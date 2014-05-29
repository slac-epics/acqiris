#include "acqiris_drv.hh"
#include "acqiris_daq.hh"

#include <AcqirisD1Import.h>
#include <epicsTime.h>
#include <evrTime.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "acqiris_sync.hh"

#define ACQ_TRACE  1
int acq_debug = 0;

#define SUCCESSREAD(x) (((x)&0x80000000)==VI_SUCCESS)

/* If the trigger isn't rearmed after this many samples, rearm it anyway! */
#define	TRIG_BLOCK_PROTECTION	10
#define ONE_FID_PS              2777777777LL
#define HALF_FID_PS             1333333333LL
#define TS_ERROR_PS             HALF_FID_PS

extern "C"
{
    void acqiris_daq_thread(void *arg)
    {
        acqiris_driver_t* ad = reinterpret_cast<acqiris_driver_t*>(arg);

        acqirisSyncObject *sobj = new acqirisSyncObject(ad);
        Synchronizer  *sync = new Synchronizer(sobj);

        sync->poll();
    }
}

// Initialize, return 0 if OK.
int acqirisSyncObject::Init(void)
{
    m_delay = &delay;
    delay   = -0.5;  // MCB - To correct the rounding in timesync.

    /* The parameters to read all of the data. */
    memset(&readParams, 0, sizeof(readParams));
    readParams.dataType         = ReadInt16;
    readParams.readMode         = ReadModeStdW;
    readParams.nbrSegments      = 1;
    readParams.firstSampleInSeg = 0;
    readParams.segmentOffset    = 0;
    readParams.firstSegment     = 0;
    readParams.nbrSamplesInSeg  = acqiris->maxsamples;
    readParams.dataArraySize    = (acqiris->maxsamples+acqiris->extra)*sizeof(short);
    readParams.segDescArraySize = sizeof(AqSegmentDescriptor);

    /* The parameters to read one sample, just to clear the device. */
    dummyReadParams                 = readParams;
    dummyReadParams.nbrSamplesInSeg = 1;
    dummyReadParams.dataArraySize   = (1+acqiris->extra)*sizeof(short);

    sprintf(name, "Acqiris(%d)", acqiris->module);

    dummy_buf = (char *)malloc(readParams.dataArraySize);

    epicsEventWait(acqiris->run_semaphore);
    printf("acqiris_daq_thread(%d) is running!\n", acqiris->module);

    m_event = acqiris->trigger;
    m_gen   = acqiris->gen;

    return 0;
}

// Get some data and return it encapsulated.
DataObject *acqirisSyncObject::Acquire(void)
{
    const long timeout = 4000; /* MCB - 4s is essentially forever. */
    int id = acqiris->id;

    // Save the last timestamp.
    acq_ts = ((((unsigned long long)segDesc[0].timeStampHi) << 32) | 
              ((unsigned long long)segDesc[0].timeStampLo));

    for (;;) {
        /*
         * Start a fresh cycle of acquisition and wait for it to complete.
         */
        ViStatus status = AcqrsD1_acquire(id);	
        if (status != VI_SUCCESS) {
            if (acq_debug & ACQ_TRACE) {
                printf("Acquire fails?!?\n");
            }
        }
        status = AcqrsD1_waitForEndOfAcquisition(id, timeout);
        if (status != VI_SUCCESS) {
            AcqrsD1_stopAcquisition(id); // Should also reset the board here...?
            acqiris->timeouts++;
            if (acq_debug & ACQ_TRACE) {
                printf("T%d:%x\n", acqiris->module, lastfid);
            }
            continue;
        } else {
            if (acq_debug & ACQ_TRACE) {
                printf("A%d:%x\n", acqiris->module, lastfid);
            }
        }

#if 0
        // Count the rearm flags.
        unsigned short rarm_flag = 0;
        if((acqiris->version>0)&&(trigger_skip_count < TRIG_BLOCK_PROTECTION)){
            for (int channel=0; channel<acqiris->nchannels; channel++)
                rarm_flag+=(*(acqiris->data[channel].rarm_ptr)); 
        }
        if (rarm_flag==acqiris->nchannels) {
            // Not rearmed, just skip it!
            FlushData();
            continue;
        }
#endif

        unsigned readerrors_old = acqiris->readerrors;
        trigger_skip_count = 0;

        epicsMutexLock(acqiris->daq_mutex);

        //--Trigger Processing: copy all channels into PC memory and load the timestamps.
        for (int channel=0; channel<acqiris->nchannels; channel++) {
            // Set the re-arm flag so the EPICS records will clear it when done.
            *(acqiris->data[channel].rarm_ptr) = 1;
            void* buffer = acqiris->data[channel].buffer;
            epicsMutexLock(acqiris_dma_mutex);
            status = AcqrsD1_readData(id, channel+1, &readParams, buffer, &wfDesc[channel], &segDesc[channel]);
            epicsMutexUnlock(acqiris_dma_mutex);

            if (SUCCESSREAD(status)) {
                if (acq_debug & ACQ_TRACE) {
                    unsigned long long acq_ts_now =
                        ((((unsigned long long)segDesc[channel].timeStampHi) << 32) | 
                         ((unsigned long long)segDesc[channel].timeStampLo));
                    printf("R%d-%d:%llu.%012llu\n", acqiris->module, channel,
                           acq_ts_now / 1000000000000ll, acq_ts_now % 1000000000000ll);
                }
                acqiris->data[channel].nsamples = wfDesc[channel].returnedSamplesPerSeg;
            } else {
                if (acq_debug & ACQ_TRACE) {
                    printf("R%d-%d:-1\n", acqiris->module, channel);
                }
                acqiris->data[channel].nsamples = 0;
                acqiris->readerrors++;
            }
        }

        epicsMutexUnlock(acqiris->daq_mutex);

        if (acqiris->readerrors == readerrors_old) {
            if (acq_debug & ACQ_TRACE) {
                printf("D%d\n", acqiris->module);
            }
            return new DataObject(NULL); // Really just a placeholder!
        } else {
            continue;
        }
    }
}

// Calculate the expected fiducial of this DataObject.
// acq_ts is the Acqiris time of the last acquire, and the new time is in the segment header.
int acqirisSyncObject::Fiducial(DataObject *dobj, int lastdatafid)
{
    long long acq_delta = (long long)(((((unsigned long long)segDesc[0].timeStampHi) << 32) | 
                                       ((unsigned long long)segDesc[0].timeStampLo)) - acq_ts);
    long long fiddiff = (acq_delta + HALF_FID_PS) / ONE_FID_PS;
    long long fidoff  = llabs(acq_delta - fiddiff * ONE_FID_PS);
#if 0
    printf("acq_ts = %lld, acq_delta = %lld, fiddiff = %lld, fidoff = %lld\n",
           acq_ts, acq_delta, fiddiff, fidoff);
#endif
    if (fidoff > TS_ERROR_PS)
        return -1;
    int fid = lastdatafid += fiddiff;
    while (fid > 0x1ffe0)
        fid -= 0x1ffe0;
    return fid;
}

// Receive the data and timestamp.
void acqirisSyncObject::QueueData(DataObject *dobj, epicsTimeStamp &evt_time)
{
    for (int channel=0; channel<acqiris->nchannels; channel++) {
        epicsTimeStamp* rec_time_ptr = acqiris->data[channel].time_ptr;
        if (NULL != rec_time_ptr)
            *rec_time_ptr = evt_time;
    }
    scanIoRequest(acqiris->ioscanpvt);
    acqiris->count++;
}

void acqirisSyncObject::FlushData(void)
{
    ViStatus status;

    trigger_skip_count++;
    epicsMutexLock(acqiris_dma_mutex);
    status = AcqrsD1_readData(acqiris->id, 1, &dummyReadParams, dummy_buf, &wfDesc[0], &segDesc[0]);
    epicsMutexUnlock(acqiris_dma_mutex);
    if (acq_debug & ACQ_TRACE) {
        unsigned long long ts = ((((unsigned long long)segDesc[0].timeStampHi) << 32) | 
                                 ((unsigned long long)segDesc[0].timeStampLo));
        if (SUCCESSREAD(status))
            printf("F%d:%08x:%llu.%012llu\n", acqiris->module, (int) status,
                   ts / 1000000000000ll, ts % 1000000000000ll);
        else
            printf("f%d:%08x:%llu.%012llu\n", acqiris->module, (int) status,
                   ts / 1000000000000ll, ts % 1000000000000ll);
    }
}
