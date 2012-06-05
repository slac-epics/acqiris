#include "acqiris_daq.hh"
#include "acqiris_drv.hh"

#include <AcqirisD1Import.h>
#include <epicsTime.h>
#include <evrTime.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define ACQ_RESYNC 1
#define ACQ_TRACE  2
int acq_debug = ACQ_RESYNC;

#define SUCCESSREAD(x) (((x)&0x80000000)==VI_SUCCESS)

/* If the trigger isn't rearmed after this many samples, rearm it anyway! */
#define	TRIG_BLOCK_PROTECTION	10

extern "C"
{
    /* 0 = failure, 1 = success. */
    static int advance_fifo(int id, long long acq_delta, epicsTimeStamp *evr_ts,
                            unsigned long long *idx, int trigevent)
    {
        epicsTimeStamp evr_ts_new;
        long long evr_delta = 0;
        int cnt = 0;
        int status = 0;

        if (!acq_delta)                 /* Don't move! */
            return 1;
        acq_delta /= 1000;              /* ps -> ns! */
        if (acq_debug & ACQ_TRACE) {
            printf("advance%d: idx = %lld, acq_delta = %lld.%09lld, cnt = %d, evr_delta = %lld.%09lld, ts = %d.%09d\n",
                   id, *idx,
                   acq_delta / 1000000000L, acq_delta % 1000000000L, cnt,
                   evr_delta / 1000000000L, evr_delta % 1000000000L, evr_ts->secPastEpoch, evr_ts->nsec);
        }
        while (evr_delta <= acq_delta) {
            if (status = evrTimeGetFifo(&evr_ts_new, trigevent, idx, 1))
                break;
            if (evr_ts_new.nsec > evr_ts->nsec)
                evr_delta = ((long long)(evr_ts_new.secPastEpoch - evr_ts->secPastEpoch) * 1000000000ll +
                             (long long)(evr_ts_new.nsec - evr_ts->nsec));
            else
                evr_delta = ((long long)(evr_ts_new.secPastEpoch - 1 - evr_ts->secPastEpoch) * 1000000000ll +
                             (long long)(1000000000ll + evr_ts_new.nsec - evr_ts->nsec));
            cnt++;
            if (acq_debug & ACQ_TRACE) {
                printf("advance%d: idx = %lld, acq_delta = %lld.%09lld, cnt = %d, evr_delta = %lld.%09lld, ts = %d.%09d\n",
                       id, *idx,
                       acq_delta / 1000000000L, acq_delta % 1000000000L, cnt,
                       evr_delta / 1000000000L, evr_delta % 1000000000L, evr_ts_new.secPastEpoch, evr_ts_new.nsec);
            }
            if (llabs(evr_delta - acq_delta) < 3000000) {/* Within 3ms looks like a match! */
                *evr_ts = evr_ts_new;
                if (cnt != 1 && acq_debug & ACQ_RESYNC) {
                    printf("At fiducial 0x%x, advanced %d steps!\n", evr_ts_new.nsec & 0x1ffff, cnt);
                    fflush(stdout);
                }
                return 1;
            }
        }
        if (acq_debug & ACQ_RESYNC) {
            if (status)
                printf("acqiris %d evrTimeGetFifo call failed!\n", id);
            else
                printf("acqiris %d synchronization lost at fiducial 0x%x (acq_delta=%lld ns)\n",
                       id, evr_ts_new.nsec & 0x1ffff, acq_delta);
            fflush(stdout);
        }
        return 0;
    }

    static ViStatus flush(acqiris_driver_t* ad, AqReadParameters *dummyReadParams, char *dummy_buf,
                          unsigned char *trigger_skip_count, unsigned long long *acq_ts)
    {
        AqDataDescriptor    wfDesc;
        const int nbrSegments = 1;
        AqSegmentDescriptor segDesc[nbrSegments];
        ViStatus status;
        unsigned long long ts;

        (*trigger_skip_count)++;
        epicsMutexLock(acqiris_dma_mutex);
        status = AcqrsD1_readData(ad->id, 1, dummyReadParams, dummy_buf, &wfDesc, &segDesc);
        epicsMutexUnlock(acqiris_dma_mutex);
        ts = ((((unsigned long long)segDesc[0].timeStampHi) << 32) | 
              ((unsigned long long)segDesc[0].timeStampLo));
        if (acq_ts)
            *acq_ts = ts;
        if (acq_debug & ACQ_TRACE) {
            if (SUCCESSREAD(status))
                printf("F%d:%08x:%llu.%012llu\n", ad->module, (int) status,
                       ts / 1000000000000ll, ts % 1000000000000ll);
            else
                printf("f%d:%08x:%llu.%012llu\n", ad->module, (int) status,
                       ts / 1000000000000ll, ts % 1000000000000ll);
            fflush(stdout);
        }
        return status;
    }

    void acqiris_daq_thread(void *arg)
    {
        acqiris_driver_t* ad = reinterpret_cast<acqiris_driver_t*>(arg);
	unsigned long long acq_ts = 0;
        epicsTimeStamp evt_time;
	unsigned char trigger_skip_count=0;
        const int nbrSegments = 1;
        AqReadParameters    readParams,dummyReadParams;
        AqDataDescriptor    wfDesc;
        AqSegmentDescriptor segDesc[nbrSegments];
        unsigned long long idx = -1;
        int in_sync = 0;
        int trigevent = *ad->trigger;
        unsigned int gen = *ad->gen;
        int eventvalid = trigevent > 0 && trigevent < 256;
        int do_print = 0;
        int resynching = 0;
        char *dummy_buf;
    
        if (eventvalid)
            printf("acqiris%d: Setting event trigger to %d\n", ad->module, trigevent);
        else
            printf("acqiris%d: Event trigger %d is invalid!\n", ad->module, trigevent);

        /*
         * A note on timing:
         *
         * The acqiris has its own picosecond timestamp, which we keep in acq_ts.
         * The EVR timestamps are kept in a fifo, which we index with idx.
         *
         * We assume that these are in sync.  If they are not, we do an acquisition and
         * immediately look at lastfid (the last fiducial seen by the interrupt handler).
         * We then find a timestamp in our queue that is "close", but earlier than, this
         * fiducial.  Once we are back in sync, we can use the queue from here on for
         * timing.
         *
         * Whenever we do an acquisition, we check the acqiris time delta and use this to
         * advance the EVR queue pointer appropriately.
         */

        /* The parameters to read all of the data. */
        memset(&readParams, 0, sizeof(readParams));
        readParams.dataType         = ReadInt16;
        readParams.readMode         = ReadModeStdW;
        readParams.nbrSegments      = nbrSegments;
        readParams.firstSampleInSeg = 0;
        readParams.segmentOffset    = 0;
        readParams.firstSegment     = 0;
        readParams.nbrSamplesInSeg  = ad->maxsamples;
        readParams.dataArraySize    = (ad->maxsamples+ad->extra)*nbrSegments*sizeof(short);
        readParams.segDescArraySize = nbrSegments*sizeof(AqSegmentDescriptor);

        dummy_buf = (char *)malloc(readParams.dataArraySize);

#if 0
        dummyReadParams=readParams;
#else
        /* The parameters to read one sample, just to clear the device. */
        dummyReadParams=readParams;
        dummyReadParams.nbrSamplesInSeg  = 1;
        dummyReadParams.dataArraySize  = (1+ad->extra)*nbrSegments*sizeof(short);
#endif

        printf("acqiris_daq_thread(%d) has been created!\n", ad->module);fflush(stdout);

        while (1) {
            epicsEventWait(ad->run_semaphore);
            printf("acqiris_daq_thread(%d) is running!\n", ad->module);fflush(stdout);
            /*
             * We just woke up.  Assume we're out of sync!
             */
            in_sync = 0;
            resynching = 0;
            do { 
                const long timeout = 4000; /* MCB - 4s is forever.  If we don't come 
                                              back by then, skip it! */
                int id = ad->id;

                /*
                 * Start a fresh cycle of acquisition and wait for it to complete.
                 */
                ViStatus status = AcqrsD1_acquire(id);	
                if (status != VI_SUCCESS) {
                    if (acq_debug & ACQ_TRACE) {
                        printf("Acquire fails?!?\n");
                        fflush(stdout);
                    }
                }
                status = AcqrsD1_waitForEndOfAcquisition(id, timeout);
                if (status != VI_SUCCESS) {
                    AcqrsD1_stopAcquisition(id); // Should also reset the board here...?
                    ad->timeouts++;
                    if (acq_debug & ACQ_TRACE) {
                        printf("T%d:%x\n", ad->module, lastfid);
                        fflush(stdout);
                    }
                    continue;
                } else {
                    if (acq_debug & ACQ_TRACE) {
                        printf("A%d:%x\n", ad->module, lastfid);
                        fflush(stdout);
                    }
                }

                if (gen != *ad->gen) {
                    /* The trigger event changed, so print a message and force a resync! */
                    trigevent = *ad->trigger;
                    gen = *ad->gen;
                    in_sync = 0;
                    resynching = 0;
                    
                    if (eventvalid) {
                        eventvalid = trigevent > 0 && trigevent < 256;
                        if (eventvalid)
                            printf("acqiris %d: Setting event trigger to %d\n", ad->module, trigevent);
                        else
                            printf("acqiris %d: Event trigger %d is invalid!\n", ad->module, trigevent);
                    } else {
                        eventvalid = trigevent > 0 && trigevent < 256;
                        if (eventvalid)
                            printf("acqiris %d: Setting event trigger to %d\n", ad->module, trigevent);
                    }
                }
 

                /*
                 * At this point, we have just done a data acquisition.
                 *
                 * We drop the data if:
                 *     - We are not re-armed.
                 *     - We are not in sync.
                 *     - The timing event isn't valid.
                 *
                 * Otherwise, we read it and process it.
                 *
                 * We arrive here twice during a resync.  The first time is when we have
                 * just fallen out of sync, and resynching is 0.  In this case, the data
                 * could be from an old trigger, so we drop it and set resynching to 1.
                 *
                 * The second time, we know the data has come from a "new" trigger.  We
                 * take a look at the most recent fiducial seen by the interrupt handler
                 * (lastfid) and assume that the trigger time has to be earlier, but "close".
                 * We find the proper timestamp in the timestamp queue, and declare ourselves
                 * synchronized.
                 *
                 * If the user tries to reconfigure or if we receive a bad fiducial during
                 * this, we just start over.
                 */
                if (!in_sync && eventvalid) {
                    if (!resynching) {
                        resynching = 1;
                        if (acq_debug & ACQ_RESYNC) {
                            printf("acqiris %d resynchronizing at actual fiducial 0x%x.\n",
                                   ad->module, lastfid);
                            fflush(stdout);
                        }
                        flush(ad, &dummyReadParams, dummy_buf, &trigger_skip_count, NULL);
                        continue;
                    } else {
                        int savefid, newfid;

                        resynching = 0;

                        /*
                         * lastfid is saved in the event interrupt handler.  It is probably the
                         * most accurate timestamp we have.  It should be "close" to the trigger
                         * timestamp.
                         *
                         * It's a little weird though, since the fiducial event has an index one
                         * less than the other events that arrive in the same interrupt.  The
                         * acqiris seems to return very quickly, so we're cheating here by adding
                         * 2.  It seems to work though.
                         */
                        savefid = lastfid + 2;

                        /* Get the current time! */
                        status = evrTimeGetFifo(&evt_time, trigevent, &idx, MAX_TS_QUEUE);
                        newfid = evt_time.nsec & 0x1ffff;
                        if (newfid == 0x1ffff) {
                            if (acq_debug & ACQ_RESYNC) {
                                printf("acqiris %d: Bad fiducial at time %08x:%08x!\n", ad->module,
                                       evt_time.secPastEpoch, evt_time.nsec);
                                fflush(stdout);
                            }
                            flush(ad, &dummyReadParams, dummy_buf, &trigger_skip_count, NULL);
                            continue;
                        }

                        if (acq_debug & ACQ_RESYNC) {
                            printf("acqiris %d resync sees timestamp %d.%09d fiducial 0x%x at actual fiducial 0x%x.\n",
                                   ad->module, evt_time.secPastEpoch, evt_time.nsec, newfid, savefid);
                            fflush(stdout);
                        }

                        if (FID_GT(newfid, savefid)) {
                            /*
                             * The timestamp is more recent than the most recent fiducial?!?
                             *
                             * We must have prematurely rotated the timestamp buffers.  As long
                             * as we are close, nothing is seriously wrong here.
                             */
                            if (FID_DIFF(newfid, savefid) > 2) {
                                printf("acqiris %d is looking into the distant future?!?\n", ad->module);
                                fflush(stdout);
                                /*
                                 * I'd write code to fix this, but we should never be
                                 * here in the first place!
                                 */
                                flush(ad, &dummyReadParams, dummy_buf, &trigger_skip_count, NULL);
                                continue;
                            }

                            /*
                             * Go back one event.
                             */
                            status = evrTimeGetFifo(&evt_time, trigevent, &idx, -1);
                            newfid = evt_time.nsec & 0x1ffff;
                            if (newfid == 0x1ffff) {
                                if (acq_debug & ACQ_RESYNC) {
                                    printf("acqiris %d resync sees a bad fiducial, restarting!\n", ad->module);
                                    fflush(stdout);
                                }
                                flush(ad, &dummyReadParams, dummy_buf, &trigger_skip_count, NULL);
                                continue;
                            }
                            if (acq_debug & ACQ_RESYNC) {
                                printf("acqiris %d is moving back to timestamp fiducial 0x%x at index %lld.\n",
                                       ad->module, newfid, idx);
                                fflush(stdout);
                            }
                        }
                        /*
                         * Our trigger was close to savefid, but the timestamps have not been rotated
                         * yet.  Wait for them to rotate until we get something close!
                         */
                        while (FID_DIFF(savefid, newfid) > 5) {
                            unsigned long long idx2;
                            do
                                status = evrTimeGetFifo(&evt_time, trigevent, &idx2, MAX_TS_QUEUE);
                            while (idx == idx2 || gen != *ad->gen);
                            idx = idx2;
                            newfid = evt_time.nsec & 0x1ffff;
                            if (gen != *ad->gen || newfid == 0x1ffff)
                                break;
                        }
                        if (gen != *ad->gen || newfid == 0x1ffff) {
                            /* This is just bad.  Flush and hope for better the next time we're here. */
                            if (acq_debug & ACQ_RESYNC) {
                                printf("acqiris %d resync failed with fiducial 0x%x, restarting!\n",
                                       ad->module, (evt_time.nsec & 0x1ffff));
                                fflush(stdout);
                            }
                            flush(ad, &dummyReadParams, dummy_buf, &trigger_skip_count, NULL);
                            continue;
                        }
                        /*
                         * We should probably check that we are, indeed, close.  What if newfid skipped
                         * *past* our real fiducial?  (Unlikely, I know.)
                         */
                        if (SUCCESSREAD(flush(ad, &dummyReadParams, dummy_buf, &trigger_skip_count, &acq_ts))) {
                            in_sync = 1;
                            do_print = 1; /* Talk the next time through the loop! */
                            if (acq_debug & ACQ_RESYNC) {
                                printf("acqiris %d resync established with index %lld at timestamp %d.%09d fiducial 0x%x at actual fiducial 0x%x (now=0x%x).\n",
                                       ad->module, idx, evt_time.secPastEpoch, evt_time.nsec, newfid, savefid, lastfid);
                                fflush(stdout);
                            }
                        } else {
                            if (acq_debug & ACQ_RESYNC) {
                                printf("acqiris %d resync failed to read from acqiris, restarting!\n",
                                       ad->module);
                                fflush(stdout);
                            }
                        }
                        continue;
                    }
                }

                // Count the rearm flags.
                unsigned short rarm_flag;
                rarm_flag=0;
                if((ad->version>0)&&(trigger_skip_count < TRIG_BLOCK_PROTECTION)){
                    for (int channel=0; channel<ad->nchannels; channel++)
                        rarm_flag+=(*(ad->data[channel].rarm_ptr)); 
                }

                if (rarm_flag==ad->nchannels || !in_sync || !eventvalid || resynching) {
                    unsigned long long acq_ts_now;
                    if (acq_debug & ACQ_RESYNC) {
                        printf("acqiris %d rarm=%d, sync=%d, valid=%d, resync=%d\n",
                               ad->module, rarm_flag, in_sync, eventvalid, resynching);
                        fflush(stdout);
                    }
                    status = flush(ad, &dummyReadParams, dummy_buf, &trigger_skip_count, &acq_ts_now);
                    if (SUCCESSREAD(status) && in_sync) {
                        in_sync = advance_fifo(ad->module, acq_ts_now - acq_ts, &evt_time,
                                               &idx, trigevent);
                        acq_ts = acq_ts_now;
                        if (acq_debug & ACQ_RESYNC) {
                            printf("acqiris %d is flushing index %lld at fiducial 0x%x (lastfid = 0x%x, acq_ts=%llu, rarm=%d).\n",
                                   ad->module, idx, evt_time.nsec & 0x1ffff, lastfid, acq_ts, rarm_flag);
                            fflush(stdout);
                        }
                    }
                } else {
                    /* The external source has cleared at least one of the re-arm flags. */
                    unsigned readerrors_old = ad->readerrors;
                    trigger_skip_count=0;

                    epicsMutexLock(ad->daq_mutex);

                    //--Trigger Processing: copy all channels into PC memory and load the timestamps.
                    for (int channel=0; channel<ad->nchannels; channel++) {
                        // Set the re-arm flag so the EPICS records will clear it when done.
                        *(ad->data[channel].rarm_ptr) = 1;
                        void* buffer = ad->data[channel].buffer;
                        epicsMutexLock(acqiris_dma_mutex);
                        status = AcqrsD1_readData(id, channel+1, &readParams, buffer, &wfDesc, &segDesc);
                        epicsMutexUnlock(acqiris_dma_mutex);

                        if (SUCCESSREAD(status)) {
                            unsigned long long acq_ts_now =
                                ((((unsigned long long)segDesc[0].timeStampHi) << 32) | 
                                 ((unsigned long long)segDesc[0].timeStampLo));
                            epicsTimeStamp* rec_time_ptr = ad->data[channel].time_ptr;
                            if (acq_debug & ACQ_TRACE) {
                                printf("R%d-%d:%llu.%012llu\n", ad->module, channel,
                                       acq_ts_now / 1000000000000ll, acq_ts_now % 1000000000000ll);
                                fflush(stdout);
                            }

                            /* This *should* only advance for channel 0. */
                            if (!advance_fifo(ad->module, acq_ts_now - acq_ts, &evt_time,
                                              &idx, trigevent)) {
                                in_sync = 0;
                                do_print = 0;
                            }
                            acq_ts = acq_ts_now;

                            if (do_print) {
                                unsigned int newfid = evt_time.nsec & 0x1ffff;
                                do_print = 0;
                                if (newfid == 0x1ffff) {
                                    if (acq_debug & ACQ_RESYNC) {
                                        printf("acqiris %d has a bad fiducial (lastfid = 0x%x), resynching!\n",
                                               ad->module, lastfid);
                                        fflush(stdout);
                                    }
                                    in_sync = 0;
                                    break;
                                } else {
                                    if (acq_debug & ACQ_RESYNC) {
                                        printf("acqiris %d is fully resynched with index %lld at fiducial 0x%x (lastfid = 0x%x, acq_ts=%llu).\n",
                                               ad->module, idx, newfid, lastfid, acq_ts);
                                        fflush(stdout);
                                    }
                                }
                            }
                            
                            if (NULL!=rec_time_ptr)
                                *rec_time_ptr = evt_time;
                            ad->data[channel].nsamples = wfDesc.returnedSamplesPerSeg;
                        } else {
                            if (acq_debug & ACQ_TRACE) {
                                printf("R%d-%d:-1\n", ad->module, channel);
                                fflush(stdout);
                            }
                            ad->data[channel].nsamples = 0;
                            ad->readerrors++;
                        }
                    }

                    epicsMutexUnlock(ad->daq_mutex);

                    if (do_print) {
                        printf("acqiris %d failed to read data after resync, restarting!\n", ad->module);
                        fflush(stdout);
                        do_print = 0;
                        in_sync = 0;
                    }

                    if (in_sync && ad->readerrors == readerrors_old)
                        scanIoRequest(ad->ioscanpvt);  /* It all worked!  Move on! */
                    ad->count++;
                }
            } while (ad->running);
        }
    }
}
