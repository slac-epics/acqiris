#include "acqiris_daq.hh"
#include "acqiris_drv.hh"

#include <AcqirisD1Import.h>
#include <epicsTime.h>
//#include <evrTime.h>

#include <stdio.h>

#define SUCCESSREAD(x) (((x)&0x80000000)==VI_SUCCESS)
#define	DEBUG_CHANNEL	ad->nchannels
//#define	DEBUG_CHANNEL	0
#define	TRIG_BLOCK_PROTECTION	10

#define STORE_DEBUG(time_a)		debug_buffer[ad->data[DEBUG_CHANNEL].write_ptr]=time_a; \
		ad->data[DEBUG_CHANNEL].write_ptr+=1;	\
		if (ad->data[DEBUG_CHANNEL].write_ptr ==AQ_DBG_SAMPLE_COUNT)	\
			{ (*debug_ch_rearm_ptr)=0;ad->data[DEBUG_CHANNEL].write_ptr=0;}
/*
#define STORE_DEBUG(time_a)		debug_buffer[ad->data[DEBUG_CHANNEL].write_ptr]=time_a; \
		printf("REARM was found to be 1 D[%d]=%d\n",ad->data[DEBUG_CHANNEL].write_ptr, \
			debug_buffer[ad->data[DEBUG_CHANNEL].write_ptr]);	\
		ad->data[DEBUG_CHANNEL].write_ptr+=1;	\
		if (ad->data[DEBUG_CHANNEL].write_ptr ==AQ_DBG_SAMPLE_COUNT)	\
			{ (*debug_ch_rearm_ptr)=0;ad->data[DEBUG_CHANNEL].write_ptr=0;}
*/


extern "C"
{
  void acqiris_daq_thread(void *arg)
  {
	unsigned long timeStampHi,timeStampLo;
	short* debug_ch_rearm_ptr;
	short free_running_counter;
	epicsTimeStamp ept_A,ept_B,ept_A_B;
	epicsTimeStamp* rec_time_ptr;
	unsigned char trigger_skip_count=0;
	unsigned char debug_skip_count;
	unsigned char debug_timeout_count;
    acqiris_driver_t* ad = reinterpret_cast<acqiris_driver_t*>(arg);
    int nchannels = ad->nchannels;
    int extra = ad->extra;
	short dummy_buf;

    const int nbrSegments = 1;
    int nbrSamples = ad->maxsamples;

    AqReadParameters    readParams,dummyReadParams;
    AqDataDescriptor    wfDesc;
    AqSegmentDescriptor segDesc[nbrSegments];

    readParams.dataType         = ReadInt16;
    readParams.readMode         = ReadModeStdW;
    readParams.nbrSegments      = nbrSegments;
    readParams.firstSampleInSeg = 0;
    readParams.firstSegment     = 0;
    readParams.segmentOffset    = 0;
    readParams.segDescArraySize = nbrSegments*sizeof(AqSegmentDescriptor);
    readParams.nbrSamplesInSeg  = nbrSamples;
    readParams.dataArraySize    = (nbrSamples+extra)*nbrSegments*sizeof(short);

    dummyReadParams=readParams;
    dummyReadParams.nbrSamplesInSeg  = 1;
    dummyReadParams.dataArraySize  = 1*sizeof(short);
    dummyReadParams.nbrSegments  = 1;
    dummyReadParams.segDescArraySize = 1*sizeof(AqSegmentDescriptor);

    while (1) {
      epicsEventWait(ad->run_semaphore);
	free_running_counter=0;
      do { 
	free_running_counter++;
        const long timeout = 1000; /* ms */
        int id = ad->id;

        AcqrsD1_acquire(id);	//Start a fresh cycle of acquisition

	//Wait for the acq to complete
        ViStatus status = AcqrsD1_waitForEndOfAcquisition(id, timeout);

	//Record time stamp (this should be the absolute first thing to do after a scan)
	epicsTimeGetEvent(&ept_A, 1);
	epicsTimeGetEvent(&ept_B, 140);
//	evrTimeGet(&ept_B, 140);

printf("Acq done, PULSEID=0x%x (FC:%d)\t",
((ept_A).nsec & (0x0001FFFF)),free_running_counter
);


	//Decide whether to use this acq or not
	unsigned short rarm_flag;
	rarm_flag=0;
	if((ad->version>0)&&(trigger_skip_count < TRIG_BLOCK_PROTECTION)){
//printf("Adding rearm pointers of valid channels\n");
	  for (int channel=0; channel<nchannels; channel++)
	    { rarm_flag+=(*(ad->data[channel].rarm_ptr)); }
	  }

//printf("(Version %d) RarmSum=%d\n",ad->version,rarm_flag);
	if(rarm_flag==ad->nchannels)
	  {trigger_skip_count++;
debug_skip_count++;
		epicsMutexLock(acqiris_dma_mutex);
		status = AcqrsD1_readData(id, 1, &dummyReadParams, &dummy_buf, &wfDesc, &segDesc);
		epicsMutexUnlock(acqiris_dma_mutex);
		  timeStampHi= (unsigned long)(segDesc->timeStampHi);
		  timeStampLo= (unsigned long)(segDesc->timeStampLo);
///printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXSkipping Trig\n");
	  }
	else
	  {/*This means the external source has cleared the at least one of the
	  re-arm flags. Hence it is ok to use this trigger*/
//printf("\tTaking this trigger\t");
	  trigger_skip_count=0;

	  // Sanity check on hardware (if it went into time out or not. If so. don't bother
	  // about further processing
	  if (status != VI_SUCCESS) {
	    AcqrsD1_stopAcquisition(id); //Should also reset the board here...?
	    ad->timeouts++;
		debug_timeout_count++;
	  } else {

	    //  All checks are passed.
	    epicsMutexLock(ad->daq_mutex);

	  //--Trigger Processing: 1. Load time stamp to appropriate records
	  for (int channel=0; channel<nchannels; channel++)
	    { rec_time_ptr=ad->data[channel].time_ptr;
	    if (NULL!=rec_time_ptr){*rec_time_ptr=ept_A;} }

	    //--Trigger Processing: 2. copy all channels into PC memory.
	    for (int channel=0; channel<nchannels; channel++)
		{
		// Set the re-arm flag so the EPICS records will clear it when done.
		*(ad->data[channel].rarm_ptr)=1;
		void* buffer = ad->data[channel].buffer;
		epicsMutexLock(acqiris_dma_mutex);
		status = AcqrsD1_readData(id, channel+1, &readParams, buffer, &wfDesc, &segDesc);
		epicsMutexUnlock(acqiris_dma_mutex);
//-----Test purposes: take over the first few samples.
reinterpret_cast<short*> (ad->data[channel].buffer)[0]=free_running_counter;
/*
reinterpret_cast<short*> (ad->data[channel].buffer)[1]=-6+channel;
reinterpret_cast<short*> (ad->data[channel].buffer)[2]=-7+channel;
reinterpret_cast<short*> (ad->data[channel].buffer)[3]=-9+channel;
reinterpret_cast<short*> (ad->data[channel].buffer)[4]=30000+channel;
reinterpret_cast<short*> (ad->data[channel].buffer)[5]=-12+channel;
reinterpret_cast<short*> (ad->data[channel].buffer)[6]=-13+channel;
reinterpret_cast<short*> (ad->data[channel].buffer)[7]=-14+channel;
reinterpret_cast<short*> (ad->data[channel].buffer)[8]=-14+channel;
reinterpret_cast<short*> (ad->data[channel].buffer)[9]=-12+channel;
reinterpret_cast<short*> (ad->data[channel].buffer)[10]=20000+channel;
*/
//-----Test purposes: take over the first few samples.

		if (SUCCESSREAD(status))
		  {
//printf("\tDMA success.\n");
		  ad->data[channel].nsamples = wfDesc.returnedSamplesPerSeg;
		  timeStampHi= (unsigned long)(segDesc->timeStampHi);
		  timeStampLo= (unsigned long)(segDesc->timeStampLo);
		  }
		else
		  {
//printf("\tDMA fail.\n");
		  ad->data[channel].nsamples = 0;
		  ad->readerrors++;
		  timeStampHi= 0;
		  timeStampLo= 0;
		  }
		}
//printf("2\n");

            epicsMutexUnlock(ad->daq_mutex);
            scanIoRequest(ad->ioscanpvt);
            ad->count++;
	    }
	  }
//	ept_A_B=ept_B-ept_A;
//printf("dt=%d\n",ept_A.secPastEpoch,ept_A_B.nsec);
/////////////////////////////////////////////////////////////////////////
	  //--------------------------------------------------------
	  //--Debug Dump. This part is common to skipped or non-skipped
	  //  triggers.
	  debug_ch_rearm_ptr=ad->data[DEBUG_CHANNEL].rarm_ptr;
	  if( (NULL!=debug_ch_rearm_ptr) && (0< (*debug_ch_rearm_ptr) ) )
	    {ad->data[DEBUG_CHANNEL].read_ptr=0;
	    short*debug_buffer= reinterpret_cast<short*> (ad->data[DEBUG_CHANNEL].buffer);
//debug_timeout_count=5;
//debug_skip_count=4;
printf("dumping Debug channel at %d\n",ad->data[DEBUG_CHANNEL].write_ptr);

	    //Dump data here. Once done, then set the following to 0.

//	    STORE_DEBUG((unsigned short)(ept_A.nsec))		//2
//	    STORE_DEBUG((unsigned short)(ept_B.nsec))		//3
	    STORE_DEBUG((unsigned short)free_running_counter)		//4
//	    STORE_DEBUG( (unsigned short)debug_timeout_count)
	    STORE_DEBUG( (((unsigned short)debug_skip_count)<<8)&(0xffff) |((unsigned short)debug_timeout_count))			//1
	    STORE_DEBUG((unsigned short)(timeStampLo))		//4
	    STORE_DEBUG((unsigned short)(timeStampLo>>16))	//5
	    STORE_DEBUG((unsigned short)(timeStampHi))		//6
	    STORE_DEBUG((unsigned short)(timeStampHi>>16))	//7
/*
*/

//	    STORE_DEBUG(debug_skip_count)
//	    STORE_DEBUG((unsigned short)(ept_A.nsec))
//	    STORE_DEBUG((unsigned short)((ept_A.nsec)>>16))

//	    STORE_DEBUG((unsigned short)(timeStampLo))
//	    STORE_DEBUG((unsigned short)(timeStampLo>>16))
//	    STORE_DEBUG((unsigned short)(timeStampHi))
//	    STORE_DEBUG((unsigned short)(timeStampHi>>16))

	    }
	  else
	    { ad->data[DEBUG_CHANNEL].write_ptr=0;debug_skip_count=0; debug_timeout_count=0;
	}
	  //  End   Processing Debug Channel. This includes any dbug variable dump if needed.
	  //--------------------------------------------------------
/////////////////////////////////////////////////////////////////////////

	//End of processing the record
	} while (ad->running);
      }
  }
}
