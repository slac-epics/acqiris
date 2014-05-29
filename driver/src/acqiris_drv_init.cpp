#include "acqiris_drv.hh"
#include "acqiris_daq.hh"

#include <AcqirisD1Import.h>
#include <AcqirisImport.h>

#include <epicsExport.h>
#include <dbAccess.h>
#include <iocsh.h>

#include <stdio.h>
#include <string.h>
#include "evrTime.h"

extern "C" {
  acqiris_driver_t acqiris_drivers[MAX_DEV];
  unsigned nbr_acqiris_drivers = 0;
  epicsMutexId acqiris_dma_mutex;

  static void check_version_number(ViInt32 id)
  {
    const char* versionstr[] = {
      "Kernel Driver",
      "EEPROM Common Section",
      "EEPROM Instrument Section",
      "CPLD Firmware"
    };
    const ViInt32 expected[4] = { ACQIRIS_KVERSION, 0x3, 0x8, 0x2a};
    ViInt32 found[4];
    for (int v=0; v<4; v++) {
      Acqrs_getVersion(id, v+1, found+v);
      if (((found[v] & 0xffff0000) != (expected[v] & 0xffff0000)) ||
	  ((found[v] & 0x0000ffff) <  (expected[v] & 0x0000ffff))) {
	fprintf(stderr, "*** Unexpected version 0x%x for %s, expected 0x%x\n",
		(unsigned)found[v], versionstr[v], (unsigned)expected[v]);
      }
    }
  }

  static int acqiris_find_devices()
  {
    ViInt32 nbrInstruments;
    ViStatus status = Acqrs_getNbrInstruments(&nbrInstruments);
    if (status != VI_SUCCESS) {
        fprintf(stderr, "Cannot get number of instruments?!?\n");
        return 0;
    }

    int module;
    for (module=0; module<nbrInstruments; ) {
      acqiris_driver_t* ad = &acqiris_drivers[module];
      ViString options = "cal=0 dma=1";
      char name[20];
      sprintf(name, "PCI::INSTR%d", module);
      status = Acqrs_InitWithOptions(name, 
				     VI_FALSE, VI_TRUE, 
				     options, 
				     (ViSession*)&ad->id);
      if (status != VI_SUCCESS) {
	fprintf(stderr, "*** Init failed (%x) for instrument %s\n", 
		(unsigned)status, name);
	continue;
      }
      Acqrs_getNbrChannels(ad->id, (ViInt32*)&ad->nchannels);
      Acqrs_getInstrumentInfo(ad->id, "MaxSamplesPerChannel", &ad->maxsamples);
      Acqrs_getInstrumentInfo(ad->id, "TbSegmentPad", &ad->extra);
      check_version_number(ad->id);
      ad->module = module;
      module++;
    }

    return module;
  }

  static int acqirisInit(int order)
  {
    static epicsUInt32 ev140 = 140;
    nbr_acqiris_drivers = acqiris_find_devices();
    if (!nbr_acqiris_drivers) {
      fprintf(stderr, "*** Could not find any acqiris devices\n");
      return -1;
    }
    acqiris_dma_mutex = epicsMutexMustCreate();
    for (unsigned module=0; module<nbr_acqiris_drivers; module++) {
      int channel;
      char name[32];
      acqiris_driver_t* ad = &acqiris_drivers[module];

	ad->run_semaphore = epicsEventMustCreate(epicsEventEmpty);

      ad->daq_mutex = epicsMutexMustCreate();
      ad->count = 0;
      ad->trigger = &ev140;
      ad->gen = &ev140;

      for(channel=0; channel<ad->nchannels; channel++) {
	int size = (ad->maxsamples+ad->extra)*sizeof(short);
	ad->data[channel].nsamples = 0;
	ad->data[channel].buffer = new char[size];
        ad->data[channel].rarm_ptr = NULL;	//Pointer to re-arm field of record.
						//This will be later filled in by init_record.
	ad->data[channel].write_ptr=0;
	ad->data[channel].read_ptr=0;
        ad->data[channel].time_ptr = NULL;	//Pointer to re-arm field of record.
      }
        //--Variables pertaining to debug channel.
	ad->data[ad->nchannels].nsamples =AQ_DBG_SAMPLE_COUNT;
	ad->data[ad->nchannels].buffer = new char[AQ_DBG_SAMPLE_COUNT*sizeof(short)];//Debug channel is
										//short int type only
        ad->data[ad->nchannels].rarm_ptr = NULL;	//Pointer to re-arm field of record.
	ad->data[ad->nchannels].write_ptr=0;
	ad->data[ad->nchannels].read_ptr=0;
        ad->data[ad->nchannels].time_ptr = NULL;	//Pointer to re-arm field of record.

      snprintf(name, sizeof(name), "tacqirisdaq%u", module);
      scanIoInit(&ad->ioscanpvt);
      epicsThreadMustCreate(name, 
			    epicsThreadPriorityHigh + 5, // MCB epicsThreadPriorityMin?!?
			    5000000,
			    acqiris_daq_thread, 
			    ad); 
    }    
    return 0;
  }

  static int acqirisSetTrigger(int module, char *trigger)
  {
    DBADDR trigaddr;
    if (dbNameToAddr(trigger, &trigaddr)) {
        printf("No PV trigger named %s!\n", trigger);
        return 1;
    }
    acqiris_drivers[module].trigger = (epicsUInt32 *) trigaddr.pfield;
    acqiris_drivers[module].gen = MAX_EV_TRIGGERS + (epicsUInt32 *) trigaddr.pfield;
    return 0;
  }

  static const iocshArg acqirisInitArg0 = {"nSamples",iocshArgInt};
  static const iocshArg * const acqirisInitArgs[1] = {&acqirisInitArg0};
  static const iocshFuncDef acqirisInitFuncDef =
    {"acqirisInit",1,acqirisInitArgs};

  static void acqirisInitCallFunc(const iocshArgBuf *arg)
  {
    acqirisInit(arg[0].ival);
  }

  static const iocshArg acqirisSetTriggerArg0 = {"module",iocshArgInt};
  static const iocshArg acqirisSetTriggerArg1 = {"trigger", iocshArgString};
  static const iocshArg * const acqirisSetTriggerArgs[2] = {&acqirisSetTriggerArg0,
                                                            &acqirisSetTriggerArg1};
  static const iocshFuncDef acqirisSetTriggerFuncDef =
    {"acqirisSetTrigger",2,acqirisSetTriggerArgs};

  static void acqirisSetTriggerCallFunc(const iocshArgBuf *arg)
  {
    acqirisSetTrigger(arg[0].ival, arg[1].sval);
  }

static const iocshArg		acqdebugArg0	= { "level",	iocshArgInt };
static const iocshArg	*	acqdebugArgs[1]	= { &acqdebugArg0 };
static const iocshFuncDef   acqdebugFuncDef	= { "acqdebug", 1, acqdebugArgs };
extern int acq_debug;
static void  acqdebugCall( const iocshArgBuf * args )
{
    acq_debug = args[0].ival;
}

  void acqirisRegistrar()
  {
    iocshRegister(&acqirisInitFuncDef,acqirisInitCallFunc);
    iocshRegister(&acqirisSetTriggerFuncDef,acqirisSetTriggerCallFunc);
    iocshRegister(&acqdebugFuncDef,acqdebugCall);
  }

  epicsExportRegistrar(acqirisRegistrar);
}
