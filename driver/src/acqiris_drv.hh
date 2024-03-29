#ifndef ACQIRIS_DRV_HH
#define ACQIRIS_DRV_HH

#define MAX_CHANNEL 10
#define MAX_DEV 10
#define AQ_DBG_SAMPLE_COUNT	6600

#include <dbScan.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsThread.h>
#include <epicsTime.h>

extern "C" {
struct acqiris_data_t {
  unsigned nsamples;
  void* buffer;
  short* rarm_ptr;
  unsigned read_ptr;
  unsigned write_ptr;
  epicsTimeStamp* time_ptr;
};

struct acqiris_driver_t {
  int module;
  int nchannels;
  int id;
  epicsUInt32 *trigger;
  epicsUInt32 *gen;
  double      *delay;
  IOSCANPVT ioscanpvt;
  epicsEventId run_semaphore;
  epicsMutexId daq_mutex;
  int running;
  int extra;
  int maxsamples;
  unsigned count;
  unsigned timeouts;
  unsigned readerrors;
  unsigned truncated;
  unsigned char version;
  acqiris_data_t data[MAX_CHANNEL];
  char *sync;
  int do_ts;
};
typedef struct acqiris_driver_t ad_t;

extern acqiris_driver_t acqiris_drivers[];
extern unsigned nbr_acqiris_drivers;
extern epicsMutexId acqiris_dma_mutex;
} // extern "C"

#define SUCCESS(x) (((x)&0x80000000) == 0)

#endif
