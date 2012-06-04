#include "acqiris_dev.hh"
#include "acqiris_drv.hh"

#include <dbAccess.h>
#include <recGbl.h>
#include <devSup.h>
#include <epicsExport.h>
#include <alarm.h>
#include <link.h>

#include <stdio.h>

static int acqiris_bad_field(void* record, 
			     const char* message, 
			     const char* fieldname)
{
  fprintf(stderr, "acqiris_init_record: %s %s\n", message, fieldname);
  recGblRecordError(S_db_badField, record, message);
  return S_db_badField;
}

template<class T> int acqiris_init_record(T* record, DBLINK link)
{


  if (link.type != INST_IO) {
    return acqiris_bad_field(record, "wrong link type", "");
  }
  struct instio* pinstio = &link.value.instio;
  if (!pinstio->string) {
    return acqiris_bad_field(record, "invalid link", "");
  }

  acqiris_record_t* arc = new acqiris_record_t;
  const char* sinp = pinstio->string;
  int status;
#ifdef MCB
	printf("Inp=%s\n",sinp);
#endif
  unsigned int version;
  status = sscanf(sinp, "M%u C%d %s V%u", &arc->module, &arc->channel, arc->name, &version);
  if (status != 4)
	{
	version=0;
	status = sscanf(sinp, "M%u C%d %s", &arc->module, &arc->channel, arc->name);
	if (status != 3)
		{
		status = sscanf(sinp, "M%u %s", &arc->module, arc->name);
		if (status != 2)  { 
			delete arc;
			return acqiris_bad_field(record, "cannot parse INP field", sinp);
			}
		}
  	}
  acqiris_driver_t* ad=&(acqiris_drivers[arc->module]);
  if(arc->channel<0) {
      arc->channel=ad->nchannels;
      printf("Inp=%s, Adjusted a -ve Channel to Last Channel...%d\n", sinp, arc->channel);
  }
  ad->version=ad->version>version?ad->version:version;	//Unfortunately versions are set channel wise in
				// the .template and .substitution files. Take the highest specified version
				// amongst all channels as the version of the board. Default version is 0.
#ifdef MCB
  printf("M=%d, C=%d, V=%d, N=%s\n",arc->module,arc->channel,ad->version, arc->name);
#endif

  record->dpvt = arc;

  status = acqiris_init_record_specialized(record);
  if (status) {
    record->dpvt = 0;
    delete arc;
    return acqiris_bad_field(record, "cannot find record name", sinp);
  }
#ifdef MCB
if(arc->channel==ad->nchannels){
  printf("*************************************************\n");
  printf("*                                               *\n");
  printf("*    acqiris_init_record                        *\n");
  printf("*                                               *\n");
  printf("*      Satyajit_copy                            *\n");
  printf("*                                               *\n");
  printf("*      INP=%s                                   *\n",sinp);
  printf("*      MM=%d C=%d                           *\n",arc->module, arc->channel);
  printf("*   Version %d                                   *\n",ad->version);
  printf("*   Date: 3/29/10                                *\n");
  printf("*************************************************\n");
	}
#endif
  return 0;
}

template<class T> int acqiris_read_record(T* record)
{
  acqiris_record_t* arc = reinterpret_cast<acqiris_record_t*>(record->dpvt);
  ad_t* ad = &acqiris_drivers[arc->module];
  int status = !ad->run_semaphore || acqiris_read_record_specialized(record);
  if (status) {
    record->nsta = UDF_ALARM;
    record->nsev = INVALID_ALARM;
    return -1;
  }
  return 0;
}

template<class T> int acqiris_write_record(T* record)
{
  acqiris_record_t* arc = reinterpret_cast<acqiris_record_t*>(record->dpvt);
  ad_t* ad = &acqiris_drivers[arc->module];
  int status = !ad->run_semaphore || acqiris_write_record_specialized(record);
  if (status) {
    record->nsta = UDF_ALARM;
    record->nsev = INVALID_ALARM;
    return -1;
  }
  return 0;
}

#include <longinRecord.h>
#include <longoutRecord.h>
#include <aiRecord.h>
#include <aoRecord.h>
#include <mbboRecord.h>
#include <waveformRecord.h>

template int acqiris_init_record(longinRecord*,   DBLINK);
template int acqiris_init_record(longoutRecord*,  DBLINK);
template int acqiris_init_record(aiRecord*,       DBLINK);
template int acqiris_init_record(aoRecord*,       DBLINK);
template int acqiris_init_record(mbboRecord*,     DBLINK);
template int acqiris_init_record(waveformRecord*, DBLINK);
template int acqiris_read_record(longinRecord*);
template int acqiris_read_record(longoutRecord*);
template int acqiris_read_record(aiRecord*);
template int acqiris_read_record(aoRecord*);
template int acqiris_read_record(mbboRecord*);
template int acqiris_read_record(waveformRecord*);
template int acqiris_write_record(longoutRecord*);
template int acqiris_write_record(aoRecord*);
template int acqiris_write_record(mbboRecord*);

