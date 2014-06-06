#ifndef ACQIRIS_SYNC_HH
#define ACQIRIS_SYNC_HH

#include"timesync.h"

class acqirisSyncObject : public SyncObject {
public:
    acqirisSyncObject(acqiris_driver_t *_acqiris);
    ~acqirisSyncObject()               {};
    DataObject *Acquire(void);
    int CheckError(DataObject *dobj)   { return 0; }
    const char *Name(void)             { return name; }
    int Fiducial(DataObject *dobj, int lastdatafid);
    int Attributes(void)               { return HasTime; }
    void QueueData(DataObject *dobj, epicsTimeStamp &evt_time);
    void FlushData(void);
private:
    acqiris_driver_t *acqiris;
    AqReadParameters    readParams, dummyReadParams;
    AqDataDescriptor    wfDesc[4];
    AqSegmentDescriptor segDesc[4];
    char name[40];
    double delay;
    unsigned long long acq_ts;
    int trigger_skip_count;
    char *dummy_buf;
};
#endif
