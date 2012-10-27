#include <AcqirisD1Import.h>
#include <AcqirisImport.h>
#include<stdio.h>
#include<stdlib.h>

int main(int argc, char **argv)
{
    ViInt32 nbrInstruments;
    ViStatus status;
    int module;
    int id[20];
    char name[20];

    status = Acqrs_getNbrInstruments(&nbrInstruments);

    if (status != VI_SUCCESS) {
        fprintf(stderr, "Cannot get number of instruments?!?\n");
        return 0;
    }
    printf("Found %d instruments!\n", nbrInstruments);
    for (module=0; module<nbrInstruments; module++) {
        ViString options = "cal=0 dma=1";
        sprintf(name, "PCI::INSTR%d", module);
        status = Acqrs_InitWithOptions(name, VI_FALSE, VI_TRUE, options, (ViSession*)&id[module]);
        if (status != VI_SUCCESS) {
            fprintf(stderr, "*** Init failed (%x) for instrument %s\n", (unsigned)status, name);
            continue;
        }

        status = Acqrs_calibrate(id[module]); /* 0xbffa4900 - ACQ_TIMEOUT */
        printf("Calibration returns %x\n", status);
    }
}
