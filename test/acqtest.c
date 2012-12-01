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
}
