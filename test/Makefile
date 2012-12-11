ACQDIR=/reg/g/pcds/package/external/acqiris_3.3a

TARGETS = acqconf	acqtest
default:	$(TARGETS)

%: %.c
	g++	-m32 -O3 -o $@ -g \
		-D_POSIX_C_SOURCE=199506L	\
		-D_POSIX_THREADS	\
		-D_XOPEN_SOURCE=500 \
	    -DOSITHREAD_USE_DEFAULT_STACK	\
		-D_ACQIRIS	\
		-D_LINUX	\
		-D_X86_	\
		-DUNIX \
	    -D_BSD_SOURCE	\
		-Dlinux	\
		-D_REENTRANT \
	    -ansi	\
		-I$(ACQDIR)/include	\
	    -Wl,-rpath,$(ACQDIR)/lib	\
		-L$(ACQDIR)/lib	\
		-lAqDrv4	\
		-lpthread	$<

clean:
	@/bin/rm -f $(TARGETS)
