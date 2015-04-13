CC=gcc
CFLAGS=-O3 -W -Wall -g
LDFLAGS=-lm -lfftw3 -g

CFLAGS+=$(shell pkg-config --cflags librtlsdr)
LDFLAGS+=$(shell pkg-config --libs librtlsdr)

OBJS = wf_sync.o wf_prstables.o wf_maths.o fic.o misc.o viterbi.o dab_tables.o input_wf.o depuncture.o input_sdr.o sdr_fifo.o sdr_sync.o

all: dab2eti sdr2eti eti2mpa

eti2mpa: eti2mpa.o
	$(CC) -o eti2mpa eti2mpa.o

dab2eti.o: dab2eti.c dab.h
	$(CC) $(CFLAGS) -c -o dab2eti.o dab2eti.c

wf_sync.o: wf_sync.c
	$(CC) $(CFLAGS) -c -o wf_sync.o wf_sync.c

wf_prstables.o: wf_prstables.c
	$(CC) $(CFLAGS) -c -o wf_prstables.o wf_prstables.c

wf_maths.o: wf_maths.c
	$(CC) $(CFLAGS) -c -o wf_maths.o wf_maths.c

dab2eti: $(OBJS) dab2eti.o
	$(CC) $(LDFLAGS) -o dab2eti dab2eti.o $(OBJS)

sdr2eti: $(OBJS) sdr2eti.o
	$(CC) $(LDFLAGS) -o sdr2eti sdr2eti.o $(OBJS)

clean:
	rm -f dab2eti dab2eti.o sdr2eti sdr2eti.o eti2mpa eti2mpa.o $(OBJS) *~
