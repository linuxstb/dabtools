CC=gcc
CFLAGS=-O3 -W -Wall -g
LDFLAGS=-lm -lfftw3 -lpthread -g

CFLAGS+=$(shell pkg-config --cflags librtlsdr)
LDFLAGS+=$(shell pkg-config --libs librtlsdr)

OBJS = wf_sync.o wf_prstables.o wf_maths.o fic.o misc.o viterbi.o dab_tables.o input_wf.o depuncture.o input_sdr.o sdr_fifo.o sdr_sync.o dab.o

all: dab2eti eti2mpa

dab2eti: $(OBJS) dab2eti.o
	$(CC) -o dab2eti dab2eti.o $(OBJS) $(LDFLAGS)

eti2mpa: eti2mpa.o
	$(CC) -o eti2mpa eti2mpa.o

clean:
	rm -f dab2eti dab2eti.o eti2mpa eti2mpa.o $(OBJS) *~
