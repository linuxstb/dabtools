
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "dab.h"
#include "fic.h"

void usage(void)
{
  fprintf(stderr,"Usage: etiplayer N\n");
  fprintf(stderr,"          N = sub-channel ID to output to stdout\n");
}

int main(int argc, char* argv[])
{
  uint8_t buf[6144];  /* Main buffer for an ETI frame */
  int subchanid;
  int i,n;

  if (argc != 2) {
    usage();
    return 1;
  }

  subchanid = atoi(argv[1]);
  int data_offset = 0;
  int data_length = -1;

  fprintf(stderr,"Decoding channel %d\n",subchanid);
  while (1) {
    n = read(0,buf,6144);
    if (n != 6144) {
      fprintf(stderr,"Read error, exiting\n");
      return 1;
    }

    /* TODO: Check Sync etc */
    int FICF = (buf[5] & 0x80) >> 7;
    int NST = buf[5] & 0x7f;
    //fprintf(stderr,"FICF=%d, NST=%d\n",FICF,NST);

    if (data_length == -1) {
      for (i=0;i<NST;i++) {
        int SCID = (buf[8+4*i] & 0xfc) >> 2;
        int SAD = ((buf[8+4*i] & 0x03) << 8) | buf[8+4*i+1];
        int TPL = (buf[8+4*i+2] & 0xfc) >> 2;
        int STL = ((buf[8+4*i+2] & 0x03) << 8) | buf[8+4*i+3];
        if (subchanid == SCID) {
          data_length = STL * 8;
          fprintf(stderr,"Extracting channel: SCID=%d, SAD=%d, TPL=%d, STL=%d\n",SCID,SAD,TPL,STL);
          break;
        } else {
          data_offset += STL * 8;
        }
      }
      //fprintf(stderr,"Extracting from offset %d, length=%d\n",data_offset,data_length);

      if (data_length == -1) {
        fprintf(stderr,"Could not find sub-channel with ID %d\n",subchanid);
        return 1;
      }
    }

    write(1,buf + 12 + 4*NST + FICF*96 + data_offset, data_length);
  }
}
