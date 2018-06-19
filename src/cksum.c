#include <htc.h>
#include "cksum.h"


// the return type (_D_TYPE) is based on definitions in cksum.h
_D_TYPE cksum(void){
   _D_TYPE  sum;
   unsigned short addr;
   unsigned int counter;

	addr = _START;	// point to start of checksum range
	sum = _OFFSET;		// initial offset (usually zero)
	counter = _SIZE;

	while(counter--){
#if _BYTEOP == 1
   		unsigned short dataread;
		dataread = FLASH_READ(addr);
		_SUM(sum,dataread & 0xff);		// read a word and accumulate a byte
		_SUM(sum,dataread>>8);			// accumulate the upper byte
#else
		_SUM(sum,FLASH_READ(addr));		// read and accumulate a word
#endif
#if BYTEOP == 3
	#error Checksum algorithm +3 and -3 not supported in this demo
#endif
#if _BYTEOP == 4
		addr++;	// read and accumulate the upper word
		_SUM(sum,((unsigned long)FLASH_READ(addr)<<16L));		// read and accumulate a byte
#endif
		addr++;	// select address for next loop iteration
	}
	return sum;
}

