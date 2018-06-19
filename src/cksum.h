
/**** These are typical settings and ****
 **** should be adjusted to suit those **
 **** used in the --CHECKSUM option. ****/
#define _OFFSET 0
#define _ALGORITHM 2     // can be -4,-3,-2,-1,1,2,3,4
#define _RESULT_WIDTH 2  // byte size of result
#define _START 0L
#define _END (_ROMSIZE - _RESULT_WIDTH - 1)
/********************************************/
/** All settings below this line are fixed **/
/********************************************/

/* The data type being returned is
 * determined from the above settings. */
#if (_RESULT_WIDTH == 1)
 #define _D_TYPE unsigned char
#elif (_RESULT_WIDTH == 2)
 #define _D_TYPE unsigned int
#else
 #define _D_TYPE unsigned long
#endif

// Determine whether an addition or subtraction algorithm is used.
#if (_ALGORITHM < 0)
 #define _SUM(x,y) x -= y
 #define _BYTEOP (_ALGORITHM * -1)
#else
 #define _SUM(x,y) x += y
 #define _BYTEOP _ALGORITHM
#endif

#if _RESULT_WIDTH < _BYTEOP
	#error Significant data losses in checksum calculation. Increase byte-width of result
#endif

// Calculate the number of iterations through the loop.
#if _BYTEOP > 2
 #define _SIZE ( (_END - _START + 1) / 2 )
#else
 #define _SIZE ( _END - _START + 1 )
#endif

#if (_END - _START + 1) % _SIZE
 #error Range is not a multiple of this selected algorithm
#endif

// Function prototypes
extern unsigned _D_TYPE cksum(void);
extern unsigned _D_TYPE _checksum0;
