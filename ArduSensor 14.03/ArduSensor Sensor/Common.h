/*
 * Common.h
 *
 * Created: 13.08.2012 21:09:49
 *  Author: Cmc
 */ 


#ifndef COMMON_H_
#define COMMON_H_

#define BIT(x) (1 << (x))
#define SETBITS(x,y) ((x) |= (y))
#define CLEARBITS(x,y) ((x) &= (~(y)))
#define SETBIT(x,y) SETBITS((x), (BIT((y))))
#define CLEARBIT(x,y) CLEARBITS((x), (BIT((y))))
#define BITSET(x,y) ((x) & (BIT(y)))
#define BITCLEAR(x,y) !BITSET((x), (y))
#define BITSSET(x,y) (((x) & (y)) == (y))
#define BITSCLEAR(x,y) (((x) & (y)) == 0)
#define BITVAL(x,y) (((x)>>(y)) & 1)

 

#endif /* COMMON_H_ */

/*
This stuff is for watchdog conf. Basically it changes the WDTCSR register based on user input millisecond. 
*/
#if WDTIMEO == 250
#define __WDCONF__ 0b01000100

#elif WDTIMEO == 500
#define __WDCONF__ 0b01000101

#elif WDTIMEO == 1000
#define __WDCONF__ 0b01000110

#elif WDTIMEO == 2000
#define __WDCONF__ 0b01000111

#elif WDTIMEO == 4000
#define __WDCONF__ 0b01100000

#elif WDTIMEO == 8000
#define __WDCONF__ 0b01000001

#else
#define __WDCONF__ 0b01000111 // Default is 2 sec

#endif