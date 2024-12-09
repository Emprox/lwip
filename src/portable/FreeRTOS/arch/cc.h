/*
 * cc.h
 *
 *  Created on: 3 lut 2017
 *      Author: Robson
 */

#ifndef LWIP_PORTABLE_ARCH_CC_H_
#define LWIP_PORTABLE_ARCH_CC_H_


#include<Console/Console.h>
#include<debugware/debugware.h>
#include<globalroutines.h>
//#include<stdlib.h>
#include<time.h>


//#define BYTE_ORDER					LITTLE_ENDIAN
#define LWIP_PLATFORM_DIAG(mes)		DEBUG_PRINTF mes
#define LWIP_TIMEVAL_PRIVATE		0


#if DEBUGWARE_DEBUG
#define LWIP_PLATFORM_ASSERT(x)		do{LOG_PUTS("LWIP ASSERT.\r\n");dwDEBUG_STOP(0);}while(0)
#else
#define LWIP_PLATFORM_ASSERT(x)		do{LOG_PRINTF("LWIP ASSERT: \"%s\" at line %d in %s\r\n", x, __LINE__, __FILE__);\
                                     vSystemRestart();} while(0)
#endif

#define LWIP_RAND()					ui32RNG_Get()



#endif /* LWIP_PORTABLE_ARCH_CC_H_ */
