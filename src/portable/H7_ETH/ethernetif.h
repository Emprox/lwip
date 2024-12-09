/*--------------------------------------------------------------------------/
/  Module – ethernetif.h													/
/---------------------------------------------------------------------------/
/
/ Copyright (C) 2019, eprojects Michał Koziak, all right reserved.
/
/ Description:
/
/ Created on: Jul 30, 2020
/ Author: Robson
/ Modified by: 
/--------------------------------------------------------------------------*/

#ifndef LWIP_PORTABLE_F2_ETH_ETHERNETIF_H_
#define LWIP_PORTABLE_F2_ETH_ETHERNETIF_H_


#include<lwip/netif.h>


err_t ethernetif_init(struct netif *netif);


#endif /* LWIP_PORTABLE_F2_ETH_ETHERNETIF_H_ */
