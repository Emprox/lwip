/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */

#include<lwip/tcpip.h>
#include<lwip/sys.h>

#if LWIP_ETHERNET

#include<lwip/def.h>
#include<lwip/mem.h>
#include<lwip/pbuf.h>
#include<lwip/stats.h>
#include<lwip/snmp.h>
#include<lwip/ethip6.h>
#include<lwip/etharp.h>
#if LWIP_NETIF_API
#include<lwip/netifapi.h>
#endif
#include<stm32f2xx_hal.h>

/* The time to block waiting for input. */
#define TIME_WAITING_FOR_INPUT		(200)

/* Stack size of the interface thread */
#define INTERFACE_THREAD_STACK_SIZE ( 1024 )
#define INTERFACE_THREAD_PRIORITY	(tskIDLE_PRIORITY+configMAX_PRIORITIES-1)

#define IFNAME0 's'
#define IFNAME1 't'

/**
 * Helper struct to hold private data used to operate your ethernet interface.
 * Keeping the ethernet address of the MAC in this struct is not necessary
 * as it is already kept in the struct netif.
 * But this is only an example, anyway...
 */
//struct ethernetif
//{
//	struct eth_addr *ethaddr;
///* Add whatever per-interface state that is needed here. */
//};

__ALIGN_BEGIN ETH_DMADescTypeDef DMARxDscrTab[ETH_RXBUFNB] __ALIGN_END;/* Ethernet Rx MA Descriptor */
__ALIGN_BEGIN ETH_DMADescTypeDef DMATxDscrTab[ETH_TXBUFNB] __ALIGN_END;/* Ethernet Tx DMA Descriptor */
__ALIGN_BEGIN uint8_t Rx_Buff[ETH_RXBUFNB][ETH_RX_BUF_SIZE] __ALIGN_END; /* Ethernet Receive Buffer */
__ALIGN_BEGIN uint8_t Tx_Buff[ETH_TXBUFNB][ETH_TX_BUF_SIZE] __ALIGN_END; /* Ethernet Transmit Buffer */

/* Semaphore to signal incoming packets */
sys_sem_t xSemaphore;
/* Global Ethernet handle */
ETH_HandleTypeDef heth;

/**
 * @brief  Ethernet Rx Transfer completed callback
 * @param  heth: ETH handle
 * @retval None
 */
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
	sys_arch_sem_signal_fromisr(&xSemaphore);
}

/**
 * Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf*	low_level_input	( struct netif *netif )
{
//	struct ethernetif *ethernetif = netif->state;
	struct pbuf *p = NULL;
	struct pbuf *q = NULL;
	uint16_t len = 0;
	uint8_t *buffer;
	__IO ETH_DMADescTypeDef *dmarxdesc;
	uint32_t bufferoffset = 0;
	uint32_t payloadoffset = 0;
	uint32_t byteslefttocopy = 0;
	uint32_t i = 0;

	/* get received frame */
	if (HAL_ETH_GetReceivedFrame_IT(&heth) != HAL_OK)
		return NULL;

	/* Obtain the size of the packet and put it into the "len"
	 variable. */
	len = heth.RxFrameInfos.length;
	buffer = (uint8_t*) heth.RxFrameInfos.buffer;

	if (len > 0)
	{
#if ETH_PAD_SIZE
  len += ETH_PAD_SIZE; /* allow room for Ethernet padding */
#endif

		/* We allocate a pbuf chain of pbufs from the pool. */
		p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
	}

	if (p != NULL)
	{
#if ETH_PAD_SIZE
    pbuf_remove_header(p, ETH_PAD_SIZE); /* drop the padding word */
#endif

		dmarxdesc = heth.RxFrameInfos.FSRxDesc;
		bufferoffset = 0;

		/* We iterate over the pbuf chain until we have read the entire
		 * packet into the pbuf. */
		for (q = p; q != NULL; q = q->next)
		{
			/* Read enough bytes to fill this pbuf in the chain. The
			 * available data in the pbuf is given by the q->len
			 * variable.
			 * This does not necessarily have to be a memcpy, you can also preallocate
			 * pbufs for a DMA-enabled MAC and after receiving truncate it to the
			 * actually received size. In this case, ensure the tot_len member of the
			 * pbuf is the sum of the chained pbuf len members.
			 */
			byteslefttocopy = q->len;
			payloadoffset = 0;

			/* Check if the length of bytes to copy in current pbuf is bigger than Rx buffer size*/
			while ((byteslefttocopy + bufferoffset) > ETH_RX_BUF_SIZE)
			{
				/* Copy data to pbuf */
				memcpy((uint8_t*) ((uint8_t*) q->payload + payloadoffset),
						(uint8_t*) ((uint8_t*) buffer + bufferoffset),
						(ETH_RX_BUF_SIZE - bufferoffset));

				/* Point to next descriptor */
				dmarxdesc =
						(ETH_DMADescTypeDef*) (dmarxdesc->Buffer2NextDescAddr);
				buffer = (uint8_t*) (dmarxdesc->Buffer1Addr);

				byteslefttocopy = byteslefttocopy
						- (ETH_RX_BUF_SIZE - bufferoffset);
				payloadoffset = payloadoffset
						+ (ETH_RX_BUF_SIZE - bufferoffset);
				bufferoffset = 0;
			}
			/* Copy remaining data in pbuf */
			memcpy((uint8_t*) ((uint8_t*) q->payload + payloadoffset),
					(uint8_t*) ((uint8_t*) buffer + bufferoffset),
					byteslefttocopy);
			bufferoffset = bufferoffset + byteslefttocopy;
		}

		MIB2_STATS_NETIF_ADD(netif, ifinoctets, p->tot_len);
		if (((u8_t*) p->payload)[0] & 1)
		{
			/* broadcast or multicast packet*/
			MIB2_STATS_NETIF_INC(netif, ifinnucastpkts);
		}
		else
		{
			/* unicast packet*/
			MIB2_STATS_NETIF_INC(netif, ifinucastpkts);
		}
#if ETH_PAD_SIZE
    pbuf_add_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

		LINK_STATS_INC(link.recv);
	}
	else
	{
		LINK_STATS_INC(link.memerr);
		LINK_STATS_INC(link.drop);MIB2_STATS_NETIF_INC(netif, ifindiscards);
	}

	/* Release descriptors to DMA */
	/* Point to first descriptor */
	dmarxdesc = heth.RxFrameInfos.FSRxDesc;
	/* Set Own bit in Rx descriptors: gives the buffers back to DMA */
	for (i = 0; i < heth.RxFrameInfos.SegCount; i++)
	{
		dmarxdesc->Status |= ETH_DMARXDESC_OWN;
		dmarxdesc = (ETH_DMADescTypeDef*) (dmarxdesc->Buffer2NextDescAddr);
	}

	/* Clear Segment_Count */
	heth.RxFrameInfos.SegCount = 0;

	/* When Rx Buffer unavailable flag is set: clear it and resume reception */
	if ((heth.Instance->DMASR & ETH_DMASR_RBUS) != (uint32_t) RESET)
	{
		/* Clear RBUS ETHERNET DMA flag */
		heth.Instance->DMASR = ETH_DMASR_RBUS;
		/* Resume DMA reception */
		heth.Instance->DMARPDR = 0;
	}

	return p;
}

/**
 * This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */
static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
	err_t errval;
//	struct ethernetif *ethernetif = netif->state;
	struct pbuf *q;
	uint8_t *buffer = (uint8_t*) (heth.TxDesc->Buffer1Addr);
	__IO ETH_DMADescTypeDef *DmaTxDesc;
	uint32_t framelength = 0;
	uint32_t bufferoffset = 0;
	uint32_t byteslefttocopy = 0;
	uint32_t payloadoffset = 0;
	DmaTxDesc = heth.TxDesc;
	bufferoffset = 0;

#if ETH_PAD_SIZE
  pbuf_remove_header(p, ETH_PAD_SIZE); /* drop the padding word */
#endif
	/* copy frame from pbufs to driver buffers */
	for (q = p; q != NULL; q = q->next)
	{
		/* Is this buffer available? If not, goto error */
		if ((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t) RESET)
		{
			errval = ERR_USE;
			goto error;
		}

		/* Get bytes in current lwIP buffer */
		byteslefttocopy = q->len;
		payloadoffset = 0;

		/* Check if the length of data to copy is bigger than Tx buffer size*/
		while ((byteslefttocopy + bufferoffset) > ETH_TX_BUF_SIZE)
		{
			/* Copy data to Tx buffer*/
			memcpy((uint8_t*) ((uint8_t*) buffer + bufferoffset),
					(uint8_t*) ((uint8_t*) q->payload + payloadoffset),
					(ETH_TX_BUF_SIZE - bufferoffset));

			/* Point to next descriptor */
			DmaTxDesc = (ETH_DMADescTypeDef*) (DmaTxDesc->Buffer2NextDescAddr);

			/* Check if the buffer is available */
			if ((DmaTxDesc->Status & ETH_DMATXDESC_OWN) != (uint32_t) RESET)
			{
				errval = ERR_USE;
				goto error;
			}

			buffer = (uint8_t*) (DmaTxDesc->Buffer1Addr);

			byteslefttocopy = byteslefttocopy
					- (ETH_TX_BUF_SIZE - bufferoffset);
			payloadoffset = payloadoffset + (ETH_TX_BUF_SIZE - bufferoffset);
			framelength = framelength + (ETH_TX_BUF_SIZE - bufferoffset);
			bufferoffset = 0;
		}

		/* Copy the remaining bytes */
		memcpy((uint8_t*) ((uint8_t*) buffer + bufferoffset),
				(uint8_t*) ((uint8_t*) q->payload + payloadoffset),
				byteslefttocopy);
		bufferoffset = bufferoffset + byteslefttocopy;
		framelength = framelength + byteslefttocopy;
	}

	/* Prepare transmit descriptors to give to DMA */
	HAL_ETH_TransmitFrame(&heth, framelength);

	errval = ERR_OK;

	MIB2_STATS_NETIF_ADD(netif, ifoutoctets, p->tot_len);
	if (((u8_t*) p->payload)[0] & 1)
	{
		/* broadcast or multicast packet*/
		MIB2_STATS_NETIF_INC(netif, ifoutnucastpkts);
	}
	else
	{
		/* unicast packet */
		MIB2_STATS_NETIF_INC(netif, ifoutucastpkts);
	}
	/* increase ifoutdiscards or ifouterrors on error */

#if ETH_PAD_SIZE
  pbuf_add_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

	LINK_STATS_INC(link.xmit);

	error: if (errval != ERR_OK)
	{
		MIB2_STATS_NETIF_INC(netif, ifouterrors);
	}

	/* When Transmit Underflow flag is set, clear it and issue a Transmit Poll Demand to resume transmission */
	if ((heth.Instance->DMASR & ETH_DMASR_TUS) != (uint32_t) RESET)
	{
		/* Clear TUS ETHERNET DMA flag */
		heth.Instance->DMASR = ETH_DMASR_TUS;

		/* Resume DMA transmission*/
		heth.Instance->DMATPDR = 0;
	}
	return errval;
}

static void vprvEthTask(void *argument)
{
	struct pbuf *p;
	struct netif *netif = (struct netif*) argument;

	for (;;)
	{
		if (sys_arch_sem_wait(&xSemaphore, TIME_WAITING_FOR_INPUT)
				!= SYS_ARCH_TIMEOUT)
		{
			do
			{
				LOCK_TCPIP_CORE();
				p = low_level_input(netif);
				if (p != NULL)
				{
					if (netif->input(p, netif) != ERR_OK)
					{
						pbuf_free(p);
					}
				}
				UNLOCK_TCPIP_CORE();
			} while (p != NULL);
		}

		{
			uint32_t regvalue = 0;

			/* Read PHY_BSR*/
			HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &regvalue);

			regvalue &= PHY_LINKED_STATUS;

			/* Check whether the netif link down and the PHY link is up */
			if (!netif_is_link_up(netif) && (regvalue))
			{
				/* network cable is connected */

				__IO uint32_t tickstart = 0;
				/* Restart the auto-negotiation */
				if (heth.Init.AutoNegotiation != ETH_AUTONEGOTIATION_DISABLE)
				{
					/* Enable Auto-Negotiation */
					HAL_ETH_WritePHYRegister(&heth, PHY_BCR,
							PHY_AUTONEGOTIATION);

					/* Get tick */
					tickstart = HAL_GetTick();

					/* Wait until the auto-negotiation will be completed */
					do
					{
						HAL_ETH_ReadPHYRegister(&heth, PHY_BSR, &regvalue);

						/* Check for the Timeout ( 1s ) */
						if ((HAL_GetTick() - tickstart) > 1000)
						{
							/* In case of timeout */
							goto error;
						}
					} while (((regvalue & PHY_AUTONEGO_COMPLETE)
							!= PHY_AUTONEGO_COMPLETE));

					/* Read the result of the auto-negotiation */
					HAL_ETH_ReadPHYRegister(&heth, PHY_SR, &regvalue);

					/* Configure the MAC with the Duplex Mode fixed by the auto-negotiation process */
					if ((regvalue & PHY_DUPLEX_STATUS) != (uint32_t) RESET)
					{
						/* Set Ethernet duplex mode to Full-duplex following the auto-negotiation */
						heth.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
					}
					else
					{
						/* Set Ethernet duplex mode to Half-duplex following the auto-negotiation */
						heth.Init.DuplexMode = ETH_MODE_HALFDUPLEX;
					}
					/* Configure the MAC with the speed fixed by the auto-negotiation process */
					if (regvalue & PHY_SPEED_STATUS)
					{
						/* Set Ethernet speed to 10M following the auto-negotiation */
						heth.Init.Speed = ETH_SPEED_10M;
					}
					else
					{
						/* Set Ethernet speed to 100M following the auto-negotiation */
						heth.Init.Speed = ETH_SPEED_100M;
					}
				}
				else /* AutoNegotiation Disable */
				{
					error:
					/* Check parameters */
					assert_param(IS_ETH_SPEED(heth.Init.Speed));
					assert_param(IS_ETH_DUPLEX_MODE(heth.Init.DuplexMode));

					/* Set MAC Speed and Duplex Mode to PHY */
					HAL_ETH_WritePHYRegister(&heth, PHY_BCR,
							((uint16_t) (heth.Init.DuplexMode >> 3)
									| (uint16_t) (heth.Init.Speed >> 1)));
				}

				/* ETHERNET MAC Re-Configuration */
				HAL_ETH_ConfigMAC(&heth, (ETH_MACInitTypeDef*) NULL);

				/* Restart MAC interface */
				HAL_ETH_Start(&heth);

#if LWIP_NETIF_API
				netifapi_netif_set_link_up(netif);
#else
				netif_set_link_up(netif);
#endif
			}
			else if (netif_is_link_up(netif) && (!regvalue))
			{
				/* network cable is dis-connected */
#if LWIP_NETIF_API
				netifapi_netif_set_link_down(netif);
#else
				netif_set_link_down(netif);
#endif

			    /* Stop MAC interface */
			    HAL_ETH_Stop(&heth);
			}
		}
	}
}

/**
 * In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void low_level_init(struct netif *netif)
{
//	struct ethernetif *ethernetif = netif->state;

	/* set MAC hardware address length */
	netif->hwaddr_len = ETHARP_HWADDR_LEN;

	/* set MAC hardware address */
	/* set MAC hardware address */
	netif->hwaddr[0] = 0x00;
	netif->hwaddr[1] = 0x80;
	netif->hwaddr[2] = 0xE1;
	netif->hwaddr[3] = 0x00;
	netif->hwaddr[4] = 0x00;
	netif->hwaddr[5] = 0x01;

	/* maximum transfer unit */
	netif->mtu = 1500;

	/* device capabilities */
	/* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
	netif->flags =
			NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;

#if LWIP_IPV6 && LWIP_IPV6_MLD
  /*
   * For hardware/netifs that implement MAC filtering.
   * All-nodes link-local is handled by default, so we must let the hardware know
   * to allow multicast packets in.
   * Should set mld_mac_filter previously. */
  if (netif->mld_mac_filter != NULL) {
    ip6_addr_t ip6_allnodes_ll;
    ip6_addr_set_allnodes_linklocal(&ip6_allnodes_ll);
    netif->mld_mac_filter(netif, &ip6_allnodes_ll, NETIF_ADD_MAC_FILTER);
  }
#endif /* LWIP_IPV6 && LWIP_IPV6_MLD */

	uint32_t regvalue = 0;
	HAL_StatusTypeDef hal_eth_init_status;

	/* Init ETH */

	heth.Instance = ETH;
	heth.Init.AutoNegotiation = ETH_AUTONEGOTIATION_DISABLE;
	heth.Init.Speed = ETH_SPEED_100M;
	heth.Init.DuplexMode = ETH_MODE_FULLDUPLEX;
	heth.Init.PhyAddress = LAN8742A_PHY_ADDRESS;
	heth.Init.MACAddr = netif->hwaddr;
	heth.Init.RxMode = ETH_RXINTERRUPT_MODE;
	heth.Init.ChecksumMode = ETH_CHECKSUM_BY_HARDWARE;
	heth.Init.MediaInterface = ETH_MEDIA_INTERFACE_RMII;

	hal_eth_init_status = HAL_ETH_Init(&heth);

	if (hal_eth_init_status == HAL_OK)
	{
		/* Set netif link flag */
		netif->flags |= NETIF_FLAG_LINK_UP;
	}
	/* Initialize Tx Descriptors list: Chain Mode */
	HAL_ETH_DMATxDescListInit(&heth, DMATxDscrTab, &Tx_Buff[0][0], ETH_TXBUFNB);

	/* Initialize Rx Descriptors list: Chain Mode  */
	HAL_ETH_DMARxDescListInit(&heth, DMARxDscrTab, &Rx_Buff[0][0], ETH_RXBUFNB);

	/* Accept broadcast address and ARP traffic */
	/* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
#if LWIP_ARP
	netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
#else
    netif->flags |= NETIF_FLAG_BROADCAST;
  #endif /* LWIP_ARP */

	/* create a binary semaphore used for informing ethernetif of frame reception */
	sys_sem_new(&xSemaphore, 1);

	/* create the task that handles the ETH_MAC */
	sys_thread_new("EthIf", vprvEthTask, netif, INTERFACE_THREAD_STACK_SIZE,
			INTERFACE_THREAD_PRIORITY);
	/* Enable MAC and DMA transmission and reception */
	HAL_ETH_Start(&heth);

	/* Read Register Configuration */
	HAL_ETH_ReadPHYRegister(&heth, PHY_ISFR, &regvalue);
	regvalue |= (PHY_ISFR_INT4);

	/* Enable Interrupt on change of link status */
	HAL_ETH_WritePHYRegister(&heth, PHY_ISFR, regvalue);

	/* Read Register Configuration */
	HAL_ETH_ReadPHYRegister(&heth, PHY_ISFR, &regvalue);
}

/**
 * Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init(struct netif *netif)
{
//	struct ethernetif *ethernetif;

	LWIP_ASSERT("netif != NULL", (netif != NULL));

//	ethernetif = mem_malloc(sizeof(struct ethernetif));
//	if (ethernetif == NULL)
//	{
//		LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_init: out of memory\n"));
//		return ERR_MEM;
//	}

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

	/*
	 * Initialize the snmp variables and counters inside the struct netif.
	 * The last argument should be replaced with your link speed, in units
	 * of bits per second.
	 */
	MIB2_INIT_NETIF(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

//	netif->state = ethernetif;
	netif->name[0] = IFNAME0;
	netif->name[1] = IFNAME1;
	/* We directly use etharp_output() here to save a function call.
	 * You can instead declare your own function an call etharp_output()
	 * from it if you have to do some checks before sending (e.g. if link
	 * is available...) */
#if LWIP_IPV4
	netif->output = etharp_output;
#endif /* LWIP_IPV4 */
#if LWIP_IPV6
  netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */
	netif->linkoutput = low_level_output;

//	ethernetif->ethaddr = (struct eth_addr*) &(netif->hwaddr[0]);

	/* initialize the hardware */
	low_level_init(netif);

	return ERR_OK;
}

#endif	// LWIP_ETHERNET
