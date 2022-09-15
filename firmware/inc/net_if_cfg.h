#ifndef NET_IF_CFG_H
#define NET_IF_CFG_H

#include "lwip/netif.h"

/* Structure containing network interfaces configuration. */
typedef struct {
    u8_t num;                               /* Interface number */
    uint8_t hwaddr[NETIF_MAX_HWADDR_LEN];   /* MAC address */
    uint8_t has_dhcp;                       /* Variable containing information whether dhcp is enabled or not */
    uint8_t has_auto_ip;                    /* Variable containing information whether auto ip is enabled or not */
    uint8_t has_IPv6;                       /* Variable containing information whether ipv6 is enabled or not */
    u8_t ip_addr[4];                        /* Board's ip address. Needs to be set if dhcp is turned off. */
    u8_t netmask[4];                        /* Board's netmask. Needs to be set if dhcp is turned off. */
    u8_t gw[4];                             /* Board's network gateway. Needs to be set if dhcp is turned off. */
    const char *hostname;                   /* Board's hostname used for dns. */
    char name[2];                           /* Interface name */
} netif_custom_t;

/* Number of active network interfaces. */
#define ENETIF_NUMBER 1

/* Number of Ethernet Interfaces for the stack */
#define ETHIF_NUMBER ENETIF_NUMBER

/* Network interfaces configuration. */
extern netif_custom_t *netif_cfg[ENETIF_NUMBER];

/* Ethernet MII mode - speed. */
#define ENET_MIIMODE                  ENET_RMII_MODE
#define ENET_MIISPEED                 ENET_MII_SPEED_100M


#ifndef FEATURE_ENET_RX_CONFIG_PAYLOAD_LEN_CHECK
#define FEATURE_ENET_RX_CONFIG_PAYLOAD_LEN_CHECK        0U
#endif /* FEATURE_ENET_RX_CONFIG_PAYLOAD_LEN_CHECK */

#ifndef FEATURE_ENET_RX_CONFIG_STRIP_CRC_FIELD
#define FEATURE_ENET_RX_CONFIG_STRIP_CRC_FIELD          0U
#endif /* FEATURE_ENET_RX_CONFIG_STRIP_CRC_FIELD */

#ifndef FEATURE_ENET_RX_CONFIG_REMOVE_PADDING
#define FEATURE_ENET_RX_CONFIG_REMOVE_PADDING           0U
#endif /* FEATURE_ENET_RX_CONFIG_REMOVE_PADDING */

#ifndef FEATURE_ENET_HAS_TX_CONFIG
#define FEATURE_ENET_HAS_TX_CONFIG                      0U
#endif /* FEATURE_ENET_HAS_TX_CONFIG */

#ifndef FEATURE_ENET_HAS_ACCELERATOR
#define FEATURE_ENET_HAS_ACCELERATOR                    0U
#endif /* FEATURE_ENET_HAS_ACCELERATOR */


#endif /* NET_IF_CFG_H */
