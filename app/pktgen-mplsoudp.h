/*
 * Copyright (C) 2014 Semihalf.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * pktgen-mplsoudp.h -- definitions of MPLS-o-UDP packet header constants
 *
 */

#ifndef _PKTGEN_MPLSOUDP_H_
#define _PKTGEN_MPLSOUDP_H_

/* Example label: 0x00030 == 48 decimal */
#define MPLSOUDP_MPLS_LABEL 0x00030140

/* VM MACs */
#define MPLSOUDP_INNER_ETHER_SRC_MAC {0x02, 0x94, 0x47, 0x54, 0xf4, 0x2f}
#define MPLSOUDP_INNER_ETHER_DST_MAC {0x02, 0x94, 0x47, 0x54, 0xf4, 0x3f}

/* 0xC0A80003 == 192.168.0.3 */
#define MPLSOUDP_INNER_IPV4_SRC_ADDR 0xC0A80003
#define MPLSOUDP_INNER_IPV4_DST_ADDR 0xC0A80004

 /*                        MPLS Header Format
 *
 *    0                   1                   2                   3
 *    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   |                 Label                 | EXP |S|     TTL       |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 */
/* MPLSoUDP Header */
typedef struct mplsoudpHdr_s {
    uint16_t            sport;        /* Source port value */
    uint16_t            dport;        /* Destination port value */
    uint16_t            len;          /* Length of datagram + header */
    uint16_t            cksum;        /* Checksum of data and header */
    mplsHdr_t           mpls;         /* MPLS label */
    struct ether_hdr    ether;        /* Inner Ethernet header */
    ipHdr_t             ip;           /* Inner IPv4 header */
    udpHdr_t            inner_udp;    /* Inner UDP header */
} __attribute__((__packed__)) mplsoudpHdr_t;

/* The MPLSoUDP/IP Pseudo header */
typedef struct mplsoudpip_s {
    ipHdr_t          ip;              /* IPv4 overlay header */
    mplsoudpHdr_t    udp;             /* UDP header for protocol */
} __attribute__((__packed__)) mplsoudpip_t;

#endif    // _PKTGEN_MPLSOUDP_H_
