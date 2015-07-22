/*-
 * Copyright (c) <2010>, Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 *
 * - Neither the name of Intel Corporation nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * Copyright (c) <2010-2014>, Wind River Systems, Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are
 * permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * 3) Neither the name of Wind River Systems nor the names of its contributors may be
 * used to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * 4) The screens displayed by the application must contain the copyright notice as defined
 * above and can not be removed without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/* Created 2010 by Keith Wiles @ intel.com */

#include "pktgen.h"

#include "pktgen-udp.h"

/**************************************************************************//**
*
* pktgen_udp_hdr_ctor - UDP header constructor routine.
*
* DESCRIPTION
* Construct the UDP header in a packer buffer.
*
* RETURNS: N/A
*
* SEE ALSO:
*/

void
pktgen_udp_hdr_ctor(pkt_seq_t * pkt, udpip_t * uip, __attribute__ ((unused)) int type)
{
	uint16_t		tlen;

    // Zero out the header space
    memset((char *)uip, 0, sizeof(udpip_t));

    // Create the UDP header
    uip->ip.src         = htonl(pkt->ip_src_addr);
    uip->ip.dst         = htonl(pkt->ip_dst_addr);
    tlen                = pkt->pktSize - (pkt->ether_hdr_size + sizeof(ipHdr_t));

    uip->ip.len         = htons(tlen);
    uip->ip.proto       = pkt->ipProto;

	uip->udp.len		= htons(tlen);
    // Incrementing sport generate multiple UDP streams.
    uip->udp.sport      = htons(pkt->sport++);
    uip->udp.dport      = htons(pkt->dport);

	// Includes the pseudo header information
    tlen                = pkt->pktSize - pkt->ether_hdr_size;

    uip->udp.cksum      = cksum(uip, tlen, 0);
    if ( uip->udp.cksum == 0 )
        uip->udp.cksum = 0xFFFF;
}

/**************************************************************************//**
*
* pktgen_mplsoudp_hdr_ctor - MPLSoUDP header constructor routine.
*
* DESCRIPTION
* Construct the following header in a packer buffer:
*             Outer UDP/MPLS label/Inner Ethernet/Inner IP/Inner UDP
*
* RETURNS: N/A
*
* SEE ALSO:
* pktgen-mplsoudp.h
*/

void
pktgen_mplsoudp_hdr_ctor(pkt_seq_t * pkt, mplsoudpip_t * uip, __attribute__ ((unused)) int type)
{
    uint16_t udp_len = pkt->pktSize - pkt->ether_hdr_size - sizeof(ipHdr_t);
    static uint16_t inner_udp_sport = 1234;
    //uint16_t inner_ip_len = udp_len - sizeof(mplsudpip_t) - pkt->ether_hdr_size;

    // Zero out the header space
    memset((char *)uip, 0, sizeof(mplsoudpip_t));

    // Outer UDP header
    uip->udp.sport         = htons(pkt->sport);
    pkt->sport += 2;
    uip->udp.dport         = htons(pkt->dport);
    uip->udp.len           = htons(udp_len);
    uip->udp.cksum         = 0x0000;
    uip->udp.mpls.label    = htonl(MPLSOUDP_MPLS_LABEL); //48 dec

    // Inner Ethernet header
    char src[] = MPLSOUDP_INNER_ETHER_SRC_MAC;
    char dst[] = MPLSOUDP_INNER_ETHER_DST_MAC;
    memcpy(&uip->udp.ether.s_addr, src, ETHER_ADDR_LEN);
    memcpy(&uip->udp.ether.d_addr, dst, ETHER_ADDR_LEN);
    uip->udp.ether.ether_type = htons(pkt->ethType);

    // Inner IPv4 header
    uip->udp.ip.vl        = (IPv4_VERSION << 4) | (sizeof(ipHdr_t) /4);
    uip->udp.ip.tos       = 0;
    //uip->udp.ip.tlen    = htons(inner_ip_len);
    uip->udp.ip.tlen      = htons(0x002e);
    pktgen.ident          += 27; // bump by a prime number
    uip->udp.ip.ident     = htons(pktgen.ident);
    uip->udp.ip.ffrag     = 0;
    uip->udp.ip.ttl       = 4;
    uip->udp.ip.proto     = PG_IPPROTO_UDP;
    uip->udp.ip.src       = htonl(MPLSOUDP_INNER_IPV4_SRC_ADDR);
    uip->udp.ip.dst       = htonl(MPLSOUDP_INNER_IPV4_DST_ADDR);
    uip->udp.ip.cksum     = cksum(&uip->udp.ip, sizeof(ipHdr_t), 0);

    // Inner UDP header
    // Incrementing sport generate multiple UDP streams.
    uip->udp.inner_udp.sport    = htons(inner_udp_sport++);
    uip->udp.inner_udp.dport    = htons(5678);
    uip->udp.inner_udp.len      = htons(sizeof(udpHdr_t));
    uip->udp.inner_udp.cksum    = 0x0000;
}
