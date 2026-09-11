/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file rpmsg_uorb_proto.h
 *
 * Wire format of the "rpmsg-uorb" endpoint, shared by the PX4 module and the
 * remote-core firmware (boards/px4/fmu-v6xrt/cm4). Both sides compile the
 * same generated topic structs; topics are named once at subscribe or
 * advertise time and carried as the CM7's ORB_ID afterwards.
 */

#pragma once

#include <stdint.h>

#define RPMSG_UORB_EPT_NAME   "rpmsg-uorb"
#define RPMSG_UORB_VERSION    1
#define RPMSG_UORB_NAME_LEN   48	/* longest generated topic name is 37 */
#define RPMSG_UORB_TOPIC_NONE 0xFFFF

/* One rpmsg buffer (1024) minus the rpmsg header (16) and ours (8). */
#define RPMSG_UORB_MAX_PAYLOAD 1000

/* The remote drives the protocol. On bind it sends HELLO and waits for the
 * host's ACK (status 0, or -EPROTO on a version mismatch) before it sends
 * its requests; the host answers each with an ACK carrying the ORB_ID. A
 * HELLO from the host tells the remote the host restarted: it forgets every
 * id and starts over with HELLO.
 */
enum rpmsg_uorb_type {
	RPMSG_UORB_HELLO = 1,	/* arg = protocol version */
	RPMSG_UORB_SUBSCRIBE,	/* remote -> host: topic name at rate_hz (0 = every update) */
	RPMSG_UORB_UNSUBSCRIBE,	/* remote -> host */
	RPMSG_UORB_ADVERTISE,	/* remote -> host: remote publishes topic name */
	RPMSG_UORB_ACK,		/* host -> remote: hdr.arg = acked type, hdr.topic = ORB_ID, status */
	RPMSG_UORB_DATA,	/* both: hdr.topic = ORB_ID, payload = topic struct */
};

/* DATA from the remote: hdr.arg holds the low 32 bits of its 1 MHz counter,
 * the host rebuilds 64 bits from its own hrt and overwrites the struct's
 * leading uint64_t timestamp (every PX4 topic starts with it). Other
 * timestamp fields the remote fills are left as sent. DATA from the host
 * carries the full struct and no flag.
 */
#define RPMSG_UORB_F_TS32 0x01

struct rpmsg_uorb_hdr {
	uint8_t type;
	uint8_t flags;
	uint16_t topic;
	uint32_t arg;
} __attribute__((packed));

struct rpmsg_uorb_req {
	struct rpmsg_uorb_hdr hdr;
	char name[RPMSG_UORB_NAME_LEN];
	uint16_t size;		/* sizeof(struct) on the sender, checked against o_size */
	uint16_t rate_hz;
} __attribute__((packed));

struct rpmsg_uorb_ack {
	struct rpmsg_uorb_hdr hdr;
	char name[RPMSG_UORB_NAME_LEN];
	uint16_t size;
	int16_t status;		/* 0, -EPROTO version, -EINVAL unknown topic, -EMSGSIZE size mismatch, -ENOSPC table full */
} __attribute__((packed));

#ifdef __cplusplus
#define RPMSG_UORB_STATIC_ASSERT static_assert
#else
#define RPMSG_UORB_STATIC_ASSERT _Static_assert
#endif

RPMSG_UORB_STATIC_ASSERT(sizeof(struct rpmsg_uorb_hdr) == 8, "hdr");
RPMSG_UORB_STATIC_ASSERT(sizeof(struct rpmsg_uorb_req) == 60, "req");
RPMSG_UORB_STATIC_ASSERT(sizeof(struct rpmsg_uorb_ack) == 60, "ack");
