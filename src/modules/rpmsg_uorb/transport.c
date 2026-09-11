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

#include <nuttx/config.h>
#include <nuttx/rpmsg/rpmsg.h>

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include "rpmsg_uorb_proto.h"
#include "transport.h"

static struct rpmsg_endpoint g_ept;
static rpmsg_uorb_rx_t g_rx;
static const char *g_cpuname;
static bool g_registered;

static int ept_cb(struct rpmsg_endpoint *ept, void *data, size_t len, uint32_t src, void *priv)
{
	if (g_rx) {
		g_rx((const uint8_t *)data, len);
	}

	return 0;
}

static void ept_unbind(struct rpmsg_endpoint *ept)
{
	rpmsg_destroy_ept(ept);
}

static void device_created(struct rpmsg_device *rdev, void *priv)
{
	if (strcmp(rpmsg_get_cpuname(rdev), g_cpuname) != 0) {
		return;
	}

	rpmsg_create_ept(&g_ept, rdev, RPMSG_UORB_EPT_NAME, RPMSG_ADDR_ANY, RPMSG_ADDR_ANY, ept_cb, ept_unbind);
}

static void device_destroy(struct rpmsg_device *rdev, void *priv)
{
	if (g_ept.rdev == rdev) {
		rpmsg_destroy_ept(&g_ept);
	}
}

int rpmsg_uorb_transport_init(const char *cpuname, rpmsg_uorb_rx_t rx)
{
	if (g_registered) {
		return 0;
	}

	g_cpuname = cpuname;
	g_rx = rx;
	g_registered = true;
	return rpmsg_register_callback(NULL, device_created, device_destroy, NULL, NULL);
}

bool rpmsg_uorb_transport_ready(void)
{
	return is_rpmsg_ept_ready(&g_ept);
}

int rpmsg_uorb_transport_send(const void *hdr, size_t hdr_len, const void *payload, size_t payload_len)
{
	uint32_t cap = 0;

	if (!is_rpmsg_ept_ready(&g_ept)) {
		return -ENOTCONN;
	}

	uint8_t *buf = rpmsg_get_tx_payload_buffer(&g_ept, &cap, false);

	if (buf == NULL) {
		return -EAGAIN;
	}

	if (hdr_len + payload_len > cap) {
		rpmsg_release_tx_buffer(&g_ept, buf);
		return -EMSGSIZE;
	}

	memcpy(buf, hdr, hdr_len);

	if (payload_len) {
		memcpy(buf + hdr_len, payload, payload_len);
	}

	int ret = rpmsg_send_nocopy(&g_ept, buf, hdr_len + payload_len);

	if (ret < 0) {
		rpmsg_release_tx_buffer(&g_ept, buf);
		return ret;
	}

	return 0;
}
