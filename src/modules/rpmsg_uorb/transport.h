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

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* rpmsg endpoint plumbing for the uORB bridge. C because the OpenAMP headers
 * are not C++-clean. Receive runs on the rptun thread.
 */

__BEGIN_DECLS

typedef void (*rpmsg_uorb_rx_t)(const uint8_t *data, size_t len);

/* Register for rpmsg device creation on cpuname; the endpoint is created when
 * the remote comes up. Idempotent.
 */
int rpmsg_uorb_transport_init(const char *cpuname, rpmsg_uorb_rx_t rx);

/* True once the remote endpoint address is known. */
bool rpmsg_uorb_transport_ready(void);

/* Send hdr followed by payload in one rpmsg buffer without copying twice.
 * Never blocks: returns -EAGAIN when no TX buffer is free, -ENOTCONN when
 * the endpoint is not bound, -EMSGSIZE when it does not fit a buffer.
 */
int rpmsg_uorb_transport_send(const void *hdr, size_t hdr_len, const void *payload, size_t payload_len);

__END_DECLS
