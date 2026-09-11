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

#include "RpmsgUorbChannel.hpp"
#include "transport.h"

#include <px4_platform_common/log.h>
#include <uORB/uORBManager.hpp>
#include <uORB/topics/uORBTopics.hpp>

#include <errno.h>
#include <string.h>

RpmsgUorbChannel &RpmsgUorbChannel::instance()
{
	static RpmsgUorbChannel channel;
	return channel;
}

int RpmsgUorbChannel::start(const char *cpuname)
{
	if (_started) {
		return 0;
	}

	int ret = rpmsg_uorb_transport_init(cpuname, rx_trampoline);

	if (ret < 0) {
		return ret;
	}

	uORB::Manager::get_instance()->set_uorb_communicator(this);
	_started = true;
	return 0;
}

int16_t RpmsgUorbChannel::register_handler(uORBCommunicator::IChannelRxHandler *handler)
{
	_rx_handler = handler;
	return 0;
}

/* ---- Manager -> remote ------------------------------------------------- */

int16_t RpmsgUorbChannel::send_message(const char *messageName, int32_t length, uint8_t *data)
{
	/* Hot path: called for every publish of every topic. Pointer compare on
	 * o_name, no lock; the tables change rarely and a torn read costs at
	 * most one sample. meta is read once: the rptun thread may clear the
	 * slot underneath us.
	 */
	Sub *sub = nullptr;
	const orb_metadata *meta = nullptr;

	for (Sub &s : _subs) {
		meta = s.meta;

		if (meta && meta->o_name == messageName) {
			sub = &s;
			break;
		}
	}

	if (sub == nullptr || length != meta->o_size || length > RPMSG_UORB_MAX_PAYLOAD) {
		return 0;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (sub->interval_us && now - sub->last < sub->interval_us) {
		sub->rate_limited++;
		return 0;
	}

	rpmsg_uorb_hdr hdr{};
	hdr.type = RPMSG_UORB_DATA;
	hdr.topic = meta->o_id;

	/* Never block or fail a publisher: drop and count instead. */
	if (rpmsg_uorb_transport_send(&hdr, sizeof(hdr), data, length) < 0) {
		sub->dropped++;
		return 0;
	}

	sub->last = now;
	sub->sent++;
	return 0;
}

/* ---- remote -> Manager ------------------------------------------------- */

void RpmsgUorbChannel::rx_trampoline(const uint8_t *data, size_t len)
{
	instance().on_rx(data, len);
}

void RpmsgUorbChannel::on_rx(const uint8_t *data, size_t len)
{
	if (len < sizeof(rpmsg_uorb_hdr)) {
		_rx_bad++;
		return;
	}

	const rpmsg_uorb_hdr *hdr = reinterpret_cast<const rpmsg_uorb_hdr *>(data);

	switch (hdr->type) {
	case RPMSG_UORB_HELLO:
		on_hello(hdr->arg);
		break;

	case RPMSG_UORB_SUBSCRIBE:
	case RPMSG_UORB_UNSUBSCRIBE:
	case RPMSG_UORB_ADVERTISE:
		if (len < sizeof(rpmsg_uorb_req)) {
			_rx_bad++;
			return;
		}

		on_request(*reinterpret_cast<const rpmsg_uorb_req *>(data));
		break;

	case RPMSG_UORB_DATA:
		on_data(*hdr, data + sizeof(*hdr), len - sizeof(*hdr));
		break;

	default:
		_rx_bad++;
		break;
	}
}

void RpmsgUorbChannel::on_hello(uint32_t version)
{
	/* A fresh remote has empty tables; forget ours so nothing is sent to
	 * an endpoint that no longer expects it. It re-requests once acked.
	 */
	pthread_mutex_lock(&_lock);

	for (Sub &s : _subs) { s = Sub{}; }

	for (Adv &a : _advs) { a = Adv{}; }

	pthread_mutex_unlock(&_lock);
	_hello++;

	/* Runs on the rptun kernel thread, which has no stdio: outcomes are
	 * counted, not printed, and show up in status.
	 */
	if (version != RPMSG_UORB_VERSION) {
		_refused++;
	}

	send_ack(RPMSG_UORB_HELLO, nullptr, "", 0, version == RPMSG_UORB_VERSION ? 0 : -EPROTO);
}

const orb_metadata *RpmsgUorbChannel::lookup(const char *name, uint16_t size, int16_t &status)
{
	const orb_metadata *const *topics = orb_get_topics();

	for (size_t i = 0; i < orb_topics_count(); i++) {
		if (strncmp(topics[i]->o_name, name, RPMSG_UORB_NAME_LEN) == 0) {
			if (topics[i]->o_size != size) {
				status = -EMSGSIZE;
				return nullptr;
			}

			if (size > RPMSG_UORB_MAX_PAYLOAD) {
				status = -EMSGSIZE;
				return nullptr;
			}

			status = 0;
			return topics[i];
		}
	}

	status = -EINVAL;
	return nullptr;
}

void RpmsgUorbChannel::on_request(const rpmsg_uorb_req &req)
{
	char name[RPMSG_UORB_NAME_LEN];
	strncpy(name, req.name, sizeof(name) - 1);
	name[sizeof(name) - 1] = '\0';

	int16_t status = 0;
	const orb_metadata *meta = lookup(name, req.size, status);

	if (meta == nullptr) {
		_refused++;
		send_ack(req.hdr.type, nullptr, name, req.size, status);
		return;
	}

	pthread_mutex_lock(&_lock);

	switch (req.hdr.type) {
	case RPMSG_UORB_SUBSCRIBE: {
			Sub *s = find_sub(meta);

			if (s == nullptr) {
				s = find_sub(nullptr);
			}

			if (s == nullptr) {
				status = -ENOSPC;
				break;
			}

			s->interval_us = req.rate_hz ? 1000000u / req.rate_hz : 0;
			s->last = 0;
			s->meta = meta;	/* last: publishes see a complete entry */
			break;
		}

	case RPMSG_UORB_UNSUBSCRIBE: {
			Sub *s = find_sub(meta);

			if (s) { *s = Sub{}; }

			break;
		}

	case RPMSG_UORB_ADVERTISE: {
			Adv *a = find_adv(meta);

			if (a == nullptr) {
				a = find_adv(nullptr);
			}

			if (a == nullptr) {
				status = -ENOSPC;
				break;
			}

			a->meta = meta;
			break;
		}
	}

	pthread_mutex_unlock(&_lock);

	send_ack(req.hdr.type, meta, name, req.size, status);

	if (status != 0 || _rx_handler == nullptr) {
		return;
	}

	switch (req.hdr.type) {
	case RPMSG_UORB_SUBSCRIBE:
		/* Manager makes the node push its latest sample through send_message() */
		_rx_handler->process_add_subscription(meta->o_name);
		break;

	case RPMSG_UORB_UNSUBSCRIBE:
		_rx_handler->process_remove_subscription(meta->o_name);
		break;

	case RPMSG_UORB_ADVERTISE:
		/* creates and marks the node so remote data has somewhere to go */
		_rx_handler->process_remote_topic(meta->o_name);
		break;
	}
}

void RpmsgUorbChannel::on_data(const rpmsg_uorb_hdr &hdr, const uint8_t *payload, size_t len)
{
	if (hdr.topic >= orb_topics_count()) {
		_rx_bad++;
		return;
	}

	const orb_metadata *meta = orb_get_topics()[hdr.topic];
	Adv *adv = find_adv(meta);

	if (adv == nullptr) {
		_rx_bad++;
		return;
	}

	if (len != meta->o_size || len > sizeof(_rx_buf)) {
		adv->bad++;
		return;
	}

	memcpy(_rx_buf, payload, len);

	if ((hdr.flags & RPMSG_UORB_F_TS32) && len >= sizeof(uint64_t)) {
		/* remote stamps with the shared 1 MHz counter (low 32 bits of hrt) */
		const hrt_abstime now = hrt_absolute_time();
		hrt_abstime ts = (now & ~static_cast<hrt_abstime>(0xFFFFFFFF)) | hdr.arg;

		if (ts > now + 0x80000000ull) {
			ts -= 0x100000000ull;
		}

		memcpy(_rx_buf, &ts, sizeof(ts));
	}

	adv->received++;

	if (_rx_handler) {
		_rx_handler->process_received_message(meta->o_name, len, _rx_buf);
	}
}

/* ---- helpers ------------------------------------------------------------ */

void RpmsgUorbChannel::send_ack(uint8_t type, const orb_metadata *meta, const char *name, uint16_t size, int16_t status)
{
	rpmsg_uorb_ack ack{};
	ack.hdr.type = RPMSG_UORB_ACK;
	ack.hdr.arg = type;
	ack.hdr.topic = meta ? meta->o_id : RPMSG_UORB_TOPIC_NONE;
	strncpy(ack.name, name, sizeof(ack.name) - 1);
	ack.size = size;
	ack.status = status;

	if (rpmsg_uorb_transport_send(&ack, sizeof(ack), nullptr, 0) < 0) {
		_tx_errors++;
	}
}

RpmsgUorbChannel::Sub *RpmsgUorbChannel::find_sub(const orb_metadata *meta)
{
	for (Sub &s : _subs) {
		if (s.meta == meta) { return &s; }
	}

	return nullptr;
}

RpmsgUorbChannel::Adv *RpmsgUorbChannel::find_adv(const orb_metadata *meta)
{
	for (Adv &a : _advs) {
		if (a.meta == meta) { return &a; }
	}

	return nullptr;
}

void RpmsgUorbChannel::print_status()
{
	PX4_INFO("endpoint %s, hello %" PRIu32 ", requests refused %" PRIu32 ", rx bad %" PRIu32 ", tx errors %" PRIu32,
		 rpmsg_uorb_transport_ready() ? "bound" : "not bound", _hello, _refused, _rx_bad, _tx_errors);

	for (Sub &s : _subs) {
		if (s.meta) {
			PX4_INFO("  -> %-24s %4" PRIu32 " Hz  sent %" PRIu32 " dropped %" PRIu32 " rate-limited %" PRIu32,
				 s.meta->o_name, s.interval_us ? 1000000u / s.interval_us : 0u, s.sent, s.dropped, s.rate_limited);
		}
	}

	for (Adv &a : _advs) {
		if (a.meta) {
			PX4_INFO("  <- %-24s received %" PRIu32 " bad %" PRIu32, a.meta->o_name, a.received, a.bad);
		}
	}
}
