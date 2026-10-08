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

#include <uORB/uORB.h>
#include <uORB/uORBCommunicator.hpp>
#include <drivers/drv_hrt.h>

#include <pthread.h>
#include <stdint.h>

#include "rpmsg_uorb_proto.h"

/**
 * uORB communicator over an rpmsg endpoint.
 *
 * The remote pulls: it subscribes to topics it wants and advertises topics it
 * publishes, naming them once. Afterwards data carries the ORB_ID. The
 * Manager calls send_message() for every local publish, so the subscribed set
 * is checked by metadata pointer before anything else happens.
 */
class RpmsgUorbChannel : public uORBCommunicator::IChannel
{
public:
	static RpmsgUorbChannel &instance();

	int start(const char *cpuname);
	void print_status();

	// uORBCommunicator::IChannel. The remote decides what it wants, so the
	// Manager's own advertise and subscriber notifications go nowhere.
	int16_t topic_advertised(const char *messageName) override { return 0; }
	int16_t add_subscription(const char *messageName, int32_t msgRateInHz) override { return 0; }
	int16_t remove_subscription(const char *messageName) override { return 0; }
	int16_t register_handler(uORBCommunicator::IChannelRxHandler *handler) override;
	int16_t send_message(const char *messageName, int32_t length, uint8_t *data) override;

	static constexpr int MAX_SUBS = 16;	/* topics the remote receives */
	static constexpr int MAX_ADVS = 8;	/* topics the remote publishes */

private:
	RpmsgUorbChannel() = default;

	struct Sub {
		const orb_metadata *meta{nullptr};
		uint32_t interval_us{0};
		hrt_abstime last{0};
		uint32_t sent{0};
		uint32_t dropped{0};
		uint32_t rate_limited{0};
	};

	struct Adv {
		const orb_metadata *meta{nullptr};
		uint32_t received{0};
		uint32_t bad{0};
	};

	static void rx_trampoline(const uint8_t *data, size_t len);
	void on_rx(const uint8_t *data, size_t len);
	void on_hello(uint32_t version);
	void on_request(const rpmsg_uorb_req &req);
	void on_data(const rpmsg_uorb_hdr &hdr, const uint8_t *payload, size_t len);

	const orb_metadata *lookup(const char *name, uint16_t size, int16_t &status);
	void send_ack(uint8_t type, const orb_metadata *meta, const char *name, uint16_t size, int16_t status);

	Sub *find_sub(const orb_metadata *meta);
	Adv *find_adv(const orb_metadata *meta);

	uORBCommunicator::IChannelRxHandler *_rx_handler{nullptr};
	pthread_mutex_t _lock = PTHREAD_MUTEX_INITIALIZER;

	Sub _subs[MAX_SUBS];
	Adv _advs[MAX_ADVS];

	uint8_t _rx_buf[RPMSG_UORB_MAX_PAYLOAD];	/* receive runs on one thread */

	uint32_t _hello{0};
	uint32_t _rx_bad{0};
	uint32_t _tx_errors{0};
	uint32_t _refused{0};	/* HELLO with a foreign version, requests for unknown or mis-sized topics */
	bool _started{false};
};
