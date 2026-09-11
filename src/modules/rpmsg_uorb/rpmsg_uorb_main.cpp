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
 * @file rpmsg_uorb_main.cpp
 *
 * uORB bridge to the remote core over rpmsg: start, status, and a loopback
 * test that publishes debug_value and waits for the remote's
 * debug_key_value echo.
 */

#include "RpmsgUorbChannel.hpp"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_key_value.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

namespace
{

constexpr const char *REMOTE = "cm4";

int do_test(int count)
{
	uORB::Publication<debug_value_s> pub{ORB_ID(debug_value)};
	debug_key_value_s kv{};

	/* poll-driven, so the measurement is not quantised to the 1 ms tick */
	int fd = orb_subscribe(ORB_ID(debug_key_value));

	if (fd < 0) {
		PX4_ERR("subscribe failed");
		return 1;
	}

	px4_pollfd_struct_t fds{};
	fds.fd = fd;
	fds.events = POLLIN;

	while (px4_poll(&fds, 1, 0) > 0) {
		orb_copy(ORB_ID(debug_key_value), fd, &kv);	/* drain stale echoes */
	}

	uint64_t min = UINT64_MAX, max = 0, total = 0;
	int ok = 0;

	for (int i = 0; i < count; i++) {
		debug_value_s v{};
		v.timestamp = hrt_absolute_time();
		v.ind = static_cast<int8_t>(i & 0x7f);
		v.value = static_cast<float>(i);

		const hrt_abstime start = hrt_absolute_time();
		pub.publish(v);

		bool got = false;

		while (!got && hrt_elapsed_time(&start) < 100000) {
			if (px4_poll(&fds, 1, 100) <= 0) {
				break;
			}

			orb_copy(ORB_ID(debug_key_value), fd, &kv);
			got = static_cast<int>(kv.value) == i;
		}

		if (!got) {
			PX4_WARN("no echo for %d", i);
			continue;
		}

		const uint64_t rtt = hrt_elapsed_time(&start);
		ok++;
		total += rtt;
		min = rtt < min ? rtt : min;
		max = rtt > max ? rtt : max;
	}

	orb_unsubscribe(fd);

	if (ok == 0) {
		PX4_ERR("no echoes: is the remote started and subscribed to debug_value?");
		return 1;
	}

	PX4_INFO("%d/%d echoes via %s, key \"%s\"", ok, count, REMOTE, kv.key);
	PX4_INFO("publish -> remote -> publish: min %" PRIu64 " us, avg %" PRIu64 " us, max %" PRIu64 " us", min, total / ok, max);
	PX4_INFO("last echo timestamp %" PRIu64 " us, now %" PRIu64 " us", kv.timestamp, hrt_absolute_time());
	return ok == count ? 0 : 1;
}

void usage()
{
	PRINT_MODULE_DESCRIPTION("uORB bridge to the remote core over rpmsg.");
	PRINT_MODULE_USAGE_NAME("rpmsg_uorb", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Register as the uORB communicator");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Endpoint state and per-topic counters");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Loopback: publish debug_value, expect debug_key_value from the remote");
	PRINT_MODULE_USAGE_ARG("<count>", "Iterations (default 100)", true);
}

} // namespace

extern "C" __EXPORT int rpmsg_uorb_main(int argc, char *argv[]);

int rpmsg_uorb_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return 1;
	}

	if (strcmp(argv[1], "start") == 0) {
		int ret = RpmsgUorbChannel::instance().start(REMOTE);

		if (ret < 0) {
			PX4_ERR("start failed: %d", ret);
			return 1;
		}

		return 0;
	}

	if (strcmp(argv[1], "status") == 0) {
		RpmsgUorbChannel::instance().print_status();
		return 0;
	}

	if (strcmp(argv[1], "test") == 0) {
		return do_test(argc > 2 ? atoi(argv[2]) : 100);
	}

	usage();
	return 1;
}
