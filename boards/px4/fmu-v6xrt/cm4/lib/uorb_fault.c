#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_vect.h>

#include "fault.h"
#include "libc.h"
#include "rpmsg_remote.h"
#include "rpmsg_uorb_proto.h"
#include "uorb_lite.h"

/* uORB bridge fault kinds, on top of the runtime's (fault.c). */

static const char *const g_kinds[] = {
	"runt", "badtype", "badtopic", "badlen", "badsize", "unknown", "flood",
};

static int send_uorb(const void *msg, uint16_t len)
{
	return rp_send_service(uorb_lite_service(), msg, len);
}

static int send_data(uint16_t topic, uint16_t payload_len)
{
	uint8_t buf[sizeof(struct rpmsg_uorb_hdr) + sizeof(struct debug_key_value_s)];
	struct rpmsg_uorb_hdr *uh = (void *)buf;

	memset(buf, 0, sizeof(buf));
	uh->type = RPMSG_UORB_DATA;
	uh->flags = RPMSG_UORB_F_TS32;
	uh->topic = topic;
	uh->arg = uorb_now_us();
	return send_uorb(buf, (uint16_t)(sizeof(*uh) + payload_len));
}

static int send_advertise(const char *name, uint16_t size)
{
	struct rpmsg_uorb_req req;

	memset(&req, 0, sizeof(req));
	req.hdr.type = RPMSG_UORB_ADVERTISE;
	req.hdr.topic = RPMSG_UORB_TOPIC_NONE;
	str_append(req.name, sizeof(req.name), name);
	req.size = size;
	return send_uorb(&req, sizeof(req));
}

int fault_inject_app(const char *kind, char *reply, size_t cap)
{
	int kv = uorb_handle("debug_key_value");
	int kv_id = uorb_topic_id(kv);

	if (strncmp(kind, "list", 5) == 0) {
		for (size_t i = 0; i < sizeof(g_kinds) / sizeof(g_kinds[0]); i++) {
			str_append(reply, cap, g_kinds[i]);
			str_append(reply, cap, " ");
		}

		return 1;
	}

	if (strncmp(kind, "runt", 5) == 0) {
		int ret = send_uorb("\x06\x01\x00", 3);
		fault_reply(reply, cap, kind, "sent 3 bytes, expect rx bad +1, rp_send ");
		str_append_u32(reply, cap, (uint32_t)ret);
		return 1;
	}

	if (strncmp(kind, "badtype", 8) == 0) {
		struct rpmsg_uorb_hdr uh;
		memset(&uh, 0, sizeof(uh));
		uh.type = 0x7f;
		send_uorb(&uh, sizeof(uh));
		fault_reply(reply, cap, kind, "sent type 0x7f, expect rx bad +1");
		return 1;
	}

	if (strncmp(kind, "badtopic", 9) == 0) {
		send_data(0xfffe, 8);
		send_data(0, 8);
		fault_reply(reply, cap, kind, "sent ORB id 0xfffe and 0, expect rx bad +2");
		return 1;
	}

	if (strncmp(kind, "badlen", 7) == 0) {
		if (kv_id < 0) {
			fault_reply(reply, cap, kind, "debug_key_value not acknowledged");
			return 1;
		}

		send_data((uint16_t)kv_id, sizeof(struct debug_key_value_s) - 1);
		fault_reply(reply, cap, kind, "sent debug_key_value with size - 1, expect <- debug_key_value bad +1");
		return 1;
	}

	if (strncmp(kind, "badsize", 8) == 0) {
		send_advertise("debug_vect", sizeof(struct debug_vect_s) + 1);
		fault_reply(reply, cap, kind, "advertised debug_vect with sizeof + 1, expect requests refused +1");
		return 1;
	}

	if (strncmp(kind, "unknown", 8) == 0) {
		send_advertise("no_such_topic", 8);
		fault_reply(reply, cap, kind, "advertised no_such_topic, expect requests refused +1");
		return 1;
	}

	if (strncmp(kind, "flood", 6) == 0) {
		struct debug_key_value_s kv_msg;
		uint32_t ok = 0;

		memset(&kv_msg, 0, sizeof(kv_msg));
		str_append(kv_msg.key, sizeof(kv_msg.key), "flood");

		for (uint32_t i = 0; i < 64; i++) {
			kv_msg.timestamp = uorb_now_us();
			kv_msg.value = (float)i;

			if (uorb_publish(kv, &kv_msg, sizeof(kv_msg)) == 0) {
				ok++;
			}
		}

		fault_reply(reply, cap, kind, "");
		str_append_u32(reply, cap, ok);
		str_append(reply, cap, "/64 sent, rest refused by a full TX ring (16 buffers)");
		return 1;
	}

	return 0;
}
