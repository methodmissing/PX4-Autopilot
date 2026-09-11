#include "uorb_lite.h"
#include "libc.h"
#include "rpmsg_remote.h"
#include "rpmsg_uorb_proto.h"

#define GPT5_CNT (*(volatile uint32_t *)(0x400FC000u + 0x24))

enum { ROLE_NONE, ROLE_SUB, ROLE_ADV };

struct topic {
	const char *name;
	uint16_t size;
	uint16_t rate_hz;
	uint16_t id;		/* ORB_ID from the ACK, RPMSG_UORB_TOPIC_NONE until then */
	uint8_t role;
	uint8_t sent;		/* request already sent to the CM7 */
	int16_t status;
	uorb_cb_t cb;
};

static struct topic g_topics[UORB_LITE_MAX_TOPICS];
static int g_ready;		/* HELLO acknowledged, requests may flow */
static int16_t g_hello_status;
static int g_svc = -1;

static void uorb_lite_rx(const struct rpmsg_hdr *hdr);
static void uorb_lite_bound(void);

void uorb_lite_init(void)
{
	g_svc = rp_register_service(RPMSG_UORB_EPT_NAME, uorb_lite_rx, uorb_lite_bound);
}

/* Service index for fault injection (lib/uorb_fault.c). */
int uorb_lite_service(void)
{
	return g_svc;
}

uint32_t uorb_now_us(void)
{
	return GPT5_CNT;
}

static void send_req(struct topic *t)
{
	struct rpmsg_uorb_req req;

	memset(&req, 0, sizeof(req));
	req.hdr.type = t->role == ROLE_ADV ? RPMSG_UORB_ADVERTISE : RPMSG_UORB_SUBSCRIBE;
	req.hdr.topic = RPMSG_UORB_TOPIC_NONE;
	str_append(req.name, sizeof(req.name), t->name);
	req.size = t->size;
	req.rate_hz = t->rate_hz;

	if (rp_send_service(g_svc, &req, sizeof(req)) == 0) {
		t->sent = 1;
	}
}

static void flush_requests(void)
{
	for (int i = 0; i < UORB_LITE_MAX_TOPICS; i++) {
		if (g_topics[i].role != ROLE_NONE && !g_topics[i].sent) {
			send_req(&g_topics[i]);
		}
	}
}

static void send_hello(void)
{
	struct rpmsg_uorb_hdr hello;

	memset(&hello, 0, sizeof(hello));
	hello.type = RPMSG_UORB_HELLO;
	hello.arg = RPMSG_UORB_VERSION;
	rp_send_service(g_svc, &hello, sizeof(hello));
}

static void uorb_lite_bound(void)
{
	send_hello();
}

static struct topic *by_id(uint16_t id, uint8_t role)
{
	for (int i = 0; i < UORB_LITE_MAX_TOPICS; i++) {
		if (g_topics[i].role == role && g_topics[i].id == id) {
			return &g_topics[i];
		}
	}

	return 0;
}

static struct topic *by_name(const char *name)
{
	for (int i = 0; i < UORB_LITE_MAX_TOPICS; i++) {
		if (g_topics[i].role != ROLE_NONE && strncmp(g_topics[i].name, name, RPMSG_UORB_NAME_LEN) == 0) {
			return &g_topics[i];
		}
	}

	return 0;
}

static void uorb_lite_rx(const struct rpmsg_hdr *hdr)
{
	const struct rpmsg_uorb_hdr *uh = (const void *)hdr->data;

	if (hdr->len < sizeof(*uh)) {
		return;
	}

	switch (uh->type) {
	case RPMSG_UORB_HELLO:
		/* CM7 side restarted: every id is stale, start over */
		g_ready = 0;

		for (int i = 0; i < UORB_LITE_MAX_TOPICS; i++) {
			g_topics[i].sent = 0;
			g_topics[i].id = RPMSG_UORB_TOPIC_NONE;
		}

		send_hello();
		break;

	case RPMSG_UORB_ACK: {
			const struct rpmsg_uorb_ack *ack = (const void *)hdr->data;

			if (hdr->len < sizeof(*ack)) {
				return;
			}

			if (ack->hdr.arg == RPMSG_UORB_HELLO) {
				g_hello_status = ack->status;
				g_ready = ack->status == 0;

				if (g_ready) {
					flush_requests();
				}

				break;
			}

			struct topic *t = by_name(ack->name);

			if (t) {
				t->status = ack->status;
				t->id = ack->status == 0 ? ack->hdr.topic : RPMSG_UORB_TOPIC_NONE;
			}

			break;
		}

	case RPMSG_UORB_DATA: {
			struct topic *t = by_id(uh->topic, ROLE_SUB);
			uint16_t len = hdr->len - sizeof(*uh);

			if (t && t->cb && len == t->size) {
				t->cb(hdr->data + sizeof(*uh), len);
			}

			break;
		}

	default:
		break;
	}
}

int16_t uorb_lite_hello_status(void)
{
	return g_ready ? 0 : (g_hello_status ? g_hello_status : -1);
}

static int add(const char *name, uint16_t size, uint16_t rate_hz, uorb_cb_t cb, uint8_t role)
{
	if (size > RPMSG_UORB_MAX_PAYLOAD) {
		return -1;
	}

	for (int i = 0; i < UORB_LITE_MAX_TOPICS; i++) {
		struct topic *t = &g_topics[i];

		if (t->role != ROLE_NONE) {
			continue;
		}

		t->name = name;
		t->size = size;
		t->rate_hz = rate_hz;
		t->cb = cb;
		t->id = RPMSG_UORB_TOPIC_NONE;
		t->sent = 0;
		t->status = 0;
		t->role = role;

		if (g_ready) {
			send_req(t);
		}

		return i;
	}

	return -1;
}

int uorb_subscribe(const char *name, uint16_t size, uint16_t rate_hz, uorb_cb_t cb)
{
	return add(name, size, rate_hz, cb, ROLE_SUB);
}

int uorb_advertise(const char *name, uint16_t size)
{
	return add(name, size, 0, 0, ROLE_ADV);
}

int uorb_handle(const char *name)
{
	struct topic *t = by_name(name);

	return t ? (int)(t - g_topics) : -1;
}

int uorb_topic_id(int handle)
{
	if (handle < 0 || handle >= UORB_LITE_MAX_TOPICS || g_topics[handle].id == RPMSG_UORB_TOPIC_NONE) {
		return -1;
	}

	return g_topics[handle].id;
}

int uorb_publish(int handle, const void *data, uint16_t size)
{
	uint8_t buf[sizeof(struct rpmsg_uorb_hdr) + RPMSG_UORB_MAX_PAYLOAD];
	struct rpmsg_uorb_hdr *uh = (void *)buf;

	if (handle < 0 || handle >= UORB_LITE_MAX_TOPICS) {
		return -1;
	}

	struct topic *t = &g_topics[handle];

	if (t->role != ROLE_ADV || t->id == RPMSG_UORB_TOPIC_NONE || size != t->size) {
		return -1;
	}

	uh->type = RPMSG_UORB_DATA;
	uh->flags = RPMSG_UORB_F_TS32;
	uh->topic = t->id;
	uh->arg = uorb_now_us();
	memcpy(buf + sizeof(*uh), data, size);

	return rp_send_service(g_svc, buf, (uint16_t)(sizeof(*uh) + size));
}
