#pragma once

#include <stdint.h>

/* Minimal uORB client over the "rpmsg-uorb" endpoint: subscribe to CM7
 * topics, advertise and publish CM4 topics. Names are sent once; data
 * carries the CM7's ORB_ID afterwards. Protocol in
 * src/modules/rpmsg_uorb/rpmsg_uorb_proto.h.
 */

#define UORB_LITE_MAX_TOPICS 8

typedef void (*uorb_cb_t)(const void *data, uint16_t size);

/* Register the rpmsg-uorb endpoint with the runtime; call before rp_init(). */
void uorb_lite_init(void);

/* Both return a handle >= 0 or -1 when the table is full. Requests are sent
 * as soon as the endpoint is bound; acknowledgements fill in the ORB_ID.
 */
int uorb_subscribe(const char *name, uint16_t size, uint16_t rate_hz, uorb_cb_t cb);
int uorb_advertise(const char *name, uint16_t size);

/* Send one sample; the header carries uorb_now_us() so the CM7 rebuilds a
 * 64-bit timestamp. -1 when the topic is not acknowledged yet or the send
 * failed.
 */
int uorb_publish(int handle, const void *data, uint16_t size);

/* Shared 1 MHz counter: GPT5, the CM7's hrt timer. Low 32 bits of hrt. */
uint32_t uorb_now_us(void);

/* Handle of a subscribed or advertised topic by name, -1 if none. */
int uorb_handle(const char *name);

/* ORB_ID acknowledged by the CM7 for a handle, -1 until acknowledged. */
int uorb_topic_id(int handle);

/* Runtime service index of the endpoint, -1 before uorb_lite_init(). */
int uorb_lite_service(void);

/* 0 once the CM7 acknowledged HELLO, -1 before, else the CM7's status (-EPROTO). */
int16_t uorb_lite_hello_status(void);
