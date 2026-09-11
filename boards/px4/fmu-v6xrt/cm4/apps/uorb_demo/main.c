#include <uORB/topics/debug_array.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/sensor_gyro.h>

#include "gyro_analyser.h"
#include "libc.h"
#include "mu.h"
#include "rpmsg_remote.h"
#include "status.h"
#include "uorb_lite.h"

/* Loopback: every debug_value from the CM7 comes back as debug_key_value
 * with key "cm4" and the same value (rpmsg_uorb test).
 */
static int g_kv;

static void on_debug_value(const void *data, uint16_t size)
{
	const struct debug_value_s *in = data;
	struct debug_key_value_s out;

	(void)size;
	memset(&out, 0, sizeof(out));
	out.timestamp = uorb_now_us();
	out.value = in->value;
	str_append(out.key, sizeof(out.key), "cm4");
	uorb_publish(g_kv, &out, sizeof(out));
}

int main(void)
{
	mu_init();
	cm4_status[0] = CM4_STATE_MU;

	uorb_lite_init();
	uorb_subscribe("debug_value", sizeof(struct debug_value_s), 0, on_debug_value);
	g_kv = uorb_advertise("debug_key_value", sizeof(struct debug_key_value_s));

	/* Workload: per-IMU gyro statistics from every sample, 1 Hz debug_array */
	uorb_subscribe("sensor_gyro", sizeof(struct sensor_gyro_s), 0, gyro_analyser_sample);
	gyro_analyser_init(uorb_advertise("debug_array", sizeof(struct debug_array_s)));

	rp_init();
	cm4_status[0] = CM4_STATE_READY;

	for (;;) {
		if (mu_rx_pending) {
			mu_rx_pending = 0;
			cm4_status[2]++;
			rp_process();

		} else {
			__asm volatile("wfi");
		}

		/* runs after every kick; gyro traffic keeps this well under 1 ms */
		gyro_analyser_poll(uorb_now_us());
	}
}
