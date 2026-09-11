#include <uORB/topics/debug_array.h>
#include <uORB/topics/sensor_gyro.h>

#include "gyro_analyser.h"
#include "libc.h"
#include "uorb_lite.h"

struct slot {
	uint32_t device_id;
	uint32_t n;
	uint32_t last_us;
	uint32_t max_gap_us;
	uint32_t window_start_us;
	float sum[3];
	float sumsq[3];
	float peak[3];
	float mean_prev[3];	/* previous window's mean, removed before rms/peak/zcr */
	uint32_t crossings[3];
	int8_t sign[3];
};

static struct slot g_slots[GYRO_ANALYSER_SLOTS];
static int g_handle = -1;

void gyro_analyser_init(int debug_array_handle)
{
	g_handle = debug_array_handle;
}

static struct slot *slot_for(uint32_t device_id, uint32_t now)
{
	struct slot *free_slot = 0;

	for (int i = 0; i < GYRO_ANALYSER_SLOTS; i++) {
		if (g_slots[i].device_id == device_id) {
			return &g_slots[i];
		}

		if (!free_slot && g_slots[i].device_id == 0) {
			free_slot = &g_slots[i];
		}
	}

	if (free_slot) {
		memset(free_slot, 0, sizeof(*free_slot));
		free_slot->device_id = device_id;
		/* stagger publications so the three slots do not land together */
		free_slot->window_start_us = now - (uint32_t)((free_slot - g_slots) * 333333u);
	}

	return free_slot;
}

void gyro_analyser_sample(const void *data, uint16_t size)
{
	const struct sensor_gyro_s *g = data;
	uint32_t now = uorb_now_us();

	if (size != sizeof(*g) || g->device_id == 0) {
		return;
	}

	struct slot *s = slot_for(g->device_id, now);

	if (!s) {
		return;
	}

	if (s->last_us) {
		uint32_t gap = now - s->last_us;

		if (gap > s->max_gap_us) {
			s->max_gap_us = gap;
		}
	}

	s->last_us = now;
	s->n++;

	const float v[3] = { g->x, g->y, g->z };

	for (int a = 0; a < 3; a++) {
		float hp = v[a] - s->mean_prev[a];
		float mag = __builtin_fabsf(hp);
		int8_t sign = hp >= 0.0f ? 1 : -1;

		s->sum[a] += v[a];
		s->sumsq[a] += hp * hp;

		if (mag > s->peak[a]) {
			s->peak[a] = mag;
		}

		if (s->sign[a] != 0 && sign != s->sign[a]) {
			s->crossings[a]++;
		}

		s->sign[a] = sign;
	}
}

static void publish(struct slot *s, int idx, uint32_t now)
{
	struct debug_array_s out;
	float dt = (float)(now - s->window_start_us) * 1e-6f;

	if (s->n == 0 || dt <= 0.0f) {
		return;
	}

	memset(&out, 0, sizeof(out));
	out.timestamp = now;
	out.id = (uint16_t)idx;
	str_append(out.name, sizeof(out.name), "cm4gyro");
	out.data[0] = (float)(s->device_id & 0xFFFFu);
	out.data[1] = (float)(s->device_id >> 16);
	out.data[2] = (float)s->n / dt;
	out.data[3] = (float)s->max_gap_us;

	for (int a = 0; a < 3; a++) {
		float mean = s->sum[a] / (float)s->n;
		out.data[4 + a] = mean;
		out.data[7 + a] = __builtin_sqrtf(s->sumsq[a] / (float)s->n);
		out.data[10 + a] = s->peak[a];
		out.data[13 + a] = (float)s->crossings[a] / (2.0f * dt);
		s->mean_prev[a] = mean;
		s->sum[a] = 0.0f;
		s->sumsq[a] = 0.0f;
		s->peak[a] = 0.0f;
		s->crossings[a] = 0;
	}

	s->n = 0;
	s->max_gap_us = 0;
	s->window_start_us = now;

	uorb_publish(g_handle, &out, sizeof(out));
}

void gyro_analyser_poll(uint32_t now_us)
{
	if (g_handle < 0) {
		return;
	}

	for (int i = 0; i < GYRO_ANALYSER_SLOTS; i++) {
		struct slot *s = &g_slots[i];

		if (s->device_id && now_us - s->window_start_us >= 1000000u) {
			publish(s, i, now_us);
		}
	}
}
