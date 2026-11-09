#include "foc_speed_source.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static const uint8_t table[8] = {255, 60, 119, 80, 180, 19, 160, 255};
static const float dt = 1.0f / 15000;
static void switching(void) {
	foc_speed_source_t s = {0};
	for (int i = 0; i < 1500; i++)
		foc_speed_source_update(&s, table, 5, dt, 3800, 4000, 3000);
	assert(s.fast_mode && s.fast_blend == 1);
	for (int i = 0; i < 200; i++)
		foc_speed_source_update(&s, table, 5, dt, 3500, 3600, 3000);
	assert(s.fast_mode);
	foc_speed_source_update(&s, table, 5, dt, 3200, 3300, 3000);
	assert(!s.fast_mode);
	foc_speed_source_update(&s, table, 0, dt, 1500, 1500, 3000);
	assert(s.initialized);
	foc_speed_source_update(&s, table, 5, dt, NAN, 1500, 3000);
	assert(!s.initialized);
	uint8_t invalid[8] = {255, 1, 1, 2, 3, 4, 5, 255};
	foc_speed_source_update(&s, invalid, 5, dt, 1500, 1500, 3000);
	assert(!s.initialized);
}
static void source_hysteresis(void) {
	for (int dir = -1; dir <= 1; dir += 2) {
		foc_speed_source_t s = {0};
		for (int i = 0; i < 5000; i++) {
			// Hall states do not gate PLL feedback.
			float pll = dir * (200 + i * .3f), fast = dir * 2900;
			foc_speed_source_update(&s, table, 5, dt, pll, fast, 3000);
			assert(s.fast_blend == 0);
		}
		// Invalid Hall samples do not replace or filter PLL feedback.
		foc_speed_source_update(&s, table, 0, dt, dir * 100, dir * 2000, 3000);
		assert(s.fast_blend == 0 && s.initialized);
		for (int i = 0; i < 1500; i++)
			foc_speed_source_update(&s, table, 5, dt, dir * 3800, dir * 4100, 3000);
		assert(s.fast_blend == 1);
		// Hold inside the band, then leave on its lower (still sensorless) side.
		for (int i = 0; i < 100; i++)
			foc_speed_source_update(&s, table, 0, dt, dir * 3400, dir * 3900, 3000);
		assert(s.fast_mode && s.fast_blend == 1);
		float previous = s.fast_blend;
		for (int i = 0; i < 1500; i++) {
			foc_speed_source_update(&s, table, 0, dt, dir * 3200, dir * 3900, 3000);
			assert(!s.fast_mode);
			// Blend progresses continuously.
			assert(fabsf(s.fast_blend - previous) <= dt / HALL_FAST_BLEND_SECONDS + 1e-7f);
			previous = s.fast_blend;
		}
		assert(s.fast_blend == 0);
	}
	puts("Both-direction source hysteresis, table validation and 50ms handoff passed");
}

int main(void) {
	switching();
	source_hysteresis();
	puts("PLL/Fast source switching tests passed");
	return 0;
}
