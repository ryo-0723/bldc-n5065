/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "foc_speed_source.h"

#include <math.h>
#include <string.h>

static bool configure(foc_speed_source_t *state, const uint8_t table[8]) {
	memset(state, 0, sizeof(*state));
	state->hall = -1;
	memcpy(state->table, table, sizeof(state->table));

	int count = 0;
	for (int i = 0; i < 8; i++) {
		if (table[i] > 200) {
			continue;
		}
		count++;
		for (int j = 0; j < i; j++) {
			if (table[j] <= 200 && table[i] % 200 == table[j] % 200) {
				return false;
			}
		}
	}
	state->initialized = count == 6;
	return state->initialized;
}

void foc_speed_source_update(foc_speed_source_t *state, const uint8_t table[8], int hall, float dt,
		float pll_rpm, float fast_rpm, float sensorless_erpm) {
	if (!isfinite(dt) || dt <= 0.0f || !isfinite(pll_rpm) || !isfinite(fast_rpm) ||
			!isfinite(sensorless_erpm) || sensorless_erpm <= 0.0f) {
		state->initialized = false;
		return;
	}

	// Configuration writes invalidate initialized. Also detect table changes
	// at Hall transitions, matching the previous implementation's reset timing.
	bool changed = !state->initialized;
	if (state->initialized && hall != state->hall) {
		changed = memcmp(state->table, table, sizeof(state->table)) != 0;
	}
	if (changed && !configure(state, table)) {
		return;
	}

	if (fabsf(pll_rpm) >= sensorless_erpm + HALL_FAST_ENTER_MARGIN_ERPM) {
		state->fast_mode = true;
	} else if (fabsf(pll_rpm) <= sensorless_erpm + HALL_FAST_EXIT_MARGIN_ERPM) {
		state->fast_mode = false;
	}

	state->hall = hall >= 0 && hall < 8 && table[hall] <= 200 ? hall : -1;
	float target = state->fast_mode ? 1.0f : 0.0f;
	float step = dt / HALL_FAST_BLEND_SECONDS;
	state->fast_blend += fmaxf(-step, fminf(step, target - state->fast_blend));
}
