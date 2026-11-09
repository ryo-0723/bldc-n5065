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

#ifndef FOC_SPEED_SOURCE_H_
#define FOC_SPEED_SOURCE_H_

#include <stdbool.h>
#include <stdint.h>

// Both thresholds are above foc_sl_erpm, after Hall/observer angle blending.
#define HALL_FAST_ENTER_MARGIN_ERPM 700.0f
#define HALL_FAST_EXIT_MARGIN_ERPM 200.0f
#define HALL_FAST_BLEND_SECONDS 0.050f

// Ki remains the common VESC Tool setting.
#define HALL_FAST_PID_KP 0.00265f
#define HALL_FAST_PID_FULL_ERPM 10000.0f
#define HALL_FAST_PID_GAIN_SECONDS 0.050f

typedef struct {
	uint8_t table[8];
	int hall;
	float fast_blend;
	bool initialized;
	bool fast_mode;
} foc_speed_source_t;

void foc_speed_source_update(foc_speed_source_t *state, const uint8_t table[8], int hall, float dt,
		float pll_rpm, float fast_rpm, float sensorless_erpm);

#endif
