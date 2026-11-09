#include "foc_math.h"
#include "utils_math.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static void geometry(int dir, float rpm) {
	mc_configuration c = {0};
	motor_all_state_t m = {0};
	m.m_conf = &c;
	// Midpoint geometry: sector widths 14.4..108 degrees, ratio 7.5.
	const uint8_t table[8] = {255, 4, 60, 16, 180, 0, 132, 255};
	memcpy(c.foc_hall_table, table, 8);
	c.foc_sensor_mode = FOC_SENSOR_MODE_HALL;
	c.foc_speed_soure = FOC_SPEED_SRC_CORRECTED;
	c.s_pid_speed_source = S_PID_SPEED_SRC_FAST;
	c.foc_hall_interp_erpm = 1000;
	c.foc_sl_erpm_start = 3000;
	c.foc_sl_erpm = 3500;
	c.foc_pll_kp = 2000;
	c.foc_pll_ki = 30000;
	m.m_ang_hall_int_prev = -1;
	const float dt = 1.0f / 15000;
	double turns = .37, sum = 0, sq = 0;
	int n = 0;
	for (int i = 0; i < 60000; i++) {
		turns += dir * rpm / 60 * dt;
		float phase = (float)((turns - floor(turns)) * 2 * M_PI);
		float distance = 10;
		int hall = -1;
		for (int h = 0; h < 8; h++)
			if (table[h] <= 200) {
				float d = fabsf(utils_angle_difference_rad(phase, table[h] / 200.0f * 2 * M_PI));
				if (d < distance) {
					distance = d;
					hall = h;
				}
			}
		float angle = foc_correct_hall(phase, dt, &m, hall);
		foc_pll_run(angle, dt, &m.m_pll_phase, &m.m_pll_speed, &c);
		float feedback = foc_selected_speed(&m);
		assert(isfinite(angle) && isfinite(feedback));
		assert(fabsf(feedback - RADPS2RPM_f(m.m_pll_speed)) < .001f);
		if (i > 30000) {
			sum += feedback;
			sq += feedback * feedback;
			n++;
		}
	}
	double mean = sum / n, sigma = sqrt(fmax(0, sq / n - mean * mean));
	printf("unequal Hall widths dir%d rpm%.0f: PLL mean %.2f std %.2f ERPM\n", dir, (double)rpm,
		   mean, sigma);
	assert(fabs(mean - dir * rpm) < fmax(25, rpm * .03));
	if (rpm > 1000)
		assert(sigma < rpm * .03);

	// Zero-initialized interval and standstill must not generate NaN.
	memset(&m, 0, sizeof(m));
	m.m_conf = &c;
	m.m_ang_hall_int_prev = -1;
	for (int i = 0; i < 15000; i++) {
		float a = foc_correct_hall(0, dt, &m, 5);
		foc_pll_run(a, dt, &m.m_pll_phase, &m.m_pll_speed, &c);
		assert(isfinite(a) && isfinite(foc_selected_speed(&m)));
	}
}
int main(void) {
	for (int d = -1; d <= 1; d += 2) {
		geometry(d, 300);
		geometry(d, 1500);
		geometry(d, 2800);
	}
	puts("Actual Hall-angle / legacy-PLL unequal-width tests passed");
	return 0;
}
