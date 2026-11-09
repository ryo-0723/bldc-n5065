#include "foc_math.h"
#include "utils_math.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
int main(void) {
	mc_configuration c = {0};
	motor_all_state_t m = {0};
	m.m_conf = &c;
	const int table[8] = {255, 60, 119, 80, 180, 19, 160, 255};
	const int states[6] = {5, 1, 3, 2, 6, 4};
	const float widths[6] = {102, 44, 31, 107, 39, 37};
	for (int i = 0; i < 8; i++)
		c.foc_hall_table[i] = table[i];
	c.foc_sensor_mode = FOC_SENSOR_MODE_HALL;
	c.foc_speed_soure = FOC_SPEED_SRC_CORRECTED;
	c.s_pid_speed_source = S_PID_SPEED_SRC_FAST;
	c.foc_hall_interp_erpm = 1000;
	c.foc_sl_erpm_start = 2000;
	c.foc_sl_erpm = 3000;
	m.m_ang_hall_int_prev = -1;
	m.m_hall_dt_diff_last = 1;
	m.m_hall_dt_diff_now = 1;
	double turns = 0;
	const float dt = 1.0f / 15000;
	for (int i = 0; i < 45000; i++) {
		float t = i * dt, rpm = 2200 + 1800 * sinf(t * 5.0f);
		if (t > 1.5f)
			rpm = -rpm;
		turns += rpm / 60 * dt;
		float phase = (float)((turns - floor(turns)) * 6.28318530718);
		float angle = phase * 180 / 3.14159265359f, edge = 0;
		int hall = states[5];
		for (int j = 0; j < 6; j++) {
			edge += widths[j];
			if (angle < edge) {
				hall = states[j];
				break;
			}
		}
		m.m_pll_speed = RPM2RADPS_f(rpm);
		m.m_speed_est_fast = m.m_pll_speed;
		if (i % 1777 == 0)
			hall = 0;
		float out = foc_correct_hall(phase, dt, &m, hall);
		if (i % 37 == 0)
			printf("%a\n", (double)out);
	}
	return 0;
}
