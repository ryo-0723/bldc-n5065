// Deterministic signal replay for comparison with the pre-cleanup source.
#include "foc_math.h"
#include "utils_math.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
int main(void) {
	for (int scenario = 0; scenario < 12; scenario++) {
		mc_configuration c = {0};
		motor_all_state_t m = {0};
		m.m_conf = &c;
		const uint8_t table[8] = {255, 60, 119, 80, 181, 19, 160, 255};
		memcpy(c.foc_hall_table, table, 8);
		c.foc_sensor_mode = FOC_SENSOR_MODE_HALL;
		c.foc_speed_soure = FOC_SPEED_SRC_CORRECTED;
		c.s_pid_speed_source = S_PID_SPEED_SRC_FAST;
		c.foc_sl_erpm = 1800;
		c.foc_sl_erpm_start = 1300;
		c.foc_hall_interp_erpm = 300;
		c.foc_pll_kp = 720;
		c.foc_pll_ki = 37500;
		c.s_pid_kp = .00085f;
		c.s_pid_ki = scenario == 6 ? 0 : .015f;
		c.s_pid_kd = scenario == 7 ? 0 : .000006f;
		c.s_pid_kd_filter = .2f;
		c.s_pid_allow_braking = scenario != 8;
		c.s_pid_min_erpm = 200;
		c.s_pid_ramp_erpms_s = 20000;
		c.lo_current_max = 53;
		c.l_current_max_scale = 1;
		c.l_max_erpm = 25000;
		c.l_min_erpm = -25000;
		m.m_ang_hall_int_prev = -1;
		m.m_hall_dt_diff_last = scenario == 9 ? 0 : 1;
		const float dt = 1.0f / 15000;
		double turns = 0;
		m.m_control_mode = CONTROL_MODE_SPEED;
		for (int i = 0; i < 45000; i++) {
			float t = i * dt;
			float rpm =
				t < 1 ? 10000 * t : (t < 2 ? 10000 - 20000 * (t - 1) : -10000 + 10000 * (t - 2));
			if (scenario % 2)
				rpm = -rpm;
			turns += rpm / 60 * dt;
			float phase = (float)((turns - floor(turns)) * 2 * M_PI);
			float nearest = 10;
			int hall = -1;
			for (int h = 0; h < 8; h++)
				if (c.foc_hall_table[h] <= 200) {
					float diff = fabsf(
						utils_angle_difference_rad(phase, c.foc_hall_table[h] / 200.0f * 2 * M_PI));
					if (diff < nearest) {
						nearest = diff;
						hall = h;
					}
				}
			if (scenario == 2 && i % 131 == 0)
				hall = 0;
			if (scenario == 3 && i == 17000)
				c.foc_hall_table[4] = 180;
			if (scenario == 4 && i == 18000)
				c.foc_hall_table[4] = 160;
			if (scenario == 4 && i == 22000)
				c.foc_hall_table[4] = 181;
			if (scenario == 5)
				c.s_pid_speed_source = t < 1 ? S_PID_SPEED_SRC_PLL : S_PID_SPEED_SRC_FASTER;
			if (scenario == 10)
				m.m_phase_override = t > 1 && t < 2;
			if (scenario == 11)
				m.m_control_mode = t > 1 && t < 1.2f ? CONTROL_MODE_CURRENT : CONTROL_MODE_SPEED;
			float angle = foc_correct_hall(phase, dt, &m, hall);
			foc_pll_run(angle, dt, &m.m_pll_phase, &m.m_pll_speed, &c);
			m.m_speed_est_fast = RPM2RADPS_f(rpm + 20 * sinf(t * 400));
			m.m_speed_est_faster = RPM2RADPS_f(rpm + 30 * sinf(t * 600));
			m.m_speed_command_rpm = t < 1 ? 10000 : -10000;
			if (i % 15 == 0) {
				foc_run_pid_control_speed(true, dt * 15, &m);
				float kp, kd;
				foc_speed_pid_gains(&m, &kp, &kd);
				printf("%d,%d,%a,%a,%a,%a,%a,%a,%a,%a\n", scenario, i, (double)angle,
					   (double)foc_selected_speed(&m), (double)m.m_iq_set, (double)m.m_speed_i_term,
					   (double)m.m_speed_d_filter, (double)m.m_speed_gain_weight, (double)kp,
					   (double)kd);
			}
		}
	}
	return 0;
}
