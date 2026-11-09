#include "foc_math.h"
#include "utils_math.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
int main(void) {
	mc_configuration c = {0};
	motor_all_state_t m = {0};
	m.m_conf = &c;
	m.m_pll_speed = RPM2RADPS_f(1111);
	m.m_speed_est_fast = RPM2RADPS_f(2222);
	m.m_speed_est_faster = RPM2RADPS_f(3333);
	c.s_pid_speed_source = S_PID_SPEED_SRC_PLL;
	assert(fabsf(foc_selected_speed(&m) - 1111) < .01f);
	c.s_pid_speed_source = S_PID_SPEED_SRC_FAST;
	assert(fabsf(foc_selected_speed(&m) - 2222) < .01f);
	c.foc_sensor_mode = FOC_SENSOR_MODE_HALL;
	c.foc_speed_soure = FOC_SPEED_SRC_CORRECTED;
	assert(foc_hall_speed_selected(&c));
	assert(fabsf(foc_selected_speed(&m) - 1111) < .01f);
	m.m_speed_source.initialized = true;
	assert(fabsf(foc_selected_speed(&m) - 1111) < .01f); // PLL is read directly.
	m.m_control_mode = CONTROL_MODE_SPEED;
	m.m_speed_pid_set_rpm = 1500;
	c.s_pid_kp = .0014f;
	c.s_pid_allow_braking = true;
	c.lo_current_max = 53;
	c.l_current_max_scale = 1;
	foc_run_pid_control_speed(true, .001f, &m);
	assert(fabsf(m.m_iq_set - (1500 - 1111) * .0014f / 20 * 53) < .0001f);
	m.m_speed_source.fast_blend = .5f;
	assert(fabsf(foc_selected_speed(&m) - 1666.5f) < .01f);
	m.m_speed_source.fast_blend = 1;
	assert(fabsf(foc_selected_speed(&m) - 2222) <
		   .01f); // Completed handoff is the original current Fast value.
	m.m_phase_override = true;
	assert(fabsf(foc_selected_speed(&m) - 2222) < .01f);
	m.m_phase_override = false;
	c.foc_sensor_mode = FOC_SENSOR_MODE_SENSORLESS;
	assert(fabsf(foc_selected_speed(&m) - 2222) < .01f);
	c.s_pid_speed_source = S_PID_SPEED_SRC_FASTER;
	assert(fabsf(foc_selected_speed(&m) - 3333) < .01f);
	puts("PID/source selection integration passed");
	return 0;
}
