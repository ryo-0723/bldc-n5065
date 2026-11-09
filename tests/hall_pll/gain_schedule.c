#include "foc_math.h"
#include "utils_math.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static mc_configuration c;
static motor_all_state_t m;
static void init(int dir) {
	memset(&c, 0, sizeof(c));
	memset(&m, 0, sizeof(m));
	m.m_conf = &c;
	c.foc_sensor_mode = FOC_SENSOR_MODE_HALL;
	c.foc_speed_soure = FOC_SPEED_SRC_CORRECTED;
	c.s_pid_speed_source = S_PID_SPEED_SRC_FAST;
	c.foc_sl_erpm = 1800;
	c.s_pid_kp = .00085f;
	c.s_pid_ki = .012f;
	c.s_pid_kd = .000006f;
	c.s_pid_kd_filter = .2f;
	c.s_pid_allow_braking = true;
	c.lo_current_max = 53;
	c.l_current_max_scale = 1;
	m.m_control_mode = CONTROL_MODE_SPEED;
	m.m_speed_source.initialized = true;
	m.m_pll_speed = RPM2RADPS_f(dir * 1500);
	m.m_speed_est_fast = m.m_pll_speed;
	m.m_speed_pid_set_rpm = dir * 1600;
}
static void speed(float rpm) {
	m.m_pll_speed = RPM2RADPS_f(rpm);
	m.m_speed_est_fast = m.m_pll_speed;
	m.m_speed_pid_set_rpm = rpm + 100;
}
static void run(int n) {
	for (int i = 0; i < n; i++)
		foc_run_pid_control_speed(true, .001f, &m);
}
int main(void) {
	for (int dir = -1; dir <= 1; dir += 2) {
		init(dir);
		run(1);
		float kp, kd;
		foc_speed_pid_gains(&m, &kp, &kd);
		assert(kp == c.s_pid_kp && kd == c.s_pid_kd);
		m.m_speed_source.fast_mode = true;
		m.m_speed_source.fast_blend = 1;
		speed(dir * 2500);
		run(100);
		foc_speed_pid_gains(&m, &kp, &kd);
		assert(fabsf(kp - c.s_pid_kp) < 1e-8f && kd == 0 && m.m_speed_d_filter == 0);
		speed(dir * 6250);
		run(100);
		foc_speed_pid_gains(&m, &kp, &kd);
		assert(fabsf(kp - (c.s_pid_kp + .00265f) * .5f) < 1e-7f && kd == 0);
		speed(dir * 10000);
		run(100);
		foc_speed_pid_gains(&m, &kp, &kd);
		assert(fabsf(kp - .00265f) < 1e-8f && kd == 0);
		speed(dir * 1500);
		m.m_speed_source.fast_blend = 0;
		m.m_speed_source.fast_mode = false;
		run(100);
		foc_speed_pid_gains(&m, &kp, &kd);
		assert(kp == c.s_pid_kp && kd == c.s_pid_kd);
	}
	// Coefficient changes do not inject a current step (small common Ki).
	init(1);
	c.s_pid_ki = 1e-8f;
	c.s_pid_kd = 0;
	run(1);
	float before = m.m_iq_set;
	speed(10000);
	m.m_speed_prev_error = 100;
	m.m_speed_source.fast_mode = true;
	m.m_speed_source.fast_blend = 1;
	run(100);
	assert(fabsf(m.m_iq_set - before) < .0001f);
	// Changes in real error still see the new high P gain.
	before = m.m_iq_set;
	m.m_speed_pid_set_rpm += 100;
	run(1);
	assert(fabsf(m.m_iq_set - before - 100 * .00265f / 20 * 53) < .0001f);
	// D is removed immediately, its prior output transferred once to I.
	init(1);
	c.s_pid_ki = 1e-8f;
	run(1);
	m.m_speed_d_filter = .01f;
	before = (100 * c.s_pid_kp / 20 + .01f + m.m_speed_i_term) * 53;
	m.m_speed_source.fast_mode = true;
	run(1);
	assert(m.m_speed_d_filter == 0 && fabsf(m.m_iq_set - before) < .0001f);
	// Ki remains the Tool setting at high speed; no hidden multiplier.
	init(1);
	c.s_pid_kd = 0;
	m.m_speed_source.fast_mode = true;
	m.m_speed_source.fast_blend = 1;
	m.m_speed_gain_weight = 1;
	speed(10000);
	c.s_pid_ki = .012f;
	run(1);
	assert(fabsf(m.m_speed_i_term - 100 * .012f * .001f / 20) < 1e-8f);
	c.s_pid_ki = .015f;
	float old = m.m_speed_i_term;
	run(1);
	assert(fabsf(m.m_speed_i_term - old - 100 * .015f * .001f / 20) < 1e-8f);
	// Ki=0 disables compensation, preserves zero I, but P still schedules.
	init(1);
	c.s_pid_ki = 0;
	c.s_pid_kd = 0;
	run(1);
	speed(10000);
	m.m_speed_source.fast_mode = true;
	m.m_speed_source.fast_blend = 1;
	run(100);
	assert(m.m_speed_i_term == 0 && fabsf(m.m_iq_set - 100 * .00265f / 20 * 53) < .0001f);
	// Other sensor/source modes retain the original Tool gains.
	float kp, kd;
	c.foc_sensor_mode = FOC_SENSOR_MODE_SENSORLESS;
	foc_speed_pid_gains(&m, &kp, &kd);
	assert(kp == c.s_pid_kp && kd == c.s_pid_kd);
	// Compensation and subsequent I updates retain the original bounded state.
	init(1);
	c.s_pid_kd = 0;
	run(1);
	m.m_speed_i_term = .99f;
	speed(10000);
	m.m_speed_pid_set_rpm = 0;
	m.m_speed_gain_weight = 1;
	m.m_speed_source.fast_mode = true;
	m.m_speed_source.fast_blend = 1;
	run(1);
	assert(fabsf(m.m_speed_i_term) <= 1 && fabsf(m.m_iq_set) <= 53);
	// Stopping clears scheduling history.
	m.m_control_mode = CONTROL_MODE_CURRENT;
	run(1);
	assert(m.m_speed_gain_weight == 0 && !m.m_speed_gain_active);
	puts("Gain schedule: signed endpoints, D disable, bounded handoff, Tool Ki, Ki=0, source "
		 "fallback passed");
	return 0;
}
