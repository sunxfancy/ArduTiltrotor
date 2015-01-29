/* 
* @Author: sxf
* @Date:   2015-01-22 17:02:34
* @Last Modified by:   sxf
* @Last Modified time: 2015-01-24 21:21:57
* @file MotorsTiltrotor.cpp
* @brief 这是倾转旋翼机的专用发动机类
*/
#include <AP_HAL.h>
#include <AP_Math.h>
#include "MotorsTiltrotor.h"

extern const AP_HAL::HAL& hal;

void MotorsTiltrotor::Init() {
	// call parent Init function to set-up throttle curve
	AP_Motors::Init();

	// setup the motors
	setup_motors();

	// enable fast channels or instant pwm
	set_update_rate(_speed_hz);

	// disable CH5-8 from being used as an aux output (i.e. for camera gimbal, etc)
	RC_Channel_aux::disable_aux_channel(MOTORS_CH_AILERON);
	RC_Channel_aux::disable_aux_channel(MOTORS_CH_TAIL);
	RC_Channel_aux::disable_aux_channel(MOTORS_CH_YAW);
	RC_Channel_aux::disable_aux_channel(MOTORS_CH_TILTROTOR);
}

void MotorsTiltrotor::setup_motors()
{
	remove_all_motors();
	// 这里第一个参数是发动机号，其次两个是滚转roll、俯仰pitch的参数，最后一个参数是测试排序
	add_motor_raw(AP_MOTORS_MOT_1,	0.6,	0.6,	-0.8,	1); // 左前
	add_motor_raw(AP_MOTORS_MOT_2,	-0.6,	0.6,	0.8,	2); // 右前
	add_motor_raw(AP_MOTORS_MOT_3,	1.0,	-1.0,	1.0,	3); // 左后
	add_motor_raw(AP_MOTORS_MOT_4,	-1.0,	-1.0,	-1.0,	4); // 右后
};


// set update rate to motors - a value in hertz
void MotorsTiltrotor::set_update_rate(uint16_t speed_hz)
{
	int8_t i;

	// record requested speed
	_speed_hz = speed_hz;

	// check each enabled motor
	uint32_t mask = 0;
	for (i = 0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
		if (motor_enabled[i]) {
			mask |= 1U << pgm_read_byte(&_motor_to_channel_map[i]);
		}
	}
	hal.rcout->set_freq(mask, _speed_hz);
}

// set frame orientation (normally + or X)
void MotorsTiltrotor::set_frame_orientation(uint8_t new_orientation)
{
	// return if nothing has changed
	if (new_orientation == _flags.frame_orientation) {
		return;
	}

	// call parent
	AP_Motors::set_frame_orientation(new_orientation);

	// setup the motors
	setup_motors();

	// enable fast channels or instant pwm
	set_update_rate(_speed_hz);
}

// enable - starts allowing signals to be sent to motors
void MotorsTiltrotor::enable()
{
	int8_t i;

	// enable output channels
	for (i = 0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
		if (motor_enabled[i]) {
			hal.rcout->enable_ch(pgm_read_byte(&_motor_to_channel_map[i]));
		}
	}
}

// output_min - sends minimum values out to the motors
void MotorsTiltrotor::output_min()
{
	int8_t i;

	// set limits flags
	limit.roll_pitch = true;
	limit.yaw = true;
	limit.throttle_lower = true;
	limit.throttle_upper = false;

	// fill the motor_out[] array for HIL use and send minimum value to each motor
	for (i = 0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
		if (motor_enabled[i]) {
			hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[i]), _rc_throttle.radio_min);
		}
	}
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint16_t MotorsTiltrotor::get_motor_mask()
{
	uint16_t mask = 0;
	for (uint8_t i = 0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
		if (motor_enabled[i]) {
			mask |= 1U << i;
		}
	}
	return mask;
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
void MotorsTiltrotor::output_armed()
{
	int8_t i;
	int16_t out_min_pwm = _rc_throttle.radio_min + _min_throttle;      // minimum pwm value we can send to the motors
	int16_t out_max_pwm = _rc_throttle.radio_max;                      // maximum pwm value we can send to the motors
	int16_t out_mid_pwm = (out_min_pwm + out_max_pwm) / 2;                  // mid pwm value we can send to the motors
	int16_t out_best_thr_pwm;  // the is the best throttle we can come up which provides good control without climbing
	float rpy_scale = 1.0; // this is used to scale the roll, pitch and yaw to fit within the motor limits

	int16_t rpy_out[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.
	int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final outputs sent to the motors

	int16_t rpy_low = 0;    // lowest motor value
	int16_t rpy_high = 0;   // highest motor value
	int16_t yaw_allowed;    // amount of yaw we can fit in
	int16_t thr_adj;        // the difference between the pilot's desired throttle and out_best_thr_pwm (the throttle that is actually provided)

	// initialize limits flag
	limit.roll_pitch = false;
	limit.yaw = false;
	limit.throttle_lower = false;
	limit.throttle_upper = false;

	// Throttle is 0 to 1000 only
	// To-Do: we should not really be limiting this here because we don't "own" this _rc_throttle object
	if (_rc_throttle.servo_out <= 0) {
		_rc_throttle.servo_out = 0;
		limit.throttle_lower = true;
	}
	if (_rc_throttle.servo_out >= _max_throttle) {
		_rc_throttle.servo_out = _max_throttle;
		limit.throttle_upper = true;
	}

	// capture desired roll, pitch, yaw and throttle from receiver
	_rc_roll.calc_pwm();
	_rc_pitch.calc_pwm();
	_rc_throttle.calc_pwm();
	_rc_yaw.calc_pwm();

	// if we are not sending a throttle output, we cut the motors
	if (_rc_throttle.servo_out == 0) {
		// range check spin_when_armed
		if (_spin_when_armed_ramped < 0) {
			_spin_when_armed_ramped = 0;
		}
		if (_spin_when_armed_ramped > _min_throttle) {
			_spin_when_armed_ramped = _min_throttle;
		}
		for (i = 0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
			// spin motors at minimum
			if (motor_enabled[i]) {
				motor_out[i] = _rc_throttle.radio_min + _spin_when_armed_ramped;
			}
		}

		// Every thing is limited
		limit.roll_pitch = true;
		limit.yaw = true;

	}
	else {

		// check if throttle is below limit
		if (_rc_throttle.servo_out <= _min_throttle) {  // perhaps being at min throttle itself is not a problem, only being under is
			limit.throttle_lower = true;
		}

		// calculate roll and pitch for each motor
		// set rpy_low and rpy_high to the lowest and highest values of the motors
		for (i = 0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
			if (motor_enabled[i]) {
				rpy_out[i] = _rc_roll.pwm_out * _roll_factor[i] +
					_rc_pitch.pwm_out * _pitch_factor[i];

				// record lowest roll pitch command
				if (rpy_out[i] < rpy_low) {
					rpy_low = rpy_out[i];
				}
				// record highest roll pich command
				if (rpy_out[i] > rpy_high) {
					rpy_high = rpy_out[i];
				}
			}
		}

		// calculate throttle that gives most possible room for yaw (range 1000 ~ 2000) which is the lower of:
		//      1. mid throttle - average of highest and lowest motor (this would give the maximum possible room margin above the highest motor and below the lowest)
		//      2. the higher of:
		//            a) the pilot's throttle input
		//            b) the mid point between the pilot's input throttle and hover-throttle
		//      Situation #2 ensure we never increase the throttle above hover throttle unless the pilot has commanded this.
		//      Situation #2b allows us to raise the throttle above what the pilot commanded but not so far that it would actually cause the copter to rise.
		//      We will choose #1 (the best throttle for yaw control) if that means reducing throttle to the motors (i.e. we favour reducing throttle *because* it provides better yaw control)
		//      We will choose #2 (a mix of pilot and hover throttle) only when the throttle is quite low.  We favour reducing throttle instead of better yaw control because the pilot has commanded it
		int16_t motor_mid = (rpy_low + rpy_high) / 2;
		out_best_thr_pwm = min(out_mid_pwm - motor_mid, max(_rc_throttle.radio_out, (_rc_throttle.radio_out + _hover_out) / 2));

		// calculate amount of yaw we can fit into the throttle range
		// this is always equal to or less than the requested yaw from the pilot or rate controller
		yaw_allowed = min(out_max_pwm - out_best_thr_pwm, out_best_thr_pwm - out_min_pwm) - (rpy_high - rpy_low) / 2;
		yaw_allowed = max(yaw_allowed, AP_MOTORS_MATRIX_YAW_LOWER_LIMIT_PWM);

		if (_rc_yaw.pwm_out >= 0) {
			// if yawing right
			if (yaw_allowed > _rc_yaw.pwm_out) {
				yaw_allowed = _rc_yaw.pwm_out; // to-do: this is bad form for yaw_allows to change meaning to become the amount that we are going to output
			}
			else{
				limit.yaw = true;
			}
		}
		else{
			// if yawing left
			yaw_allowed = -yaw_allowed;
			if (yaw_allowed < _rc_yaw.pwm_out) {
				yaw_allowed = _rc_yaw.pwm_out; // to-do: this is bad form for yaw_allows to change meaning to become the amount that we are going to output
			}
			else{
				limit.yaw = true;
			}
		}

		// add yaw to intermediate numbers for each motor
		rpy_low = 0;
		rpy_high = 0;
		for (i = 0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
			if (motor_enabled[i]) {
				rpy_out[i] = rpy_out[i] +
					yaw_allowed * _yaw_factor[i];

				// record lowest roll+pitch+yaw command
				if (rpy_out[i] < rpy_low) {
					rpy_low = rpy_out[i];
				}
				// record highest roll+pitch+yaw command
				if (rpy_out[i] > rpy_high) {
					rpy_high = rpy_out[i];
				}
			}
		}

		// check everything fits
		thr_adj = _rc_throttle.radio_out - out_best_thr_pwm;

		// calc upper and lower limits of thr_adj
		int16_t thr_adj_max = max(out_max_pwm - (out_best_thr_pwm + rpy_high), 0);

		// if we are increasing the throttle (situation #2 above)..
		if (thr_adj > 0) {
			// increase throttle as close as possible to requested throttle
			// without going over out_max_pwm
			if (thr_adj > thr_adj_max){
				thr_adj = thr_adj_max;
				// we haven't even been able to apply full throttle command
				limit.throttle_upper = true;
			}
		}
		else if (thr_adj < 0){
			// decrease throttle as close as possible to requested throttle
			// without going under out_min_pwm or over out_max_pwm
			// earlier code ensures we can't break both boundaries
			int16_t thr_adj_min = min(out_min_pwm - (out_best_thr_pwm + rpy_low), 0);
			if (thr_adj > thr_adj_max) {
				thr_adj = thr_adj_max;
				limit.throttle_upper = true;
			}
			if (thr_adj < thr_adj_min) {
				thr_adj = thr_adj_min;
				limit.throttle_lower = true;
			}
		}

		// do we need to reduce roll, pitch, yaw command
		// earlier code does not allow both limit's to be passed simultainiously with abs(_yaw_factor)<1
		if ((rpy_low + out_best_thr_pwm) + thr_adj < out_min_pwm){
			rpy_scale = (float)(out_min_pwm - thr_adj - out_best_thr_pwm) / rpy_low;
			// we haven't even been able to apply full roll, pitch and minimal yaw without scaling
			limit.roll_pitch = true;
			limit.yaw = true;
		}
		else if ((rpy_high + out_best_thr_pwm) + thr_adj > out_max_pwm){
			rpy_scale = (float)(out_max_pwm - thr_adj - out_best_thr_pwm) / rpy_high;
			// we haven't even been able to apply full roll, pitch and minimal yaw without scaling
			limit.roll_pitch = true;
			limit.yaw = true;
		}

		// add scaled roll, pitch, constrained yaw and throttle for each motor
		for (i = 0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
			if (motor_enabled[i]) {
				motor_out[i] = out_best_thr_pwm + thr_adj +
					rpy_scale*rpy_out[i];
			}
		}

		// adjust for throttle curve
		if (_throttle_curve_enabled) {
			for (i = 0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
				if (motor_enabled[i]) {
					motor_out[i] = _throttle_curve.get_y(motor_out[i]);
				}
			}
		}
		// clip motor output if required (shouldn't be)
		for (i = 0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
			if (motor_enabled[i]) {
				motor_out[i] = constrain_int16(motor_out[i], out_min_pwm, out_max_pwm);
			}
		}
	}

	// send output to each motor
	for (i = 0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
		if (motor_enabled[i]) {
			hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[i]), motor_out[i]);
		}
	}
}

// output_disarmed - sends commands to the motors
void MotorsTiltrotor::output_disarmed()
{
	// Send minimum values to all motors
	output_min();
}

// output_test - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void MotorsTiltrotor::output_test(uint8_t motor_seq, int16_t pwm)
{
	// exit immediately if not armed
	if (!_flags.armed) {
		return;
	}

	// loop through all the possible orders spinning any motors that match that description
	for (uint8_t i = 0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
		if (motor_enabled[i] && _test_order[i] == motor_seq) {
			// turn on this motor
			hal.rcout->write(pgm_read_byte(&_motor_to_channel_map[i]), pwm);
		}
	}
}

// add_motor
void MotorsTiltrotor::add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order)
{
	// ensure valid motor number is provided
	if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS) {

		// increment number of motors if this motor is being newly motor_enabled
		if (!motor_enabled[motor_num]) {
			motor_enabled[motor_num] = true;
		}

		// set roll, pitch, thottle factors and opposite motor (for stability patch)
		_roll_factor[motor_num] = roll_fac;
		_pitch_factor[motor_num] = pitch_fac;
		_yaw_factor[motor_num] = yaw_fac;

		// set order that motor appears in test
		_test_order[motor_num] = testing_order;

		// disable this channel from being used by RC_Channel_aux
		RC_Channel_aux::disable_aux_channel(_motor_to_channel_map[motor_num]);
	}
}

// add_motor using just position and prop direction - assumes that for each motor, roll and pitch factors are equal
void MotorsTiltrotor::add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order)
{
	add_motor(motor_num, angle_degrees, angle_degrees, yaw_factor, testing_order);
}

// add_motor using position and prop direction. Roll and Pitch factors can differ (for asymmetrical frames)
void MotorsTiltrotor::add_motor(int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order)
{
	add_motor_raw(
		motor_num,
		cosf(radians(roll_factor_in_degrees + 90)),
		cosf(radians(pitch_factor_in_degrees)),
		yaw_factor,
		testing_order);
}

// remove_motor - disabled motor and clears all roll, pitch, throttle factors for this motor
void MotorsTiltrotor::remove_motor(int8_t motor_num)
{
	// ensure valid motor number is provided
	if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS) {
		// disable the motor, set all factors to zero
		motor_enabled[motor_num] = false;
		_roll_factor[motor_num] = 0;
		_pitch_factor[motor_num] = 0;
		_yaw_factor[motor_num] = 0;
	}
}

// remove_all_motors - removes all motor definitions
void MotorsTiltrotor::remove_all_motors()
{
	for (int8_t i = 0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
		remove_motor(i);
	}
}


