/**
* @Author: sxf
* @Date:   2015-01-22 15:56:50
* @Last Modified by:   ERROR: ld.so: object './sublime_text_fcitx.so' from LD_PRELOAD cannot be preloaded (cannot open shared object file): ignored.
sxf
* @Last Modified time: 2015-01-24 23:50:59
* @file MotorsTiltrotor.h
* @brief 这是倾转旋翼机的专用发动机类
*/

#ifndef MOTORS_TILTROTOR_H
#define MOTORS_TILTROTOR_H

#include <AP_Common.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include "AP_Motors.h"    // Parent Motors Matrix library

// 定义舵机
#define MOTORS_CH_AILERON    CH_5 // 副翼 aileron
#define MOTORS_CH_TAIL		 CH_6 // 水平尾
#define MOTORS_CH_YAW	     CH_7 // 偏航
#define MOTORS_CH_TILTROTOR	 CH_8 // 倾转舵机

/// @class      MotorsTiltrotor
/// @brief	这是一个专用的发动机控制类
class MotorsTiltrotor : public AP_Motors {
public:

    /// Constructor
	MotorsTiltrotor(
		RC_Channel& rc_roll, 
		RC_Channel& rc_pitch,
		RC_Channel& rc_throttle,
		RC_Channel& rc_yaw, 
		RC_Channel& rc_tiltrotor,
		uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT
	):
		AP_Motors(rc_roll, rc_pitch, rc_throttle, rc_yaw, speed_hz),
		_rc_tiltrotor(rc_tiltrotor) 
	{};

	// init
	virtual void        Init();

	// set update rate to motors - a value in hertz
	// you must have setup_motors before calling this
	virtual void        set_update_rate(uint16_t speed_hz);

	// set frame orientation (normally + or X)
	virtual void        set_frame_orientation(uint8_t new_orientation);

	// enable - starts allowing signals to be sent to motors
	virtual void        enable();

	// output_test - spin a motor at the pwm value specified
	//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
	//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
	virtual void        output_test(uint8_t motor_seq, int16_t pwm);

	// output_min - sends minimum values out to the motors
	virtual void        output_min();

	// add_motor using just position and yaw_factor (or prop direction)
	void                add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order);

	// add_motor using separate roll and pitch factors (for asymmetrical frames) and prop direction
	void                add_motor(int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order);

	// remove_motor
	void                remove_motor(int8_t motor_num);

	// remove_all_motors - removes all motor definitions
	void                remove_all_motors();

	// setup_motors - configures the motors for a given frame type - should be overwritten by child classes
	virtual void        setup_motors();

	// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
	//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
	virtual uint16_t    get_motor_mask();

protected:
	// output - sends commands to the motors
	virtual void        output_armed();
	virtual void        output_disarmed();

	// add_motor using raw roll, pitch, throttle and yaw factors
	void                add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order);

	float               _roll_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to roll
	float               _pitch_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to pitch
	float               _yaw_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to yaw (normally 1 or -1)
	uint8_t             _test_order[AP_MOTORS_MAX_NUM_MOTORS];  // order of the motors in the test sequence
    
//	RC_Channel&			_rc_aileron;	// 副翼的通道
//	RC_Channel&         _rc_tail;       // REV parameter used from this channel to determine direction of tail servo movement
//	RC_Channel&			_rc_yaw;		// 偏航
	RC_Channel&			_rc_tiltrotor;  // 倾转

};




#endif // MOTORS_TILTROTOR_H


