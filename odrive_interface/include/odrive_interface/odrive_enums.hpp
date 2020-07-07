#ifndef ODRIVE_ENUMS_HPP_
#define ODRIVE_ENUMS_HPP_

#define AXIS_STATE_UNDEFINED                  0
#define AXIS_STATE_IDLE                       1
#define AXIS_STATE_STARTUP_SEQUENCE           2
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE  3
#define AXIS_STATE_MOTOR_CALIBRATION          4
#define AXIS_STATE_SENSORLESS_CONTROL         5
#define AXIS_STATE_ENCODER_INDEX_SEARCH       6
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION 7
#define AXIS_STATE_CLOSED_LOOP_CONTROL        8
#define AXIS_STATE_LOCKIN_SPIN                9
#define AXIS_STATE_ENCODER_DIR_FIND           10


#define ERROR_NONE  0x00

//    class axis:
#define ERROR_INVALID_STATE               0x01 //<! an invalid state was requested
#define ERROR_DC_BUS_UNDER_VOLTAGE        0x02
#define ERROR_DC_BUS_OVER_VOLTAGE         0x04
#define ERROR_CURRENT_MEASUREMENT_TIMEOUT 0x08
#define ERROR_BRAKE_RESISTOR_DISARMED     0x10 //<! the brake resistor was unexpectedly disarmed
#define ERROR_MOTOR_DISARMED              0x20 //<! the motor was unexpectedly disarmed
#define ERROR_MOTOR_FAILED                0x40 
#define ERROR_SENSORLESS_ESTIMATOR_FAILED 0x80
#define ERROR_ENCODER_FAILED              0x100 
#define ERROR_CONTROLLER_FAILED           0x200
#define ERROR_POS_CTRL_DURING_SENSORLESS  0x400
#define ERROR_WATCHDOG_TIMER_EXPIRED      0x800

//    class motor:
#define ERROR_PHASE_RESISTANCE_OUT_OF_RANGE 0x0001
#define ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE 0x0002
#define ERROR_ADC_FAILED                    0x0004
#define ERROR_DRV_FAULT                     0x0008
#define ERROR_CONTROL_DEADLINE_MISSED       0x0010
#define ERROR_NOT_IMPLEMENTED_MOTOR_TYPE    0x0020
#define ERROR_BRAKE_CURRENT_OUT_OF_RANGE    0x0040
#define ERROR_MODULATION_MAGNITUDE          0x0080
#define ERROR_BRAKE_DEADTIME_VIOLATION      0x0100
#define ERROR_UNEXPECTED_TIMER_CALLBACK     0x0200
#define ERROR_CURRENT_SENSE_SATURATION      0x0400
#define ERROR_CURRENT_UNSTABLE              0x1000

//    class encoder:
#define ERROR_UNSTABLE_GAIN            0x01
#define ERROR_CPR_OUT_OF_RANGE         0x02
#define ERROR_NO_RESPONSE              0x04
#define ERROR_UNSUPPORTED_ENCODER_MODE 0x08
#define ERROR_ILLEGAL_HALL_STATE       0x10
#define ERROR_INDEX_NOT_FOUND_YET      0x20

//    class controller:
#define ERROR_OVERSPEED 0x01

#define MOTOR_TYPE_HIGH_CURRENT 0
#define MOTOR_TYPE_LOW_CURRENT  1
#define MOTOR_TYPE_GIMBAL       2

#define CTRL_MODE_VOLTAGE_CONTROL    0
#define CTRL_MODE_CURRENT_CONTROL    1
#define CTRL_MODE_VELOCITY_CONTROL   2
#define CTRL_MODE_POSITION_CONTROL   3
#define CTRL_MODE_TRAJECTORY_CONTROL 4

#define ENCODER_MODE_INCREMENTAL 0
#define ENCODER_MODE_HALL        1

#endif
