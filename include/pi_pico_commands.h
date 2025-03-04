#pragma once

#define CMD_CALIBRATE 0x63
#define CMD_POSITION_UPDATE 0x70
#define CMD_POSITION_REQUEST 0x72
#define CMD_ERROR 0x65
#define CMD_NULL 0x00
#define CMD_DUMMY 0xFF
#define LEFT_STEPPER_MOTOR 0x6C
#define RIGHT_STEPPER_MOTOR 0x72
#define IS_VALID_MOTOR(m) ((m) == LEFT_STEPPER_MOTOR || (m) == RIGHT_STEPPER_MOTOR)
