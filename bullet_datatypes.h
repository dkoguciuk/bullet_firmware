/**
 *  @file		bullet_datatypes.h
 *  @author     Daniel Koguciuk <daniel.koguciuk@gmail.com>
 *  @date       04.09.2016r.
 *  @version    0.1
 *
 *  @brief		Structures definitions for both firmware and software of the Bullet robot.
 */


#ifndef BULLET_DATATYPES_H
#define BULLET_DATATYPES_H


#include <stdint.h>                 // ints with 8 & 16 bits

#define BULLET_BROADCAST_ID               0xFE

typedef enum BulletSPICommand
{
    BULLET_COMMAND_PING			                = 0x01,
    BULLET_COMMAND_READ			                = 0x02,
    BULLET_COMMAND_WRITE		                = 0x03,
    BULLET_COMMAND_READ_ALL		                = 0x04,
    BULLET_COMMAND_WRITE_ALL	                = 0x05,
    BULLET_COMMAND_CHANGE_MODE                  = 0xFF
} BulletSPICommand;

typedef enum BulletPingInstruction
{
    BULLET_INSTRUCTION_COMMUNICATION            = 0x00,
    BULLET_INSTRUCTION_SERVO                    = 0x01,
    BULLET_INSTRUCTION_SERVOS_ALL               = 0x02
} BulletPingInstruction;

typedef enum BulletReadWriteInstruction
{
    BULLET_INSTRUCTION_MODEL_NUMBER             = 0x00,
    BULLET_INSTRUCTION_FIRMWARE_VERSION         = 0x02,
    BULLET_INSTRUCTION_ID                       = 0x03,
    BULLET_INSTRUCTION_BAUDRATE                 = 0x04,
    BULLET_INSTRUCTION_RETURN_DELAY_TIME        = 0x05,
    BULLET_INSTRUCTION_CW_ANGLE_LIMIT           = 0x06,
    BULLET_INSTRUCTION_CCW_ANGLE_LIMIT          = 0x08,
    BULLET_INSTRUCTION_HIGHEST_LIMIT_TEMPERATURE= 0x0B,
    BULLET_INSTRUCTION_LOWEST_LIMIT_VOLTAGE     = 0x0C,
    BULLET_INSTRUCTION_HIGHEST_LIMIT_VOLTAGE    = 0x0D,
    BULLET_INSTRUCTION_MAX_TORQUE               = 0x0E,
    BULLET_INSTRUCTION_STATUS_RETURN_LEVEL      = 0x10,
    BULLET_INSTRUCTION_ALARM_LED                = 0x11,
    BULLET_INSTRUCTION_ALARM_SHUTDOWN           = 0x12,
    BULLET_INSTRUCTION_TORQUE_ENABLE            = 0x18,
    BULLET_INSTRUCTION_LED                      = 0x19,
    BULLET_INSTRUCTION_CW_COMPLIANCE_MARGIN     = 0x1A,
    BULLET_INSTRUCTION_CCW_COMPLIANCE_MARGIN    = 0x1B,
    BULLET_INSTRUCTION_CW_COMPLIANCE_SLOPE      = 0x1C,
    BULLET_INSTRUCTION_CCW_COMPLIANCE_SLOPE     = 0x1D,
    BULLET_INSTRUCTION_GOAL_POSITION            = 0x1E,
    BULLET_INSTRUCTION_MOVING_SPEED             = 0x20,
    BULLET_INSTRUCTION_TORQUE_LIMIT             = 0x22,
    BULLET_INSTRUCTION_PRESENT_POSITION         = 0x24,
    BULLET_INSTRUCTION_PRESENT_SPEED            = 0x26,
    BULLET_INSTRUCTION_PRESENT_LOAD             = 0x28,
    BULLET_INSTRUCTION_PRESENT_VOLTAGE          = 0x2A,
    BULLET_INSTRUCTION_PRESENT_TEMPERATURE      = 0x2B,
    BULLET_INSTRUCTION_REGISTERED_INSTRUCTION   = 0x2C,
    BULLET_INSTRUCTION_MOVING                   = 0x2E,
    BULLET_INSTRUCTION_LOCK                     = 0x2F,
    BULLET_INSTRUCTION_PUNCH                    = 0x30
} BulletReadWriteInstruction;

typedef enum BulletStatus
{
    BULLET_SUCCESS 				= 0,
    BULLET_ERROR_UNKNOWN 		= 1,
    BULLET_ERROR_TIMEOUT		= 2,
    BULLET_ERROR_SPI_CRC		= 3,
    BULLET_ERROR_MODE_MISMATCH	= 4,

    BULLET_ERROR_IOCTL          = 100,
    BULLET_ERROR_SPI_RCR_IN     = 101,

    BULLET_ASKING				= 0xFF
} BulletStatus;


typedef enum BulletSTMMode
{
    BULLET_MODE_INITIALIZING	= 0,
    BULLET_MODE_PREOPERATIONAL	= 1,
    BULLET_MODE_OPERATIONAL		= 2,
    BULLET_MODE_STOPPED			= 3
} BulletSTMMode;


#endif /* BULLET_DATATYPES_H */
