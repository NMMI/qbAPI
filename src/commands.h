// Copyright (c) 2012, qbrobotics.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// - Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * \file        commands.h
 *
 *  \brief      Definitions for QB Move commands, parameters and packages.
 *
 *  \details
 *  This file is included in the qbMove firmware, in its libraries and
 *  applications. It contains all definitions that are necessary for the
 *  contruction of communication packages.
 *
 *  It includes definitions for all of the device commands, parameters and also
 *  the size of answer packages.
 *
**/

#ifndef COM_COMMANDS_DEFINITIONS_H_INCLUDED
#define COM_COMMANDS_DEFINITIONS_H_INCLUDED

#define NUM_OF_SENSORS 3
#define API_VERSION "v5.5.0"

//==============================================================================
//                                                                      COMMANDS
//==============================================================================


/** \name qbMove Commands
 * \{
**/

enum qbmove_command
{

//=========================================================     general commands

    CMD_PING                    = 0,    ///< Asks for a ping message
    CMD_SET_PARAM               = 1,    ///< Command for setting a parameter to be
                                        ///  stored in the device memory
    CMD_GET_PARAM               = 2,    ///< Command for getting stored parameters
    CMD_STORE_PARAMS            = 3,    ///< Stores all parameters in memory and
                                        ///  loads them
    CMD_STORE_DEFAULT_PARAMS    = 4,    ///< Store current parameters as factory parameters
    CMD_RESTORE_PARAMS          = 5,    ///< Restore default factory parameters
    CMD_GET_INFO                = 6,    ///< Asks for a string of information about

    CMD_SET_VALUE               = 7,    ///< Not used
    CMD_GET_VALUE               = 8,    ///< Not used

    CMD_BOOTLOADER              = 9,    ///< Sets the bootloader modality to update the
                                        ///  firmware
    CMD_INIT_MEM                = 10,   ///< Initialize the memory with the defalut values

    CMD_CALIBRATE               = 11,   ///< Starts the stiffness calibration of the qbMove
                                        ///  or the hand closure and opening calibration

//=========================================================     QB Move commands

    CMD_ACTIVATE            = 128,  ///< Command for activating/deactivating
                                    ///  the device
    CMD_GET_ACTIVATE        = 129,  ///< Command for getting device activation
                                    ///  state
    CMD_SET_INPUTS          = 130,  ///< Command for setting reference inputs
    CMD_GET_INPUTS          = 131,  ///< Command for getting reference inputs
    CMD_GET_MEASUREMENTS    = 132,  ///< Command for asking device's
                                    ///  position measurements
    CMD_GET_CURRENTS        = 133,  ///< Command for asking device's
                                    ///  current measurements
    CMD_GET_CURR_AND_MEAS   = 134,  ///< Command for asking device's
                                    ///  measurements and currents
    CMD_SET_POS_STIFF       = 135,  ///< Command for setting stiffness and position
                                    ///  of the qbMove's shaft
    CMD_GET_EMG             = 136,  ///< Command for getting the emg sensors measurements

    CMD_GET_VELOCITIES      = 137,  ///< Command for asking device's
                                    ///  current velocity of motors and pulley
    CMD_GET_COUNTERS        = 138
};

/** \} */
//==============================================================================
//                                                                    PARAMETERS
//==============================================================================
/** \name qbMove Parameters */
/** \{ */

enum qbmove_parameter
{

    PARAM_ID                     = 0,   ///< Device's ID number
    PARAM_PID_CONTROL            = 1,   ///< PID Control proportional constant
    PARAM_STARTUP_ACTIVATION     = 2,   ///< Start up activation byte
    PARAM_INPUT_MODE             = 3,   ///< Input mode

    PARAM_CONTROL_MODE           = 4,   ///< Choose the kind of control between
                                        ///  position control, current control
                                        ///  or direct PWM value input
    PARAM_MEASUREMENT_OFFSET     = 5,   ///< Adds a constant offset to the
                                        ///  measurements
    PARAM_MEASUREMENT_MULTIPLIER = 6,   ///< Adds a multiplier to the
                                        ///  measurements
    PARAM_POS_LIMIT_FLAG         = 7,   ///< Enable/disable position limiting
    PARAM_POS_LIMIT              = 8,   ///< Position limit values
                                        ///  [ int32     | int32     | int32     | int32     ]
                                        ///  [ INF_LIM_1 | SUP_LIM_1 | INF_LIM_2 | SUP_LIM_2 ]

    PARAM_MAX_STEP_POS           = 9,   ///< Used to slow down movements for positive values
    PARAM_MAX_STEP_NEG           = 10,  ///< Used to slow down movements for negative values
    PARAM_POS_RESOLUTION         = 11,  ///< Angle resolution for inputs and
                                        ///  measurements. Used during
                                        ///  communication.
    PARAM_CURRENT_LIMIT          = 12,  ///< Limit for absorbed current
    PARAM_EMG_CALIB_FLAG         = 13,  ///< Enable calibration on startup
    PARAM_EMG_THRESHOLD          = 14,  ///< Minimum value to have effect
    PARAM_EMG_MAX_VALUE          = 15,  ///< Maximum value of EMG
    PARAM_EMG_SPEED              = 16,  ///< Closure speed when using EMG
    PARAM_SC_BAND                = 17,  ///< Short-Circuit band
    PARAM_PID_CURR_CONTROL       = 18,  ///< PID current control
    PARAM_DOUBLE_ENC_ON_OFF      = 19,  ///< Double Encoder Y/N
    PARAM_MOT_HANDLE_RATIO       = 20,  ///< Multiplier between handle and motor
    PARAM_MOTOR_SUPPLY           = 21,  ///< Motor supply voltage of the hand
    PARAM_DEFLECTION_CONTROL     = 22

};


//===================================================     resolution definitions

enum qbmove_resolution
{
    RESOLUTION_360      = 0,
    RESOLUTION_720      = 1,
    RESOLUTION_1440     = 2,
    RESOLUTION_2880     = 3,
    RESOLUTION_5760     = 4,
    RESOLUTION_11520    = 5,
    RESOLUTION_23040    = 6,
    RESOLUTION_46080    = 7,
    RESOLUTION_92160    = 8
};

//==============================================================     input modes

enum qbmove_input_mode
{
    INPUT_MODE_EXTERNAL = 0,            ///< References through external
                                        ///  commands (default)
    INPUT_MODE_ENCODER3 = 1,            ///< Encoder 3 drives all inputs

    INPUT_MODE_EMG_PROPORTIONAL = 2,    ///< Use EMG measure to proportionally
                                        ///  drive the position of the motor 1
    INPUT_MODE_EMG_INTEGRAL     = 3,    ///< Use 2 EMG signals to drive motor
                                        ///  position
    INPUT_MODE_EMG_FCFS         = 4,    ///< Use 2 EMG. First reaching threshold
                                        ///  wins and its value defines hand closure
    INPUT_MODE_EMG_FCFS_ADV     = 5     ///< Use 2 EMG. First reaching threshold
                                        ///  wins and its value defines hand closure
                                        ///  Wait for both EMG to lower under threshold
};

//============================================================     control modes

enum qbmove_control_mode
{
    CONTROL_ANGLE           = 0,        ///< Classic position control
    CONTROL_PWM             = 1,        ///< Direct PWM value
    CONTROL_CURRENT         = 2,        ///< Current control (beta)
    CURR_AND_POS_CONTROL    = 3         ///< Current control (beta)
};

//============================================================     supply types

enum motor_supply_tipe {
    MAXON_24V               = 0,
    MAXON_12V               = 1
};

//====================================================     acknowledgment values
enum acknowledgment_values
{
    ACK_ERROR           = 0,
    ACK_OK              = 1
};


/** \} */

//==============================================================================
//                                                                   INFORMATION
//==============================================================================
/** \name QB Move Information Strings */
/** \{ */
#define INFO_ALL        0 ///< All system information.

/** \} */

// ----------------------------------------------------------------------------
#endif

/* [] END OF FILE */