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
 *  This file is included in the QB Move firmware, in its libraries and 
 *  applications. It contains all definitions that are necessary for the 
 *  contruction of communication packages.
 *  
 *  It includes definitions for all of the device commands, parameters and also 
 *  the size of answer packages.
 *
**/

#ifndef COM_COMMANDS_DEFINITIONS_H_INCLUDED
#define COM_COMMANDS_DEFINITIONS_H_INCLUDED

//==============================================================================
//                                                                      COMMANDS
//==============================================================================


/** \name QB Move Commands
 * \{
**/

enum qbmove_command
{

//=========================================================     general commands

    CMD_PING    		    = 0,    ///< Asks for a ping message
    CMD_SET_PARAM		    = 1,    ///< Command for setting a parameter to be 
                                    ///  stored in the device memory
    CMD_GET_PARAM		    = 2,    ///< Command for getting stored parameters    
    CMD_STORE_PARAMS        = 3,    ///< Stores all parameters in memory and 
                                    ///  loads them
    CMD_RESTORE_PARAMS      = 4,    ///< Restore default factory parameters
    CMD_GET_INFO		    = 5,    ///< Asks for a string of information about
    CMD_SET_VALUE           = 6,
    CMD_GET_VALUE           = 7,


//=========================================================     QB Move commands

    CMD_ACTIVATE		    = 128,  ///< Command for activating/deactivating 
                                    ///  the device
    CMD_GET_ACTIVATE	    = 129,  ///< Command for getting device activation
                                    ///  state
    CMD_SET_INPUTS		    = 130,  ///< Command for setting reference inputs
    CMD_GET_INPUTS		    = 131,  ///< Command for getting reference inputs
    CMD_GET_MEASUREMENTS    = 132,  ///< Command for asking device's
    CMD_GET_CURRENTS    	= 133,  ///< Command for asking device's

                                    ///  measurements


};

/** \} */
//==============================================================================
//                                                                    PARAMETERS
//==============================================================================
/** \name QB Move Parameters */
/** \{ */

enum qbmove_parameter
{
    //////////////////////////   implemented   //////////////////////////
    
    PARAM_ID		    	    = 0,    ///< Device's ID number
    PARAM_CONTROL_K		        = 1,    ///< PID Control proportional constant
    PARAM_STARTUP_ACTIVATION    = 2,    ///< Start up activation byte
    PARAM_INPUT_MODE            = 3,    ///< Input mode

    /////////////////////////   not implemented   ////////////////////////
    
    PARAM_POS_RESOLUTION        = 4,    ///< Angle resolution for inputs and
                                        ///  measurements. Used during
                                        ///  communication.
    PARAM_POS_MULTIPLIER        = 5,    ///< Multiplier constant that amplifies
                                        ///  input references. Useful when using
                                        ///  a joystick
    PARAM_POS_OFFSET            = 6,    ///< Adds a constant offset to the input
                                        ///  references. Useful when using a 
                                        ///  joystick.
    PARAM_MEAS_FILTER           = 7,    ///< Measurement filter, should be from
                                        ///  0 to 1. 32 bit float number.
    PARAM_CONTROL_DEADZONE      = 8,    ///< Applies a deadzone to the 
                                        ///  controller
    PARAM_MEASUREMENT_OFFSET    = 9,    ///< Adds a constant offset to the
                                        ///  measurements
    PARAM_MEASUREMENT_MULTIPLIER = 10   ///< Adds a multiplier to the 
                                        ///  measurements                                    
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

enum qbmove_mode
{
    INPUT_MODE_EXTERNAL = 0,        ///< References through external
                                    ///  commands (default)
    INPUT_MODE_ENCODER3 = 1         ///< Encoder 3 drives all inputs
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