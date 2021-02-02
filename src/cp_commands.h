// Copyright (c) 2016-2020, Mattia Poggiani.
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
 * \file        cp_commands.h
 *
 *  \brief      Definitions for additional commands, parameters and packages.
 *
 *  \details
 *  This file is included in the board firmware, in its libraries and
 *  applications. It contains all definitions that are necessary for the
 *  contruction of communication packages.
 *
 *  It includes definitions for all of the device commands, parameters and also
 *  the size of answer packages.
 *
**/

#ifndef ADDITIONAL_COM_COMMANDS_DEFINITIONS_H_INCLUDED
#define ADDITIONAL_COM_COMMANDS_DEFINITIONS_H_INCLUDED

#include "commands.h"

#ifdef API_VERSION
	#undef API_VERSION
	#define API_VERSION "v. 7.0 Centro Piaggio"
#endif

//==============================================================================
//                                                                      COMMANDS
//==============================================================================

#define NUM_OF_ADDITIONAL_EMGS  6       /*!< Number of additional emg channels.*/
#define NUM_OF_INPUT_EMGS		2

/** \name Additional Commands
 * \{
**/

enum additional_command
{
    CMD_GET_IMU_READINGS        = 161,  // Retrieve accelerometers, gyroscopes and magnetometers readings    
    CMD_GET_IMU_PARAM           = 162,  // Retrieve and set IMU parameters
    CMD_GET_ENCODER_CONF        = 163,  // Get encoder configuration
    CMD_GET_ENCODER_RAW         = 164,  // Get all encoder raw values
	CMD_GET_ADC_CONF			= 165,	// Get ADC configuration
    CMD_GET_ADC_RAW  			= 166,	// Get ADC raw values  
    CMD_GET_SD_SINGLE_FILE      = 167   // Get a single file of the SD card given the path
};


// ----------------------------------------------------------------------------
#endif

/* [] END OF FILE */