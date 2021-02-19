
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
 * \file        cp_communications.h
 *
 * \brief       Library of functions for SERIAL PORT communication additional to standard qbmove_communications.
 *              Function Prototypes.
 *
 * \details
 *
 *  This library contains all necessary functions for communicating with a board when
 *  using a USB to RS485 connector that provides a Virtual COM interface.
**/

 /**
* \mainpage     qbAPI Libraries
*
* \brief        Those functions allows to use the board through a serial port
*
* \version      7.1
*
* \author       Mattia Poggiani
*
* \date         February 19th, 2021
*
* \details      This is a set of functions that allows to use the board  
*               via a serial port.
*/

#ifndef CP_SERIALPORT_H_INCLUDED
#define CP_SERIALPORT_H_INCLUDED

#include "qbmove_communications.h"

			   
//===========================================================  commGetImuReadings

/** This function gets IMU readings from IMU board connected to the serial port.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id              The device's id number.
 *  \param  imu_table      	IMU table configuration.
 *  \param  imus_magcal   	IMU magnetometer calibratino parameters vector.
 *  \param  n_imu      		Number of connected IMUs.
 *
 *  \return imu_values 		Vector of imu readings.

**/

int commGetImuReadings(comm_settings *comm_settings_t,
               int id, uint8_t* imu_table, uint8_t* imus_magcal, int n_imu, float* imu_values);
			   

//============================================================     commGetIMUParamList

/** This function gets all the parameters that are stored in the board memory and sets
    one of them if requested
*
*  \param  comm_settings_t     A _comm_settings_ structure containing info about the
*                              communication settings.
*
*  \param  id                  The device's id number.
*  \param  index               The index relative to the parameter to be get.
*  \param  values              An array with the parameter values.
*  \param  value_size          The byte size of the parameter to be get
*  \param  num_of_values       The size of the array of the parameter to be get
*  \param  buffer              The array where the parameters' values and descriptions are saved

*
*  \par Example
*  \code

    comm_settings   comm_settings_t;
    int             device_id = 65;
    unsigned char   aux_string[2000];
    int             index = 0;
    int             value_size = 0;
    int             num_of_values = 0;

    // Get parameters
    commGetIMUParamList(&comm_settings_t, device_id, index, NULL, value_size, num_of_values, aux_string);
    string_unpacking_and_printing(aux_string);

    // Set parameters

    float           val[5];
    val[0] = 1;
    val[1] = 1;
    val[2] = 0;
	val[3] = 0;
	val[4] = 0;
    index = 2;
    value_size = 6;
    num_of_values = 5;
    commGetIMUParamList(&comm_settings_t, device_id, index, pid, value_size, num_of_values, NULL);

*  \endcode
**/

int commGetIMUParamList(comm_settings *comm_settings_t, int id, unsigned short index,
                    void *values, unsigned short value_size, unsigned short num_of_values, 
                    uint8_t *buffer);

					
//===========================================================  commGetEncoderConf

/** This function gets encoder map from board connected to the serial port.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id              	The device's id number.
 *  \param  num_enc_line     	Number of encoder lines.
 *  \param  num_enc_per_line    Number of encoder per line.
 *
 *  \return encoder_map		Vector of encoder map.

**/

int commGetEncoderConf(comm_settings *comm_settings_t, int id, uint8_t* num_enc_line, uint8_t* num_enc_per_line, uint8_t* enc_map);

//===========================================================  commGetEncoderRawValues

/** This function gets encoder raw readings from board connected to the serial port.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id              The device's id number.
 *
 *  \return encoder_val 	Vector of encoder map.

**/

int commGetEncoderRawValues(comm_settings *comm_settings_t, int id, uint8_t enc_total, uint16_t enc_val[]);

//===========================================================  commGetADCConf

/** This function gets ADC map from board connected to the serial port.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id              The device's id number.
 *
 *  \return num_ch			Number of channels.
 *  \return adc_map			Vector of adc map.

**/

int commGetADCConf(comm_settings *comm_settings_t, int id, uint8_t* num_ch, uint8_t* adc_map);		

//======================================================     commGetADCRaw

/** This function gets position measurements from a board connected to the serial
 *  port.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id                  The device's id number.
 *  \param  num_channels        Number of channels.
 *  \param  adc        			ADC raw values.
 *
 *  \return Returns 0 if communication was ok, -1 otherwise.
 *
 *  \par Example
 *  \code

    comm_settings   comm_settings_t;
    int             device_id = 65;
    short int       adc[3];

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

    if(!commGetADCRaw(&comm_settings_t, DEVICE_ID, adc))
        printf("ADC raw: %d\t%d\t%d\n",adc[0], adc[1], adc[2]);
    else
        puts("Couldn't retrieve measurements.");

    closeRS485(&comm_settings_t);

 *  \endcode

**/

int commGetADCRawValues( comm_settings *comm_settings_t, int id, uint8_t num_channels, short int adc[3] );

//======================================================     commGetSDFile

/** This function gets SD card files.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id                  The device's id number.
 *  \param  filename            The name of the file to retrieve.
 *  \param  buffer              The returned file.
 *
 *  \return Returns 0 if communication was ok, -1 otherwise.
 *
 *  \par Example
 *  \code

    comm_settings   comm_settings_t;
    int             device_id = 65;
    char            filename[19] = "USR01\2020\01\31\Param0.csv";
    char            buffer[10000] = "";

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

    if(commGetSDFile(&comm_settings_t, DEVICE_ID, filename, buffer))
        puts("Couldn't retrieve SD filesystem.");

    closeRS485(&comm_settings_t);

 *  \endcode

**/

int commGetSDFile(comm_settings *comm_settings_t, int id, char* filename, char *buffer);
	   
// ----------------------------------------------------------------------------
#endif

/* [] END OF FILE */
