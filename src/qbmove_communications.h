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
 * \file        qbmove_communications.h
 *
 * \brief       Library of functions for SERIAL PORT communication with a QB Move.
 *              Function Prototypes.
 *
 *  \details
 *
 *  This library contains all necessary functions for communicating with a QB Move when
 *  using a USB to RS485 connector that provides a Virtual COM interface.
**/


#ifndef QBMOVE_SERIALPORT_H_INCLUDED
#define QBMOVE_SERIALPORT_H_INCLUDED


#if (defined(_WIN32) || defined(_WIN64))
    #include <windows.h>
#else
    #define HANDLE  int
    #define INVALID_HANDLE_VALUE    -1
#endif

#if !(defined(_WIN32) || defined(_WIN64)) && !(defined(__APPLE__)) //only for linux
    #include <termios.h>
#endif

#include "commands.h"


//==============================================================================
//                                                              structures/enums
//==============================================================================

typedef struct comm_settings comm_settings;

struct comm_settings
{
    HANDLE file_handle;
};


//==============================================================================
//                                                          function definitions
//==============================================================================

/** \name Virtual COM (RS485) functions */
/** \{ */

//===========================================================     RS485listPorts

/** This function is used to return a list of available serial ports. A maximum of 10
 *  ports are found. THIS FUNCTIONS IS NOT IMPLEMENTED FOR LINUX YET.
 *
 *  \todo Make LINUX version of this function.
 *
 *  \param  list_of_ports   An array of strings with the serial ports paths.
 *
 *  \return Returns the number of serial ports found.
 *
 *  \par Example
 *  \code

    int  i, num_ports;
    char list_of_ports[10][255];

    num_ports = RS485listPorts(ports);

    for(i = 0; i < num_ports; ++i)
    {
        puts(ports[i]);
    }

 *  \endcode
**/

int RS485listPorts( char list_of_ports[10][255] );

//================================================================     openRS485

/** This function is used to open a serial port for using with the QB Move.
 *
 *  \param port_s The string to the serial port path.
 *
 *  \return Returns the file descriptor associated to the serial port.
 *
 *  \par Example
 *  \code

    comm_settings comm_settings_t;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-FTU6OC47");
    if(comm_settings_t.file_handle == INVALID_HANDLE_VALUE)
    {
    // ERROR
    }

 *  \endcode
**/

#if !(defined(_WIN32) || defined(_WIN64)) && !(defined(__APPLE__)) //only for linux
    void openRS485( comm_settings *comm_settings_t, const char *port_s, int BAUD_RATE = B460800);
#else
    void openRS485( comm_settings *comm_settings_t, const char *port_s, int BAUD_RATE = 460800);
#endif


//===============================================================     closeRS485


/** This function is used to close a serial port being used with the QB Move.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \par Example
 *  \code

    comm_settings comm_settings_t;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-FTU6OC47");
    closeRS485(&comm_settings_t);

 *  \endcode
**/

void closeRS485( comm_settings *comm_settings_t );

/** \} */


/** \name QB Move Commands */
/** \{ */

//================================================================     RS485read

/** This function is used to read a package from the device.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id              The device's id number.
 *  \param  package         Package will be stored here.
 *
 *  \return Returns package length if communication was ok, -1 otherwise.
 *
 *  \par Example
 *  \code

     char package_in[15];		// output data buffer
     char package_out[15];		// output data buffer
     int package_out_size;
     int id = 65;
     int measurements[3];

     package_out_size = pkgGetMeasurements(id, package_out);

     RS485write(comm_settings_t, id, package_out, package_out_size);

     if (RS485read(comm_settings_t, id, package_in, 1))
         return -1;

     processReplyGetMeasurements(package_in, measurements);


 *  \endcode
**/

int RS485read(  comm_settings *comm_settings_t,
                int id,
                char *package );

//=========================================================     RS485ListDevices

int RS485ListDevices( comm_settings *comm_settings_t, char list_of_ids[255] );

//=============================================================     RS485GetInfo

/** This function is used to ping the serial port for a QB Move and
 *  get information about the device. ONLY USE WHEN ONE DEVICE IS CONNECTED
 *  ONLY.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  buffer          Buffer that stores a string with information about
 *                          the device. BUFFER SIZE MUST BE AT LEAST 500.
 *
 *
 *  \par Example
 *  \code

    comm_settings    comm_settings_t;
    char        auxstring[500];

    openRS485(&comm_settings_t,"/dev/tty.usbserial-FTU6OC47");
    RS485GetInfo(&comm_settings_t, auxstring);
    puts(auxstring);
    closeRS485(&comm_settings_t);

 *  \endcode
**/

void RS485GetInfo( comm_settings *comm_settings_t, char *buffer );

//================================================================     commPing

/** This function is used to ping the QB Move.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id              The device's id number.
 *  \param  buffer          Buffer that stores a string with information about
 *                          the device. BUFFER SIZE MUST BE AT LEAST 500.
 *
 *  \return Returns 0 if ping was ok, -1 otherwise.
 *  \par Example
 *  \code

comm_settings comm_settings_t;
int     device_id = 65;

   openRS485(&comm_settings_t,"/dev/tty.usbserial-FTU6OC47");
   if ( commPing(&comm_settings_t, device_id) )
        puts("Device exists.");
    else
        puts("Device does not exist.");

   closeRS485(&comm_settings_t);

*  \endcode


**/

int commPing( comm_settings *comm_settings_t, int id );

//=============================================================     commActivate



/** This function activates or deactivates a QB Move connected to
 *  the serial port.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id              The device's id number.
 *  \param  activate        TRUE to turn motors on.
 *                          FALSE to turn motors off.
 *  \par Example
 *  \code

    comm_settings comm_settings_t;
    int     device_id = 65;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-FTU6OC47");
    commActivate(&comm_settings_t, device_id, TRUE);
    closeRS485(&comm_settings_t);

 *  \endcode
**/

void commActivate(  comm_settings *comm_settings_t,
                    int id,
                    char activate );

//============================================================     commSetInputs

/** This function send reference inputs to a QB Move connected to the serial
 *  port.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id              The device's id number.
 *  \param  inputs          Input references.
 *
 *  \par Example
 *  \code

    comm_settings comm_settings_t;
    int     device_id = 65;
    short int inputs[2];

    openRS485(&comm_settings_t,"/dev/tty.usbserial-FTU6OC47");

    inputs[0]   = 1000;
    inputs[1]   = -1000;
    commSetInputs(&comm_settings_t, device_id, inputs);
    closeRS485(&comm_settings_t);

 *  \endcode
**/

void commSetInputs( comm_settings *comm_settings_t, int id, short int inputs[]);


//============================================================     commSetPosStiff

void commSetPosStiff(comm_settings *comm_settings_t, int id, short int inputs[]);

//============================================================     commGetInputs

/** This function gets input references from a QB Move connected to the serial
 *  port.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id              The device's id number.
 *  \param  inputs          Input references.
 *
 *  \return Returns 0 if communication was ok, -1 otherwise.
 *
 *  \par Example
 *  \code

    comm_settings comm_settings_t;
    int     device_id = 65;
    short int inputs[2];

    openRS485(&comm_settings_t,"/dev/tty.usbserial-FTU6OC47");

    if(!commGetInputs(&comm_settings_t, DEVICE_ID, inputs))
        printf("Inputs: %d\t%d\n",inputs[0], inputs[1]);
    else
        puts("Couldn't retrieve measurements.");

    closeRS485(&comm_settings_t);

 *  \endcode

**/

int commGetInputs(  comm_settings *comm_settings_t,
                    int id,
                    short int inputs[2] );

//======================================================     commGetMeasurements

/** This function gets measurements from a QB Move connected to the serial
 *  port.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id              The device's id number.
 *  \param  measurements    Measurements.
 *
 *  \return Returns 0 if communication was ok, -1 otherwise.
 *
 *  \par Example
 *  \code

    comm_settings comm_settings_t;
    int     device_id = 65;
    short int measurements[3];

    openRS485(&comm_settings_t,"/dev/tty.usbserial-FTU6OC47");

    if(!commGetMeasurements(&comm_settings_t, DEVICE_ID, measurements))
        printf("Measurements: %d\t%d\t%d\n",measurements[0], measurements[1], measurements[2]);
    else
        puts("Couldn't retrieve measurements.");

    closeRS485(&comm_settings_t);

 *  \endcode

**/

int commGetMeasurements(    comm_settings *comm_settings_t,
                            int id,
                            short int measurements[3] );


//======================================================     commGetCurrents

/** This function gets currents from a QB Move connected to the serial
*  port.
*
*  \param  comm_settings_t     A _comm_settings_ structure containing info about the
*                              communication settings.
*
*  \param  id              The device's id number.
*  \param  currents    Currents.
*
*  \return Returns 0 if communication was ok, -1 otherwise.
*
*  \par Example
*  \code

   comm_settings comm_settings_t;
   int     device_id = 65;
   short int currents[2];

   openRS485(&comm_settings_t,"/dev/tty.usbserial-FTU6OC47");

   if(!commGetMeasurements(&comm_settings_t, DEVICE_ID, currents))
       printf("Measurements: %d\t%d\t%d\n",currents[0], currents[1], currents[2]);
   else
       puts("Couldn't retrieve measurements.");

   closeRS485(&comm_settings_t);

*  \endcode

**/

int commGetCurrents(    comm_settings *comm_settings_t,
                           int id,
                           short int currents[2] );

//=======================================================     commGetCurrAndMeas

// Values is a short int array of 2 + NUM_OF_SENSORS (for the qbmove is 5)

int commGetCurrAndMeas( comm_settings *comm_settings_t,
                        int id,
                        short int *values);

//===============================================================     commGetEmg

int commGetEmg(comm_settings *comm_settings_t,
               int id,
               short int emg[2]);

//========================================================     commGetVelocities
int commGetVelocities(comm_settings *comm_settings_t, int id, short int measurements[]);


//==========================================================     commGetActivate

/** This function gets the activation status of a QB Move connected to the serial
 *  port.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id              The device's id number.
 *  \param  activation      Activation status.
 *
 *  \return Returns 0 if communication was ok, -1 otherwise.
 *
 *  \par Example
 *  \code

    comm_settings comm_settings_t;
    int     device_id = 65;
    char    activation_status;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-FTU6OC47");

    if(!commGetActivate(&comm_settings_t, DEVICE_ID, activation_status))
        printf("Activation status: %d\n", &activation_status);
    else
        puts("Couldn't retrieve measurements.");

    closeRS485(&comm_settings_t);

 *  \endcode

**/

int commGetActivate(    comm_settings *comm_settings_t,
                        int id,
                        char *activate );


//==============================================================     commGetInfo

/** This function is used to ping the QB Move and get information about the
 *  device.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id              The device's id number.
 *  \param  buffer          Buffer that stores a string with information about
 *                          the device. BUFFER SIZE MUST BE AT LEAST 500.
 *  \param  info_type       Information to be retrieved.
 *
 *  \par Example
 *  \code

    comm_settings comm_settings_t;
    char    auxstring[500];
    int     device_id = 65;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-FTU6OC47");
    commGetInfo(&comm_settings_t, device_id, INFO_ALL, auxstring);
    puts(auxstring);
    closeRS485(&comm_settings_t);

 *  \endcode
**/

int commGetInfo(    comm_settings *comm_settings_t,
                    int id,
                    short int info_type,
                    char *info );

/** \} */


/** \name QB Move Parameters */
/** \{ */


//============================================================     commSetParam

/** This function sets a parameter that remains stored in the QB Move memory.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id                  The device's id number.
 *  \param  param_type          The parameter to be set.
 *  \param  values              An array with the parameter values.
 *  \param  num_of_values       The size of the values array.
 *
 *  \par Example
 *  \code

    comm_settings comm_settings_t;
    int     device_id = 65;
    float   control_k = 0.1;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-FTU6OC47");
    commSetParam(&comm_settings_t, global_args.device_id,
        PARAM_CONTROL_K, &global_args.control_k, 1);
    closeRS485(&comm_settings_t);

 *  \endcode
**/

int commSetParam(  comm_settings *comm_settings_t,
                    int id,
                    enum qbmove_parameter type,
                    void *values,
                    unsigned short num_of_values );

//============================================================     commGetParam

/** This function gets a parameter that is stored in the QB Move memory.
*
*  \param  comm_settings_t     A _comm_settings_ structure containing info about the
*                              communication settings.
*
*  \param  id                  The device's id number.
*  \param  param_type          The parameter to be get.
*  \param  values              An array with the parameter values.
*  \param  num_of_values       The size of the values array.

*
*  \par Example
*  \code

    comm_settings comm_settings_t;
    int     device_id = 65;
    float   control_k;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-FTU6OC47");

    if( commGetParams(&comm_settings_t, device_id, PARAM_CONTROL_K,
                                           &control_k, 1) )
   {
       printf("Control constant: %f", control_k);
   }
    closeRS485(&comm_settings_t);

*  \endcode
**/

int commGetParam(comm_settings *comm_settings_t,
                    int id,
                    enum qbmove_parameter type,
                    void *values,
                    unsigned short num_of_values );

//============================================================     commStoreParams

/** This function stores all parameters that were set in the QB Move memory.
*
*  \param  comm_settings_t     A _comm_settings_ structure containing info about the
*                              communication settings.
*
*  \param  id                  The device's id number.
*
*  \par Example
*  \code

    comm_settings comm_settings_t;
    int     device_id = 65;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-FTU6OC47");

    commStoreParams(&comm_settings_t, device_id)

    closeRS485(&comm_settings_t);

*  \endcode
**/


int commStoreParams( comm_settings *comm_settings_t, int id);

// TODO
int commStoreDefaultParams( comm_settings *comm_settings_t, int id);

//==========================================================     commRestoreParams


/** This function restores the factory default parameters.
*
*  \param  comm_settings_t  A _comm_settings_ structure containing info about
*                           the communication settings.
*
*  \param  id               The device's id number.
*
*  \par Example
*  \code

    comm_settings comm_settings_t;
    int     device_id = 65;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-FTU6OC47");

    commRestoreParams(&comm_settings_t, device_id)

    closeRS485(&comm_settings_t);

*  \endcode
**/

int commRestoreParams( comm_settings *comm_settings_t, int id );

///TODO

int commInitMem(comm_settings *comm_settings_t, int id);

//==========================================================     timevaldiff

long timevaldiff (struct timeval *starttime, struct timeval *finishtime);


//=================================================================     checksum


/** This functions returns an 8 bit LCR checksum over the lenght of a buffer.
 *
 * \param   data_buffer Buffer.
 * \param   data_length Buffer length.
 *
 *  \par Example
 *  \code

    char    aux;
    char    buffer[5];

    buffer  = "abcde";
    aux     = checksum(buffer,5);
    printf("Checksum: %d", (int) aux)

 *  \endcode

**/

 /**
  * \name General Functions
  * \{
 **/

char checksum ( char * data_buffer, int data_length );

/** \} */


int commBootloader(comm_settings *comm_settings_t, int id);

int commCalibrate(comm_settings *comm_settings_t, int id);

int commHandCalibrate(comm_settings *comm_settings_t, int id, short int speed, short int repetitions);

// ----------------------------------------------------------------------------
#endif

/* [] END OF FILE */