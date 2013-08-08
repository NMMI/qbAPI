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
 *  \public
 *  \file       qbmove_packages.h
 *
 *  \brief      Library of low-level functions for communicating with 
 *              a QB Move. Function prototypes.
 *
 *  \details
 *
 *  This library contains all necessary functions for preparing the communication packages
 *  that are used for interfacing with the device.
 *  These packages are __independent of the communication protocol__ used (like RS485 for 
 *  example). The library also contains functions for processing the packages received from
 *  the device.
 *
*/

#ifndef QBMOVEAPI_H_INCLUDED
#define QBMOVEAPI_H_INCLUDED

//==============================================================================
//                                                          function definitions
//==============================================================================


/**
 * \name Preparing Packages
 *  Functions for preparing the communication packages
 *  that are used for interfacing with the device.
 * \{
**/

//==============================================================     pkgActivate

/** This function prepares the package that activates or deactivates a QB 
 *  Move motors.
 * 
 *  \param  id              The device's id number.
 *  \param  activate        TRUE to turn motors on.
 *                          FALSE to turn motors off.
 *  \param  pkg_buffer      Package will be stored in this buffer. Be careful
 *                          that the buffer size is big enough for storing 
 *                          the package. 
 *
 *  \return Returns package size.
 *
 *  \par Example
 *  \code

    int device_id = 65;
    char package[15];
    int n_bytes;
    
    n_bytes = pkgActivate(device_id, TRUE, package);

 *  \endcode   
**/

int pkgActivate(int id, char activate, char pkg_buffer[]);

/** This function prepares the package that asks for the activation status.
 * 
 *  \param  id              The device's id number.
 *  \param  pkg_buffer      Package will be stored in this buffer. Be careful
 *                          that the buffer size is big enough for storing 
 *                          the package. 
 *
 *  \return Returns package size.
 *
 *  \par Example
 *  \code

    int device_id = 65;
    char package[15];
    int n_bytes;
        
    n_bytes = pkgGetActivate(device_id, package);

 *  \endcode   
**/

int pkgGetActivate(int id, char pkg_buffer[]);

//=============================================================     pkgSetInputs

/** This function prepares the package that sets reference inputs to a QB Move.
 * 
 *  \param  id              The device's id number.
 *  \param  reference_a     Reference A.
 *  \param  reference_b     Reference B.
 *  \param  pkg_buffer      Package will be stored in this buffer. Be careful
 *                          that the buffer size is big enough for storing 
 *                          the package.
 *  \return Returns package size. 
 *
 *  \par Example
 *  \code

    int device_id = 65;
    short int inputs;
    char package[15];
    int n_bytes;
        
    inputs[0] = 10000;
    inputs[1] = -10000;
    n_bytes = pkgSetInputs(device_id, inputs, package);
 *  \endcode 
**/

int pkgSetInputs(int id, short int inputs[2], char pkg_buffer[]);


/** This function prepares the package that asks for the reference inputs of a QB Move.
 * 
 *  \param  id              The device's id number.
 *  \param  pkg_buffer      Package will be stored in this buffer. Be careful
 *                          that the buffer size is big enough for storing 
 *                          the package. 
 *
 *  \return Returns package size.
 *
 *  \par Example
 *  \code

    int device_id = 65;
    char package[15];
    int n_bytes;
        
    n_bytes = pkgGetInputs(device_id, package);

 *  \endcode   
**/


int pkgGetInputs(int id, char pkg_buffer[]);

//=======================================================     pkgGetMeasurements

/** This function prepares the package that ask measurements from a QB Move.
 * 
 *  \param  id              The device's id number.
 *  \param  pkg_buffer      Package will be stored in this buffer. Be careful
 *                          that the buffer size is big enough for storing 
 *                          the package.
 *
 *  \return Returns package size. 
 *  \par Example
 *  \code

    int device_id = 65;
    char package[15];
    int n_bytes;
    
    n_bytes = pkgGetMeasurements(device_id, char package);

 *  \endcode    
**/

int pkgGetMeasurements(int id, char pkg_buffer[]);

//==================================================================     pkgGetInfo

/** This function prepares the package that is used to 
 *  get information about the device. 
 *
 *  \param  id              The device's id number.
 *  \param  pkg_buffer      Package will be stored in this buffer. Be careful
 *                          that the buffer size is big enough for storing 
 *                          the package.
 *  \param  info_type       Information to be retrieved.
 *  \param  page            Indicates the page of the information string to be retrieved.
 *
 *  \par Example
 *  \code

    int device_id = 65;
    char package[500];
    int n_bytes;
        
    n_bytes = pkgGetInfo(device_id, package, 0);

 *  \endcode
*/


int pkgGetInfo(int id,  unsigned short int info_type, unsigned char page, char pkg_buffer[]);

//==================================================================     pkgSetParam

/** This function prepares the package that is used to 
 *  set a parameter stored in the device's memory.
 *
 *  \param  id              The device's id number.
 *  \param  param_type      Parameter to be set.
 *  \param  values          The parameter value.
 *  \param  value_size      The parameter value size (in bytes).
 *  \param  num_of_values   The size of the values array. 
 *  \param  pkg_buffer      Package will be stored in this buffer. Be careful
 *                          that the buffer size is big enough for storing 
 *                          the package.
 *
 *  \par Example
 *  \code

    int device_id = 65;
    unsigned char new_id = 10;
    char package[20];
    int n_bytes;
        
    n_bytes = pkgSetParam(device_id, PARAM_CONTROL_K, &new_id, sizeof(new_id), package);

 *  \endcode
*/

int pkgSetParam(    int id,
                    unsigned short int param_type,
                    void * values,
                    unsigned short int value_size,
                    unsigned short num_of_values,            
                    char pkg_buffer[]);

//==================================================================     pkgGetParam

/** This function prepares the package that is used to 
 *  get a parameter stored in the device's memory.
 *
 *  \param  id              The device's id number.
 *  \param  param_type      Parameter to be retrieved.
 *  \param  pkg_buffer      Package will be stored in this buffer. Be careful
 *                          that the buffer size is big enough for storing 
 *                          the package.
 *
 *  \par Example
 *  \code

    int device_id = 65;
    char package[20];
    int n_bytes;
        
    n_bytes = pkgGetParam(device_id, PARAM_CONTROL_K, package);

 *  \endcode
*/


int pkgGetParam(int id, unsigned short int param_type, char pkg_buffer[]);


//===========================================================     pkgStoreParams

/** This function prepares the package that is used to 
 *  store all parameter the were set in the device's memory.
 *
 *  \param  id              The device's id number.
 *  \param  pkg_buffer      Package will be stored in this buffer. Be careful
 *                          that the buffer size is big enough for storing 
 *                          the package.
 *
 *  \par Example
 *  \code

    int device_id = 65;
    char package[20];
    int n_bytes;
        
    n_bytes = pkgStoreParams(device_id, package);

 *  \endcode
*/

int pkgStoreParams(int id, char pkg_buffer[]);

//=========================================================     pkgRestoreParams

/** This function prepares the package that is used to 
 *  restores the factory default parameters.
 *
 *  \param  id              The device's id number.
 *  \param  pkg_buffer      Package will be stored in this buffer. Be careful
 *                          that the buffer size is big enough for storing 
 *                          the package.
 *
 *  \par Example
 *  \code

    int device_id = 65;
    char package[20];
    int n_bytes;
        
    n_bytes = pkgRestoreParams(device_id, package);

 *  \endcode
*/

int pkgRestoreParams(int id, char pkg_buffer[]);

//==================================================================     pkgPing

/** This function prepares the package that is used to ping the QB Move and 
 *  get information about the device. 
 *
 *  \param  id              The device's id number.
 *  \param  pkg_buffer      Package will be stored in this buffer. Be careful
 *                          that the buffer size is big enough for storing 
 *                          the package.
 *
 *  \par Example
 *  \code

    int device_id = 65;
    char package[15];
    int n_bytes;
        
    n_bytes = pkgPing(device_id, package);

 *  \endcode
*/

int pkgPing(int id, char pkg_buffer[]);


/** \} */

/**
 * \name Processing Packages
 *  Functions for processing the packages received from the device.
 * \{
**/

//=================================================     processReplyGetMeasurements

/** This function interprets the package with measurements from a QB Move.
 * 
 *  \param  pkg_buffer      Package received from the QB Move.
 *  \param  measurements    Measurements will be stored here.
 *  \param  package_size    Package size.
 *
 *  \par Example
 *  \code

    short int measurements[3];
    processReplyGetMeasurements(package_in, measurements, package_in_size);
    
 *  \endcode
**/

void processReplyGetMeasurements(char pkg_buffer[], short int measurements[3], int package_size);

//==================================================     processReplyGetActivate

/** This function interprets the package with the activation status of a QB Move.
 * 
 *  \param  pkg_buffer      Package received from the QB Move.
 *  \param  activate        Activation status will be stored here.
 *  \param  package_size    Package size.
 *
 *  \par Example
 *  \code

    
    char activate;    
    processReplyGetActivate(package_in, &activate, package_in_size);


 *  \endcode
**/

void processReplyGetActivate(char pkg_buffer[], char *activate, int package_size);

//====================================================     processReplyGetInputs


/** This function interprets the package with inputs from a QB Move.
 * 
 *  \param  pkg_buffer      Package received from the QB Move.
 *  \param  inputs          Inputs will be stored here.
 *  \param  package_size    Package size.
 *
 *
 *  \par Example
 *  \code

    short int inputs[2];
    processReplyGetInputs(package_in, inputs, package_in_size);

 *  \endcode
**/


void processReplyGetInputs(char pkg_buffer[], short int inputs[2], int package_size);

//======================================================     processReplyGetInfo


/** This function interprets the package with information about the device. 
 * 
 *  \param  pkg_buffer      Package received from the QB Move.
 *  \param  inputs          Inputs will be stored here.
 *  \param  package_size    Package size.
 *
 *  \return Returns the number of pages the information string has.
 *
 *  \par Example
 *  \code

    char info[1000];
    processReplyGetInfo(package_in, info, package_in_size);

 *  \endcode
**/

unsigned char processReplyGetInfo(char pkg_buffer[], char *info, int package_size);

//======================================================     processReplyGetParam


/** This function interprets the package with a parameter of the device. 
 * 
 *  \param  pkg_buffer      Package received from the QB Move.
 *  \param  values          The parameter value.
 *  \param  value_size      The parameter value size (in bytes).
 *  \param  num_of_values   The size of the values array. 
 *  \param  package_size    Package size.
 *
 *
 *  \par Example
 *  \code

float control_k;
processReplyGetParam(package_in, PARAM_CONTROL_K, *control_k, sizeof(control_k), package_in_size);

 *  \endcode
**/


void processReplyGetParam(  char pkg_buffer[], 
                            void *values,
                            unsigned short int value_size,
                            unsigned short num_of_values,                           
                            int package_size);

/** \} */

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
#endif