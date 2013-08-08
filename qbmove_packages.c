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
 * \file         qbmove_packages.c
 *
 *  \brief      Library of low-level functions for communicating with 
 *              a QB Move. Implementation.
 *
 *  \details    
 *
Check the \ref qbmove_packages.h "qbmove_packages.h file reference" for a complete description of
the public functions implemented in qbmove_communications.c
 *
**/

//=================================================================     includes

#include "qbmove_packages.h"
#include "commands.h"
#include "string.h"


//===========================================     public fuctions implementation

/// @cond C_FILES

//==============================================================================
//                                                                   pkgActivate
//==============================================================================

int pkgActivate(int id, char activate, char pkg_buffer[])
{

    pkg_buffer[0] = CMD_ACTIVATE;                       // command
    pkg_buffer[1] = activate ? 3 : 0; 
    pkg_buffer[2] = checksum(pkg_buffer, 2);      // checksum    
    
    return 3;
}

//==============================================================================
//                                                                pkgGetActivate
//==============================================================================

int pkgGetActivate(int id, char pkg_buffer[])
{    
    pkg_buffer[0] = CMD_GET_ACTIVATE;             // command
    pkg_buffer[1] = CMD_GET_ACTIVATE;             // checksum
    
    return 2;
}

//==============================================================================
//                                                                  pkgSetInputs
//==============================================================================

int pkgSetInputs(int id, short int inputs[2], char pkg_buffer[])
{

    pkg_buffer[0] = CMD_SET_INPUTS;                // command
    pkg_buffer[1] = ((char *) &inputs[0])[1];
    pkg_buffer[2] = ((char *) &inputs[0])[0];
    pkg_buffer[3] = ((char *) &inputs[1])[1];
    pkg_buffer[4] = ((char *) &inputs[1])[0];
    pkg_buffer[5] = checksum(pkg_buffer, 5);   // checksum    

    return 6;
}

//==============================================================================
//                                                                pkgGetActivate
//==============================================================================

int pkgGetInputs(int id, char pkg_buffer[])
{    
    pkg_buffer[0] = CMD_GET_INPUTS;             // command
    pkg_buffer[1] = CMD_GET_INPUTS;             // checksum
    
    return 2;
}


//==============================================================================
//                                                            pkgGetMeasurements
//==============================================================================

int pkgGetMeasurements(int id, char pkg_buffer[])
{    

    pkg_buffer[0] = CMD_GET_MEASUREMENTS;             // command
    pkg_buffer[1] = CMD_GET_MEASUREMENTS;             // checksum
    
    return 2;
}

//==============================================================================
//                                                                   pkgSetParam
//==============================================================================


int pkgSetParam(    int id,
                    unsigned short int param_type,
                    void * values,
                    unsigned short int value_size,
                    unsigned short num_of_values,            
                    char pkg_buffer[])
{

    unsigned short int i, h;

	pkg_buffer[0] = CMD_SET_PARAM;		            // command
    pkg_buffer[1] = ((char *) &param_type)[1];      // parameter type
    pkg_buffer[2] = ((char *) &param_type)[0];      // parameter type

    for(h = 0; h < num_of_values; ++h)
    {
        for(i = 0; i < value_size; ++i)
        {
            pkg_buffer[ h * value_size +  3 + i ] = 
                ((char *) values)[ h * value_size + value_size - i - 1 ];
        }

    }


	pkg_buffer[ 3 + num_of_values * value_size ] =
	    checksum( pkg_buffer, 3 + num_of_values * value_size );	// checksum
	
    return  4 + num_of_values * value_size;
    
}

//==============================================================================
//                                                                   pkgGetParam
//==============================================================================

int pkgGetParam(int id, unsigned short int param_type, char pkg_buffer[])
{
	pkg_buffer[0] = CMD_GET_PARAM;		                // command
    pkg_buffer[1] = ((char *) &param_type)[1];          // parameter type
    pkg_buffer[2] = ((char *) &param_type)[0];          // parameter type
    
	pkg_buffer[3] = checksum (pkg_buffer, 3);	        // checksum
    
    return 4; 
}

//==============================================================================
//                                                                   pkgSetParam
//==============================================================================


int pkgStoreParams(int id, char pkg_buffer[])
{
    pkg_buffer[0] = CMD_STORE_PARAMS;                       // command
    pkg_buffer[1] = CMD_STORE_PARAMS;                       // checksum
    
    return 2;
}

//==============================================================================
//                                                                   pkgSetParam
//==============================================================================


int pkgRestoreParams(int id, char pkg_buffer[])
{
    pkg_buffer[0] = CMD_RESTORE_PARAMS;                       // command
    pkg_buffer[1] = CMD_RESTORE_PARAMS;                       // checksum
    
    return 2;
}

//==============================================================================
//                                                                     pkgPing
//==============================================================================

int pkgPing(int id, char pkg_buffer[])
{
    pkg_buffer[0] = CMD_PING;
    pkg_buffer[1] = CMD_PING;
    
    return 2;    
}

//==============================================================================
//                                                                pkgGetActivate
//==============================================================================

int pkgGetInfo(int id, unsigned short int info_type, unsigned char page, char pkg_buffer[])
{
    pkg_buffer[0] = CMD_GET_INFO;                        // command

    pkg_buffer[1] = ((unsigned char *) &info_type)[1];          // parameter type
    pkg_buffer[2] = ((unsigned char *) &info_type)[0];          // parameter type

    pkg_buffer[3] = page;    
    pkg_buffer[4] = checksum(pkg_buffer, 4);             // checksum
    
    return 5;
}


//==============================================================================
//                                                       processReplyGetActivate
//==============================================================================

void processReplyGetActivate(char pkg_buffer[], char *activate, int package_size)
{
    *activate = pkg_buffer[1];
}


//==============================================================================
//                                                          processReplyGetParam
//==============================================================================

void processReplyGetParam(  char pkg_buffer[], 
                            void *value, 
                            unsigned short int value_size, 
                            unsigned short num_of_values,                            
                            int package_size)
{
    unsigned short int i, h;
    for(h = 0; h < num_of_values; ++h)
    {
        for(i = 0; i < value_size; ++i)
        {
            ((char *) value) 
                [ h * value_size + value_size - i - 1 ] =
                pkg_buffer[ h * value_size + i + 1 ];
        }
    }
}


//==============================================================================
//                                                           processReplyGetInfo
//==============================================================================

unsigned char processReplyGetInfo(char pkg_buffer[], char *info, int package_size)
{
    strncpy(info, pkg_buffer + 2, package_size - 2);
    return pkg_buffer[1];
}


//==============================================================================
//                                                         processReplyGetInputs
//==============================================================================

void processReplyGetInputs(char pkg_buffer[], short int inputs[2], int package_size)
{
    ((char *) &inputs[0])[0] = pkg_buffer[2];
    ((char *) &inputs[0])[1] = pkg_buffer[1];
    
    ((char *) &inputs[1])[0] = pkg_buffer[4];
    ((char *) &inputs[1])[1] = pkg_buffer[3];
}

//==============================================================================
//                                                      processReplyGetMeasurements
//==============================================================================

void processReplyGetMeasurements(char pkg_buffer[], short int measurements[4], int package_size)
{
    ((char *) &measurements[0])[0] = pkg_buffer[2];
    ((char *) &measurements[0])[1] = pkg_buffer[1];
    
    ((char *) &measurements[1])[0] = pkg_buffer[4];
    ((char *) &measurements[1])[1] = pkg_buffer[3];
    
    ((char *) &measurements[2])[0] = pkg_buffer[6];
    ((char *) &measurements[2])[1] = pkg_buffer[5];

    ((char *) &measurements[3])[0] = pkg_buffer[8];
    ((char *) &measurements[3])[1] = pkg_buffer[7];
}


//========================================     private functions implementations

//==============================================================================
//																		checksum
//==============================================================================
// This functions returns an 8 bit LCR checksum over the lenght of a buffer.
//==============================================================================

char checksum ( char * data_buffer, int data_length )
{
	int i;
	char checksum = 0x00;
	for(i = 0; i < data_length; ++i)
	{
       checksum = checksum ^ data_buffer[i];
	}
	return checksum;
}

/// @endcond

/* [] END OF FILE */