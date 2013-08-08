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
 *  \file       qbmove_communications.c
 *
 *  \brief      Library of functions for SERIAL PORT communication with a 
 *              QB Move.
 *
 *              Implementation.
 *
 *  \details    
 *
 *  Check the \ref qbmove_communications.h "qbmove_communications.h file
 *  reference" for a complete description of the public functions implemented in
 *  qbmove_communications.c.
**/

//=================================================================     includes

#include "qbmove_communications.h"
#include "qbmove_packages.h"
#include "commands.h"

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */

#if !(defined(_WIN32) || defined(_WIN64))
    #include <unistd.h>  /* UNIX standard function definitions */
    #include <fcntl.h>   /* File control definitions */
    #include <errno.h>   /* Error number definitions */
    #include <termios.h> /* POSIX terminal control definitions */
    #include <sys/ioctl.h>    
    #include <dirent.h>
    #include <sys/time.h>
    #include <time.h>
#endif


//=================================================================     #defines

#if  DOXYGEN
    #define BAUD_RATE   CBR_115200(WIN) or B115200(UNIX) 
    ///< Virtual COM baud rate.

#elif (defined(_WIN32) || defined(_WIN64))
    #define BAUD_RATE   CBR_115200      ///< Virtual COM baud rate - WINDOWS
#else
	#define BAUD_RATE   460800         ///< Virtual COM baud rate - UNIX         
    //#define BAUD_RATE   B115200         ///< Virtual COM baud rate - UNIX
    //#define BAUD_RATE   230400         ///< Virtual COM baud rate - UNIX
#endif


#define BUFFER_SIZE 500
///< Size of buffers that store communication packets

// #define VERBOSE                 ///< Used for debugging

//===========================================     public fuctions implementation

/// @cond C_FILES

//==============================================================================
//                                                               RS485listPorts
//==============================================================================

int RS485listPorts( char list_of_ports[10][255] )
{
    //////////////////////////////   MAC OS X   //////////////////////////////
    #ifdef __APPLE__

    DIR     *directory;
    struct  dirent *directory_p;
    int i = 0;
    char aux_string[255];
    
    directory = opendir("/dev");

    while ( ( directory_p = readdir(directory) ) && i < 10 )
    {    
        strncpy(aux_string, directory_p->d_name, 13);
        aux_string[13] = 0;
        
        if(!strcmp(aux_string, "tty.usbserial")|!strcmp(aux_string, "tty.SLAB_USBt"))
             {
                 strcpy( list_of_ports[i], "/dev/" );
                 strcat(list_of_ports[i], directory_p->d_name);
                 i++;
             }
        }

    (void)closedir(directory);
    
    return i;
    
    //////////////////////////////   WINDOWS   //////////////////////////////
    #elif (defined(_WIN32) || defined(_WIN64))
    
    HANDLE port;
    int i, h;
    char aux_string[255];
    
    
    h = 0;
     
    for(i = 1; i < 10; ++i)
    {
        strcpy(list_of_ports[i], "");
        sprintf(aux_string, "COM%d", i);
        port = CreateFile(aux_string, GENERIC_WRITE|GENERIC_READ,
            		0, NULL, OPEN_EXISTING, 0, NULL);
        
        if( port != INVALID_HANDLE_VALUE)
        {
            strcpy(list_of_ports[h], aux_string);
            CloseHandle( port );
            h++;
        }
    }

    return h;

    #endif
    
    return 0;
}

//==============================================================================
//                                                                openRS485
//==============================================================================

void RS485InitCommSettings(comm_settings *comm_settings_t)
{
    comm_settings_t->type           = COMM_RS_485;
    comm_settings_t->file_handle    = comm_settings_t->file_handle;
    comm_settings_t->write          = RS485write;
    comm_settings_t->read           = RS485read;    
}


void openRS485(comm_settings *comm_settings_t, const char *port_s)
{

//////////////////////////////   WINDOWS CODE   //////////////////////////////

    #if (defined(_WIN32) || defined(_WIN64))
    
        DCB  dcb;                   // for serial port configuration
        COMMTIMEOUTS cts;           // for serial port configuration

//======================================================     opening serial port

                
    	comm_settings_t->file_handle = 
    	    CreateFile( port_s,   
    	                GENERIC_WRITE|GENERIC_READ,
    		            0, NULL, OPEN_EXISTING, FILE_FLAG_OPEN_REPARSE_POINT, NULL);

    	if (comm_settings_t->file_handle == INVALID_HANDLE_VALUE)
    	{
            goto error;
    	}

//==========================================     serial communication properties

    	dcb.DCBlength = sizeof (DCB); 

    	GetCommState(comm_settings_t->file_handle, &dcb);
    	dcb.BaudRate  = BAUD_RATE;
    	dcb.Parity    = NOPARITY;
    	dcb.StopBits  = ONESTOPBIT;

    	dcb.fOutxCtsFlow = FALSE;         // No CTS output flow control 
    	dcb.fOutxDsrFlow = FALSE;         // No DSR output flow control 
    	dcb.fDtrControl = FALSE;          // DTR flow control type 

    	dcb.fDsrSensitivity = FALSE;      // DSR sensitivity 
    	dcb.fTXContinueOnXoff = FALSE;     // XOFF continues Tx 
    	dcb.fOutX = FALSE;                // No XON/XOFF out flow control 
    	dcb.fInX = FALSE;                 // No XON/XOFF in flow control 
    	dcb.fErrorChar = FALSE;           // Disable error replacement 
    	dcb.fNull = FALSE;                // Disable null stripping 
    	dcb.fRtsControl = RTS_CONTROL_DISABLE;          // RTS flow control 
    	dcb.fAbortOnError = FALSE;        // Do not abort reads/writes on 
	                                      // error
    	dcb.ByteSize = 8;                 // Number of bits/byte, 4-8 

    	dcb.DCBlength = sizeof(DCB);
    	SetCommState(comm_settings_t->file_handle, &dcb);

        //Set up Input/Output buffer size 
        SetupComm(comm_settings_t->file_handle, 100, 100);

    	// timeouts
    	GetCommTimeouts(comm_settings_t->file_handle, &cts);
    	cts.ReadIntervalTimeout 		= 0;            		// msec
    	// ReadTimeout = Constant + Multiplier * Nwritten
    	cts.ReadTotalTimeoutMultiplier 	= 0;     				// msec
    	cts.ReadTotalTimeoutConstant 	= 100;       			// msec
    	// WriteTimeout = Constant + Multiplier * Nwritten
    	cts.WriteTotalTimeoutConstant 	= 100;           		// msec
    	cts.WriteTotalTimeoutMultiplier	= 0;       				// msec
    	SetCommTimeouts(comm_settings_t->file_handle, &cts);	
        RS485InitCommSettings(comm_settings_t);


        return;
    error:

        if (comm_settings_t->file_handle != INVALID_HANDLE_VALUE)
        {
            CloseHandle(comm_settings_t->file_handle);
        }

//////////////////////////////   UNIX CODE   //////////////////////////////
    #else
    
        struct termios options;

        comm_settings_t->file_handle = 
            open(port_s, O_RDWR | O_NOCTTY | O_NONBLOCK);

        if(comm_settings_t->file_handle == -1)
        {
            printf("Open failed\n");
            goto error;
        }

        // prevent multiple openings
        if (ioctl(comm_settings_t->file_handle, TIOCEXCL) == -1)
        {
            goto error;
        }

        // set communication as BLOCKING

        if(fcntl(comm_settings_t->file_handle, F_SETFL, 0) == -1)
        {
            goto error; 
        }

        if (tcgetattr(comm_settings_t->file_handle, &options) == -1)
        {
            goto error; 

        }
    
        // set baud rate
        // cfsetspeed(&options, BAUD_RATE);
        cfsetispeed(&options, BAUD_RATE);
        cfsetospeed(&options, BAUD_RATE);

        // enable the receiver and set local mode
        options.c_cflag |= (CLOCAL | CREAD);

        // enable flags
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
    
        // disable flags
        options.c_cflag &= ~CRTSCTS;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);    
        options.c_oflag &= ~OPOST;    
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;    
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;

        // save changes
        if (tcsetattr(comm_settings_t->file_handle, TCSANOW, &options) == -1)
        {
            goto error;
        }
        
        RS485InitCommSettings(comm_settings_t);
        // comm_settings_t->type        = COMM_RS_485;
        // comm_settings_t->file_handle = comm_settings_t->file_handle;
        // comm_settings_t->write = RS485write;
        // comm_settings_t->read = RS485read;
        
        return;

        error:
            if (comm_settings_t->file_handle != -1)
            {
                close(comm_settings_t->file_handle);
            }
            comm_settings_t->file_handle = INVALID_HANDLE_VALUE;
    #endif
}


//==============================================================================
//                                                                    closeRS485
//==============================================================================
// This function is used to close a serial port being used with the QB Move.
//==============================================================================

 
void closeRS485(comm_settings *comm_settings_t)
{       
#if (defined(_WIN32) || defined(_WIN64))
    CloseHandle( comm_settings_t->file_handle );
#else
    close(comm_settings_t->file_handle);    
#endif
}

//==============================================================================
//                                                                    RS485write
//==============================================================================
// This function is used to send packets.
//==============================================================================

void RS485write(comm_settings *comm_settings_t, int id, char *package, int package_size)
{

    unsigned char data_out[BUFFER_SIZE];		// output data buffer
    unsigned char package_in[BUFFER_SIZE];
    int n_bytes;

    #if (defined(_WIN32) || defined(_WIN64))
        DWORD package_size_out;					// for serial port access	
    #endif    

//========================================================     RS485 data header

    data_out[0]  = ':';
    data_out[1]  = ':';
    data_out[2] = (unsigned char) id;
    data_out[3]  = package_size;
    
    memcpy( data_out + 4, package, package_size);
    
//==============================================================	 send packet    

    #if (defined(_WIN32) || defined(_WIN64))
        WriteFile(comm_settings_t->file_handle, data_out, package_size + 4, &package_size_out, NULL);
    #else
    
// Flush the serial port if necessary
/// \todo Do the same on WINDOWS!!!

        ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
        if(n_bytes)
            read(comm_settings_t->file_handle, package_in, n_bytes);
    
        write(comm_settings_t->file_handle, data_out, package_size + 4);
    #endif
}

//==============================================================================
//                                                                   timevaldiff
//==============================================================================

/*
 * Return the difference between two timeval structures in microseconds
 */
long timevaldiff(struct timeval *starttime, struct timeval *finishtime)
{
  long usec;
  usec=(finishtime->tv_sec-starttime->tv_sec)*1000000;
  usec+=(finishtime->tv_usec-starttime->tv_usec);
  return usec;
}


//==============================================================================
//                                                                     RS485read
//==============================================================================
// This function is used to read packets from the device.
//==============================================================================

int RS485read(comm_settings *comm_settings_t, int id, char *package)
{
    unsigned char data_in[BUFFER_SIZE] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};     // output data buffer
    unsigned int package_size = 6;
    
    memcpy(package, data_in, package_size);
    
    package[1] = 0;
    package[2] = 0;
    package[3] = 0;
	package[4] = 0;
    package[5] = 0;
	package[6] = 0;

            
    

    // WINDOWS
    #if (defined(_WIN32) || defined(_WIN64))
        DWORD data_in_bytes = 0;
        
        if (!ReadFile(comm_settings_t->file_handle, data_in, 4, &data_in_bytes, NULL))
            return -1;
            
        // Control ID
        if ((id != 0) && (data_in[2] != id)) {
        	return -1;
        }
        
        package_size = data_in[3];            

        if(package_size > 8)
        {
            // Sleep((double) package_size * 0.5);
        }
 
        if (!ReadFile(comm_settings_t->file_handle, data_in, package_size, &data_in_bytes, NULL))
            return -1;
    
    // UNIX
    #else
        int n_bytes;
        struct timeval start, now;
        
        gettimeofday(&start, NULL);
        gettimeofday(&now, NULL);                 
        
        ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);        

        while((n_bytes == 0) && ( timevaldiff(&start, &now) < 3000)) 
        {
            gettimeofday(&now, NULL);                 
            ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
        }

        if (!read(comm_settings_t->file_handle, data_in, 4)) {
        	//printf("First read failed\n");
            return -1;
        }

        // Control ID
        if ((id != 0) && (data_in[2] != id)) {
        	//printf("ID failed\n");
        	return -1;
        }

            
        package_size = data_in[3];
        
        if(package_size > 8)
        {
            //usleep(package_size * 500);
        }
        
        gettimeofday(&start, NULL);
        gettimeofday(&now, NULL);                 
        
        ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);        

        while((n_bytes == 0) && ( timevaldiff(&start, &now) < 3000)) 
        {
            gettimeofday(&now, NULL);                 
            ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);            
        }
                
        if (!read(comm_settings_t->file_handle, data_in, package_size)) {
        	//printf("Second read failed\n");
            return -1;
        }
            
    #endif

    // Control checksum
    if (checksum ( (char *) data_in, package_size - 1) != (char) data_in[package_size-1])
    {
    	//printf("Checksum failed\n");
        return -1;
    }
    

    #ifdef VERBOSE
        printf("Received package size: %d \n", package_size);
    #endif
    

    memcpy(package, data_in, package_size);
        
    return package_size;
}

//==============================================================================
//                                                              RS485ListDevices
//==============================================================================

int RS485ListDevices(comm_settings *comm_settings_t, char list_of_ids[255])
{
        unsigned char package_out[BUFFER_SIZE];
        unsigned char package_in[BUFFER_SIZE];
        int package_out_size;
        int id;
        int h = 0; 
        
         #if (defined(_WIN32) || defined(_WIN64))
             int aux_int;
             int z = 0;   
          DWORD n_bytes_in;
             COMMTIMEOUTS cts_old, cts;         // for serial port configuration
     
          // timeouts
          GetCommTimeouts(comm_settings_t->file_handle, &cts_old);
          
             memcpy(&cts, &cts_old, sizeof(COMMTIMEOUTS));
          
          cts.ReadIntervalTimeout         = 0;      // msec
          // ReadTimeout = Constant + Multiplier * Nwritten
          cts.ReadTotalTimeoutMultiplier  = 0;      // msec
          cts.ReadTotalTimeoutConstant    = 5;      // msec
          // WriteTimeout = Constant + Multiplier * Nwritten
          cts.WriteTotalTimeoutConstant   = 5;      // msec
          cts.WriteTotalTimeoutMultiplier = 0;      // msec
     
          SetCommTimeouts(comm_settings_t->file_handle, &cts);    
         #else
             int n_bytes;
             ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
             if(n_bytes)
                 read(comm_settings_t->file_handle, package_in, n_bytes);
         #endif
         
         for(id = 1; id < 255; ++id)
         {
             list_of_ids[id] = 0;
             package_out_size = pkgPing(id, (char *) package_out);
             comm_settings_t->write(comm_settings_t, id, (char *) package_out, package_out_size);
     
             #if (defined(_WIN32) || defined(_WIN64))
     
                 n_bytes_in  = 1;
                 aux_int     = 0;
                 z = 0;
             
                 while(n_bytes_in)
                 {
                     ReadFile(  comm_settings_t->file_handle, 
                                package_out, 1, &n_bytes_in, NULL);
                     aux_int |= n_bytes_in;
                     
                     memcpy(package_in + z, package_out, n_bytes_in);
                     z += n_bytes_in;
                 }
             
                 if(aux_int)
                 {
                     list_of_ids[h] = package_in[2];
                     h++;            
                 }
     
             #else
                 usleep(20000);
                 ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
                 if (n_bytes >= 6)
                 {
                     read(comm_settings_t->file_handle, package_in, n_bytes);
                     list_of_ids[h] = package_in[2];
                     h++;
                 }
             #endif
         }
         
         #if (defined(_WIN32) || defined(_WIN64))
              SetCommTimeouts(comm_settings_t->file_handle, &cts_old);    
     #endif
     
         return h;
    
}


//==============================================================================
//                                                                          ping
//==============================================================================
// This function is used to ping the serial port for a QB Move and 
// get information about the device. ONLY USE WHEN ONE DEVICE IS CONNECTED
//  ONLY.
//==============================================================================

void RS485GetInfo(comm_settings *comm_settings_t, char *buffer){
    unsigned char auxstring[3];

    
#if (defined(_WIN32) || defined(_WIN64))
    DWORD n_bytes_out;					// for serial port access
	DWORD n_bytes_in;					// for serial port access
    unsigned char aux;
    int i = 0;
#else
    int bytes;  
#endif	
    
    auxstring[0] = '?'; 
    auxstring[1] = 13; 
    auxstring[2] = 10; 
        
#if (defined(_WIN32) || defined(_WIN64))
    WriteFile(comm_settings_t->file_handle, auxstring, 3, &n_bytes_out, NULL);
    n_bytes_in = 1;

    Sleep(200);

    while(n_bytes_in)
    {
        ReadFile(comm_settings_t->file_handle, &aux, 1, &n_bytes_in, NULL);
        if(n_bytes_in)
            buffer[i] = aux;
        i++;
    }    
#else        
    write(comm_settings_t->file_handle, auxstring, 3);   
    usleep(200000);
    ioctl(comm_settings_t->file_handle, FIONREAD, &bytes);
    read(comm_settings_t->file_handle, buffer, bytes);
#endif        
    
}

//==============================================================================
//                                                                      commPing
//==============================================================================

int commPing(comm_settings *comm_settings_t, int id)
{
        char package_out[BUFFER_SIZE];		// output data buffer
        char package_in[BUFFER_SIZE];		// output data buffer
        int package_out_size;
        int package_in_size;

//=================================================		preparing packet to send

        package_out_size = pkgPing(id, package_out);

        comm_settings_t->write(comm_settings_t, id, package_out, package_out_size);

        package_in_size = comm_settings_t->read(comm_settings_t, id, package_in);
        if ((package_in_size == -1) || (package_in[1] != CMD_PING))
            return -1;

        return 0;
}


//==============================================================================
//                                                                  commActivate
//==============================================================================
// This function activates or deactivates the QB Move motors.
//==============================================================================


void commActivate(comm_settings *comm_settings_t, int id, char activate)
{
    char package[BUFFER_SIZE];		// output data buffer
    int package_size;
    
    package_size = pkgActivate(id, activate, package);

    comm_settings_t->write(comm_settings_t, id, package, package_size);
}

//==============================================================================
//                                                          commGetMeasurements
//==============================================================================
// This function gets measurements from the QB Move.
//==============================================================================

int commGetActivate(comm_settings *comm_settings_t, int id, char *activate){
    char package_out[BUFFER_SIZE];		// output data buffer
    char package_in[BUFFER_SIZE];		// output data buffer
    int package_out_size;
    int package_in_size;

//=================================================		preparing packet to send

    package_out_size = pkgGetActivate(id, package_out);

    comm_settings_t->write(comm_settings_t, id, package_out, package_out_size);

    package_in_size = comm_settings_t->read(comm_settings_t, id, package_in);
    if (package_in_size == -1)
        return -1;

//==============================================================	 get packet

    processReplyGetActivate(package_in, activate,package_in_size);

    return 0;
}

//==============================================================================
//                                                               commSetInputs
//==============================================================================
// This function send reference inputs to the qb move.
//==============================================================================

void commSetInputs(comm_settings *comm_settings_t, int id, short int inputs[2])
{    
    char package[BUFFER_SIZE];		// output data buffer
    int package_size;
    
    package_size = pkgSetInputs(id, inputs, package);
    
    comm_settings_t->write(comm_settings_t, id, package, package_size);
}

//==============================================================================
//                                                                 commGetInputs
//==============================================================================
// This function gets input references from the QB Move.
//==============================================================================

int commGetInputs(comm_settings *comm_settings_t, int id, short int inputs[2]){

    char package_out[BUFFER_SIZE];		// output data buffer
    char package_in[BUFFER_SIZE];		// output data buffer
    int package_out_size;
    int package_in_size;

//=================================================		preparing packet to send

    package_out_size = pkgGetInputs(id, package_out);
    
    comm_settings_t->write(comm_settings_t, id, package_out, package_out_size);


    package_in_size = comm_settings_t->read(comm_settings_t, id, package_in);
    if (package_in_size == -1)
            return -1;

//==============================================================	 get packet

    processReplyGetInputs(package_in, inputs,package_in_size);

    return 0;
}

//==============================================================================
//                                                          commGetMeasurements
//==============================================================================
// This function gets measurements from the QB Move.
//==============================================================================

int commGetMeasurements(comm_settings *comm_settings_t, int id, short int measurements[4]){

    char package_out[BUFFER_SIZE];		// output data buffer
    char package_in[BUFFER_SIZE];		// output data buffer
    int package_out_size;
    int package_in_size;

//=================================================		preparing packet to send

    package_out_size = pkgGetMeasurements(id, package_out);

    comm_settings_t->write(comm_settings_t, id, package_out, package_out_size);


    package_in_size = comm_settings_t->read(comm_settings_t, id, package_in);

    if (package_in_size == -1)
        return -1;

    ///
    // package_in[package_in_size] = '\n';
    // printf("%s", package_in);
    ///

//==============================================================	 get packet

    processReplyGetMeasurements(package_in, measurements,package_in_size);

    return 0;
}

//==============================================================================
//                                                                   commGetInfo
//==============================================================================
// This function gets a string of information from the QB Move.
//==============================================================================

int commGetInfo(comm_settings *comm_settings_t, int id, unsigned char info_type, char *info){

    char package_out[BUFFER_SIZE];		// output data buffer
    char package_in[BUFFER_SIZE];		// output data buffer
    int package_out_size;
    int package_in_size;
    unsigned char num_of_pages;
    char aux_string[256];
    int i;

    strcpy(aux_string, "");
    strcpy(info, "");    

//=================================================		preparing packet to send

    package_out_size = pkgGetInfo(id, info_type, 0, package_out);
    comm_settings_t->write(comm_settings_t, id, package_out, package_out_size);

//==============================================================	 get packet

    package_in_size = comm_settings_t->read(comm_settings_t, id, package_in);

    if (package_in_size == -1)
        return -1;
    
    num_of_pages = processReplyGetInfo(package_in, aux_string, package_in_size);
    
    strcat(info, aux_string);
    
    if (num_of_pages > 1)
    {        
        for(i = 1; i < num_of_pages; ++i)
        {
            package_out_size = pkgGetInfo(id, info_type, 1,package_out);
            comm_settings_t->write( comm_settings_t, 
                                    id, package_out, package_out_size);

            package_in_size = 
                comm_settings_t->read(comm_settings_t, id, package_in);

            if (package_in_size == -1) return -1;
            num_of_pages = 
                processReplyGetInfo(package_in, aux_string, package_in_size);
                
            strcat(info, aux_string);
        }
    }
    
    return 0;
}


//==============================================================================
//                                                                  commSetParam
//==============================================================================
// This function send a parameter to the QB Move.
//==============================================================================

void commSetParam(  comm_settings *comm_settings_t, 
                    int id,
                    enum qbmove_parameter type, 
                    void *values, 
                    unsigned short num_of_values )
{
    char package[BUFFER_SIZE];       // output data buffer
    int package_size;
    
    void *value;
    unsigned short int value_size;
    
    switch (type){
        case PARAM_ID:
            value       = (unsigned char *) values;
            value_size  = 1;
            break;
        case PARAM_CONTROL_K:
            value       = (float *) values;
            value_size  = 4;         
            break;
        case PARAM_STARTUP_ACTIVATION:
            value       = (unsigned char *) values;
            value_size  = 1;        
            break;
        case PARAM_INPUT_MODE:
            value       = (unsigned char *) values;
            value_size  = 1;
            break;
        case PARAM_POS_RESOLUTION:
            value       = (unsigned char *) values;
            value_size  = 1;
            break;
        case PARAM_POS_MULTIPLIER:
            value       = (float *) values;
            value_size  = 4;
            break;
        case PARAM_POS_OFFSET:
            value           = (unsigned int *) values;
            value_size      = 2;
            break;
        case PARAM_MEAS_FILTER:
            value       = (float *) values;
            value_size  = 4;
            break;
        case PARAM_CONTROL_DEADZONE:
            value       = (float *) values;
            value_size  = 4;
            break;
        case PARAM_MEASUREMENT_OFFSET:
            value       = (unsigned int *) values;
            value_size  = 2;
            break;
        case PARAM_MEASUREMENT_MULTIPLIER:        
            value       = (float *) values;
            value_size  = 4;
            break;
    }

    package_size = pkgSetParam(id, type, value, value_size, num_of_values, package);    
    comm_settings_t->write(comm_settings_t, id, package, package_size);    
}

//==============================================================================
//                                                                 commGetParams
//==============================================================================

int commGetParam(comm_settings *comm_settings_t, 
                    int id,
                    enum qbmove_parameter type, 
                    void *values,
                    unsigned short num_of_values )
{
    char package_out[BUFFER_SIZE];       // output data buffer
    char package_in[BUFFER_SIZE];        // output data buffer
    int package_out_size;
    int package_in_size;
    
    unsigned short int values_size;

    switch (type){
        case PARAM_ID:
            values_size = 1;
            break;
        case PARAM_CONTROL_K:
                values_size = 4;
                break;
        case PARAM_STARTUP_ACTIVATION:
            values_size = 1;
            break;
        case PARAM_INPUT_MODE:
            values_size = 1;
            break;
        case PARAM_POS_RESOLUTION:
            values_size = 1;
            break;
        case PARAM_POS_MULTIPLIER:
            values_size = 4;
            break;
        case PARAM_POS_OFFSET:
            values_size = 2;
            break;            
        case PARAM_MEAS_FILTER:
            values_size = 4;
            break;
        case PARAM_CONTROL_DEADZONE:
            values_size = 4;
            break;
        case PARAM_MEASUREMENT_OFFSET:
            values_size = 2;        
            break;
        case PARAM_MEASUREMENT_MULTIPLIER:

            values_size = 4;
            break;
    }

//================================================      preparing packet to send
    package_out_size = pkgGetParam(id, type, package_out);
                
    comm_settings_t->write(comm_settings_t, id, package_out, package_out_size);
    package_in_size = comm_settings_t->read(comm_settings_t, id, package_in);

    if (package_in_size == -1)
            return -1;
            
//==============================================================  get packet

    processReplyGetParam(   package_in, values,
                            values_size, 
                            num_of_values,
                            package_in_size);
    
    return 0;
}

//==============================================================================
//                                                               commStoreParams
//==============================================================================

void commStoreParams( comm_settings *comm_settings_t, int id )
{
    char package[BUFFER_SIZE];      // output data buffer
    int package_size;

    package_size = pkgStoreParams(id, package);
    comm_settings_t->write(comm_settings_t, id, package, package_size);
}

//==============================================================================
//                                                               commRestoreParams
//==============================================================================

void commRestoreParams( comm_settings *comm_settings_t, int id )
{
    char package[BUFFER_SIZE];      // output data buffer
    int package_size;

    package_size = pkgRestoreParams(id, package);
    comm_settings_t->write(comm_settings_t, id, package, package_size);
}


/// @endcond


/* [] END OF FILE */
