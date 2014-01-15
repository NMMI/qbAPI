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
#include "commands.h"
#include "../definitions.h"


#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <stdint.h>

#if !(defined(_WIN32) || defined(_WIN64))
    #include <unistd.h>  /* UNIX standard function definitions */
    #include <fcntl.h>   /* File control definitions */
    #include <errno.h>   /* Error number definitions */
    #include <termios.h> /* POSIX terminal control definitions */
    #include <sys/ioctl.h>    
    #include <dirent.h>
    #include <sys/time.h>
    #include <time.h>
    #include <stdlib.h>
    #define _BSD_SOURCE
#endif

#if !(defined(_WIN32) || defined(_WIN64)) && !(defined(__APPLE__))
    #include <linux/serial.h>
#endif


//=================================================================     #defines

#if (defined(_WIN32) || defined(_WIN64))
    //#define BAUD_RATE   CBR_115200      ///< Virtual COM baud rate - WINDOWS
    #define BAUD_RATE   460800            ///< Virtual COM baud rate - WINDOWS
#elif (defined(__APPLE__))
    #define BAUD_RATE   460800
#else
    #define BAUD_RATE   B460800         ///< Virtual COM baud rate - UNIX
#endif


#define BUFFER_SIZE 500
///< Size of buffers that store communication packets

//#define VERBOSE                 ///< Used for debugging

//===========================================     public fuctions implementation

/// @cond C_FILES

////////////////
 #include <stdio.h>
#include <ctype.h>
 
#ifndef HEXDUMP_COLS
#define HEXDUMP_COLS 8
#endif
 
void hexdump(void *mem, unsigned int len)
{
    unsigned int i, j;
    
    for(i = 0; i < len + ((len % HEXDUMP_COLS) ? (HEXDUMP_COLS - len % HEXDUMP_COLS) : 0); i++)
    {
        /* print offset */
        if(i % HEXDUMP_COLS == 0)
        {
            printf("0x%06x: ", i);
        }

        /* print hex data */
        if(i < len)
        {
            printf("%02x ", 0xFF & ((char*)mem)[i]);
        }
        else /* end of block, just aligning for ASCII dump */
        {
            printf("   ");
        }
        
        /* print ASCII dump */
        if(i % HEXDUMP_COLS == (HEXDUMP_COLS - 1))
        {
            for(j = i - (HEXDUMP_COLS - 1); j <= i; j++)
            {
                if(j >= len) /* end of block, not really printing */
                {
                    putchar(' ');
                }
                else if(isprint(((char*)mem)[j])) /* printable char */
                {
                    putchar(0xFF & ((char*)mem)[j]);        
                }
                else /* other char */
                {
                    putchar('.');
                }
            }
            putchar('\n');
        }
    }
}
///////////////

//==============================================================================
//                                                               RS485listPorts
//==============================================================================

int RS485listPorts( char list_of_ports[10][255] )
{
    
    
    //////////////////////////////   WINDOWS   //////////////////////////////
    #if (defined(_WIN32) || defined(_WIN64))
    
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

    
    //////////////////////////////   UNIX   //////////////////////////////
    #else

    DIR     *directory;
    struct  dirent *directory_p;
    int i = 0;
    
    directory = opendir("/dev");

    while ( ( directory_p = readdir(directory) ) && i < 10 )
    {    
        if (strstr(directory_p->d_name, "tty.usbserial") || strstr(directory_p->d_name, "ttyUSB")) {
            strcpy(list_of_ports[i], "/dev/" );
            strcat(list_of_ports[i], directory_p->d_name);
            i++;
        }
    }

    (void)closedir(directory);
    
    return i;

    #endif
    
    return 0;
}

//==============================================================================
//                                                                openRS485
//==============================================================================


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

        // if(fcntl(comm_settings_t->file_handle, F_SETFL, O_NONBLOCK) == -1)
        // {
        //     goto error; 
        // }

        if (tcgetattr(comm_settings_t->file_handle, &options) == -1)
        {
            goto error; 

        }
    
        // set baud rate
        cfsetispeed(&options, BAUD_RATE);
        cfsetospeed(&options, BAUD_RATE);

        // enable the receiver and set local mode
        options.c_cflag |= (CLOCAL | CREAD);

        // enable flags
        options.c_cflag &= ~PARENB;
        //options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
    
        //disable flags
        options.c_cflag &= ~CRTSCTS;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);    
        options.c_oflag &= ~OPOST;    
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;    
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;

        #if !(defined __APPLE__)
            struct serial_struct serinfo;

            ioctl(comm_settings_t->file_handle, TIOCGSERIAL, &serinfo);
            serinfo.flags |= ASYNC_LOW_LATENCY;
            ioctl(comm_settings_t->file_handle, TIOCSSERIAL, &serinfo);
        #endif

        // save changes
        if (tcsetattr(comm_settings_t->file_handle, TCSANOW, &options) == -1)
        {
            goto error;
        }
        
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
 
        if (!ReadFile(comm_settings_t->file_handle, data_in, package_size, &data_in_bytes, NULL))
            return -1;
    
    // UNIX
    #else
        int n_bytes;
        struct timeval start, now;
        
        gettimeofday(&start, NULL);
        gettimeofday(&now, NULL);                 
        
        ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);        

        while((n_bytes < 4) && ( timevaldiff(&start, &now) < 4000)) 
        {
            gettimeofday(&now, NULL);                 
            ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
        }

        if (!read(comm_settings_t->file_handle, data_in, 4)) {
            return -1;
        }

        // Control ID
        if ((id != 0) && (data_in[2] != id)) {
            return -1;
        }

            
        package_size = data_in[3];
        
        gettimeofday(&start, NULL);
        gettimeofday(&now, NULL);                 
        
        ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);

        while((n_bytes < package_size) && ( timevaldiff(&start, &now) < 4000))
        {
            gettimeofday(&now, NULL);
            ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);            
        }
          
        if (!read(comm_settings_t->file_handle, data_in, package_size)) {
            return -1;
        }
            
    #endif

    // Control checksum
    if (checksum ( (char *) data_in, package_size - 1) != (char) data_in[package_size-1])
    {
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
        unsigned char package_in[BUFFER_SIZE];
        int id;
        int h = 0;
	    unsigned char data_out[BUFFER_SIZE];		// output data buffer
	    int n_bytes;

	    #if (defined(_WIN32) || defined(_WIN64))
	        DWORD package_size_out;					// for serial port access	
	    #endif    
		
        
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
             ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
             if(n_bytes)
                 read(comm_settings_t->file_handle, package_in, n_bytes);
         #endif
         
         for(id = 1; id < 255; ++id)
         {
            list_of_ids[id] = 0;
		
		    data_out[0]  = ':';
		    data_out[1]  = ':';
		    data_out[2] = (unsigned char) id;
		    data_out[3]  = 2;
	 	    data_out[4] = CMD_PING;
	 	    data_out[5] = CMD_PING;
			 
    #if (defined(_WIN32) || defined(_WIN64))
        WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
    #else
        ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
        if(n_bytes)
            read(comm_settings_t->file_handle, package_in, n_bytes);
    
        write(comm_settings_t->file_handle, data_out, 6);
    #endif

     
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
    usleep(500000);
    ioctl(comm_settings_t->file_handle, FIONREAD, &bytes);
    printf("BYTES: %d\n", bytes);
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
        int package_in_size;
	    int n_bytes;
		
		#if (defined(_WIN32) || defined(_WIN64))
		    DWORD package_size_out;					// for serial port access	
		#endif    

//=================================================		preparing packet to send


	    package_out[0]  = ':';
	    package_out[1]  = ':';
	    package_out[2] = (unsigned char) id;
	    package_out[3]  = 2;


	    package_out[4] = CMD_PING;
	    package_out[5] = CMD_PING;
		

#if (defined(_WIN32) || defined(_WIN64))
    WriteFile(comm_settings_t->file_handle, package_out, 6, &package_size_out, NULL);
#else
    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
    if(n_bytes)
        read(comm_settings_t->file_handle, package_in, n_bytes);

    write(comm_settings_t->file_handle, package_out, 6);
#endif

        package_in_size = RS485read(comm_settings_t, id, package_in);
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
    char data_out[BUFFER_SIZE];		// output data buffer
    char package_in[BUFFER_SIZE];
    int n_bytes;

    #if (defined(_WIN32) || defined(_WIN64))
        DWORD package_size_out;					// for serial port access	
    #endif    
	
	    data_out[0]  = ':';
	    data_out[1]  = ':';
	    data_out[2] = (unsigned char) id;
	    data_out[3]  = 3;
	    data_out[4] = CMD_ACTIVATE;                       // command
		data_out[5] = activate ? 3 : 0; 
		data_out[6] = checksum(data_out + 4, 2);      // checksum    
	
	#if (defined(_WIN32) || defined(_WIN64))
	    WriteFile(comm_settings_t->file_handle, data_out, 7, &package_size_out, NULL);
	#else
	    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
	    if(n_bytes)
	        read(comm_settings_t->file_handle, package_in, n_bytes);

	    write(comm_settings_t->file_handle, data_out, 7);
	#endif
	
}

//==============================================================================
//                                                               commGetActivate
//==============================================================================
// This function gets measurements from the QB Move.
//==============================================================================

int commGetActivate(comm_settings *comm_settings_t, int id, char *activate){
    char data_out[BUFFER_SIZE];		// output data buffer
    char package_in[BUFFER_SIZE];		// output data buffer
    int package_in_size;
    int n_bytes;

    #if (defined(_WIN32) || defined(_WIN64))
        DWORD package_size_out;					// for serial port access	
    #endif    
	

//=================================================		preparing packet to send

	
    data_out[0]  = ':';
    data_out[1]  = ':';
    data_out[2] = (unsigned char) id;
    data_out[3]  = 2;	
    data_out[4] = CMD_GET_ACTIVATE;             // command
    data_out[5] = CMD_GET_ACTIVATE;             // checksum
	

	#if (defined(_WIN32) || defined(_WIN64))
	    WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
	#else
	    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
	    if(n_bytes)
	        read(comm_settings_t->file_handle, package_in, n_bytes);

	    write(comm_settings_t->file_handle, data_out, 6);
	#endif


    package_in_size = RS485read(comm_settings_t, id, package_in);
    if (package_in_size == -1)
        return -1;

//==============================================================	 get packet
	
    *activate = package_in[1];

    return 0;
}

//==============================================================================
//                                                               commSetInputs
//==============================================================================
// This function send reference inputs to the qb move.
//==============================================================================

void commSetInputs(comm_settings *comm_settings_t, int id, short int inputs[2])
{    
    char data_out[BUFFER_SIZE];		// output data buffer
    unsigned char package_in[BUFFER_SIZE];
    int n_bytes;

    #if (defined(_WIN32) || defined(_WIN64))
        DWORD package_size_out;					// for serial port access	
    #endif    


    data_out[0]  = ':';
    data_out[1]  = ':';
    data_out[2] = (unsigned char) id;
    data_out[3]  = 6;


    data_out[4] = CMD_SET_INPUTS;                // command
    data_out[5] = ((char *) &inputs[0])[1];
    data_out[6] = ((char *) &inputs[0])[0];
    data_out[7] = ((char *) &inputs[1])[1];
    data_out[8] = ((char *) &inputs[1])[0];
    data_out[9] = checksum(data_out + 4, 5);   // checksum    

	#if (defined(_WIN32) || defined(_WIN64))
	    WriteFile(comm_settings_t->file_handle, data_out, 10, &package_size_out, NULL);
	#else
	    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
	    if(n_bytes)
	        read(comm_settings_t->file_handle, package_in, n_bytes);

	    write(comm_settings_t->file_handle, data_out, 10);
	#endif

}

//==============================================================================
//                                                                 commGetInputs
//==============================================================================
// This function gets input references from the QB Move.
//==============================================================================

int commGetInputs(comm_settings *comm_settings_t, int id, short int inputs[2]){

    char data_out[BUFFER_SIZE];		// output data buffer
    char package_in[BUFFER_SIZE];		// output data buffer
    int package_in_size;
    int n_bytes;

    #if (defined(_WIN32) || defined(_WIN64))
        DWORD package_size_out;					// for serial port access	
    #endif    
	

//=================================================		preparing packet to send
	
    data_out[0]  = ':';
    data_out[1]  = ':';
    data_out[2] = (unsigned char) id;
    data_out[3]  = 2;
    data_out[4] = CMD_GET_INPUTS;             // command
    data_out[5] = CMD_GET_INPUTS;             // checksum
	
    

	#if (defined(_WIN32) || defined(_WIN64))
	    WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
	#else
	    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
	    if(n_bytes)
	        read(comm_settings_t->file_handle, package_in, n_bytes);

	    write(comm_settings_t->file_handle, data_out, 6);
	#endif



    package_in_size = RS485read(comm_settings_t, id, package_in);
    if (package_in_size == -1)
            return -1;

//==============================================================	 get packet
	
	
    ((char *) &inputs[0])[0] = package_in[2];
    ((char *) &inputs[0])[1] = package_in[1];
    
    ((char *) &inputs[1])[0] = package_in[4];
    ((char *) &inputs[1])[1] = package_in[3];
	

    return 0;
}

//==============================================================================
//                                                          commGetMeasurements
//==============================================================================
// This function gets measurements from the QB Move.
//==============================================================================

int commGetMeasurements(comm_settings *comm_settings_t, int id, short int measurements[]){

    char data_out[BUFFER_SIZE];		// output data buffer
    char package_in[BUFFER_SIZE];		// output data buffer
    int package_in_size;
    int n_bytes;

    #if (defined(_WIN32) || defined(_WIN64))
        DWORD package_size_out;					// for serial port access	
    #endif    

//=================================================		preparing packet to send
	
    data_out[0]  = ':';
    data_out[1]  = ':';
    data_out[2] = (unsigned char) id;
    data_out[3]  = 2;
    data_out[4] = CMD_GET_MEASUREMENTS;             // command
    data_out[5] = CMD_GET_MEASUREMENTS;             // checksum
	

	#if (defined(_WIN32) || defined(_WIN64))
	    WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
	#else
	    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
	    if(n_bytes)
	        read(comm_settings_t->file_handle, package_in, n_bytes);

	    write(comm_settings_t->file_handle, data_out, 6);
	#endif


    package_in_size = RS485read(comm_settings_t, id, package_in);
    if (package_in_size == -1)
        return -1;

//==============================================================	 get packet
	
    ((char *) &measurements[0])[0] = package_in[2];
    ((char *) &measurements[0])[1] = package_in[1];
    
    ((char *) &measurements[1])[0] = package_in[4];
    ((char *) &measurements[1])[1] = package_in[3];
    
    ((char *) &measurements[2])[0] = package_in[6];
    ((char *) &measurements[2])[1] = package_in[5];

    #if NUM_OF_SENSORS == 4
        ((char *) &measurements[3])[0] = package_in[8];
        ((char *) &measurements[3])[1] = package_in[7];
    #endif
	

    return 0;
}

//==============================================================================
//                                                          commGetCurrents
//==============================================================================
// This function gets currents from the QB Move.
//==============================================================================

int commGetCurrents(comm_settings *comm_settings_t, int id, short int currents[2]){

    char data_out[BUFFER_SIZE];         // output data buffer
    char package_in[BUFFER_SIZE];       // output data buffer
    int package_in_size;
    int n_bytes;

    #if (defined(_WIN32) || defined(_WIN64))
        DWORD package_size_out;					// for serial port access	
    #endif    
	

//=================================================		preparing packet to send

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 2;
    data_out[4] = CMD_GET_CURRENTS;             // command
    data_out[5] = CMD_GET_CURRENTS;             // checksum

	#if (defined(_WIN32) || defined(_WIN64))
	    WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
	#else
	    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
	    if(n_bytes)
	        read(comm_settings_t->file_handle, package_in, n_bytes);

	    write(comm_settings_t->file_handle, data_out, 6);
	#endif


    package_in_size = RS485read(comm_settings_t, id, package_in);
    if (package_in_size == -1)
        return -1;
//==============================================================	 get packet

    ((char *) &currents[0])[0] = package_in[2];
    ((char *) &currents[0])[1] = package_in[1];
    
    ((char *) &currents[1])[0] = package_in[4];
    ((char *) &currents[1])[1] = package_in[3];


    return 0;
}


//==============================================================================
//                                                            commGetCurrAndMeas
//==============================================================================
// This function gets currents and measurements from the QB Move.
//==============================================================================

int commGetCurrAndMeas( comm_settings *comm_settings_t,
                        int id,
                        short int *values) {

    char data_out[BUFFER_SIZE];     // output data buffer
    char package_in[BUFFER_SIZE];       // output data buffer
    int package_in_size;
    int n_bytes;

    #if (defined(_WIN32) || defined(_WIN64))
        DWORD package_size_out;                 // for serial port access   
    #endif

    //=================================================     preparing packet to send

    
    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 2;   
    data_out[4] = CMD_GET_CURR_AND_MEAS;             // command
    data_out[5] = CMD_GET_CURR_AND_MEAS;             // checksum


    #if (defined(_WIN32) || defined(_WIN64))
        WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
    #else
        ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
        if(n_bytes)
            read(comm_settings_t->file_handle, package_in, n_bytes);

        write(comm_settings_t->file_handle, data_out, 6);
    #endif

    package_in_size = RS485read(comm_settings_t, id, package_in);
    if (package_in_size == -1)
        return -1;

    //==============================================================     get packet

    // Currents
    ((char *) &values[0])[0] = package_in[2];
    ((char *) &values[0])[1] = package_in[1];
    
    ((char *) &values[1])[0] = package_in[4];
    ((char *) &values[1])[1] = package_in[3];

    // Measurements
    ((char *) &values[2])[0] = package_in[6];
    ((char *) &values[2])[1] = package_in[5];

    ((char *) &values[3])[0] = package_in[8];
    ((char *) &values[3])[1] = package_in[7];

    ((char *) &values[4])[0] = package_in[10];
    ((char *) &values[4])[1] = package_in[9];

    return 0;
}

//==============================================================================
//                                                                   commGetInfo
//==============================================================================
// This function gets a string of information from the QB Move.
//==============================================================================

int commGetInfo(comm_settings *comm_settings_t, int id, unsigned char info_type, char *info){

    char data_out[BUFFER_SIZE];			// output data buffer
    char package_in[BUFFER_SIZE];		// output data buffer
    int n_bytes;	
    int package_in_size;
    unsigned char num_of_pages;
    char aux_string[256];
    int i;
	
	#if (defined(_WIN32) || defined(_WIN64))
	    DWORD package_size_out;					// for serial port access	
	#endif    
	

    strcpy(aux_string, "");
    strcpy(info, "");    

//=================================================		preparing packet to send

    data_out[0]  = ':';
    data_out[1]  = ':';
    data_out[2] = (unsigned char) id;
    data_out[3]  = 5;
    data_out[4] = CMD_GET_INFO;                        // command
    data_out[5] = ((unsigned char *) &info_type)[1];   // parameter type
    data_out[6] = ((unsigned char *) &info_type)[0];   // parameter type
    data_out[7] = 0;    
    data_out[8] = checksum(data_out + 4, 4);           // checksum
	
	
#if (defined(_WIN32) || defined(_WIN64))
    WriteFile(comm_settings_t->file_handle, data_out, 9, &package_size_out, NULL);
#else
    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
    if(n_bytes)
        read(comm_settings_t->file_handle, package_in, n_bytes);

    write(comm_settings_t->file_handle, data_out, 9);
#endif


//==============================================================	 get packet

    package_in_size = RS485read(comm_settings_t, id, package_in);

    if (package_in_size == -1)
        return -1;
    
	    strncpy(info, package_in + 2, package_in_size - 2);
	
    num_of_pages =  package_in[1];
    
    strcat(info, aux_string);
    
    if (num_of_pages > 1)
    {        
        for(i = 1; i < num_of_pages; ++i)
        {
						
		    data_out[0]  = ':';
		    data_out[1]  = ':';
		    data_out[2] = (unsigned char) id;
		    data_out[3]  = 5;
						
		    data_out[4] = CMD_GET_INFO;                        // command

		    data_out[5] = ((unsigned char *) &info_type)[1];          // parameter type
		    data_out[6] = ((unsigned char *) &info_type)[0];          // parameter type

		    data_out[7] = i;    
		    data_out[8] = checksum(data_out + 4, 4);             // checksum
		
		

		#if (defined(_WIN32) || defined(_WIN64))
		    WriteFile(comm_settings_t->file_handle, data_out, 9, &package_size_out, NULL);
		#else
		    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
		    if(n_bytes)
		        read(comm_settings_t->file_handle, package_in, n_bytes);

		    write(comm_settings_t->file_handle, data_out, 9);
		#endif

            package_in_size = 
                RS485read(comm_settings_t, id, package_in);

            if (package_in_size == -1) return -1;
			
		    strncpy(info, package_in + 2, package_in_size - 2);
			
            num_of_pages = package_in[1];
                
            strcat(info, aux_string);
        }
    }
    
    return 0;
}



//==============================================================================
//                                                                commBootloader
//==============================================================================
//  This function launches bootloader
//==============================================================================


int commBootloader(comm_settings *comm_settings_t, int id)
{
    char data_out[BUFFER_SIZE];     // output data buffer
    char package_in[BUFFER_SIZE];
    int package_in_size;
    int n_bytes;

    #if (defined(_WIN32) || defined(_WIN64))
        DWORD package_size_out;                 // for serial port access   
    #endif    
    
        data_out[0] = ':';
        data_out[1] = ':';
        data_out[2] = (unsigned char) id;
        data_out[3] = 2;
        data_out[4] = CMD_BOOTLOADER;       // command
        data_out[5] = CMD_BOOTLOADER;       // checksum
    
    #if (defined(_WIN32) || defined(_WIN64))
        WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
    #else
        ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
        if(n_bytes)
            read(comm_settings_t->file_handle, package_in, n_bytes);

        write(comm_settings_t->file_handle, data_out, 6);
    #endif

    package_in_size = RS485read(comm_settings_t, id, package_in);
    if (package_in_size == -1)
            return -1;

    return 0;
}


//==============================================================================
//                                                                  commSetParam
//==============================================================================
// This function send a parameter to the QB Move.
//==============================================================================

int commSetParam(  comm_settings *comm_settings_t, 
                    int id,
                    enum qbmove_parameter type, 
                    void *values, 
                    unsigned short num_of_values )
{
    char data_out[BUFFER_SIZE];		// output data buffer
    char package_in[BUFFER_SIZE];
    int package_in_size;
    int n_bytes;

    #if (defined(_WIN32) || defined(_WIN64))
        DWORD package_size_out;					// for serial port access	
    #endif    
    
    void *value;
    unsigned short int value_size, i, h;
    
    switch (type){
        case PARAM_ID:
            value       = (unsigned char *) values;
            value_size  = 1;
            break;
        case PARAM_PID_CONTROL:
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
        case PARAM_MEASUREMENT_OFFSET:
            value       = (unsigned int *) values;
            value_size  = 2;
            break;
        case PARAM_MEASUREMENT_MULTIPLIER:        
            value       = (float *) values;
            value_size  = 4;
            break;
        case PARAM_POS_LIMIT_FLAG:
        	value 		= (unsigned char *) values;
        	value_size  = 1;
            break;
        case PARAM_POS_LIMIT:
        	value 		= (int32_t *) values;
        	value_size  = 4;
            break;
        case PARAM_MAX_STEP_POS:
            value       = (int32_t *) values;
            value_size  = 4;
            break;
        case PARAM_MAX_STEP_NEG:
            value       = (int32_t *) values;
            value_size  = 4;
            break;
    }
	
	
	
    data_out[0]  = ':';
    data_out[1]  = ':';
    data_out[2]  = (unsigned char) id;
    data_out[3]  = 4 + num_of_values * value_size;
	
	data_out[4] = CMD_SET_PARAM;		            // command
    data_out[5] = ((char *) &type)[1];      // parameter type
    data_out[6] = ((char *) &type)[0];      // parameter type

    for(h = 0; h < num_of_values; ++h)
    {
        for(i = 0; i < value_size; ++i)
        {
            data_out[ h * value_size +  7 + i ] = 
                ((char *) value)[ h * value_size + value_size - i - 1 ];
        }

    }


	data_out[ 7 + num_of_values * value_size ] =
	    checksum( data_out + 4, 3 + num_of_values * value_size );	// checksum

    // utility function to print raw data
    //hexdump(data_out, 20);
	

    #if (defined(_WIN32) || defined(_WIN64))
        WriteFile(comm_settings_t->file_handle, data_out, 8 + num_of_values * value_size, &package_size_out, NULL);
    #else
        ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
        if(n_bytes)
            read(comm_settings_t->file_handle, package_in, n_bytes);

        write(comm_settings_t->file_handle, data_out, 8 + num_of_values * value_size);
    #endif

    package_in_size = RS485read(comm_settings_t, id, package_in);

    if (package_in_size == -1) {
        return -1;
    } else {
        return 0;
    }
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
    int package_in_size;
    char data_out[BUFFER_SIZE];		// output data buffer
    char package_in[BUFFER_SIZE];
    int n_bytes;

    #if (defined(_WIN32) || defined(_WIN64))
        DWORD package_size_out;					// for serial port access	
    #endif    
		    
    unsigned short int values_size;

    switch (type){
        case PARAM_ID:
            values_size = 1;
            break;
        case PARAM_PID_CONTROL:
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
        case PARAM_MEASUREMENT_OFFSET:
            values_size = 2;        
            break;
        case PARAM_MEASUREMENT_MULTIPLIER:
            values_size = 4;
            break;
        case PARAM_POS_LIMIT_FLAG:
        	values_size = 1;
        	break;
        case PARAM_POS_LIMIT:
        	values_size = 4;
        	break;
        case PARAM_MAX_STEP_POS:
            break;
        case PARAM_MAX_STEP_NEG:
            break;
    }

//================================================      preparing packet to send
    
    data_out[0]  = ':';
    data_out[1]  = ':';
    data_out[2] = (unsigned char) id;
    data_out[3]  = 4;
	data_out[4] = CMD_GET_PARAM;		                // command
    data_out[5] = ((char *) &type)[1];          		// parameter type
    data_out[6] = ((char *) &type)[0];          		// parameter type
    
	data_out[7] = checksum (data_out + 4, 3);	        // checksum

	#if (defined(_WIN32) || defined(_WIN64))
	    WriteFile(comm_settings_t->file_handle, data_out, 8, &package_size_out, NULL);
	#else
	    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
	    if(n_bytes)
	        read(comm_settings_t->file_handle, package_in, n_bytes);

	    write(comm_settings_t->file_handle, data_out, 8);
	#endif
	
    package_in_size = RS485read(comm_settings_t, id, package_in);

    if (package_in_size == -1)
            return -1;
            
//==============================================================  get packet

    unsigned short int i, h;
    for(h = 0; h < num_of_values; ++h)
    {
        for(i = 0; i < values_size; ++i)
        {
            ((char *) values) 
                [ h * values_size + values_size - i - 1 ] =
                package_in[ h * values_size + i + 1 ];
        }
    }
    
    return 0;
}

//==============================================================================
//                                                               commStoreParams
//==============================================================================

int commStoreParams( comm_settings *comm_settings_t, int id )
{

	char data_out[BUFFER_SIZE];		// output data buffer
	char package_in[BUFFER_SIZE];
    int package_in_size;
    int n_bytes;

    #if (defined(_WIN32) || defined(_WIN64))
        DWORD package_size_out;					// for serial port access	
    #endif    

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 2;
    data_out[4] = CMD_STORE_PARAMS;                       // command
    data_out[5] = CMD_STORE_PARAMS;                       // checksum

	#if (defined(_WIN32) || defined(_WIN64))
	    WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
	#else
	    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
	    if(n_bytes)
	        read(comm_settings_t->file_handle, package_in, n_bytes);

	    write(comm_settings_t->file_handle, data_out, 6);
	#endif

    usleep(100000);
    package_in_size = RS485read(comm_settings_t, id, package_in);

    if (package_in_size == -1) {
        return -1;
    } else {
        return 0;
    }
}

//==============================================================================
//                                                        commStoreDefaultParams
//==============================================================================

int commStoreDefaultParams( comm_settings *comm_settings_t, int id )
{

    char data_out[BUFFER_SIZE];     // output data buffer
    char package_in[BUFFER_SIZE];
    int package_in_size;
    int n_bytes;

    #if (defined(_WIN32) || defined(_WIN64))
        DWORD package_size_out;                 // for serial port access   
    #endif    

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 2;
    data_out[4] = CMD_STORE_DEFAULT_PARAMS;                       // command
    data_out[5] = CMD_STORE_DEFAULT_PARAMS;                       // checksum

    #if (defined(_WIN32) || defined(_WIN64))
        WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
    #else
        ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
        if(n_bytes)
            read(comm_settings_t->file_handle, package_in, n_bytes);

        write(comm_settings_t->file_handle, data_out, 6);
    #endif

    usleep(200000);
    package_in_size = RS485read(comm_settings_t, id, package_in);

    if (package_in_size == -1) {
        return -1;
    } else {
        return 0;
    }
}

//==============================================================================
//                                                               commRestoreParams
//==============================================================================

int commRestoreParams( comm_settings *comm_settings_t, int id )
{

    char data_out[BUFFER_SIZE];		// output data buffer
    char package_in[BUFFER_SIZE];
    int package_in_size;
    int n_bytes;

    #if (defined(_WIN32) || defined(_WIN64))
        DWORD package_size_out;					// for serial port access	
    #endif    

    data_out[0]  = ':';
    data_out[1]  = ':';
    data_out[2] = (unsigned char) id;
    data_out[3]  = 2;

    data_out[4] = CMD_RESTORE_PARAMS;                       // command
    data_out[5] = CMD_RESTORE_PARAMS;                       // checksum

	#if (defined(_WIN32) || defined(_WIN64))
	    WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
	#else
	    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
	    if(n_bytes)
	        read(comm_settings_t->file_handle, package_in, n_bytes);

	    write(comm_settings_t->file_handle, data_out, 6);
	#endif

    usleep(100000);
    package_in_size = RS485read(comm_settings_t, id, package_in);

    if (package_in_size == -1) {
        return -1;
    } else {
        return 0;
    }
}

//==============================================================================
//                                                                   commInitMem
//==============================================================================

int commInitMem(comm_settings *comm_settings_t, int id) {
    char data_out[BUFFER_SIZE];     // output data buffer
    char package_in[BUFFER_SIZE];
    int package_in_size;
    int n_bytes;

    #if (defined(_WIN32) || defined(_WIN64))
        DWORD package_size_out;                 // for serial port access   
    #endif    

    data_out[0]  = ':';
    data_out[1]  = ':';
    data_out[2]  = (unsigned char) id;
    data_out[3]  = 2;

    data_out[4] = CMD_INIT_MEM;                       // command
    data_out[5] = CMD_INIT_MEM;                       // checksum

    #if (defined(_WIN32) || defined(_WIN64))
        WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
    #else
        ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
        if(n_bytes)
            read(comm_settings_t->file_handle, package_in, n_bytes);

        write(comm_settings_t->file_handle, data_out, 6);
    #endif

    usleep(200000);
    package_in_size = RS485read(comm_settings_t, id, package_in);

    if (package_in_size == -1) {
        return -1;
    } else {
        return 0;
    }
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