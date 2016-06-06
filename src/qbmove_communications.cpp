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
 *  \file       qbmove_communications.cpp
 *
 *  \brief      Library of functions for SERIAL PORT communication with a
 *              qbMove or a qbHand
 *
 *  \details
 *
 *  Check the \ref qbmove_communications.h "qbmove_communications.h" file
 *  for a complete description of the public functions implemented in
 *  qbmove_communications.cpp.
**/

//=================================================================     includes

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <stdint.h>
#include <ctype.h>
#include <time.h>

#if (defined(_WIN32) || defined(_WIN64))
    #include <windows.h>
#endif

#if !(defined(_WIN32) || defined(_WIN64))
    #include <unistd.h>  /* UNIX standard function definitions */
    #include <fcntl.h>   /* File control definitions */
    #include <errno.h>   /* Error number definitions */
    #include <termios.h> /* POSIX terminal control definitions */
    #include <sys/ioctl.h>
    #include <dirent.h>
    #include <sys/time.h>
    #include <stdlib.h>
#endif

#if !(defined(_WIN32) || defined(_WIN64)) && !(defined(__APPLE__))
    #include <linux/serial.h>
#endif

#include "qbmove_communications.h"
#include "commands.h"


//=================================================================     #defines



#if (defined(_WIN32) || defined(_WIN64))
    // windows stuff

    #define usleep(X) Sleep((X) / 1000)
#elif (defined(__APPLE__))
    // apple stuff
#else
    // linux stuff
#endif


#define BUFFER_SIZE 500    ///< Size of buffers that store communication packets


//#define VERBOSE                 ///< Used for debugging

//===========================================     public fuctions implementation

/// @cond C_FILES

////////////////


#ifndef HEXDUMP_COLS
#define HEXDUMP_COLS 8
#endif

void hexdump(void *mem, unsigned int len)
{
    unsigned int i, j;

    for(i = 0; i < len + ((len % HEXDUMP_COLS) ? (HEXDUMP_COLS - len % HEXDUMP_COLS) : 0); i++) {
        // print offset
        if(i % HEXDUMP_COLS == 0) {
            printf("0x%06x: ", i);
        }

        // print hex data
        if(i < len) {
            printf("%02x ", 0xFF & ((char*)mem)[i]);
        } else {
            // end of block, just aligning for ASCII dump
            printf("   ");
        }

        // print ASCII dump
        if(i % HEXDUMP_COLS == (HEXDUMP_COLS - 1)) {
            for(j = i - (HEXDUMP_COLS - 1); j <= i; j++) {
                if(j >= len) {
                    // end of block, not really printing
                    putchar(' ');
                } else if(isprint(((char*)mem)[j])) {
                    // printable char
                    putchar(0xFF & ((char*)mem)[j]);
                } else {
                    // other chaR
                    putchar('.');
                }
            }
            putchar('\n');
        }
    }
}

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

    for(i = 1; i < 10; ++i) {
        strcpy(list_of_ports[i], "");
        sprintf(aux_string, "COM%d", i);
        port = CreateFile(aux_string, GENERIC_WRITE|GENERIC_READ,
                0, NULL, OPEN_EXISTING, 0, NULL);

        if( port != INVALID_HANDLE_VALUE) {
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

    while ( ( directory_p = readdir(directory) ) && i < 10 ) {
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


void openRS485(comm_settings *comm_settings_t, const char *port_s, int BAUD_RATE)
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

    if (comm_settings_t->file_handle == INVALID_HANDLE_VALUE) {
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
    cts.ReadIntervalTimeout         = 0;                    // msec
    // ReadTimeout = Constant + Multiplier * Nwritten
    cts.ReadTotalTimeoutMultiplier  = 0;                    // msec
    cts.ReadTotalTimeoutConstant    = 100;                  // msec
    // WriteTimeout = Constant + Multiplier * Nwritten
    cts.WriteTotalTimeoutConstant   = 100;                  // msec
    cts.WriteTotalTimeoutMultiplier = 0;                    // msec
    SetCommTimeouts(comm_settings_t->file_handle, &cts);


    return;
error:

    if (comm_settings_t->file_handle != INVALID_HANDLE_VALUE){
        CloseHandle(comm_settings_t->file_handle);
    }

//////////////////////////////   UNIX CODE   //////////////////////////////
#else

    struct termios options;

    comm_settings_t->file_handle =
        open(port_s, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if(comm_settings_t->file_handle == -1) {
        goto error;
    }

    // prevent multiple openings
    if (ioctl(comm_settings_t->file_handle, TIOCEXCL) == -1) {
        goto error;
    }

    // set communication as BLOCKING

    if(fcntl(comm_settings_t->file_handle, F_SETFL, 0) == -1) {
        goto error;
    }

    if (tcgetattr(comm_settings_t->file_handle, &options) == -1) {
        goto error;
    }

    // set baud rate
    cfsetispeed(&options, BAUD_RATE);
    cfsetospeed(&options, BAUD_RATE);

#if (defined __APPLE__)

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
    options.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR);

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

#else
    cfmakeraw(&options);

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    struct serial_struct serinfo;

    ioctl(comm_settings_t->file_handle, TIOCGSERIAL, &serinfo);
    serinfo.flags |= ASYNC_LOW_LATENCY;
    ioctl(comm_settings_t->file_handle, TIOCSSERIAL, &serinfo);
#endif

    // save changes
    if (tcsetattr(comm_settings_t->file_handle, TCSANOW, &options) == -1) {
        goto error;
    }

    return;

error:
    if (comm_settings_t->file_handle != -1) {
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

    while((n_bytes < 4) && ( timevaldiff(&start, &now) < 4000)) {
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

    while((n_bytes < package_size) && ( timevaldiff(&start, &now) < 4000)) {
        gettimeofday(&now, NULL);
        ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
    }

    if (!read(comm_settings_t->file_handle, data_in, package_size)) {
        return -1;
    }

#endif

    // Control checksum
    if (checksum ( (char *) data_in, package_size - 1) != (char) data_in[package_size - 1]) {
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
    unsigned char data_out[BUFFER_SIZE];        // output data buffer

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;                     // for serial port access
    unsigned char package_out[BUFFER_SIZE];
    int aux_int;
    int z = 0;
    DWORD n_bytes_in;
    COMMTIMEOUTS cts_old, cts;                  // for serial port configuration

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

    for(id = 1; id < 255; ++id) {
        list_of_ids[id] = 0;

        data_out[0] = ':';
        data_out[1] = ':';
        data_out[2] = (unsigned char) id;
        data_out[3] = 2;
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

        while(n_bytes_in) {
            ReadFile(  comm_settings_t->file_handle, package_out, 1, &n_bytes_in, NULL);
            aux_int |= n_bytes_in;

            memcpy(package_in + z, package_out, n_bytes_in);
            z += n_bytes_in;
        }

        if(aux_int) {
            list_of_ids[h] = package_in[2];
            h++;
        }

#else

        usleep(3000);
        ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);

        if (n_bytes >= 6) {
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
    DWORD n_bytes_out;                  // for serial port access
    DWORD n_bytes_in;                   // for serial port access
    unsigned char aux;
    int i = 0;
#else
    const int size = 512;
    int bytes;
    int count = 0;
    char aux_buffer[size];
#endif

    auxstring[0] = '?';
    auxstring[1] = 13;
    auxstring[2] = 10;

#if (defined(_WIN32) || defined(_WIN64))
    WriteFile(comm_settings_t->file_handle, auxstring, 3, &n_bytes_out, NULL);
    n_bytes_in = 1;

    Sleep(200);


    while(n_bytes_in) {
        ReadFile(comm_settings_t->file_handle, &aux, 1, &n_bytes_in, NULL);
        if(n_bytes_in)
            buffer[i] = aux;
        i++;
    }
#else
    write(comm_settings_t->file_handle, auxstring, 3);
    usleep(200000);

    while(1) {
        usleep(50000);
        if(ioctl(comm_settings_t->file_handle, FIONREAD, &bytes) < 0)
            break;
        if(bytes == 0)
            break;

        if(bytes > size)
            bytes = size;

        read(comm_settings_t->file_handle, aux_buffer, bytes);

        strncpy(buffer + count, aux_buffer, bytes);

        count += bytes;
    }

    strcpy(buffer + count, "\0");

#endif
}

//==============================================================================
//                                                                      commPing
//==============================================================================

int commPing(comm_settings *comm_settings_t, int id)
{
    char package_out[BUFFER_SIZE];      // output data buffer
    char package_in[BUFFER_SIZE];       // output data buffer
    int package_in_size;

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;                 // for serial port access
#else
    int n_bytes;
#endif

//=================================================		preparing packet to send

    package_out[0] = ':';
    package_out[1] = ':';
    package_out[2] = (unsigned char) id;
    package_out[3] = 2;
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


void commActivate(comm_settings *comm_settings_t, int id, char activate) {

    char data_out[BUFFER_SIZE];             // output data buffer

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;             // for serial port access
#else
    int n_bytes;
    char package_in[BUFFER_SIZE];
#endif

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 3;
    data_out[4] = CMD_ACTIVATE;                     // command
    data_out[5] = activate ? 3 : 0;
    data_out[6] = checksum(data_out + 4, 2);        // checksum

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

    char data_out[BUFFER_SIZE];         // output data buffer
    char package_in[BUFFER_SIZE];       // output data buffer
    int package_in_size;


#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;             // for serial port access
#else
    int n_bytes;
#endif


//=================================================		preparing packet to send

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 2;
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

void commSetInputs(comm_settings *comm_settings_t, int id, short int inputs[]) {

    char data_out[BUFFER_SIZE];         // output data buffer

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;             // for serial port access
#else
    unsigned char package_in[BUFFER_SIZE];
    int n_bytes;
#endif


    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 6;

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
//                                                               commSetPosStiff
//==============================================================================
// This function send reference position and stiffness to the qbmove
//==============================================================================

void commSetPosStiff(comm_settings *comm_settings_t, int id, short int inputs[]) {

    char data_out[BUFFER_SIZE];                 // output data buffer

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;                 // for serial port access
#else
    unsigned char package_in[BUFFER_SIZE];
    int n_bytes;
#endif


    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 6;

    data_out[4] = CMD_SET_POS_STIFF;               // command
    data_out[5] = ((char *) &inputs[0])[1];
    data_out[6] = ((char *) &inputs[0])[0];
    data_out[7] = ((char *) &inputs[1])[1];
    data_out[8] = ((char *) &inputs[1])[0];
    data_out[9] = checksum(data_out + 4, 5);        // checksum

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

int commGetInputs(comm_settings *comm_settings_t, int id, short int inputs[2]) {

    char data_out[BUFFER_SIZE];         // output data buffer
    char package_in[BUFFER_SIZE];       // output data buffer
    int package_in_size;


#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;         // for serial port access
#else
    int n_bytes;
#endif

//=================================================		preparing packet to send

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 2;
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

int commGetMeasurements(comm_settings *comm_settings_t, int id, short int measurements[]) {

    char data_out[BUFFER_SIZE];         // output data buffer
    char package_in[BUFFER_SIZE];       // output data buffer
    int package_in_size;

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;             // for serial port access
#else
    int n_bytes;
#endif

//=================================================		preparing packet to send

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 2;
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
//                                                               commGetCounters
//==============================================================================
// This function gets counters values from the QB Move.
//==============================================================================

int commGetCounters(comm_settings *comm_settings_t, int id, short unsigned int counters[]) {

    char data_out[BUFFER_SIZE];         // output data buffer
    char package_in[BUFFER_SIZE];       // output data buffer
    int package_in_size;

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;             // for serial port access
#else
    int n_bytes;
#endif

//=================================================     preparing packet to send

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 2;
    data_out[4] = CMD_GET_COUNTERS;             // command
    data_out[5] = CMD_GET_COUNTERS;             // checksum

#if (defined(_WIN32) || defined(_WIN64))
    WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
#else
    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
    if(n_bytes)
        read(comm_settings_t->file_handle, package_in, n_bytes);

    write(comm_settings_t->file_handle, data_out, 6);
#endif

    package_in_size = RS485read(comm_settings_t, id, package_in);
    if (package_in_size == -1) {
        return -1;
        printf("culo\n");
    }
//==============================================================     get packet

    ((char *) &counters[0])[0] = package_in[2];
    ((char *) &counters[0])[1] = package_in[1];
    ((char *) &counters[1])[0] = package_in[4];
    ((char *) &counters[1])[1] = package_in[3];
    ((char *) &counters[2])[0] = package_in[6];
    ((char *) &counters[2])[1] = package_in[5];
    ((char *) &counters[3])[0] = package_in[8];
    ((char *) &counters[3])[1] = package_in[7];

    ((char *) &counters[4])[0] = package_in[10];
    ((char *) &counters[4])[1] = package_in[9];
    ((char *) &counters[5])[0] = package_in[12];
    ((char *) &counters[5])[1] = package_in[11];
    ((char *) &counters[6])[0] = package_in[14];
    ((char *) &counters[6])[1] = package_in[13];
    ((char *) &counters[7])[0] = package_in[16];
    ((char *) &counters[7])[1] = package_in[15];

    ((char *) &counters[8])[0] = package_in[18];
    ((char *) &counters[8])[1] = package_in[17];
    ((char *) &counters[9])[0] = package_in[20];
    ((char *) &counters[9])[1] = package_in[19];
    ((char *) &counters[10])[0] = package_in[22];
    ((char *) &counters[10])[1] = package_in[21];
    ((char *) &counters[11])[0] = package_in[24];
    ((char *) &counters[11])[1] = package_in[23];

    ((char *) &counters[12])[0] = package_in[26];
    ((char *) &counters[12])[1] = package_in[25];
    ((char *) &counters[13])[0] = package_in[28];
    ((char *) &counters[13])[1] = package_in[27];
    ((char *) &counters[14])[0] = package_in[30];
    ((char *) &counters[14])[1] = package_in[29];
    ((char *) &counters[15])[0] = package_in[32];
    ((char *) &counters[15])[1] = package_in[31];

    ((char *) &counters[16])[0] = package_in[34];
    ((char *) &counters[16])[1] = package_in[33];
    ((char *) &counters[17])[0] = package_in[36];
    ((char *) &counters[17])[1] = package_in[35];
    ((char *) &counters[18])[0] = package_in[38];
    ((char *) &counters[18])[1] = package_in[37];
    ((char *) &counters[19])[0] = package_in[40];
    ((char *) &counters[19])[1] = package_in[39];

    return 0;
}

//==============================================================================
//                                                          commGetCurrents
//==============================================================================
// This function gets currents from the QB Move.
//==============================================================================

int commGetCurrents(comm_settings *comm_settings_t, int id, short int currents[2]) {

    char data_out[BUFFER_SIZE];         // output data buffer
    char package_in[BUFFER_SIZE];       // output data buffer
    int package_in_size;

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;             // for serial port access
#else
    int n_bytes;
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
//                                                          commGetEmg
//==============================================================================
// This function gets Emg signals from the Soft Hand
//==============================================================================

int commGetEmg(comm_settings *comm_settings_t, int id, short int emg[2]) {

    char data_out[BUFFER_SIZE];         // output data buffer
    char package_in[BUFFER_SIZE];       // output data buffer
    int package_in_size;

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;             // for serial port access
#else
    int n_bytes;
#endif

//=================================================     preparing packet to send

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 2;
    data_out[4] = CMD_GET_EMG;             // command
    data_out[5] = CMD_GET_EMG;             // checksum

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

//===============================================================     get packet

    ((char *) &emg[0])[0] = package_in[2];
    ((char *) &emg[0])[1] = package_in[1];

    ((char *) &emg[1])[0] = package_in[4];
    ((char *) &emg[1])[1] = package_in[3];

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

    char data_out[BUFFER_SIZE];         // output data buffer
    char package_in[BUFFER_SIZE];       // output data buffer
    int package_in_size;

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;             // for serial port access
#else
    int n_bytes;
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

//===============================================================     get packet

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
//                                                             commGetVelocities
//==============================================================================
// This function gets measurements from the QB Move.
//==============================================================================

int commGetVelocities(comm_settings *comm_settings_t, int id, short int measurements[]) {

    char data_out[BUFFER_SIZE];         // output data buffer
    char package_in[BUFFER_SIZE];       // output data buffer
    int package_in_size;

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;             // for serial port access
#else
    int n_bytes;
#endif

//=================================================     preparing packet to send

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 2;
    data_out[4] = CMD_GET_VELOCITIES;             // command
    data_out[5] = CMD_GET_VELOCITIES;             // checksum

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
//                                                          commGetAccelerations
//==============================================================================
// This function gets measurements from the QB Move.
//==============================================================================

int commGetAccelerations(comm_settings *comm_settings_t, int id, short int measurements[]) {

    char data_out[BUFFER_SIZE];         // output data buffer
    char package_in[BUFFER_SIZE];       // output data buffer
    int package_in_size;

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;             // for serial port access
#else
    int n_bytes;
#endif

//=================================================     preparing packet to send

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 2;
    data_out[4] = CMD_GET_ACCEL;             // command
    data_out[5] = CMD_GET_ACCEL;             // checksum

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
//                                                                   commGetInfo
//==============================================================================
// This function gets a string of information from the QB Move.
//==============================================================================

int commGetInfo(comm_settings *comm_settings_t, int id, short int info_type, char *buffer) {

    char data_out[BUFFER_SIZE];             // output data buffer

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;                 // for serial port access
    DWORD n_bytes_in = 0;
    unsigned char aux;
    int i = 0;
#else
    int bytes;
    int count = 0;
    const int size = 512;
    char aux_buffer[size];
#endif

    strcpy(buffer, "");

//=================================================		preparing packet to send

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 4;
    data_out[4] = CMD_GET_INFO;                        // command
    data_out[5] = ((unsigned char *) &info_type)[1];   // parameter type
    data_out[6] = ((unsigned char *) &info_type)[0];   // parameter type
    data_out[7] = checksum(data_out + 4, 3);           // checksum


#if (defined(_WIN32) || defined(_WIN64))
    WriteFile(comm_settings_t->file_handle, data_out, 8, &package_size_out, NULL);

    n_bytes_in = 1;

    Sleep(200);

    while(n_bytes_in) {
        ReadFile(comm_settings_t->file_handle, &aux, 1, &n_bytes_in, NULL);
        if(n_bytes_in)
            buffer[i] = aux;
        i++;
    }

#else

    write(comm_settings_t->file_handle, data_out, 8);

    usleep(200000);

    while(1) {
        usleep(50000);
        if(ioctl(comm_settings_t->file_handle, FIONREAD, &bytes) < 0)
            break;
        if(bytes == 0)
            break;

        if(bytes > size)
            bytes = size;

        read(comm_settings_t->file_handle, aux_buffer, bytes);

        strncpy(buffer + count, aux_buffer, bytes);

        count += bytes;
    }

    strcpy(buffer + count, "\0");
#endif

    return 0;
}



//==============================================================================
//                                                                commBootloader
//==============================================================================
//  This function launches bootloader
//==============================================================================


int commBootloader(comm_settings *comm_settings_t, int id) {

    char data_out[BUFFER_SIZE];             // output data buffer
    char package_in[BUFFER_SIZE];
    int package_in_size;

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;                 // for serial port access
#else
    int n_bytes;
#endif

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 2;
    data_out[4] = CMD_BOOTLOADER;           // command
    data_out[5] = CMD_BOOTLOADER;           // checksum


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
//                                                                 commCalibrate
//==============================================================================
//  This function starts the calibration of the stiffness
//==============================================================================


int commCalibrate(comm_settings *comm_settings_t, int id) {

    char data_out[BUFFER_SIZE];             // output data buffer
    char package_in[BUFFER_SIZE];
    int package_in_size;

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;                 // for serial port access
#else
    int n_bytes;
#endif

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 2;
    data_out[4] = CMD_CALIBRATE;            // command
    data_out[5] = CMD_CALIBRATE;            // checksum

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
//                                                             commHandCalibrate
//==============================================================================
//  This function start the hand calibration
//==============================================================================


int commHandCalibrate(comm_settings *comm_settings_t, int id, short int speed, short int repetitions) {

    char data_out[BUFFER_SIZE];             // output data buffer
    char package_in[BUFFER_SIZE];
    int package_in_size;

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;                 // for serial port access
#else
    int n_bytes;
#endif

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 6;
    data_out[4] = CMD_CALIBRATE;                // command
    data_out[5] = ((char *) &speed)[1];
    data_out[6] = ((char *) &speed)[0];
    data_out[7] = ((char *) &repetitions)[1];
    data_out[8] = ((char *) &repetitions)[0];
    data_out[9] = checksum(data_out + 4, 5);    // checksum

#if (defined(_WIN32) || defined(_WIN64))
    WriteFile(comm_settings_t->file_handle, data_out, 10, &package_size_out, NULL);
#else
    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
    if(n_bytes)
        read(comm_settings_t->file_handle, package_in, n_bytes);

    write(comm_settings_t->file_handle, data_out, 10);
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
                    unsigned short num_of_values ) {

    char data_out[BUFFER_SIZE];     // output data buffer
    char package_in[BUFFER_SIZE];
    int package_in_size;
    void *value;
    unsigned short int value_size, i, h;

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;         // for serial port access
#else
    int n_bytes;
#endif

    switch (type){
        case PARAM_ID:
            value       = (unsigned char *) values;
            value_size  = 1;
            break;

        case PARAM_PID_CONTROL:
            value       = (float *) values;
            value_size  = 4;
            break;

        case PARAM_PID_CURR_CONTROL:
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

        case PARAM_CONTROL_MODE:
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
            value       = (unsigned char *) values;
            value_size  = 1;
            break;

        case PARAM_POS_LIMIT:
            value       = (int *) values;
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

        case PARAM_CURRENT_LIMIT:
            value       = (int16_t *) values;
            value_size  = 2;
            break;

        case PARAM_EMG_CALIB_FLAG:
            value       = (int8_t *) values;
            value_size  = 1;
            break;

        case PARAM_EMG_THRESHOLD:
            value       = (int16_t *) values;
            value_size  = 2;
            break;

        case PARAM_EMG_MAX_VALUE:
            value       = (int32_t *) values;
            value_size  = 4;
            break;

        case PARAM_EMG_SPEED:
            value       = (uint8_t *) values;
            value_size  = 1;
            break;

        case PARAM_DOUBLE_ENC_ON_OFF:
            value       = (uint8_t *) values;
            value_size  = 1;
            break;

        case PARAM_MOT_HANDLE_RATIO:
            value       = (int8_t *) values;
            value_size  = 1;
            break;

        case PARAM_MOTOR_SUPPLY:
            value       = (uint8_t *) values;
            value_size  = 1;
            break;

		case PARAM_DEFLECTION_CONTROL:
			value 	   = (uint8_t *) values;
			value_size = 1;
			break;

        case PARAM_CURRENT_LOOKUP:
            value       = (float *) values;
            value_size  = 4;
            break;

        case PARAM_CURR_PROP_GAIN:
            value       = (float *) values;
            value_size  = 4;
            break;

        case PARAM_CURR_SAT:
            value       = (int16_t *) values;
            value_size  = 2;
            break;

        case PARAM_CURR_DEAD_ZONE:
            value       = (int16_t *) values;
            value_size  = 2;
            break;

        case PARAM_CUFF_ACTIVATION_FLAG:
            value       = (uint8_t *) values;
            value_size  = 1;
            break;

        case PARAM_POWER_TENSION:
            value       = (uint16_t *) values;
            value_size  = 2;
            break;

        default:
            return -1;
    }


    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 4 + num_of_values * value_size;

    data_out[4] = CMD_SET_PARAM;                // command
    data_out[5] = ((char *) &type)[1];          // parameter type
    data_out[6] = ((char *) &type)[0];          // parameter type

    for(h = 0; h < num_of_values; ++h) {
        for(i = 0; i < value_size; ++i) {
            data_out[ h * value_size +  7 + i ] =
                ((char *) value)[ h * value_size + value_size - i - 1 ];
        }
    }

    data_out[ 7 + num_of_values * value_size ] =
            checksum( data_out + 4, 3 + num_of_values * value_size );


#if (defined(_WIN32) || defined(_WIN64))
    WriteFile(comm_settings_t->file_handle, data_out, 8 + num_of_values * value_size, &package_size_out, NULL);
#else
    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
    if(n_bytes)
        read(comm_settings_t->file_handle, package_in, n_bytes);

    write(comm_settings_t->file_handle, data_out, 8 + num_of_values * value_size);
#endif

    package_in_size = RS485read(comm_settings_t, id, package_in);

    if ( (package_in_size == -1) || (package_in[0] == ACK_ERROR) ) {
        return -1;
    }

    if (package_in[0] == ACK_OK) {
        return 0;
    } else {
        return -1;
    }
}

//==============================================================================
//                                                                 commGetParams
//==============================================================================

int commGetParam(comm_settings *comm_settings_t,
                    int id,
                    enum qbmove_parameter type,
                    void *values,
                    unsigned short num_of_values ) {

    int package_in_size;
    char data_out[BUFFER_SIZE];         // output data buffer
    char package_in[BUFFER_SIZE];
    unsigned short int value_size;

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;             // for serial port access
#else
    int n_bytes;
#endif

    switch (type){
        case PARAM_ID:
            value_size = 1;
            break;

        case PARAM_PID_CONTROL:
            value_size = 4;
            break;

        case PARAM_PID_CURR_CONTROL:
            value_size = 4;
            break;

        case PARAM_STARTUP_ACTIVATION:
            value_size = 1;
            break;

        case PARAM_INPUT_MODE:
            value_size = 1;
            break;

        case PARAM_CONTROL_MODE:
            value_size = 1;
            break;

        case PARAM_POS_RESOLUTION:
            value_size = 1;
            break;

        case PARAM_MEASUREMENT_OFFSET:
            value_size = 2;
            break;

        case PARAM_MEASUREMENT_MULTIPLIER:
            value_size = 4;
            break;

        case PARAM_POS_LIMIT_FLAG:
            value_size = 1;
            break;

        case PARAM_POS_LIMIT:
            value_size = 4;
            break;

        case PARAM_MAX_STEP_POS:
            value_size = 4;
            break;

        case PARAM_MAX_STEP_NEG:
            value_size = 4;
            break;

        case PARAM_CURRENT_LIMIT:
            value_size = 2;
            break;

        case PARAM_EMG_CALIB_FLAG:
            value_size = 1;
            break;

        case PARAM_EMG_THRESHOLD:
            value_size = 2;
            break;

        case PARAM_EMG_MAX_VALUE:
            value_size = 4;
            break;

        case PARAM_EMG_SPEED:
            value_size = 1;
            break;

        case PARAM_DOUBLE_ENC_ON_OFF:
            value_size = 1;
            break;

        case PARAM_MOT_HANDLE_RATIO:
            value_size = 1;
            break;

        case PARAM_MOTOR_SUPPLY:
            value_size = 1;
            break;
		
		case PARAM_DEFLECTION_CONTROL:
			value_size = 1;
			break;
		
        case PARAM_CURRENT_LOOKUP:
            value_size = 4; 
            break;

        case PARAM_CURR_PROP_GAIN:
            value_size = 4;
            break;

        case PARAM_CURR_SAT:
            value_size = 2;
            break;

        case PARAM_CURR_DEAD_ZONE:
            value_size = 2;
            break;

        case PARAM_CUFF_ACTIVATION_FLAG:
            value_size = 1;
            break;

        default:
            break;
    }

//================================================      preparing packet to send

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 4;
    data_out[4] = CMD_GET_PARAM;                // command
    data_out[5] = ((char *) &type)[1];          // parameter type
    data_out[6] = ((char *) &type)[0];          // parameter type

    data_out[7] = checksum (data_out + 4, 3);   // checksum

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

    for(h = 0; h < num_of_values; ++h) {
        for(i = 0; i < value_size; ++i) {
            ((char *) values)
                [ h * value_size + value_size - i - 1 ] =
                package_in[ h * value_size + i + 1 ];
        }
    }

    return 0;
}

//==============================================================================
//                                                               commStoreParams
//==============================================================================

int commStoreParams( comm_settings *comm_settings_t, int id ) {

    char data_out[BUFFER_SIZE];         // output data buffer
    char package_in[BUFFER_SIZE];
    int package_in_size;

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;             // for serial port access
#else
    int n_bytes;
#endif

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 2;
    data_out[4] = CMD_STORE_PARAMS;         // command
    data_out[5] = CMD_STORE_PARAMS;         // checksum

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

    if ( (package_in_size == -1) || (package_in[0] == ACK_ERROR) ) {
        return -1;
    }

    if (package_in[0] == ACK_OK) {
        return 0;
    } else {
        return -1;
    }
}

//==============================================================================
//                                                        commStoreDefaultParams
//==============================================================================

int commStoreDefaultParams( comm_settings *comm_settings_t, int id ) {

    char data_out[BUFFER_SIZE];         // output data buffers
    char package_in[BUFFER_SIZE];
    int package_in_size;

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;             // for serial port access
#else
    int n_bytes;
#endif

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 2;
    data_out[4] = CMD_STORE_DEFAULT_PARAMS;         // command
    data_out[5] = CMD_STORE_DEFAULT_PARAMS;         // checksum

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
//                                                             commRestoreParams
//==============================================================================

int commRestoreParams( comm_settings *comm_settings_t, int id ) {

    char data_out[BUFFER_SIZE];         // output data buffer
    char package_in[BUFFER_SIZE];
    int package_in_size;

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;             // for serial port access
#else
    int n_bytes;
#endif

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 2;

    data_out[4] = CMD_RESTORE_PARAMS;           // command
    data_out[5] = CMD_RESTORE_PARAMS;           // checksum

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

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;                 // for serial port access
#else
    int n_bytes;
#endif

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 2;

    data_out[4] = CMD_INIT_MEM;             // command
    data_out[5] = CMD_INIT_MEM;             // checksum

#if (defined(_WIN32) || defined(_WIN64))
    WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
#else
    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
    if(n_bytes)
        read(comm_settings_t->file_handle, package_in, n_bytes);

    write(comm_settings_t->file_handle, data_out, 6);
#endif

    usleep(300000);
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

char checksum ( char * data_buffer, int data_length ) {

    int i;
    char checksum = 0x00;

    for(i = 0; i < data_length; ++i) {
       checksum = checksum ^ data_buffer[i];
    }

    return checksum;
}

/// @endcond

/* [] END OF FILE */