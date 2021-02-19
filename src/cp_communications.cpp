
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
 *  \file       cp_communications.cpp
 *
 *  \brief      Library of functions for serial port communication with a
 *              board in addition to standard qbmove_communications
 *
 *  \details
 *
 *  Check the \ref cp_communications.h "cp_communications.h" file
 *  for a complete description of the public functions implemented in
 *  cp_communications.cpp.
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

#if (defined(__APPLE__))
    #include <IOKit/IOKitLib.h>
    #include <IOKit/serial/IOSerialKeys.h>
    #include <IOKit/serial/ioss.h>
    #include <IOKit/IOBSD.h>
#endif

#include "cp_communications.h"
#include "cp_commands.h"


#define BUFFER_SIZE 500    ///< Size of buffers that store communication packets

//===========================================     public fuctions implementation

/// @cond C_FILES


//==============================================================================
//   	                                                      commGetImuReadings
//==============================================================================
// Retrieve accelerometers, gyroscopes and magnetometers readings.
//==============================================================================

int commGetImuReadings(comm_settings *comm_settings_t, int id, uint8_t* imu_table, uint8_t* imus_magcal, int n_imu, float* imu_values){
	
	char data_out[BUFFER_SIZE];         // output data buffer
    char package_in[BUFFER_SIZE];       // output data buffer
    int package_in_size;
	float acc_sf = 0, gyro_sf = 0, mag_sf = 0;
	float temp_sf = 0, temp_off = 0, temp_div = 0;	
	char* values;
	int c = 0;
	float aux_float[4];
	int16_t aux_si;
	float aux_fl;

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
    data_out[4] = CMD_GET_IMU_READINGS;             // command
    data_out[5] = CMD_GET_IMU_READINGS;             // checksum

#if (defined(_WIN32) || defined(_WIN64))
    WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
#else
    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
    if(n_bytes)
        read(comm_settings_t->file_handle, package_in, n_bytes);

    write(comm_settings_t->file_handle, data_out, 6);
#endif

	memset(package_in, 0, sizeof(package_in));
	
    package_in_size = RS485read(comm_settings_t, id, package_in);
    if (package_in_size < 0)
        return package_in_size;
	
	acc_sf 	= 0.000061037;			// Ticks to G
	gyro_sf = 0.007629627 * 8;		// Ticks to deg/s with FS +/- 2000 °/s
	mag_sf 	= 0.1465;				// Ticks to uT
	
	temp_sf = 0.00294118; // 1/340 //0.001426;
	temp_off = 36.53; //21.6;
	temp_div = 2.0;
	
	values = &package_in[1];
/*	
 	printf("SIZE: %d\n", package_in_size);
	for (int i=0; i< package_in_size; i++) {
		printf("%d,", values[i]);
	}
	printf("\n"); 
*/	
	for (int i=0; i < n_imu; i++){

		if (values[c] == ':'){
			
			
			if (imu_table[5*i + 0]) {
				((char *) &aux_si)[0] = values[c+2];
				((char *) &aux_si)[1] = values[c+1];
				aux_float[0] = (float) ( aux_si * acc_sf);

				((char *) &aux_si)[0] = values[c+4];
				((char *) &aux_si)[1] = values[c+3];
				aux_float[1] = (float) ( aux_si * acc_sf);
				((char *) &aux_si)[0] = values[c+6];
				((char *) &aux_si)[1] = values[c+5];
				aux_float[2] = (float) ( aux_si * acc_sf);

				imu_values[(3*3+4+1)*i] 	= aux_float[0];
				imu_values[(3*3+4+1)*i+1] = aux_float[1];
				imu_values[(3*3+4+1)*i+2] = aux_float[2];
				c += 6;
			}
			if (imu_table[5*i + 1]) {
				((char *) &aux_si)[0] = values[c+2];
				((char *) &aux_si)[1] = values[c+1];
				aux_float[0] = (float) ( aux_si * gyro_sf);
				((char *) &aux_si)[0] = values[c+4];
				((char *) &aux_si)[1] = values[c+3];
				aux_float[1] = (float) ( aux_si * gyro_sf);
				((char *) &aux_si)[0] = values[c+6];
				((char *) &aux_si)[1] = values[c+5];
				aux_float[2] = (float) ( aux_si * gyro_sf);

				imu_values[(3*3+4+1)*i+3] = aux_float[0];
				imu_values[(3*3+4+1)*i+4] = aux_float[1];
				imu_values[(3*3+4+1)*i+5] = aux_float[2];
				c += 6;
			}
			if (imu_table[5*i + 2]) {
				((char *) &aux_si)[0] = values[c+2];
				((char *) &aux_si)[1] = values[c+1];
				aux_float[0] = (float) (aux_si * mag_sf * (float)imus_magcal[3*i+0]);
				((char *) &aux_si)[0] = values[c+4];
				((char *) &aux_si)[1] = values[c+3];
				aux_float[1] = (float) (aux_si * mag_sf * (float)imus_magcal[3*i+1]);
				((char *) &aux_si)[0] = values[c+6];
				((char *) &aux_si)[1] = values[c+5];
				aux_float[2] = (float) (aux_si * mag_sf * (float)imus_magcal[3*i+2]);
				
				imu_values[(3*3+4+1)*i+6] = -aux_float[1];
				imu_values[(3*3+4+1)*i+7] = -aux_float[0];
				imu_values[(3*3+4+1)*i+8] = aux_float[2];
				c += 6;
			}
			
			if (imu_table[5*i + 3]) {

				((char *) &aux_fl)[0] = values[c+4];
				((char *) &aux_fl)[1] = values[c+3];
				((char *) &aux_fl)[2] = values[c+2];
				((char *) &aux_fl)[3] = values[c+1];
				aux_float[0] = (float) (aux_fl);
				((char *) &aux_fl)[0] = values[c+8];
				((char *) &aux_fl)[1] = values[c+7];
				((char *) &aux_fl)[2] = values[c+6];
				((char *) &aux_fl)[3] = values[c+5];
				aux_float[1] = (float) (aux_fl);
				((char *) &aux_fl)[0] = values[c+12];
				((char *) &aux_fl)[1] = values[c+11];
				((char *) &aux_fl)[2] = values[c+10];
				((char *) &aux_fl)[3] = values[c+9];
				aux_float[2] = (float) (aux_fl);
				((char *) &aux_fl)[0] = values[c+16];
				((char *) &aux_fl)[1] = values[c+15];
				((char *) &aux_fl)[2] = values[c+14];
				((char *) &aux_fl)[3] = values[c+13];
				aux_float[3] = (float) (aux_fl);

				imu_values[(3*3+4+1)*i+9]  = aux_float[0];
				imu_values[(3*3+4+1)*i+10] = aux_float[1];
				imu_values[(3*3+4+1)*i+11] = aux_float[2];
				imu_values[(3*3+4+1)*i+12] = aux_float[3];
				c += 16;

			}
			
			if (imu_table[5*i + 4]) {
				((char *) &aux_si)[0] = values[c+2];
				((char *) &aux_si)[1] = values[c+1];
				aux_float[0] = (float) (aux_si * (float)temp_sf + temp_off) / temp_div;
				
				imu_values[(3*3+4+1)*i+13] = aux_float[0];
				c += 2;
			}
	
			//printf("\n");
			c = c + 1;
		}
		if (values[c] == ':')
			c = c + 1;
		else {
			break;
			//printf("Break at %d\n", c);
		}	
	}
	return 0;
}

//==============================================================================
//                                                           commGetIMUParamList
//==============================================================================
// This function gets the list of parameters of the device or sets one of them
//==============================================================================

int commGetIMUParamList(comm_settings *comm_settings_t, int id, unsigned short index,
                    void *values, unsigned short value_size, unsigned short num_of_values,
                    uint8_t *param_buffer) {

    char data_out[BUFFER_SIZE];         // output data buffer
    char package_in[BUFFER_SIZE];
    int package_in_size;

#if (defined(_WIN32) || defined(_WIN64))
    DWORD package_size_out;             // for serial port access
    DWORD n_bytes_in = 0;
    unsigned char aux;
    int i = 0;
#else
    int bytes;
    int count = 0;
    const int size = 512;
    int8_t aux_buffer[size];
#endif

//================================================      preparing packet to send

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 4 + num_of_values * value_size;

    data_out[4] = CMD_GET_IMU_PARAM;             // command
    data_out[5] = ((char *) &index)[1];          // parameter type
    data_out[6] = ((char *) &index)[0];          // parameter type

    for(int h = 0; h < num_of_values; ++h) {
        for(int i = 0; i < value_size; ++i) {
            data_out[ h * value_size +  7 + i ] =
                ((char *) values)[ h * value_size + value_size - i - 1 ];
        }
    }

    data_out[ 7 + num_of_values * value_size ] =
            checksum( data_out + 4, 3 + num_of_values * value_size );

//==============================================================  get packet

    if(!index) {        //The package must be read only when asking for all parameters information
        #if (defined(_WIN32) || defined(_WIN64))
            WriteFile(comm_settings_t->file_handle, data_out, 8, &package_size_out, NULL);

            n_bytes_in = 1;

            Sleep(200);

            while(n_bytes_in) {
                if (!ReadFile(comm_settings_t->file_handle, &aux, 1, &n_bytes_in, NULL)) {
					return -1;
				}
                if(n_bytes_in)
                    param_buffer[i] = aux;
                i++;
            }

        #else

            write(comm_settings_t->file_handle, data_out, 8);

            usleep(200000);
            while(1) {
                usleep(50000);
                if(ioctl(comm_settings_t->file_handle, FIONREAD, &bytes) < 0) {
                    break;
                }
                if(bytes == 0) {
                    break;
                }
                if(bytes > size)
                    bytes = size;

				if (!read(comm_settings_t->file_handle, aux_buffer, bytes)) {
					return -1;
				}

                memcpy(param_buffer + count, aux_buffer, bytes);

                count += bytes;
            }

        #endif
		
		
		/// Check for existing command
		if (param_buffer[4] != CMD_GET_IMU_PARAM) {
			return -1;
		}
    }

    else {      //Param setting 

        #if (defined(_WIN32) || defined(_WIN64))
            WriteFile(comm_settings_t->file_handle, data_out, 8 + num_of_values * value_size, &package_size_out, NULL);
        #else
            ioctl(comm_settings_t->file_handle, FIONREAD, &bytes);
            if(bytes)
                read(comm_settings_t->file_handle, package_in, bytes);

            write(comm_settings_t->file_handle, data_out, 8 + num_of_values * value_size);
        #endif

            package_in_size = RS485read(comm_settings_t, id, package_in);

            if ( (package_in_size < 0) || (package_in[0] == ACK_ERROR) ) {
                return package_in_size;
            }

            if (package_in[0] == ACK_OK) {
                return 0;
            } else {
                return -1;
            }
    }		
	
    return 0;
}

//==============================================================================
//                                                            commGetEncoderConf
//==============================================================================
// This function gets Encoder configuration form the board.
//==============================================================================

int commGetEncoderConf(comm_settings *comm_settings_t, int id, uint8_t* num_enc_line_ret, uint8_t* num_enc_per_line_ret, uint8_t* enc_lin_map) {

    char data_out[BUFFER_SIZE];         // output data buffer
    char package_in[BUFFER_SIZE];       // output data buffer
    int package_in_size;
	uint8_t num_enc_line;
	uint8_t num_enc_per_line;
	
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
    data_out[4] = CMD_GET_ENCODER_CONF;             // command
    data_out[5] = CMD_GET_ENCODER_CONF;             // checksum

#if (defined(_WIN32) || defined(_WIN64))
    WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
#else
    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
    if(n_bytes)
        read(comm_settings_t->file_handle, package_in, n_bytes);

    write(comm_settings_t->file_handle, data_out, 6);
#endif

    package_in_size = RS485read(comm_settings_t, id, package_in);
    if (package_in_size < 0)
        return package_in_size;

	num_enc_line = package_in[1];
	num_enc_per_line = package_in[2];
	
//==============================================================	 get packet

	for (int i=0; i< num_enc_line; i++){
		for (int j=0; j< num_enc_per_line; j++){
			enc_lin_map[i*num_enc_per_line + j] = package_in[3+i*num_enc_per_line + j];
		}
	}

	*num_enc_line_ret = num_enc_line;
	*num_enc_per_line_ret = num_enc_per_line;
	
    return 0;
}

//==============================================================================
//                                                       commGetEncoderRawValues
//==============================================================================
// This function gets Encoder raw values form the board.
//==============================================================================

int commGetEncoderRawValues(comm_settings *comm_settings_t, int id, uint8_t enc_total, uint16_t enc_val[]) {
	
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
    data_out[4] = CMD_GET_ENCODER_RAW;             // command
    data_out[5] = CMD_GET_ENCODER_RAW;             // checksum

#if (defined(_WIN32) || defined(_WIN64))
    WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
#else
    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
    if(n_bytes)
        read(comm_settings_t->file_handle, package_in, n_bytes);

    write(comm_settings_t->file_handle, data_out, 6);
#endif

    package_in_size = RS485read(comm_settings_t, id, package_in);
    if (package_in_size < 0)
        return package_in_size;

//==============================================================	 get packet

	for (int i=0; i< enc_total; i++) {
		for (int j=0; j<=1; j++) {
			((char *) &enc_val[i])[j] = package_in[2*i + (2-j)];
		}
	}

    return 0;
}

//==============================================================================
//                                                            	  commGetADCConf
//==============================================================================
// This function gets ADC configuration form the board.
//==============================================================================

int commGetADCConf(comm_settings *comm_settings_t, int id, uint8_t* tot_adc_channels, uint8_t* adc_map) {

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
    data_out[4] = CMD_GET_ADC_CONF;             // command
    data_out[5] = CMD_GET_ADC_CONF;             // checksum

#if (defined(_WIN32) || defined(_WIN64))
    WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
#else
    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
    if(n_bytes)
        read(comm_settings_t->file_handle, package_in, n_bytes);

    write(comm_settings_t->file_handle, data_out, 6);
#endif

    package_in_size = RS485read(comm_settings_t, id, package_in);
    if (package_in_size < 0)
        return package_in_size;

	*tot_adc_channels = package_in[1];
	
//==============================================================	 get packet

	for (int i=0; i< (*tot_adc_channels); i++){
		adc_map[i] = package_in[2+i];
	}

    return 0;
}

//==============================================================================
//                                                       	 commGetADCRawValues
//==============================================================================
// This function gets ADC raw values from the board.
//==============================================================================

int commGetADCRawValues(comm_settings *comm_settings_t, int id, uint8_t num_channels, short int adc[]) {

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
    data_out[4] = CMD_GET_ADC_RAW;             // command
    data_out[5] = CMD_GET_ADC_RAW;             // checksum

#if (defined(_WIN32) || defined(_WIN64))
    WriteFile(comm_settings_t->file_handle, data_out, 6, &package_size_out, NULL);
#else
    ioctl(comm_settings_t->file_handle, FIONREAD, &n_bytes);
    if(n_bytes)
        read(comm_settings_t->file_handle, package_in, n_bytes);

    write(comm_settings_t->file_handle, data_out, 6);
#endif

    package_in_size = RS485read(comm_settings_t, id, package_in);
    if (package_in_size < 0)
        return package_in_size;

//==============================================================	 get packet

	for (int i=0; i < num_channels; i++) {
		
		((char *) &adc[i])[0] = package_in[2 + 2*i];
		((char *) &adc[i])[1] = package_in[1 + 2*i];
	}
	
	return 0;
}

//==============================================================================
//                                                                 commGetSDFile
//==============================================================================
// This function gets a file stored in SD card.
//==============================================================================

int commGetSDFile(comm_settings *comm_settings_t, int id, char* filename, char *buffer) {

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
    unsigned short file_length = strlen(filename);
//=================================================		preparing packet to send

    data_out[0] = ':';
    data_out[1] = ':';
    data_out[2] = (unsigned char) id;
    data_out[3] = 4 + file_length;
    data_out[4] = CMD_GET_SD_SINGLE_FILE;                        // command
    data_out[5] = ((char *) &file_length)[1];          // parameter type
    data_out[6] = ((char *) &file_length)[0];          // parameter type

 	for(int h = 0; h < file_length; h++) {
        
        data_out[ h +  7 ] = filename[ h ];
        
    }
    data_out[ 7 + file_length ] = checksum( data_out + 4, 3 + file_length );


#if (defined(_WIN32) || defined(_WIN64))
    WriteFile(comm_settings_t->file_handle, data_out, 8+file_length, &package_size_out, NULL);

    Sleep(200);

    n_bytes_in = 1;

    while(n_bytes_in) {
        ReadFile(comm_settings_t->file_handle, &aux, 1, &n_bytes_in, NULL);
        if(n_bytes_in)
            buffer[i] = aux;
        i++;
    }

#else

    write(comm_settings_t->file_handle, data_out, 8+file_length);

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
/// @endcond

/* [] END OF FILE */
