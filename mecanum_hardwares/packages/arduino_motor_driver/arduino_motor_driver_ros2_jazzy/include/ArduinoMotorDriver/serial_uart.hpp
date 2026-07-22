// Internal UART for the Jetson nano
//
// Copyright (c) 2024 Kss
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// v0.1    042524
//

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef   __SERIAL_UART_HPP
#define   __SERIAL_UART_HPP

// C library headers
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <errno.h> // Error integer and strerror() function

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>


#include <string>

/* Define macros */
#define UART_PORT_1             0U 
#define UART_PORT_2             1U 

#define UART_DATA_BIT_5         5U
#define UART_DATA_BIT_6         6U
#define UART_DATA_BIT_7         7U
#define UART_DATA_BIT_8         8U

#define UART_STOP_BIT_1         1U
#define UART_STOP_BIT_2         2U

#define UART_PARATY_NONE        0U
#define UART_PARATY_ODD         1U
#define UART_PARATY_EVEN        2U

#define UART_HWFLOW_CONTROL_ENABLE        1U
#define UART_HWFLOW_CONTROL_DISABLE       0U

#define UART_CONONICAL_MODE_ENABLE         1U
#define UART_CONONICAL_MODE_DISABLE        0U

#define UART_MAX_BUF_SIZE             255U

#define SERIAL_DATA_SLEEP_TIME         10U  //us

class SerialComm
{
    private:
        struct termios huart_;
        int ttyfd_ = 0;
        std::string ttydir_;

    public:
        bool is_init_ = false;
    private:

    public:
        SerialComm();
        virtual ~SerialComm();

        int Initialize(std::string ttydir,
                int baudrate,
                int stopbit,
                int databits,
                bool hwflow,
                bool canonicalmode,
                int paraty);
                
        int SendData(unsigned char *msg, const size_t msg_length);
        int ReadData(unsigned char *recv_msg, const size_t recv_msg_length);
        int ReadDataWithTimeout(unsigned char *recv_msg, const size_t recv_msg_length, size_t timeout_val);

        int SerialClose();

};

int get_baud(int baud);


#endif /* __JETSON_UART_HPP */