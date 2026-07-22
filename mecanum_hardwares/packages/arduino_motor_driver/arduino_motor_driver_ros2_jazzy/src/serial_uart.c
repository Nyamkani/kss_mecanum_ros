#include "ArduinoMotorDriver/serial_uart.h"

static struct termios huart[2];
static int ttyfd[2] = {NULL, NULL};

static char *ttyuart[] = {"/dev/ttyUSB0", "/dev/ttyUSB1"};


int initUARTPort(int port, 
                int baudrate,
                int stopbit,
                int databits,
                bool hwflow,
                bool canonicalmode,
                int paraty)
{
    char *ttyport_ = NULL;
    struct termios* huart_ = NULL;
    int *fid_ = NULL;

    //Select the hnadle
    if(port  == UART_PORT_1 || port  == UART_PORT_2)
    {
        ttyport_ = ttyuart[port];
        huart_ = &huart[port];
        fid_  = &ttyfd[port];
    }
    else
    {
        printf( "Port Range Error!\r\n" );
        
        return -1;
    }

    //------------------------------------------------
    //  OPEN THE UART
    //------------------------------------------------
    // The flags (defined in fcntl.h):
    //	Access modes (use 1 of these):
    //		O_RDONLY - Open for reading only.
    //		O_RDWR   - Open for reading and writing.
    //		O_WRONLY - Open for writing only.
    //	    O_NDELAY / O_NONBLOCK (same function)
    //               - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //                 if there is no input immediately available (instead of blocking). Likewise, write requests can also return
    //				   immediately with a failure status if the output can't be written immediately.
    //                 Caution: VMIN and VTIME flags are ignored if O_NONBLOCK flag is set.
    //	    O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.fid = open("/dev/ttyTHS1", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode


	*fid_ = open(ttyport_, O_RDWR | O_NOCTTY);
	
	if(*fid_ < 0)
	{
		printf( "%s : >> tty Open Fail, Try sudo [%s]\r\n ", strerror(EACCES), ttyport_);

		return -1;
	}
    printf( "Got Pid: [%d]\r\n ", *fid_);

	memset(huart_, 0, sizeof(*huart_ ));
	
    //------------------------------------------------
    // CONFIGURE THE UART
    //------------------------------------------------
    // flags defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html
    //	Baud rate:
    //         - B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200,
    //           B230400, B460800, B500000, B576000, B921600, B1000000, B1152000,
    //           B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //	CSIZE: - CS5, CS6, CS7, CS8
    //	CLOCAL - Ignore modem status lines
    //	CREAD  - Enable receiver
    //	IGNPAR = Ignore characters with parity errors
    //	ICRNL  - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //	PARENB - Parity enable
    //	PARODD - Odd parity (else even)
    
    
    huart_->c_cflag &= ~CSIZE;	            // Clears the mask for setting the data size

    // Set the data bits 
    switch(databits)
    {
        case UART_DATA_BIT_5:  huart_->c_cflag |=  CS5; break;
        case UART_DATA_BIT_6:  huart_->c_cflag |=  CS6; break;
        case UART_DATA_BIT_7:  huart_->c_cflag |=  CS7; break;
        case UART_DATA_BIT_8:  huart_->c_cflag |=  CS8; break;

        default : printf("range over. init failed.\r\n"); return -1;
    }
   
    switch(stopbit)
    {
        case UART_STOP_BIT_1:  huart_->c_cflag &= ~CSTOPB;    break;  // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
        case UART_STOP_BIT_2:  huart_->c_cflag |= CSTOPB;     break;

        default : printf("range over. init failed.\r\n"); return -1;
    }

    if(hwflow)
    {
        huart_->c_cflag |= CRTSCTS;
        
        huart_->c_iflag |= (IXON | IXOFF | IXANY);        
    }
    else
    {
        // Disable XON/XOFF flow control both input & output
        huart_->c_cflag &= ~(CRTSCTS);
        
        huart_->c_iflag &= ~(IXON | IXOFF | IXANY);          
    }
  

    switch(paraty)
    {
        case UART_PARATY_NONE:  huart_->c_cflag &= ~PARENB;  break;      
        case UART_PARATY_ODD:
        {
            huart_->c_cflag |= PARENB | PARODD;

            huart_->c_iflag &= ~(IGNBRK | BRKINT | IGNPAR | ICRNL | INLCR | ISTRIP | IGNCR | IUCLC);
   
            break;
        }

        case UART_PARATY_EVEN:
        {
            huart_->c_cflag |= PARENB;

            huart_->c_cflag &= ~PARODD;
            
            huart_->c_iflag &= ~( IGNBRK | BRKINT | IGNPAR | ICRNL | INLCR | ISTRIP | IGNCR | IUCLC);
   
            break;
        }

        default : printf("range over. init failed.\r\n"); return -1;     
    }

    if(canonicalmode)
    {
        huart_->c_lflag |= (ICANON | ECHO | ECHOE | ISIG);  // Cannonical mode

        huart_->c_oflag |= OPOST;     
    }
    else
    {
        huart_->c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode
   
        huart_->c_oflag &= ~OPOST;                           // No Output Processing          
      
        huart_->c_cc[VMIN]  = 1;       // Read at least 1 character

        huart_->c_cc[VTIME] = 0;           // Wait indefinetly
    
    }

    huart_->c_cflag |=  CREAD | CLOCAL;                  // Enable receiver,Ignore Modem Control lines


    cfsetispeed(huart_, (speed_t)baudrate);    // Set Read  Speed
    cfsetospeed(huart_,  (speed_t)baudrate);    // Set Write Speed

	// inital serial port
    // tcflush(*fid_, TCIFLUSH);
    tcflush(*fid_, TCIOFLUSH);

	tcsetattr( *fid_, TCSANOW, huart_ ); // setting serial communication

	printf( ">> tty Opened [%s]\r\n", ttyport_);

    usleep(500000);

    return 0;
}






int sendUart(int port, unsigned char *msg, const size_t msg_length)
{
  //--------------------------------------------------------------
  // TRANSMITTING BYTES
  //--------------------------------------------------------------

    int *fid_ = NULL;
    unsigned char tx_buffer[UART_MAX_BUF_SIZE] = {0,}; //for preventing string - null error

    //Select the hnadle
    if(port  == UART_PORT_1 || port  == UART_PORT_2)
    {
        fid_  = &ttyfd[port];
    }
    else
    {
        printf("Port Range Error!\r\n ");
        
        return -1;
    }

    if(msg_length > (size_t)UART_MAX_BUF_SIZE)
    {
        printf("Buffer size Error !\r\rn");

        return -1;
    }

    memcpy(tx_buffer, msg, msg_length);

    if (!(fid_) || *fid_ < 0)
    {
        printf("Invailed File Descriptor. Abort!\r\n");

        return -1;
    }

    //Filestream, bytes to write, number of bytes to write
    size_t count = write((*fid_), tx_buffer, msg_length);	

    if (count != msg_length) 
        printf("UART TX error\n");

    usleep(1000);  // 1ms sec delay

    tcflush(*fid_, TCIOFLUSH);

    return 0;
}

int readUart(int port, unsigned char *recv_msg, const size_t recv_msg_length)
{
    int *fid_ = NULL;
    unsigned char rx_buffer[UART_MAX_BUF_SIZE] = {0,}; //for preventing string - null error

    //Select the hnadle
    if(port  == UART_PORT_1 || port  == UART_PORT_2)
    {
        fid_  = &ttyfd[port];
    }
    else
    {
        printf( "Port Range Error!\r\n ");
        
        return -1;
    }

    if (*fid_ < 0)
    {
        printf("Invailed File Descriptor. Abort!\r\n");

        return -1;
    }

    if (!(fid_) || *fid_ < 0)
    {
        printf("Invailed File Descriptor. Abort!\r\n");

        return -1;
    }

    size_t total_recv_length_= 0;

    while(total_recv_length_ < recv_msg_length)
    {
        //Filestream, bytes to write, number of bytes to wrrx_lengthite
        int rx_length_ = read((*fid_), rx_buffer, sizeof(rx_buffer));	

        if (rx_length_ < 0)
        {
            printf("UART TX error\n");

            memset(recv_msg, 0, recv_msg_length);

            return -1;
        }
        else if(rx_length_ > 0)
        {
            memcpy(recv_msg, (void*)&rx_buffer[total_recv_length_], rx_length_);
        }

        total_recv_length_ += rx_length_;

        usleep(1000); 

        tcflush(*fid_, TCIOFLUSH);
    }

    return 0;
}

int readUartWithTimeout(int port, unsigned char *recv_msg, const size_t recv_msg_length, size_t timeout_val)
{
    int *fid_ = NULL;
    unsigned char rx_buffer[UART_MAX_BUF_SIZE] = {0,}; //for preventing string - null error

    //Select the hnadle
    if(port  == UART_PORT_1 || port  == UART_PORT_2)
    {
        fid_  = &ttyfd[port];
    }
    else
    {
        printf( "Port Range Error!\r\n ");
        
        return -1;
    }

    if (*fid_ < 0)
    {
        printf("Invailed File Descriptor. Abort!\r\n");

        return -1;
    }

    if (!(fid_) || *fid_ < 0)
    {
        printf("Invailed File Descriptor. Abort!\r\n");

        return -1;
    }

    fd_set set;
    struct timeval timeout;
    int rv;
    int filedesc = (*fid_);

    FD_ZERO(&set); /* clear the set */
    FD_SET(filedesc, &set); /* add our file descriptor to the set */

    timeout.tv_sec = 0;
    timeout.tv_usec = timeout_val;


    /* there was data to read */
    size_t total_recv_length_= 0;

    while(total_recv_length_ < recv_msg_length)
    {
        rv = select(filedesc + 1, &set, NULL, NULL, &timeout);
        
        if(rv == -1)
        {
            memset(recv_msg, '\0', recv_msg_length);

            return -1; /* an error accured */
        }
        else if(rv == 0)
        {
            // memset(recv_msg, 0, recv_msg_length);

            // printf("Read timeout\r\n"); /* a timeout occured */
            
            return 1;
        }
        else
        {
            //Filestream, bytes to write, number of bytes to wrrx_lengthite
            int rx_length_ = read((*fid_), rx_buffer, sizeof(rx_buffer)/sizeof(unsigned char));	

            if (rx_length_ < 0)
            {
                printf("UART TX error\n");

                memset(recv_msg, 0, recv_msg_length);

                return -1;
            }
            else if(rx_length_ > 0)
            {
                // printf("got msg_length : %d\r\n", rx_length_);
                
                memcpy(recv_msg, (void*)&rx_buffer[total_recv_length_], rx_length_);
            }

            total_recv_length_ += rx_length_;

            usleep(1); 
        }

    }

//     rv = select(filedesc + 1, &set, NULL, NULL, &timeout);
    
//     if(rv == -1)
//     {
//         return -1; /* an error accured */
//     }
//     else if(rv == 0)
//     {
//         printf("timeout\r\n"); /* a timeout occured */
        
//         return 1;
//     }
//     else
//     {
//         /* there was data to read */
//         size_t total_recv_length_= 0;

//         while(total_recv_length_ < recv_msg_length)
//         {
//             //Filestream, bytes to write, number of bytes to wrrx_lengthite
//             int rx_length_ = read((*fid_), rx_buffer, sizeof(rx_buffer)/sizeof(unsigned char));	

//             if (rx_length_ < 0)
//             {
//                 printf("UART TX error\n");

//                 memset(recv_msg, 0, recv_msg_length);

//                 return -1;
//             }
//             else if(rx_length_ > 0)
//             {
//                 // printf("got msg_length : %d\r\n", rx_length_);

//                 memcpy(recv_msg, (void*)&rx_buffer[total_recv_length_], rx_length_);
//             }

//             total_recv_length_ += rx_length_;

//             usleep(1000); 
//         }
//     }

    tcflush(*fid_, TCIOFLUSH);

    return 0;
}




void Uartclose(int port)
{
    int *fid_ = NULL;

    //Select the hnadle
    if(port  == UART_PORT_1 || port  == UART_PORT_2)
    {
        fid_  = &ttyfd[port];
        
        ttyfd[port] = 0;
    }
    else
    {
        printf( "Port Range Error!\r\n ");
        
        return;
    }

    if (!(fid_) || *fid_ < 0)
    {
        printf("Invailed File Descriptor. Abort!\r\n");

        return;
    }

    if(*fid_!= NULL)
        close(*fid_);

	printf( ">> tty Closed [%s]\r\n", ttyuart[port]);

    return;
}

////////////////////////////////////////////////////////



// int main(int argc, char *argv[]) 
// {
//     int port = UART_PORT_1;

//     initUartPort(port,                              //port
//                 B115200,                            //baudrate
//                 UART_STOP_BIT_1,                    //stopbit
//                 UART_DATA_BIT_8,                    //databit
//                 UART_HWFLOW_CONTROL_DISABLE,        //hwflow
//                 UART_CONONICAL_MODE_DISABLE,        //canonicalmode
//                 UART_PARATY_EVEN);                  //paraty
                
//     // unsigned char send_data[2] = {0xEC, 0x13};
//     unsigned char send_data[2] = {0xc8, 0x37};
//     unsigned char read_data[100] = {0,};

//     sendUart(port, send_data, sizeof(send_data));
    
//     usleep(100000);

//     readUart(port, read_data, 21);

//     for(int i = 0; i < 21; i++)
//     {
//         if(i == 0)
//             printf("recv data : ");

//         printf("%d ", read_data[i]);

//     }
//     printf("\r\n");
    
//     Uartclose(port);

//     return 0;
// }