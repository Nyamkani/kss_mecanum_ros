#include "ArduinoMotorDriver/serial_uart.hpp"


SerialComm::SerialComm(){}


SerialComm::~SerialComm()
{
    this->SerialClose();
}


int SerialComm::Initialize(std::string ttydir,
                int baudrate,
                int stopbit,
                int databits,
                bool hwflow,
                bool canonicalmode,
                int paraty)
{
    if(ttydir.empty())
    {
		printf( "tty dir is empty.\r\n ");

        return -1;
    }
    else
    {
        this->ttydir_ = ttydir;
    }

    int defined_baudrate = get_baud(baudrate);

    if(defined_baudrate < 0)
    {
		printf( "baudrate has wrong value.\r\n ");

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


	this->ttyfd_ = open(this->ttydir_.c_str(), O_RDWR | O_NOCTTY);
	
	if(this->ttyfd_ < 0)
	{
		printf( "%s : >> tty Open Fail, Try sudo [%s]\r\n ", strerror(EACCES), this->ttydir_.c_str());

		return -1;
	}
    printf( "Got Pid: [%d]\r\n ", this->ttyfd_);

	memset(&(this->huart_), 0, sizeof(this->huart_));
	
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
    
    
    this->huart_.c_cflag &= ~CSIZE;	            // Clears the mask for setting the data size

    // Set the data bits 
    switch(databits)
    {
        case UART_DATA_BIT_5:  this->huart_.c_cflag |=  CS5; break;
        case UART_DATA_BIT_6:  this->huart_.c_cflag |=  CS6; break;
        case UART_DATA_BIT_7:  this->huart_.c_cflag |=  CS7; break;
        case UART_DATA_BIT_8:  this->huart_.c_cflag |=  CS8; break;

        default : printf("range over. init failed.\r\n"); return -1;
    }
   
    switch(stopbit)
    {
        case UART_STOP_BIT_1:  this->huart_.c_cflag &= ~CSTOPB;    break;  // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
        case UART_STOP_BIT_2:  this->huart_.c_cflag |= CSTOPB;     break;

        default : printf("range over. init failed.\r\n"); return -1;
    }

    if(hwflow)
    {
        this->huart_.c_cflag |= CRTSCTS;
        
        this->huart_.c_iflag |= (IXON | IXOFF | IXANY);        
    }
    else
    {
        // Disable XON/XOFF flow control both input & output
        this->huart_.c_cflag &= ~(CRTSCTS);
        
        this->huart_.c_iflag &= ~(IXON | IXOFF | IXANY);          
    }
  

    switch(paraty)
    {
        case UART_PARATY_NONE:  this->huart_.c_cflag &= ~PARENB;  break;      
        case UART_PARATY_ODD:
        {
            this->huart_.c_cflag |= PARENB | PARODD;

            this->huart_.c_iflag &= ~(IGNBRK | BRKINT | IGNPAR | ICRNL | INLCR | ISTRIP | IGNCR | IUCLC);
   
            break;
        }

        case UART_PARATY_EVEN:
        {
            this->huart_.c_cflag |= PARENB;

            this->huart_.c_cflag &= ~PARODD;
            
            this->huart_.c_iflag &= ~( IGNBRK | BRKINT | IGNPAR | ICRNL | INLCR | ISTRIP | IGNCR | IUCLC);
   
            break;
        }

        default : printf("range over. init failed.\r\n"); return -1;     
    }

    if(canonicalmode)
    {
        this->huart_.c_lflag |= (ICANON | ECHO | ECHOE | ISIG);  // Cannonical mode

        this->huart_.c_oflag |= OPOST;     
    }
    else
    {
        this->huart_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode
   
        this->huart_.c_oflag &= ~OPOST;                           // No Output Processing          
      
        this->huart_.c_cc[VMIN]  = 1;       // Read at least 1 character

        this->huart_.c_cc[VTIME] = 0;           // Wait indefinetly
    
    }

    this->huart_.c_cflag |=  CREAD | CLOCAL;                  // Enable receiver,Ignore Modem Control lines


    cfsetispeed(&(this->huart_), (speed_t)defined_baudrate);    // Set Read  Speed
    cfsetospeed(&(this->huart_),  (speed_t)defined_baudrate);    // Set Write Speed

	// inital serial port
    // tcflush(*fid_, TCIFLUSH);
    tcflush(this->ttyfd_, TCIOFLUSH);

	tcsetattr(this->ttyfd_, TCSANOW, &(this->huart_)); // setting serial communication

	printf( ">> tty Opened [%s] with baudrate [%d]\r\n", this->ttydir_.c_str(), baudrate);

    usleep(500000);

    this->is_init_ = true;

    return 0;
}

int SerialComm::SendData(unsigned char *msg, const size_t msg_length)
{
    //--------------------------------------------------------------
    // TRANSMITTING BYTES
    //--------------------------------------------------------------
    if(!(this->is_init_))
    {
        printf("Init is not done. Send data failed.\r\n");

        return -1;
    }

    const int fid = this->ttyfd_;
    unsigned char tx_buffer[UART_MAX_BUF_SIZE] = {0,}; //for preventing string - null error

    memcpy(tx_buffer, msg, msg_length);

    //Filestream, bytes to write, number of bytes to write
    size_t count = write(fid, tx_buffer, msg_length);	

    if (count != msg_length) 
        printf("UART TX error\n");

    usleep(SERIAL_DATA_SLEEP_TIME);  

    tcflush(fid, TCIOFLUSH);

    return 0;
}


int SerialComm::ReadData(unsigned char *recv_msg, const size_t recv_msg_length)
{
    if(!(this->is_init_))
    {
        printf("Init is not done. Read data failed.\r\n");

        return -1;
    }

    const int fid = this->ttyfd_;
    unsigned char rx_buffer[UART_MAX_BUF_SIZE] = {0,}; //for preventing string - null error

    size_t total_recv_length_= 0;

    while(total_recv_length_ < recv_msg_length)
    {
        //Filestream, bytes to write, number of bytes to wrrx_lengthite
        int rx_length_ = read(fid, rx_buffer, sizeof(rx_buffer));	

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

        usleep(SERIAL_DATA_SLEEP_TIME); 

        tcflush(fid, TCIOFLUSH);
    }

    return 0;
}


int SerialComm::ReadDataWithTimeout(unsigned char *recv_msg, const size_t recv_msg_length, size_t timeout_val)
{
    if(!(this->is_init_))
    {
        printf("Init is not done. Read data failed.\r\n");

        return -1;
    }

    const int fid = this->ttyfd_;
    fd_set set;
    struct timeval timeout;
    int rv;

    FD_ZERO(&set); /* clear the set */
    FD_SET(fid, &set); /* add our file descriptor to the set */

    timeout.tv_sec = 0;
    timeout.tv_usec = timeout_val;

    unsigned char rx_buffer[UART_MAX_BUF_SIZE] = {0,}; //for preventing string - null error

    /* there was data to read */
    size_t total_recv_length_= 0;

    while(total_recv_length_ < recv_msg_length)
    {
        rv = select(fid + 1, &set, NULL, NULL, &timeout);
        
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
            int rx_length_ = read(fid, rx_buffer, sizeof(rx_buffer)/sizeof(unsigned char));	

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

    tcflush(fid, TCIOFLUSH);

    return 0;
}


int SerialComm::SerialClose()
{
    const int fid = this->ttyfd_;

    if(this->is_init_)
    {
        close(fid);

	    printf( ">> tty Closed [%s]\r\n", this->ttydir_.c_str());  
    }


    this->is_init_ = false;

    return 0 ;
}


int get_baud(int baud)
{
    switch (baud) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
    case 2500000:
        return B2500000;
    case 3000000:
        return B3000000;
    case 3500000:
        return B3500000;
    case 4000000:
        return B4000000;
    default: 
        return -1;
    }
}


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