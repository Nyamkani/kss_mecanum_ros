#include "ArduinoMotorDriver/arduino_motor_driver.hpp"

////////////////////////////////





///////////////////////////////

ArduinoMotorDriver::ArduinoMotorDriver(){};


ArduinoMotorDriver::ArduinoMotorDriver(int no_of_motors)
{
    this->no_of_motors_ = no_of_motors;
}

ArduinoMotorDriver::ArduinoMotorDriver(int port, int no_of_motors)
{
    this->no_of_motors_ = no_of_motors;
}


ArduinoMotorDriver::~ArduinoMotorDriver()
{
    
}

int ArduinoMotorDriver::SerialInitialize(const std::string& ttydir, const int32_t baud_rate)
{
    this->Serial_.SerialClose();

    if(this->Serial_.Initialize(ttydir,
                            baud_rate,                            //baudrate
                            UART_STOP_BIT_1,                    //stopbit
                            UART_DATA_BIT_8,                    //databit
                            UART_HWFLOW_CONTROL_DISABLE,        //hwflow
                            UART_CONONICAL_MODE_DISABLE,        //canonicalmode
                            UART_PARATY_NONE)  < 0)             //paraty
    {
        this->state_ = DriverState::sError;

        return -1;
    }

    return 0;
}





int ArduinoMotorDriver::Initialize(const std::string& ttydir, const int32_t baud_rate)
{
    if(!(this->isHWInit))
    {
        //something to init hw
        this->isHWInit = true;
    }

    this->motor_data_.clear();

    this->motor_data_.shrink_to_fit();

    this->motor_data_.resize(this->no_of_motors_, MotorData());

    if(this->SerialInitialize(ttydir, baud_rate) < 0 )
        return -1;


    this->state_ = DriverState::sRun;

    return 0;
}

int ArduinoMotorDriver::Write(const ArduinoCommand cmd)
{
         //write pwm speed mm/s-> rpm -> pwm
        //1. Get command and args from queue
        //ArduinoCommand cmd = this->command_queue_.front();

        //2. send data with command data
        int arg_length = cmd.args.size();
        std::string write_data_buf;
        
        write_data_buf.append(sizeof(char), cmd.command);

        for(int i = 0; i < arg_length; i++)
        {
            write_data_buf.append(" ");
            write_data_buf.append(std::to_string(cmd.args[i]));
        }

        write_data_buf.append(" ");
        write_data_buf.append("\r"); //EOD

        if(this->Serial_.SendData((unsigned char*)write_data_buf.c_str(), write_data_buf.length()) < 0)
            return -1;


        // usleep(100*1000);

        //3. Read data with "OK"
        std::string read_data;

        unsigned char read_buf[MAX_BUF];

        memset(read_buf, 0, sizeof(read_buf));

        bool check = false;
        int utime = 1000; //1ms

        int repeat = 0;

        while(!(check))
        {
            if(this->Serial_.ReadDataWithTimeout(read_buf, sizeof(read_buf), utime) < 0)
                continue;

            read_data.append((char*)read_buf);

            int length = read_data.length();

            // if(read_data[length -1 ] == '\n' && read_data[length -2])
            //    check = true;

            if (read_data != "OK\r\n")
                check = true;

            if((repeat++)> 100)
                return -1;
        }

        // this->Serial_.ReadDataWithTimeout(read_buf, sizeof(read_buf), utime);

        //4. delete first command data
        // this->command_queue_.erase(this->command_queue_.begin());


    return 0;
}

int ArduinoMotorDriver::Read(const ArduinoCommand cmd)
{
        //read encoders
        // ArduinoCommand cmd;
        std::string write_data_buf;

        //1. send data with read encoder or rpms
        write_data_buf.append(sizeof(char), cmd.command);

        write_data_buf.append(" ");
        write_data_buf.append("\r"); //EOD

        if(this->Serial_.SendData((unsigned char*)write_data_buf.c_str(), write_data_buf.length()) < 0)
            return -1;

        //2. Read data with args
        std::string read_data;

        unsigned char read_buf[MAX_BUF];

        memset(read_buf, 0, sizeof(read_buf));

        bool check = false;
        int utime = 1000;//10ms
        int repeat = 0;

        while(!(check))
        {
            if(this->Serial_.ReadDataWithTimeout(read_buf, sizeof(read_buf), utime) < 0)
                continue;

            read_data.append((char*)read_buf);

            int length = read_data.length();

            if(read_data[length -1 ] == '\n' && read_data[length -2])
               check = true;

            // if (read_data != "OK\r\n")
 
            if((repeat++)> 100)
                return -1;
        }

        //3. parsing data
        std::vector<std::string> args;
        std::vector<int> buf;

        args.resize(this->no_of_motors_);
        buf.resize(this->no_of_motors_);

        int itr = 0;
        int length = read_data.length();

        for(int i = 0; i < length; i++)
        {
            if(read_data[i] != ' ')
            {
                if(read_data[i] == '\r' || read_data[i] == '\n')
                {
                    args[itr] += '\0';

                    break;
                }
                else
                    args[itr].append(sizeof(char), read_data[i]);
            }
            else
            {
                // args[itr].append("\0");
                args[itr] += '\0';
                itr++;
            }

        }

        try
        {
            switch(cmd.command)
            {
                case 'e': //read encoder
                {
                    for(int i = 0; i< this->no_of_motors_; i++)
                    {
                        buf[i] = std::stoi(args[i]);

                        // printf("args[%d] = %s, buf[%d] = %d\r\n", i, args[i].c_str(), i, buf[i]);
                    }


                    break;
                }

                case 'z': //read rpm
                {
                    for(int i = 0; i< this->no_of_motors_; i++)
                        buf[i] = std::stoi(args[i]);

                    break;
                }

                default: break;
            }
        }
        catch(const std::exception& e)
        {
            return -1;
        }
        
        switch(cmd.command)
        {
            case 'e': //read encoder
            {
                for(int i = 0; i< this->no_of_motors_; i++)
                    this->motor_data_[i].encoder = buf[i];

                // printf("encoder = %d, %d, %d, %d\r\n", this->motor_data_[0].encoder, 
                //                                         this->motor_data_[1].encoder,
                //                                         this->motor_data_[2].encoder,
                //                                         this->motor_data_[3].encoder);
                    
                break;
            }

            case 'z': //read rpm
            {
                for(int i = 0; i< this->no_of_motors_; i++)
                    this->motor_data_[i].rpm = buf[i];

                break;
            }

            default: break;
        }

        if((this->read_queue_)++ >1)
            this->read_queue_ = 0;

    return 0;
}


int ArduinoMotorDriver::MainWorker()
{
    if(!(this->command_queue_.empty()))
    {
        //write pwm speed mm/s-> rpm -> pwm
        //1. Get command and args from queue
        ArduinoCommand cmd = this->command_queue_.front();

        //2. do Write thing
        if(this->Write(cmd) < 0);
            return -1;

        //3. delete first command data
        this->command_queue_.erase(this->command_queue_.begin());

    }
    else
    {
        //read encoders
        ArduinoCommand cmd;
        std::string write_data_buf;

        //1. send data with read encoder or rpms
        int read_cmd = this->read_queue_;

        switch(read_cmd)
        {
            case 0: //read encoder
            {
                cmd.command = 'e';

                break;
            }

            case 1: //read rpm
            {
                cmd.command = 'z';

                break;
            }

            default: break;
        }

        //2. do thing
        if(this->Read(cmd) < 0);
            return -1;


        if((this->read_queue_)++ >1)
            this->read_queue_ = 0;

    }

    return 0;
}


// int ArduinoMotorDriver::Drive()
// {

//     DriverState state = this->state_;

//     switch(state)
//     {
//         case DriverState::sHWInit:
//         {
//             this->state_ = DriverState::sInit;

//             break;
//         }

//         case DriverState::sInit:
//         {
//             if(this->Initialize() < 0)
//                 this->state_ = DriverState::sError;
//             else
//                 this->state_ = DriverState::sRun;

//             break;
//         }

//         case DriverState::sRun:
//         {
//             if(this->MainWorker() < 0);
//                 // this->state_ = DriverState::sError;

//             break;
//         }

//         case DriverState::sError:
//         {


//             break;
//         }

//         default: break;
//     }

//     return 0;
// }



void ArduinoMotorDriver::SetMacanummVelData(int vel1, int vel2, int vel3, int vel4)
{

    ArduinoCommand cmd;
    
    cmd.command = 'm';
    cmd.args.push_back(vel1);
    cmd.args.push_back(vel2);
    cmd.args.push_back(vel3);
    cmd.args.push_back(vel4);

    this->command_queue_.push_back(cmd);

    return;
}


int ArduinoMotorDriver::ReadRPM(int motor)
{
    int select = motor -1;

    if (select < 0 || select >= this->motor_data_.size())
        return 0;

    return this->motor_data_.at(select).rpm;
}

int ArduinoMotorDriver::ReadEncoder(int motor)
{
    int select = motor -1;

    if (select < 0 || select >= this->motor_data_.size())
        return 0;

    return this->motor_data_.at(select).encoder;
}
