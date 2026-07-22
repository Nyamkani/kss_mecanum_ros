#include <wt901c/wt901c.hpp>


static volatile char s_cDataUpdate = 0;
const int c_uiBaud[] = {2400 , 4800 , 9600 , 19200 , 38400 , 57600 , 115200 , 230400 , 460800 , 921600};

//wt901c
Wt901c::Wt901c()
{


}

Wt901c::~Wt901c()
{
    if(this->is_init_)
    {
        close(this->fd_);
    }
}


int Wt901c::Initialize()
{
    if(this->is_init_)
    {
        close(this->fd_);
    }

    if((this->fd_ = serial_open((unsigned char*)this->dir_, this->s_iCurBaud_) < 0))
    {
        printf("open %s fail\n", this->dir_);
        return -1;
    }
    else 
    {
        printf("open %s success\n", this->dir_);
    }


	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitRegisterCallBack(this->SensorDataUpdata);


	AutoScanSensor((unsigned char*)this->dir_);

	this->is_init_ = true;

    return 0;
}

int Wt901c::Initialize(std::string dir, int baud_rate)
{
    if(this->is_init_)
    {
        close(this->fd_);
    }

	this->dir_ = dir.c_str();
	this->s_iCurBaud_ = baud_rate;

    if((this->fd_ = serial_open((unsigned char*)this->dir_, this->s_iCurBaud_) < 0))
    {
        printf("open %s fail\n", this->dir_);
        return -1;
    }
    else 
    {
        printf("open %s success\n", this->dir_);
    }


	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitRegisterCallBack(this->SensorDataUpdata);

	AutoScanSensor((unsigned char*)this->dir_);

	this->is_init_ = true;

    return 0;
}



int Wt901c::Drive()
{
	int i , ret;
 
	while(serial_read_data(this->fd_, this->cBuff, 1))
	{
		WitSerialDataIn(cBuff[0]);
	}
	#if WT901C_DPRINT
		printf("\n");
	#endif

	//   Delayms(500);
	
	if(s_cDataUpdate)
	{
		for(i = 0; i < 3; i++)
		{
			this->fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
			this->fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
			this->fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			this->fMag[i] = sReg[HX+i];
		}

		if(s_cDataUpdate & ACC_UPDATE)
		{	
			#if WT901C_DPRINT
				printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
			#endif

			s_cDataUpdate &= ~ACC_UPDATE;
		}
		if(s_cDataUpdate & GYRO_UPDATE)
		{			
			#if WT901C_DPRINT
				printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);	
			#endif
			
			s_cDataUpdate &= ~GYRO_UPDATE;
		}
		if(s_cDataUpdate & ANGLE_UPDATE)
		{			
			#if WT901C_DPRINT
				printf("angle:%.3f %.3f %.3f\r\n", fAngle[0], fAngle[1], fAngle[2]);
			#endif
			
			s_cDataUpdate &= ~ANGLE_UPDATE;
		}
		if(s_cDataUpdate & MAG_UPDATE)
		{			
			#if WT901C_DPRINT
				// printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
				printf("mag:%d %d %d\r\n", fMag[0], fMag[1], fMag[2]);
			#endif
			
			s_cDataUpdate &= ~MAG_UPDATE;
		}
	}
     
	return 0;
}




void Wt901c::SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
	return;
}

void Wt901c::Delayms(uint16_t ucMs)
{ 
	usleep(ucMs*1000);
	
	return;
}
 
void Wt901c::AutoScanSensor(unsigned char* dev)
{
	int i, iRetry;
	char cBuff[1];
	
	// for(i = 1; i < 10; i++)
	// {
	// 	serial_close(this->fd_);
	// 	this->s_iCurBaud_ = c_uiBaud[i];
	// 	this->fd_ = serial_open(dev , c_uiBaud[i]);
		
	// 	iRetry = 2;
	// 	do
	// 	{
	// 		s_cDataUpdate = 0;
	// 		WitReadReg(AX, 3);
	// 		this->Delayms(200);
	// 		while(serial_read_data(this->fd_, (unsigned char*)cBuff, 1))
	// 		{
	// 			WitSerialDataIn(cBuff[0]);
	// 		}
	// 		if(s_cDataUpdate != 0)
	// 		{
	// 			printf("%d baud find sensor\r\n\r\n", c_uiBaud[i]);
	// 			return ;
	// 		}
	// 		iRetry--;
	// 	}while(iRetry);		
	// }

	serial_close(this->fd_);
	this->fd_ = serial_open(dev , this->s_iCurBaud_);

	iRetry = 2;
	do
	{
		s_cDataUpdate = 0;
		WitReadReg(AX, 3);
		this->Delayms(200);
		while(serial_read_data(this->fd_, (unsigned char*)cBuff, 1))
		{
			WitSerialDataIn(cBuff[0]);
		}
		if(s_cDataUpdate != 0)
		{
			printf("%d baud find sensor\r\n\r\n", this->s_iCurBaud_);
			return ;
		}
		iRetry--;
	}while(iRetry);		


	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");

	return;
}


/////////////////////////////////////////////////////////////////////////////////////////////

const char node_name[] = "ImuSensor_Node";

ImuSensorNode::ImuSensorNode() : Node(node_name)
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    //joint state
    this->imu_msg_pubs_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/imu/raw_data", 
            qos_profile );

    this->mag_msg_pubs_ = this->create_publisher<sensor_msgs::msg::MagneticField>(
            "/imu/mag", 
            qos_profile );

	this->declare_parameter("serial_port", "/dev/ttyUSB0");
    this->declare_parameter("serial_baudrate", 115200);
	this->declare_parameter("x_axis_inverted", false);
    this->declare_parameter("y_axis_inverted", false);
    this->declare_parameter("z_axis_inverted", false);

	this->serial_port_ = this->get_parameter("serial_port").as_string();
    this->serial_baudrate_ = this->get_parameter("serial_baudrate").as_int();
	this->x_axis_inverted_ = this->get_parameter("x_axis_inverted").as_bool();
	this->y_axis_inverted_ = this->get_parameter("y_axis_inverted").as_bool();
	this->z_axis_inverted_ = this->get_parameter("z_axis_inverted").as_bool();
}



ImuSensorNode::~ImuSensorNode()
{


}


int ImuSensorNode::Initialize()
{
	if(this->sensor_.Initialize(this->serial_port_, this->serial_baudrate_) < 0 )
		return -1;

	if(this->sensor_.Drive() <0)
		return -2;

	// for(int i = 0; i < 3; i++)
	// {
	// 	this->init_fAcc[i] = this->GetAcc(i);
	// 	this->init_fGyro[i] = this->GetGyro(i);
	// 	this->init_fAngle[i] = this->GetAngle(i);
	// 	this->init_fMag[i] = this->GetMag(i);
	// }

	return 0;
}

int ImuSensorNode::Drive()
{
	return this->sensor_.Drive();
}

int ImuSensorNode::PublishImuMsg()
{
    sensor_msgs::msg::Imu imu;

	imu.header.stamp = this->get_clock()->now();
    imu.header.frame_id = "imu_link";

    tf2::Quaternion q_angle;

	double angle_to_rad = (M_PI/180.0f);
	double angle_rad[3], gyro_rad[3], acc_rad[3];

	for (int i = 0; i < 3; i++)
	{
		angle_rad[i] = this->GetAngle(i)*angle_to_rad;
		gyro_rad[i] = this->GetGyro(i)*angle_to_rad;
		acc_rad[i] = this->GetAcc(i);
	}

    q_angle.setRPY(angle_rad[0], angle_rad[1], angle_rad[2]); //angle x,y, z

	imu.orientation.x = q_angle.x();
	imu.orientation.y = q_angle.y();
	imu.orientation.z = q_angle.z();
	imu.orientation.w = q_angle.w();

	imu.angular_velocity.x = gyro_rad[0];
	imu.angular_velocity.y = gyro_rad[1];
	imu.angular_velocity.z = gyro_rad[2];

	imu.linear_acceleration.x = acc_rad[0];
	imu.linear_acceleration.y = acc_rad[1];
	imu.linear_acceleration.z = acc_rad[2];

    this->imu_msg_pubs_->publish(imu);

    return 0;
}

int ImuSensorNode::PublishMagMsg()
{
    sensor_msgs::msg::MagneticField mag;

	mag.header.stamp = this->get_clock()->now();
    mag.header.frame_id = "imu_link";

	mag.magnetic_field.x = this->GetMag(0);
	mag.magnetic_field.y = this->GetMag(1);
	mag.magnetic_field.z = this->GetMag(2);

    this->mag_msg_pubs_->publish(mag);

    return 0;
}



float ImuSensorNode::GetAcc(int id)
{
	if(id >= 3)	
		return 0.0f;

	int invert = 1;

	switch(id)
	{
		case 1: if((this->x_axis_inverted_)) invert = -1; break;
		case 2: if((this->y_axis_inverted_)) invert = -1; break;
		case 3: if((this->z_axis_inverted_)) invert = -1; break;
	}

	return (invert*this->sensor_.fAcc[id]) - this->init_fAcc[id];
}

float ImuSensorNode::GetGyro(int id)
{
	if(id >= 3)	
		return 0.0f;
		
	int invert = 1;

	switch(id)
	{
		case 1: if((this->x_axis_inverted_)) invert = -1; break;
		case 2: if((this->y_axis_inverted_)) invert = -1; break;
		case 3: if((this->z_axis_inverted_)) invert = -1; break;
	}
	
	return (invert*this->sensor_.fGyro[id]) - this->init_fGyro[id];
}

float ImuSensorNode::GetAngle(int id)
{
	if(id >= 3)	
		return 0.0f;

	int invert = 1;

	// switch(id)
	// {
	// 	case 1: if((this->x_axis_inverted_)) invert = -1; break;
	// 	case 2: if((this->y_axis_inverted_)) invert = -1; break;
	// 	case 3: if((this->z_axis_inverted_)) invert = -1; break;
	// }
	
	return (invert*this->sensor_.fAngle[id]) - this->init_fAngle[id];
}

float ImuSensorNode::GetMag(int id)
{
	if(id >= 3)	
		return 0.0f;
		
	int invert = 1;

	switch(id)
	{
		case 1: if((this->x_axis_inverted_)) invert = -1; break;
		case 2: if((this->y_axis_inverted_)) invert = -1; break;
		case 3: if((this->z_axis_inverted_)) invert = -1; break;
	}
	
	return (invert*this->sensor_.fMag[id]) - this->init_fMag[id];
}