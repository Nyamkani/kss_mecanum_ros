#include <ArduinoMotorDriver/mecanum_motor_controller.hpp>

//test
// int main(int argc, char* argv[])
// {
//     ArduinoMotorDriver driver(0, 4);

//     int flag = 0;


//     if(driver.Initialize()<0)
//     {
//         printf("error occur, Init failed\r\n");

//         return 0;
//     }

//         printf("init done.\r\n");

//     driver.SetMacanummVelData(144,144,144,144);


//      while(1)
//      {
//         if(flag%1 == 0)
//             driver.Drive();




//         usleep(100*1000);

//         if(flag++ >10)
//         {
//             flag = 0;

//             printf("rpm = %d,%d,%d,%d\r\n", driver.ReadRPM(1),  driver.ReadRPM(2),  driver.ReadRPM(3),  driver.ReadRPM(4));
//         }


//      }


//     return 0;
// }



int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    auto mmc = std::make_shared<MacanumMotorDriver>();

    if(mmc->Initialize() <0)
    {
        printf("MMC Init Failed\r\n");

        rclcpp::shutdown();

        return 0;   
    }


    //while(rclcpp::ok())
    //{
        rclcpp::spin(mmc);


    //}

    rclcpp::shutdown();

    return 0;
}
