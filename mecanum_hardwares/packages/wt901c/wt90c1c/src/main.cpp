#include <wt901c/wt901c.hpp>

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);

    //time
    // std::chrono::system_clock::time_point prev_time;
    auto next_tick = std::chrono::steady_clock::now();

    int tick = 0;
    bool mag_pub_tick = false;
    bool imu_pub_tick = false;
    bool drive_tick = false;

    auto imu = std::make_shared<ImuSensorNode>();

    if(imu->Initialize() <0)
    {
        printf("imu Init Failed\r\n");

        rclcpp::shutdown();

        return 0;   
    }


    while(rclcpp::ok())
    {
        // auto start_time = std::chrono::system_clock::now();
        // auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(start_time - prev_time);

        // if(dt.count() >= 1)  //1ms tick
        // {
        //     if(tick++ >= 1000)
        //         tick = 0;

        //     if(tick %20 == 0)  //20ms?
        //     {
        //         drive_tick = true;
        //     }

        //     if(tick %20 == 0)  //20ms?
        //     {
        //         imu_pub_tick = true;
        //     }

        //     if(tick %100 == 0)  //100ms?
        //     {
        //         mag_pub_tick = true;
        //     }


        //     prev_time = start_time;
        // }
        // else
        // {
        //     if (drive_tick)
        //     {
        //         imu->Drive();

        //         drive_tick = false;
        //     }

        //     if(imu_pub_tick)
        //     {
        //         //do publish
        //         imu->PublishImuMsg();

        //         imu_pub_tick = false;
        //     }
        //     else if(mag_pub_tick)
        //     {
        //         //do publish
        //         imu->PublishMagMsg();

        //         mag_pub_tick = false;
        //     }
        // }
        // usleep(1);
        next_tick += std::chrono::milliseconds(1); 


        if (++tick >= 1000) tick = 0;

        if (tick % 10 == 0) {
            drive_tick = true;
            imu_pub_tick = true;
        }

        if (tick % 50 == 0) {
            mag_pub_tick = true;
        }


        if (drive_tick) {
            imu->Drive();
            drive_tick = false;
        }
        if (imu_pub_tick) {
            imu->PublishImuMsg();
            imu_pub_tick = false;
        }
        if (mag_pub_tick) {
            imu->PublishMagMsg();
            mag_pub_tick = false;
        }

        std::this_thread::sleep_until(next_tick);
    }

    rclcpp::shutdown();

    return 0;
}
