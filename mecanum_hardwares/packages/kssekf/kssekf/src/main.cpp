#include <kssekf/kssekf_node.hpp>


int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    auto mmc = std::make_shared<KssEKF>();

    if(mmc->Initialize() <0)
    {
        printf("KssEKF Init Failed\r\n");

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
