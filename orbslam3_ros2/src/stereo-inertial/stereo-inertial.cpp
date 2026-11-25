#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-inertial-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify [do_equalize]" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    /*if(argc == 4)
    {
        argv[4] = "false";
    }*/
    const char *doEqual = (argc >= 5) ? argv[4] : "false";

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    /* - замена
    bool visualization = true;
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, visualization);
    */
    bool visualization = true;
    
    // Если в аргументах запуска передан режим pure_stereo или вы хотите переключаться
    ORB_SLAM3::System::eSensor sensor_type = ORB_SLAM3::System::IMU_STEREO;
    
    // Проверяем имя исполняемого файла или аргументы (если запускаем 'stereo')
    if (std::string(argv[0]).find("stereo-node") == std::string::npos && 
        std::string(argv[0]).find("stereo-inertial") == std::string::npos) {
        sensor_type = ORB_SLAM3::System::STEREO;
        std::cout << "SLAM Mode: Pure Stereo (No IMU)" << std::endl;
    } else {
        std::cout << "SLAM Mode: Stereo-Inertial (With IMU)" << std::endl;
    }

    ORB_SLAM3::System pSLAM(argv[1], argv[2], sensor_type, visualization);
    //

    // auto node = std::make_shared<StereoInertialNode>(&pSLAM, argv[2], argv[3], argv[4]); // Замена
    auto node = std::make_shared<StereoInertialNode>(&pSLAM, argv[2], argv[3], doEqual); // Вставка
    std::cout << "============================" << std::endl;

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
