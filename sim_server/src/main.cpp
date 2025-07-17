#include "ros2_node.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Ros2Node>());  // 运行ros节点，程序会一直阻塞在这里
    rclcpp::shutdown();

    return 0;
}
