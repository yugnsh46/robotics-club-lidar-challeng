#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarListener : public rclcpp::Node
{
public:
    LidarListener() : Node("lidar_listener")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&LidarListener::scanCallback, this, std::placeholders::_1)
        );
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        int center = msg->ranges.size() / 2;
        RCLCPP_INFO(this->get_logger(), "Front distance: %.2f m", msg->ranges[center]);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarListener>());
    rclcpp::shutdown();
    return 0;
}
