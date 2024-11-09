#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include "CollisionDetector/CollisionDetector.h"

#define MEASURE_INFERENCE_TIME 0

class DemoNode : public rclcpp::Node
{
public:
    DemoNode() : Node("demo_node")
    {
        //* Create a timer that calls the timerCallback function every 1.0 second
        _timer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&DemoNode::timerCallback, this));

        //* Create collision detector
        _collision_detector = std::make_shared<collision_detector::CollisionDetector>();

        RCLCPP_INFO(this->get_logger(), "Demo node is ready");
    }

private:
    rclcpp::TimerBase::SharedPtr _timer;
    std::shared_ptr<collision_detector::CollisionDetector> _collision_detector;

    void timerCallback()
    {
        std::vector<float> input(10, 0.0);
        std::vector<bool> radar_confidence = _collision_detector->checkRadarConfidence(input, 0.5);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DemoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
