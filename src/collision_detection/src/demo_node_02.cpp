/*std libs*/
#include <iostream>
#include <vector>
#include <sstream>
#include <chrono>
#include <cmath>

/*ROS2 libs*/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>

/*user libs*/
#include "CollisionDetector/CollisionDetector.h"

#define MEASURE_INFERENCE_TIME 0

class DemoNode : public rclcpp::Node
{
public:
    DemoNode() : Node("demo_node")
    {

    }

private:
    int _temp;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DemoNode>();

    std::shared_ptr<collision_detector::CollisionDetector> collision_detector = std::make_shared<collision_detector::CollisionDetector>();
    std::vector<std::vector<float>> test_data = {
        /* r1, r2, r3, r4, q1, q2, q3, q4, q5, q6 */
        {1.0, 1.0, 0.26, 1.0, -0.524729, -0.414567, 0.848874, -1.92067, -0.425858, 1.71819},
        {0.48, 0.22, 1.0, 0.5, -0.484377, -0.520259, 0.65377, -1.09321, -2.93977, 0.0808745},
    };

    // for (auto &data : test_data)
    std::vector<float> data{10, 0};
    for(int i = 0; i < 2; i++)
    {
        /* prepare input tensor */
        /* r1, r2, r3, r4, s_q4, c_q4, s_q5, c_q5, s_q6, c_q6*/
        if(i == 0)
        {
            data = {1.0, 1.0, 0.26, 1.0, 0, -1, 0, 1, 0, -1};
        }
        else
        {
            data = {0.48, 0.22, 1.0, 0.5, 0, -1, 0, 1, 0, 1};
        }

        /* check radar confidence */
        std::vector<bool> radar_confidence = collision_detector->checkRadarConfidence(data, 0.5);
        std::ostringstream oss;
        for (size_t index = 0; index < radar_confidence.size(); index++)
        {
            oss << "Radar confidence " << index << ": " << radar_confidence[index] << " ";
        }
        RCLCPP_INFO(rclcpp::get_logger("collision_detector"), oss.str().c_str());
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
