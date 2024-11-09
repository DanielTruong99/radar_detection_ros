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
        //* Create a timer that calls the timerCallback function 500 hz
        _timer = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&DemoNode::timerCallback, this));

        //* Create a publisher that publishes Float32MultiArray messages on the topic "radar_confidence"
        _publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>("radar_confidence", 10);

        //* Create collision detector
        _collision_detector = std::make_shared<collision_detector::CollisionDetector>();

        //* read csv file
        this->_readCsv("/home/ryz2/DanielWorkspace/radar_detection_ros/Cleaned_Data_without_Empty_Rows.csv");

        RCLCPP_INFO(this->get_logger(), "Demo node is ready");
    }

private:
    rclcpp::TimerBase::SharedPtr _timer;
    std::shared_ptr<collision_detector::CollisionDetector> _collision_detector;
    std::vector<std::vector<float>> _csv_data;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr _publisher;

    inline void _readCsv(const char *file_path)
    {
        std::ifstream file(file_path);
        std::string line;
        int row_count = 0;
        while (std::getline(file, line))
        {
            row_count++;
            if (row_count == 1)
            {
                continue;
            }

            std::vector<float> row;
            std::stringstream ss(line);
            std::string item;
            while(std::getline(ss, item, ','))
            {
                float converted_data = std::stof(item);
                converted_data = converted_data == 9999 ? 1.0 : converted_data;
                row.push_back(converted_data);
            }
            _csv_data.push_back(row);
        }
        file.close();
    }

    void timerCallback()
    {
        static int row_index = 0;
        std::vector<float> input(_csv_data[row_index].begin(), _csv_data[row_index].begin() + 4);
        for (size_t index = 4; index < _csv_data[row_index].size(); index++)
        {
            if (index % 2 != 0)
            {
                continue;
            }

            input.push_back(std::sin(_csv_data[row_index][index]));
            input.push_back(std::cos(_csv_data[row_index][index + 1]));
        }

        std::vector<bool> radar_confidence = _collision_detector->checkRadarConfidence(input, 0.7);
        std_msgs::msg::Float32MultiArray msg;
        for(size_t index = 0; index < radar_confidence.size(); index++)
        {
            msg.data.push_back(radar_confidence[index] == true ? 1.0 : 0.0);
        }
        _publisher->publish(msg);

        row_index++;
        row_index = row_index % _csv_data.size();
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
