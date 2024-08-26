#include "ros/ros.h"
#include "std_msgs/String.h"
#include <chrono>
#include "ColisionDetector/ColisionDetector.h"

#define MEASURE_INFERENCE_TIME 0

class DemoNode
{
    public:
        DemoNode():
            _node_handler(ros::NodeHandle()),
            _publisher(_node_handler.advertise<std_msgs::String>("demo_output_topic", 1000))
        {
            //* Create a timer that calls the timerCallback function every 1.0 second
            _timer = _node_handler.createTimer(ros::Duration(1.0), &DemoNode::timerCallback, this);

           //* Create colsion detector
            _colision_detector = colision_detector::ColisionDetector();

            ROS_INFO("Demo node is ready");
        }
    
    private:
        ros::NodeHandle _node_handler;
        ros::Publisher _publisher;
        ros::Timer _timer;
        colision_detector::ColisionDetector _colision_detector;

        void timerCallback(const ros::TimerEvent& event)
        {
            //! Fake radar sensor data
            float v = 1.0;
            float d = 0.3;
            float alpha = 30.0;
            
            //! Colision detection result
            float threshold = 0.5;
            bool is_colision = false;

            #if MEASURE_INFERENCE_TIME 
                auto start = std::chrono::high_resolution_clock::now();
            #endif

            is_colision = _colision_detector.detectColision(v, alpha, d, threshold);

            #if MEASURE_INFERENCE_TIME
                auto end = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
                std::stringstream ss; ss << "Inference Time: " << duration.count() << " microseconds";
                ROS_INFO("%s", ss.str().c_str());
            #endif
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "demo");
    DemoNode demo_node;
    ros::spin();
    return 0;
}