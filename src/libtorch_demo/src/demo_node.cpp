#include "ros/ros.h"
#include "std_msgs/String.h"
#include "torch/torch.h"
#include "torch/csrc/jit/serialization/import.h"
#include <chrono>

#define MEASURE_INFERENCE_TIME 1

class DemoNode
{
    public:
        DemoNode():
            _node_handler(ros::NodeHandle()),
            _publisher(_node_handler.advertise<std_msgs::String>("demo_output_topic", 1000))
        {
            //* Create a timer that calls the timerCallback function every 1.0 second
            _timer = _node_handler.createTimer(ros::Duration(1.0), &DemoNode::timerCallback, this);

            /*
                ! Load the model
                  * Load the model from the file
                  * Set the model to evaluation mode
                  * Set the model to run on the CPU
                  * Disable gradient calculation
            */
            try
            {
                _colision_detector = torch::jit::load("/home/ryz2/catkin_ws/src/libtorch_demo/models/colision_detector.pt");
           }
           catch(const c10::Error& e)
           {
               ROS_ERROR("Error loading the model: %s", e.what());

           }
           _colision_detector.eval();
           _colision_detector.to(at::kCPU);
           torch::NoGradGuard no_grad;
        }
    
    private:
        ros::NodeHandle _node_handler;
        ros::Publisher _publisher;
        ros::Timer _timer;
        torch::jit::script::Module _colision_detector;

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
            /*
                ! Run the model
                  * Create a tensor from the input data
                  * Create a vector of IValues
                  * Forward pass
                  * Compute the output tensor with threshold
            */
            torch::Tensor input_tensor = torch::tensor({v, alpha, d}).unsqueeze(0);
            std::vector<torch::jit::IValue> inputs;
            inputs.push_back(input_tensor);
            torch::Tensor output = _colision_detector.forward(inputs).toTensor();
            is_colision = output.item<float>() > threshold;

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