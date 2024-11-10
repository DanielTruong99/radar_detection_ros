#include "CollisionDetector.h"


namespace collision_detector
{
    CollisionDetector::CollisionDetector()
    {
        /*
            ! Load the model
              * Load the model from the file
              * Set the model to evaluation mode
              * Set the model to run on the CPU
              * Disable gradient calculation
        */
        try
        {
            _model = torch::jit::load("/home/ryz2/DanielWorkspace/radar_detection_ros/models/colision_detector.pt");
        }
        catch (const c10::Error &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("collision_detector"), "Error loading the model: %s", e.what()); 
        }
        _model.eval();
        _model.to(at::kCPU);

        torch::NoGradGuard no_grad;
    }


    std::vector<bool> CollisionDetector::checkRadarConfidence(std::vector<float> &input, float threshold)
    {
        /*
            ! Run the model
            * Create a tensor from the input data
            * Create a vector of IValues
            * Forward pass
        */

        //! Create a tensor from the input data
        torch::Tensor input_tensor = torch::zeros({1, static_cast<long>(input.size())});
        this->_updateInputs(input, input_tensor);

        //! Set the updated input tensor in _inputs
        _inputs.clear();
        _inputs.push_back(input_tensor);
        auto temp = _inputs[0].toTensor();
        for (int i = 0; i < temp.size(1); i++)
        {
            RCLCPP_INFO(rclcpp::get_logger("collision_detector"), "input elements %d: %f", i, temp[0][i].item<float>());
        }

        //! Compute the network output
        torch::Tensor output = _model.forward(_inputs).toTensor();
        for (int i = 0; i < output.size(1); i++)
        {
            RCLCPP_INFO(rclcpp::get_logger("collision_detector"), "Output element %d: %f", i, output[0][i].item<float>());
        }

        //! Apply the threshold
        std::vector<bool> radar_confident;
        for (int i = 0; i < output.size(1); i++)
        {
            radar_confident.push_back(output[0][i].item<float>() > threshold);
        }
        return radar_confident;
    }
}