#include "ColisionDetector.h"

namespace colision_detector
{
    ColisionDetector::ColisionDetector()
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
            _model = torch::jit::load("/home/ryz2/catkin_ws/src/libtorch_demo/models/colision_detector.pt");
        }
        catch(const c10::Error& e)
        {
            ROS_ERROR("Error loading the model: %s", e.what());
        }
        _model.eval();
        _model.to(at::kCPU);
        torch::NoGradGuard no_grad;

    }

    bool ColisionDetector::detectColision(float &v, float &alpha, float &d, float &threshold)
    {
        /*
            ! Run the model
            * Create a tensor from the input data
            * Create a vector of IValues
            * Forward pass
        */
        // torch::NoGradGuard no_grad;

        //! Create a tensor from the input data
        static bool is_init = true;
        if(is_init)
        {
            torch::Tensor input_tensor = torch::zeros({1, 3});
            _inputs.push_back(input_tensor);
            is_init = false;
        }
        this->_updateInputs(v, alpha, d);

        //! Compute the network output
        torch::Tensor output = _model.forward(_inputs).toTensor();

        //! Apply the threshold
        bool is_colision = output.item<float>() > threshold;
        return is_colision;
    }

    std::vector<bool> checkRadarConfident(std::vector<float> &input, float &threshold = 0.5)
    {
        /*
            ! Run the model
            * Create a tensor from the input data
            * Create a vector of IValues
            * Forward pass
        */
        //! Create a tensor from the input data
        static bool is_init = true;
        if (is_init)
        {
            torch::Tensor input_tensor = torch::zeros({1, 10});
            _inputs.push_back(input_tensor);
            is_init = false;
        }
        this->_updateInputs(input);

        //! Compute the network output
        torch::Tensor output = _model.forward(inputs).toTensor();

        //! Apply the threshold
        std::vector<bool> radar_confident;
        for (int i = 0; i < output.size(1); i++)
        {
            radar_confident.push_back(output[0][i].item<float>() > threshold);
        }
        return radar_confident;
    }
}
