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

        //! Create a tensor from the input data
        torch::Tensor input_tensor = torch::tensor({0.0, 0.0, 0.0}).unsqueeze(0);
        _inputs.push_back(input_tensor);
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
        this->_updateInputs(v, alpha, d);

        //! Compute the network output
        torch::Tensor output = _model.forward(_inputs).toTensor();

        //! Apply the threshold
        bool is_colision = output.item<float>() > threshold;
        return is_colision;
    }

}
