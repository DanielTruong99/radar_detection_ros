#ifndef _COLLISIONDETECTOR_H_
#define _COLLISIONDETECTOR_H_

#include <cstring>
#include "rclcpp/rclcpp.hpp"
#include "torch/torch.h"
#include "torch/csrc/jit/serialization/import.h"

namespace collision_detector
{
    class CollisionDetector
    {
    public:
        CollisionDetector();
        std::vector<bool> checkRadarConfidence(std::vector<float> &input, float threshold);

    private:
        torch::jit::script::Module _model;

        //! Data holder
        std::vector<torch::jit::IValue> _inputs;


        inline void _updateInputs(std::vector<float> &input, torch::Tensor &tensor)
        {
            for (size_t index = 0; index < input.size(); index++)
            {
                tensor[0][index] = input[index];
            }
        }
    };
}

#endif // _COLLISIONDETECTOR_H_