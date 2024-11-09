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

        bool detectCollision(float &v, float &alpha, float &d, float &threshold);
        std::vector<bool> checkRadarConfidence(std::vector<float> &input, float threshold);

    private:
        torch::jit::script::Module _model;

        //! Data holder
        std::vector<torch::jit::IValue> _inputs;

        inline void _updateInputs(float &v, float &alpha, float &d)
        {
            torch::Tensor tensor = _inputs[0].toTensor();
            tensor[0][0] = v;
            tensor[0][1] = alpha;
            tensor[0][2] = d;
            _inputs[0] = tensor;
        }

        inline void _updateInputs(std::vector<float> &input)
        {
            torch::Tensor tensor = _inputs[0].toTensor();
            for (size_t index = 0; index < input.size(); index++)
            {
                tensor[0][index] = input[index];
            }
            _inputs[0] = tensor;
        }
    };
}

#endif // _COLLISIONDETECTOR_H_