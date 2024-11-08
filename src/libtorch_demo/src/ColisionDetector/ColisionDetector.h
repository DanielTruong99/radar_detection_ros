#ifndef _COLISIONDETECTOR_H_
#define _COLISIONDETECTOR_H_

#include "ros/ros.h"
#include "torch/torch.h"
#include "torch/csrc/jit/serialization/import.h"

namespace colision_detector
{
    class ColisionDetector
    {
        public:
            ColisionDetector();
            
            bool detectColision(float& v, float& alpha, float& d, float& threshold);
            std::vector<bool> checkRadarConfident(std::vector<float> &input, float &threshold = 0.5);

        private:
            torch::jit::script::Module _model;

            //! Data holder
            std::vector<torch::jit::IValue> _inputs;

            inline void _updateInputs(float& v, float& alpha, float& d)
            {
                torch::Tensor tensor = _inputs[0].toTensor();
                tensor[0][0] = v; tensor[0][1] = alpha; tensor[0][2] = d;
                _inputs[0] = tensor;
            }

            inline void _updateInputs(std::vector<float> &input)
            {
                torch::Tensor tensor = _inputs[0].toTensor();
                for (size_t index = 0; index < input.size(); index++)
                {
                    tensor[0][i] = input[i];
                }
                _inputs[0] = tensor;
            }
    };
}


#endif // _COLISIONDETECTOR_H_