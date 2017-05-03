// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <open_chisel/MultiDistVoxel.h>
#include <stdexcept>
#include <iostream>
namespace chisel
{

    MultiDistVoxel::MultiDistVoxel() :
            DistVoxel(), expanded(false)
    {

        expanded_sdf_.resize(8, 99999); //todo(kdaun) remove dummy initialization
        expanded = true;

    }

    MultiDistVoxel::~MultiDistVoxel()
    {

    }


    float MultiDistVoxel::GetExpandedSDF(int level) const
    {
        if(level > expanded_sdf_.size() || !expanded)
        {
            if(level > expanded_sdf_.size())
            {
                std::out_of_range e("Requested level > expanded_sdf_.size(), " + std::to_string(level) + " > " + std::to_string(expanded_sdf_.size()));
                throw(e);
            }
        }
        if(level == 0)
            return sdf;
        else
            return expanded_sdf_[level - 1];
    }

    void MultiDistVoxel::SetExpandedSDF(std::vector<float> expanded_sdf)
    {
        expanded_sdf_ = expanded_sdf;
        expanded = true;
    }

    void MultiDistVoxel::SetExpandedSDF(int level, float value)
    {
        expanded_sdf_[level - 1] = value;
        expanded = true;
    }

    void MultiDistVoxel::SetExpandedSDFSize(int n_level)
    {
        expanded_sdf_.resize(n_level, 99999);
        expanded = true;
    }

} // namespace chisel 
