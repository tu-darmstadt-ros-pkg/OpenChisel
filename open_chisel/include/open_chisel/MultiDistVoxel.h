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

#ifndef MULTIDISTVOXEL_H_
#define MULTIDISTVOXEL_H_

#include <limits>
#include <stdint.h>

#include <open_chisel/FixedPointFloat.h>
#include <open_chisel/DistVoxel.h>

namespace chisel
{

    class MultiDistVoxel : DistVoxel
    {
        public:
            MultiDistVoxel();
            virtual ~MultiDistVoxel();
            inline int GetExpansionLevels() const {return expanded_sdf_.size();}
            inline bool GetExpanded() const {return expanded;}
            // returns expanded sdf, level 0 is original sdf
            float GetExpandedSDF(int level) const;
            void SetExpandedSDF(std::vector<float> expanded_sdf);

        protected:

           bool expanded;
           // stores expanded sdf data, expansion level increases with index
           // index 0 contains first expanded level
           std::vector<float> expanded_sdf_;
    };

} // namespace chisel 

#endif // MULTIDISTVOXEL_H_
