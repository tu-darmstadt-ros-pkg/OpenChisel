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

#include <open_chisel/Chunk.h>

namespace chisel
{

    Chunk::Chunk()
    {
        // TODO Auto-generated constructor stub

    }

    Chunk::Chunk(const ChunkID id, const Eigen::Vector3i& nv, float r, bool useColor) :
            ID(id), numVoxels(nv), voxelResolutionMeters(r)
    {
        AllocateDistVoxels();

        if(useColor)
        {
            AllocateColorVoxels();
        }

        origin = Vec4(numVoxels(0) * ID(0) * voxelResolutionMeters, numVoxels(1) * ID(1) * voxelResolutionMeters, numVoxels(2) * ID(2) * voxelResolutionMeters, 0.0f);
    }

    Chunk::~Chunk()
    {

    }

    void Chunk::AllocateDistVoxels()
    {
        int totalNum = GetTotalNumVoxels();
        voxels.clear();
        voxels.resize(totalNum, DistVoxel());
    }

    void Chunk::AllocateColorVoxels()
    {
        int totalNum = GetTotalNumVoxels();
        colors.clear();
        colors.resize(totalNum, ColorVoxel());
    }

    AABB Chunk::ComputeBoundingBox()
    {
        Vec4 pos = origin;
        Vec4 size(numVoxels.x() * voxelResolutionMeters, numVoxels.y() * voxelResolutionMeters, numVoxels.z() * voxelResolutionMeters, 0.0f);
        return AABB(pos, pos + size);
    }

    Vec4 Chunk::GetWorldCoords(const Point4& coords) const
    {
      return (coords.cast<float>() + Vec4(0.5, 0.5, 0.5, 0.0)) * voxelResolutionMeters + origin;
    }

    Point4 Chunk::GetVoxelCoords(const Vec4& relativeCoords) const
    {
      const float roundingFactor = 1.0f / (voxelResolutionMeters);

      return Point4( static_cast<int>(std::floor(relativeCoords(0) * roundingFactor)),
                     static_cast<int>(std::floor(relativeCoords(1) * roundingFactor)),
                     static_cast<int>(std::floor(relativeCoords(2) * roundingFactor)),
                     0);
    }

    Vec4 Chunk::GetWorldCoords(const VoxelID& voxelID) const
    {
      Vec4 voxelCoords(voxelID % numVoxels(0),
                       voxelID / numVoxels(0) % numVoxels(1),
                       voxelID / (numVoxels(0)* numVoxels(1)),
                       0.0f );

      return (voxelCoords + Vec4(0.5, 0.5, 0.5, 0.0)) * voxelResolutionMeters + origin;
    }

    VoxelID Chunk::GetVoxelID(const Vec4& relativePos) const
    {
        return GetVoxelID(GetVoxelCoords(relativePos));
    }

    void Chunk::ComputeStatistics(ChunkStatistics* stats)
    {
        assert(stats != nullptr);

        for (const DistVoxel& vox : voxels)
        {
            float weight = vox.GetWeight();
            if (weight > 0)
            {
                float sdf = vox.GetSDF();
                if (sdf < 0)
                {
                    stats->numKnownInside++;
                }
                else
                {
                    stats->numKnownOutside++;
                }
            }
            else
            {
                stats->numUnknown++;
            }

            stats->totalWeight += weight;

        }
    }

    Vec3 Chunk::GetColorAt(const Vec4& relativedPos)
    {
        Point4 coords = GetVoxelCoords(relativedPos);

        if (IsCoordValid(coords.x(), coords.y(), coords.z()))
        {
            const ColorVoxel& color = GetColorVoxel(coords.x(), coords.y(), coords.z());
            float maxVal = static_cast<float>(std::numeric_limits<uint8_t>::max());
            return Vec3(static_cast<float>(color.GetRed()) / maxVal, static_cast<float>(color.GetGreen()) / maxVal, static_cast<float>(color.GetBlue()) / maxVal);
        }

        return Vec3::Zero();
    }


} // namespace chisel 
