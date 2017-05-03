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

#ifndef CHUNK_H_
#define CHUNK_H_

#include <boost/shared_ptr.hpp>
#include <vector>
#include <Eigen/Core>

#include <open_chisel/geometry/AABB.h>
#include "mesh/Mesh.h"
#include "DistVoxel.h"
#include "ColorVoxel.h"

namespace chisel
{

    typedef Eigen::Vector3i ChunkID;
    typedef int VoxelID;

    struct ChunkStatistics
    {
            size_t numKnownInside;
            size_t numKnownOutside;
            size_t numUnknown;
            float totalWeight;
    };

    template<class VoxelType>
    class Chunk
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Chunk() {}

            Chunk(const ChunkID id, const Eigen::Vector3i& numVoxels, float resolution, bool useColor) :
                ID(id), numVoxels(numVoxels), voxelResolutionMeters(resolution)
            {
                AllocateDistVoxels();
                if(useColor)
                {
                    AllocateColorVoxels();
                }
                origin = Vec3(numVoxels(0) * ID(0) * voxelResolutionMeters, numVoxels(1) * ID(1) * voxelResolutionMeters, numVoxels(2) * ID(2) * voxelResolutionMeters);
            }

            virtual ~Chunk(){}

            void AllocateDistVoxels()
            {
                int totalNum = GetTotalNumVoxels();
                voxels.clear();
                voxels.resize(totalNum, VoxelType());
            }

            void AllocateColorVoxels()
            {
                int totalNum = GetTotalNumVoxels();
                colors.clear();
                colors.resize(totalNum, ColorVoxel());
            }

            inline const ChunkID& GetID() const { return ID; }
            inline ChunkID& GetIDMutable() { return ID; }
            inline void SetID(const ChunkID& id) { ID = id; }

            inline bool HasColors() const { return !colors.empty(); }
            inline bool HasVoxels() const { return !voxels.empty(); }
            inline const std::vector<VoxelType>& GetVoxels() const { return voxels; }

            inline const Eigen::Vector3i& GetNumVoxels() const { return numVoxels; }
            inline float GetVoxelResolutionMeters() const { return voxelResolutionMeters; }

            inline const VoxelType& GetDistVoxel(const VoxelID& voxelID) const { return voxels[voxelID]; }
            inline VoxelType& GetDistVoxelMutable(const VoxelID& voxelID) { return voxels[voxelID]; }

            inline const ColorVoxel& GetColorVoxel(const VoxelID& voxelID) const { return colors[voxelID]; }
            inline ColorVoxel& GetColorVoxelMutable(const VoxelID& voxelID) { return colors[voxelID]; }

            inline const VoxelType& GetDistVoxel(int x, int y, int z) const
            {
                return GetDistVoxel(GetVoxelID(x, y, z));
            }

            inline VoxelType& GetDistVoxelMutable(int x, int y, int z)
            {
                return GetDistVoxelMutable(GetVoxelID(x, y, z));
            }

            inline const VoxelType& GetDistVoxel(const Vec3& relativePos) const
            {
                return GetDistVoxel(GetVoxelID(relativePos));
            }

            inline VoxelType& GetDistVoxelMutable(const Vec3& relativePos)
            {
                return GetDistVoxelMutable(GetVoxelID(relativePos));
            }

            inline const ColorVoxel& GetColorVoxel(int x, int y, int z) const
            {
                return GetColorVoxel(GetVoxelID(x, y, z));
            }

            inline ColorVoxel& GetColorVoxelMutable(int x, int y, int z)
            {
                return GetColorVoxelMutable(GetVoxelID(x, y, z));
            }

            inline const ColorVoxel& GetColorVoxel(const Vec3& relativePos) const
            {
                return GetColorVoxel(GetVoxelID(relativePos));
            }

            inline ColorVoxel& GetColorVoxelMutable(const Vec3& relativePos)
            {
                return GetColorVoxelMutable(GetVoxelID(relativePos));
            }

            inline bool IsCoordValid(int x, int y, int z) const
            {
                return (x >= 0 && x < numVoxels(0) && y >= 0 && y < numVoxels(1) && z >= 0 && z < numVoxels(2));
            }

            inline Vec3 GetCenterWorldCoords() const
            {
                return origin + (numVoxels.cast<float>() * 0.5f * voxelResolutionMeters);
            }

            Point3 GetVoxelCoords(const Vec3& relativeCoords) const
            {
              const float roundingFactor = 1.0f / (voxelResolutionMeters);

              return Point3(static_cast<int>(std::floor(relativeCoords(0) * roundingFactor)),
                            static_cast<int>(std::floor(relativeCoords(1) * roundingFactor)),
                            static_cast<int>(std::floor(relativeCoords(2) * roundingFactor)));
            }

            Vec3 GetWorldCoords(const Point3& coords) const
            {
              return (coords.cast<float>() + Vec3(0.5, 0.5, 0.5)) * voxelResolutionMeters + origin;
            }

            Vec3 GetWorldCoords(const VoxelID& voxelID) const
            {
              Point3 voxelCoords(voxelID % numVoxels(0),
                                 voxelID / numVoxels(0) % numVoxels(1),
                                 voxelID / (numVoxels(0)*numVoxels(1)));

              return GetWorldCoords(voxelCoords);
            }

            Point3 GetLocalCoords(const VoxelID& voxelID) const
            {
              Point3 voxelCoords(voxelID % numVoxels(0),
                                 voxelID / numVoxels(0) % numVoxels(1),
                                 voxelID / (numVoxels(0)*numVoxels(1)));

              return voxelCoords;
            }

            inline VoxelID GetVoxelID(const Vec3& relativePos) const
            {
                return GetVoxelID(GetVoxelCoords(relativePos));
            }

            inline VoxelID GetVoxelID(const Point3& coords) const
            {
                return GetVoxelID(coords.x(), coords.y(), coords.z());
            }

            inline VoxelID GetVoxelID(int x, int y, int z) const
            {
                return (z * numVoxels(1) + y) * numVoxels(0) + x;
            }

            inline size_t GetTotalNumVoxels() const
            {
                return numVoxels(0) * numVoxels(1) * numVoxels(2);
            }

            inline const std::vector<ColorVoxel>& GetColorVoxels() const
            {
                return colors;
            }

            void ComputeStatistics(ChunkStatistics* stats)
            {
                assert(stats != nullptr);

                for (const VoxelType& vox : voxels)
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

            AABB ComputeBoundingBox()
            {
                Vec3 pos = origin;
                Vec3 size = numVoxels.cast<float>() * voxelResolutionMeters;
                return AABB(pos, pos + size);
            }

            inline const Vec3& GetOrigin() const { return origin; }

            Vec3 GetColorAt(const Vec3& relativedPos)
            {
                Point3 coords = GetVoxelCoords(relativedPos);

                if (IsCoordValid(coords.x(), coords.y(), coords.z()))
                {
                    const ColorVoxel& color = GetColorVoxel(coords.x(), coords.y(), coords.z());
                    float maxVal = static_cast<float>(std::numeric_limits<uint8_t>::max());
                    return Vec3(static_cast<float>(color.GetRed()) / maxVal, static_cast<float>(color.GetGreen()) / maxVal, static_cast<float>(color.GetBlue()) / maxVal);
                }

                return Vec3::Zero();
            }

            void SetMesh(MeshPtr mesh) { this->mesh = mesh; }
            MeshConstPtr GetMesh() const { return mesh; }

        protected:
            ChunkID ID;
            Eigen::Vector3i numVoxels;
            float voxelResolutionMeters;
            Vec3 origin;
            std::vector<VoxelType> voxels;
            std::vector<ColorVoxel> colors;
            MeshPtr mesh;
    };

    template<class VoxelType>
    using ChunkPtr = boost::shared_ptr<Chunk<VoxelType>>;

    template<class VoxelType>
    using ChunkConstPtr = boost::shared_ptr<const Chunk<VoxelType>>;

    typedef std::vector<ChunkID, Eigen::aligned_allocator<ChunkID> > ChunkIDList;

} // namespace chisel 

#endif // CHUNK_H_ 
