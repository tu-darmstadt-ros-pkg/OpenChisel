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

    class Chunk
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Chunk();
            Chunk(const ChunkID id, const Eigen::Vector3i& numVoxels, float resolution, bool useColor);
            virtual ~Chunk();

            void AllocateDistVoxels();
            void AllocateColorVoxels();

            inline const ChunkID& GetID() const { return ID; }
            inline ChunkID& GetIDMutable() { return ID; }
            inline void SetID(const ChunkID& id) { ID = id; }

            inline bool HasColors() const { return !colors.empty(); }
            inline bool HasVoxels() const { return !voxels.empty(); }
            inline const std::vector<DistVoxel>& GetVoxels() const { return voxels; }

            inline const Eigen::Vector3i& GetNumVoxels() const { return numVoxels; }
            inline float GetVoxelResolutionMeters() const { return voxelResolutionMeters; }

            inline const DistVoxel& GetDistVoxel(const VoxelID& voxelID) const { return voxels[voxelID]; }
            inline DistVoxel& GetDistVoxelMutable(const VoxelID& voxelID) { return voxels[voxelID]; }
            inline const ColorVoxel& GetColorVoxel(const VoxelID& voxelID) const { return colors[voxelID]; }
            inline ColorVoxel& GetColorVoxelMutable(const VoxelID& voxelID) { return colors[voxelID]; }

            inline Vec3 GetCenterWorldCoords() const
            {
                return origin + (numVoxels.cast<float>() * 0.5f * voxelResolutionMeters);
            }

            Point3 GetVoxelCoords(const Vec3& relativeCoords) const;

            Vec3 GetWorldCoords(const VoxelID& voxelID) const;

            inline VoxelID GetVoxelID(const Point3& coords) const
            {
                return GetVoxelID(coords.x(), coords.y(), coords.z());
            }

            inline VoxelID GetVoxelID(int x, int y, int z) const
            {
                return (z * numVoxels(1) + y) * numVoxels(0) + x;
            }

            inline const DistVoxel& GetDistVoxel(int x, int y, int z) const
            {
                return GetDistVoxel(GetVoxelID(x, y, z));
            }

            inline DistVoxel& GetDistVoxelMutable(int x, int y, int z)
            {
                return GetDistVoxelMutable(GetVoxelID(x, y, z));
            }

            inline const ColorVoxel& GetColorVoxel(int x, int y, int z) const
            {
                return GetColorVoxel(GetVoxelID(x, y, z));
            }

            inline ColorVoxel& GetColorVoxelMutable(int x, int y, int z)
            {
                return GetColorVoxelMutable(GetVoxelID(x, y, z));
            }

            inline bool IsCoordValid(int x, int y, int z) const
            {
                return (x >= 0 && x < numVoxels(0) && y >= 0 && y < numVoxels(1) && z >= 0 && z < numVoxels(2));
            }


            inline size_t GetTotalNumVoxels() const
            {
                return numVoxels(0) * numVoxels(1) * numVoxels(2);
            }

            inline const std::vector<ColorVoxel>& GetColorVoxels() const
            {
                return colors;
            }

            void ComputeStatistics(ChunkStatistics* stats);

            AABB ComputeBoundingBox();

            inline const Vec3& GetOrigin() const { return origin; }

            Vec3 GetColorAt(const Vec3& relativedPos);

            VoxelID GetVoxelID(const Vec3& relativePos) const;

            void SetMesh(MeshPtr mesh) { this->mesh = mesh; }
            MeshConstPtr GetMesh() const { return mesh; }

        protected:
            ChunkID ID;
            Eigen::Vector3i numVoxels;
            float voxelResolutionMeters;
            Vec3 origin;
            std::vector<DistVoxel> voxels;
            std::vector<ColorVoxel> colors;
            MeshPtr mesh;
    };

    typedef boost::shared_ptr<Chunk> ChunkPtr;
    typedef boost::shared_ptr<const Chunk> ChunkConstPtr;
    typedef std::vector<ChunkID, Eigen::aligned_allocator<ChunkID> > ChunkIDList;

} // namespace chisel 

#endif // CHUNK_H_ 
