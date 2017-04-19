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

    typedef chisel::Point4 ChunkID;
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
            Chunk(const ChunkID id, const Point4& numVoxels, float resolution, bool useColor);
            virtual ~Chunk();

            void AllocateDistVoxels();
            void AllocateColorVoxels();

            inline const ChunkID& GetID() const { return ID; }
            inline ChunkID& GetIDMutable() { return ID; }
            inline void SetID(const ChunkID& id) { ID = id; }

            inline bool HasColors() const { return !colors.empty(); }
            inline bool HasVoxels() const { return !voxels.empty(); }
            inline const std::vector<DistVoxel>& GetVoxels() const { return voxels; }

            inline const Point4& GetNumVoxels() const { return numVoxels; }
            inline float GetVoxelResolutionMeters() const { return voxelResolutionMeters; }

            inline const DistVoxel& GetDistVoxel(const VoxelID& voxelID) const { return voxels[voxelID]; }
            inline DistVoxel& GetDistVoxelMutable(const VoxelID& voxelID) { return voxels[voxelID]; }

            inline const ColorVoxel& GetColorVoxel(const VoxelID& voxelID) const { return colors[voxelID]; }
            inline ColorVoxel& GetColorVoxelMutable(const VoxelID& voxelID) { return colors[voxelID]; }

            inline Vec4 GetCenterWorldCoords() const
            {
                return origin + Vec4(numVoxels.x() * 0.5f * voxelResolutionMeters, numVoxels.y() * 0.5f * voxelResolutionMeters, numVoxels.z() * 0.5f * voxelResolutionMeters, 0.0f);
            }

            VoxelID GetVoxelID(const Vec4& relativePos) const;

            inline const DistVoxel& GetDistVoxel(int x, int y, int z) const
            {
                return GetDistVoxel(GetVoxelID(x, y, z));
            }

            inline VoxelID GetVoxelID(const Point4& coords) const
            {
                return GetVoxelID(coords.x(), coords.y(), coords.z());
            }

            inline DistVoxel& GetDistVoxelMutable(int x, int y, int z)
            {
                return GetDistVoxelMutable(GetVoxelID(x, y, z));
            }

            inline const DistVoxel& GetDistVoxel(const Vec4& relativePos) const
            {
                return GetDistVoxel(GetVoxelID(relativePos));
            }

            inline DistVoxel& GetDistVoxelMutable(const Vec4& relativePos)
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

            inline const ColorVoxel& GetColorVoxel(const Vec4& relativePos) const
            {
                return GetColorVoxel(GetVoxelID(relativePos));
            }

            inline ColorVoxel& GetColorVoxelMutable(const Vec4& relativePos)
            {
                return GetColorVoxelMutable(GetVoxelID(relativePos));
            }

            inline bool IsCoordValid(int x, int y, int z) const
            {
                return (x >= 0 && x < numVoxels(0) && y >= 0 && y < numVoxels(1) && z >= 0 && z < numVoxels(2));
            }

            Point4 GetVoxelCoords(const Vec4& relativeCoords) const;

            Vec4 GetWorldCoords(const Point4& coords) const;
            Vec4 GetWorldCoords(const VoxelID& voxelID) const;

            /*inline VoxelID GetVoxelID(const Vec3& relativePos) const
            {
                return GetVoxelID(GetVoxelCoords(relativePos));
            }*/

            /*inline VoxelID GetVoxelID(const Point3& coords) const
            {
                return GetVoxelID(coords.x(), coords.y(), coords.z());
            }*/

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

            void ComputeStatistics(ChunkStatistics* stats);

            AABB ComputeBoundingBox();

            inline const Vec4& GetOrigin() const { return origin; }

            Vec3 GetColorAt(const Vec4& relativedPos);

            void SetMesh(MeshPtr mesh) { this->mesh = mesh; }
            MeshConstPtr GetMesh() const { return mesh; }

        protected:
            ChunkID ID;
            Point4 numVoxels;
            float voxelResolutionMeters;
            Vec4 origin;
            std::vector<DistVoxel> voxels;
            std::vector<ColorVoxel> colors;
            MeshPtr mesh;
    };

    typedef boost::shared_ptr<Chunk> ChunkPtr;
    typedef boost::shared_ptr<const Chunk> ChunkConstPtr;
    typedef std::vector<ChunkID, Eigen::aligned_allocator<ChunkID> > ChunkIDList;

} // namespace chisel 

#endif // CHUNK_H_ 
