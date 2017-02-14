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

#ifndef CHUNKMANAGER_H_
#define CHUNKMANAGER_H_

#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/mesh/Mesh.h>
#include <open_chisel/ColorVoxel.h>
#include <open_chisel/DistVoxel.h>
#include <open_chisel/pointcloud/PointCloud.h>
#include <open_chisel/Chunk.h>
#include <open_chisel/geometry/Frustum.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

namespace chisel
{
    // Spatial hashing function from Matthias Teschner
    // Optimized Spatial Hashing for Collision Detection of Deformable Objects
    struct ChunkHasher
    {
            // Three large primes are used for spatial hashing.
            static constexpr size_t p1 = 73856093;
            static constexpr size_t p2 = 19349663;
            static constexpr size_t p3 = 8349279;

            std::size_t operator()(const ChunkID& key) const
            {
                return ( key(0) * p1 ^ key(1) * p2 ^ key(2) * p3);
            }

            std::size_t operator()(const std::pair<ChunkID, VoxelID>& key) const
            {
                return operator()(key.first);
            }

            std::size_t operator()(const std::pair<ChunkPtr, VoxelID>& key) const
            {
                return operator()(key.first->GetID());
            }
    };

    struct VoxelHasher
    {
            std::size_t operator()(const VoxelID& key) const
            {
                return key;
            }

            std::size_t operator()(const std::pair<ChunkID, VoxelID>& key) const
            {
                return operator()(key.second);
            }

            std::size_t operator()(const std::pair<ChunkPtr, VoxelID>& key) const
            {
                return operator()(key.second);
            }
    };



    typedef std::unordered_map<ChunkID, ChunkPtr, ChunkHasher> ChunkMap;
    typedef std::unordered_map<ChunkID, Vec3, ChunkHasher> ChunkSet;
    typedef std::unordered_map<ChunkID, MeshPtr, ChunkHasher> MeshMap;
    typedef std::unordered_set<std::pair<ChunkPtr, VoxelID>, VoxelHasher> VoxelSet;
    typedef std::unordered_map<ChunkID, VoxelSet, ChunkHasher> ChunkVoxelMap;

    struct IncrementalChanges
    {
      ChunkSet deletedChunks;
      ChunkMap updatedChunks;
      ChunkMap addedChunks;

      // for all chunks contained in the updated chunk set, the updated and carved voxel ids are stored
      ChunkVoxelMap updatedVoxels;
      ChunkVoxelMap carvedVoxels;

      IncrementalChanges()
      {
      }

      IncrementalChanges(const IncrementalChanges& obj)
      {
        deletedChunks.insert(obj.deletedChunks.begin(), obj.deletedChunks.end());
        updatedChunks.insert(obj.updatedChunks.begin(), obj.updatedChunks.end());
        addedChunks.insert(obj.addedChunks.begin(), obj.addedChunks.end());
        updatedVoxels.insert(obj.updatedVoxels.begin(), obj.updatedVoxels.end());
        carvedVoxels.insert(obj.carvedVoxels.begin(), obj.carvedVoxels.end());
      }

      ~IncrementalChanges()
      {
      }

      void clear()
      {
        deletedChunks.clear();
        updatedChunks.clear();
        addedChunks.clear();
        updatedVoxels.clear();
        carvedVoxels.clear();
      }

      ChunkMap merge(const ChunkMap& a, const ChunkMap& b) const
      {
        ChunkMap temp(a);
        temp.insert(b.begin(), b.end());
        return temp;
      }

      ChunkSet mergeToSet(const ChunkMap& a, const ChunkMap& b) const
      {
        ChunkSet chunkset;

        for (const auto& chunk : a)
          chunkset.emplace(chunk.first, chunk.second->GetOrigin());

        for (const auto& chunk : b)
          chunkset.emplace(chunk.first, chunk.second->GetOrigin());

        return chunkset;
      }

      inline ChunkMap getChangedChunks() const
      {
        return merge(addedChunks, updatedChunks);
      }

      inline ChunkMap getAddedChunks() const
      {
        return addedChunks;
      }

      inline ChunkSet getChunkSet(const ChunkMap& chunks) const
      {
        ChunkSet chunkset;

        for (auto& chunk: chunks)
        {
          chunkset.emplace(chunk.first, chunk.second->GetOrigin());
        }

        return chunkset;
      }

      void RememberAddedChunk(ChunkPtr chunk)
      {
        addedChunks.emplace(chunk->GetID(), chunk);
      }

      void RememberUpdatedChunk(ChunkPtr chunk)
      {
        updatedChunks.emplace(chunk->GetID(), chunk);
      }

      void RememberUpdatedChunk(ChunkPtr chunk, const Vec3& chunk_size_meters, ChunkSet& meshes_to_update)
      {
        RememberUpdatedChunk(chunk);

        ChunkID chunk_id = chunk->GetID();
        for (int dx = -1; dx <= 1; dx++)
        {
            for (int dy = -1; dy <= 1; dy++)
            {
                for (int dz = -1; dz <= 1; dz++)
                {
                    meshes_to_update[chunk_id + ChunkID(dx, dy, dz)] = chunk->GetOrigin() + Vec3(dx, dy, dz).cwiseProduct(chunk_size_meters);
                }
            }
        }
      }

      void RememberDeletedChunk(ChunkPtr chunk)
      {
        const ChunkID& chunk_id = chunk->GetID();
        deletedChunks.emplace(chunk_id, chunk->GetOrigin());
        updatedChunks.erase(chunk_id);
        addedChunks.erase(chunk_id);
        updatedVoxels.erase(chunk_id);

        for (size_t voxel_id = 0; voxel_id < chunk->GetTotalNumVoxels(); voxel_id++)
        {
          const chisel::DistVoxel& voxel = chunk->GetDistVoxel(voxel_id);
          if (voxel.IsValid())
            RememberCarvedVoxel(chunk, voxel_id);
        }
      }

      void RememberUpdatedVoxel(ChunkPtr chunk, const VoxelID& voxelID)
      {
        const ChunkID& chunk_id = chunk->GetID();
        updatedVoxels[chunk_id].insert(std::make_pair(chunk, voxelID));

        // delete any carve information
        if (RemoveFromChunkVoxelMap(carvedVoxels, chunk, voxelID))
          deletedChunks.erase(chunk_id);
      }

      void RememberCarvedVoxel(ChunkPtr chunk, const VoxelID& voxelID)
      {
        const ChunkID& chunk_id = chunk->GetID();
        carvedVoxels[chunk_id].insert(std::make_pair(chunk, voxelID));

        // delete any add and update information
        if (RemoveFromChunkVoxelMap(updatedVoxels, chunk, voxelID))
          addedChunks.erase(chunk_id);
      }

      bool RemoveFromChunkVoxelMap(ChunkVoxelMap& map, ChunkPtr chunk, const VoxelID& voxelID)
      {
        // find voxel entry
        auto itr = map.find(chunk->GetID());
        if (itr != map.end())
        {
          VoxelSet& voxel_set = itr->second;

          // remove entry from voxelset
          voxel_set.erase(std::make_pair(chunk, voxelID));

          // also delete voxel set itself if empty
          if (voxel_set.empty())
          {
            map.erase(itr);
            return true;
          }
        }

        return false;
      }
    };

    typedef boost::shared_ptr<IncrementalChanges> IncrementalChangesPtr;
    typedef boost::shared_ptr<const IncrementalChanges> IncrementalChangesConstPtr;

    class ChunkManager
    {
        public:
            ChunkManager();
            ChunkManager(const Eigen::Vector3i& chunkSize, float voxelResolution, bool color);
            virtual ~ChunkManager();

            inline const ChunkMap& GetChunks() const { return *chunks; }
            inline ChunkMap& GetMutableChunks() { return *chunks; }

            inline boost::shared_ptr<ChunkMap> GetMutableChunksPointer() { return chunks; }
            inline boost::shared_ptr<const ChunkMap> GetChunksPointer() const { return chunks; }
            inline void SetChunksPointer(boost::shared_ptr<ChunkMap> data) {chunks = data; }

            inline const Vec3& GetRoundingFactor() const { return roundingFactor; }

            inline bool HasChunk(const ChunkID& chunk) const
            {
                return chunks->find(chunk) != chunks->end();
            }

            inline ChunkPtr GetChunk(const ChunkID& chunk) const
            {
                auto itr = chunks->find(chunk);
                if (itr != chunks->end())
                  return itr->second;

                return ChunkPtr();
            }

            inline void AddChunk(const ChunkPtr& chunk)
            {
                chunks->insert(std::make_pair(chunk->GetID(), chunk));
            }

            inline bool RemoveChunk(const ChunkID& chunk)
            {
                ChunkPtr chunk_ptr = GetChunk(chunk);

                if (chunk_ptr)
                {
                    RememberDeletedChunk(chunk_ptr);
                    chunks->erase(chunk_ptr->GetID());
                    return true;
                }

                return false;
            }

            inline bool RemoveChunk(const ChunkPtr& chunk)
            {
                return RemoveChunk(chunk->GetID());
            }

            inline bool HasChunk(int x, int y, int z) const { return HasChunk(ChunkID(x, y, z)); }
            inline ChunkPtr GetChunk(int x, int y, int z) const { return GetChunk(ChunkID(x, y, z)); }

            inline ChunkPtr GetChunkAt(const Vec3& pos) const
            {
                return GetChunk(GetIDAt(pos));
            }

            inline ChunkID GetIDAt(const Vec3& pos) const
            {
                return ChunkID(static_cast<int>(std::floor(pos(0) * roundingFactor(0))),
                               static_cast<int>(std::floor(pos(1) * roundingFactor(1))),
                               static_cast<int>(std::floor(pos(2) * roundingFactor(2))));
            }

            inline void SetThreadingParameters(unsigned int maxThreads, unsigned int threadTreshold)
            {
              this->maxThreads = maxThreads;
              this->threadTreshold = threadTreshold;
            }

            const DistVoxel* GetDistanceVoxel(const Vec3& pos) const;
            DistVoxel* GetDistanceVoxelMutable(const Vec3& pos);
            const ColorVoxel* GetColorVoxel(const Vec3& pos) const;
            ColorVoxel* GetColorVoxelMutable(const Vec3& pos);

            void GetChunkIDsIntersecting(const AABB& box, ChunkIDList* chunkList);
            void GetChunkIDsIntersecting(const Frustum& frustum, ChunkIDList* chunkList);
            void GetChunkIDsIntersecting(const PointCloud& cloud, const Transform& cameraTransform, float truncation, float minDist, float maxDist, ChunkIDList* chunkList);
            void CreateChunk(const ChunkID& id);
            void ClearPassedVoxels(const Vec3& start, const Vec3& end, ChunkSet* updatedChunks);

            void GenerateMesh(const ChunkPtr& chunk, Mesh* mesh);
            void ColorizeMesh(Mesh* mesh);
            Vec3 InterpolateColor(const Vec3& colorPos);

            void CacheCentroids();
            void ExtractBorderVoxelMesh(const ChunkPtr& chunk, const Eigen::Vector3i& index, const Eigen::Vector3f& coordinates, VertIndex* nextMeshIndex, Mesh* mesh);
            void ExtractInsideVoxelMesh(const ChunkPtr& chunk, const Eigen::Vector3i& index, const Vec3& coords, VertIndex* nextMeshIndex, Mesh* mesh);

            inline const MeshMap& GetAllMeshes() const { return *allMeshes; }
            inline MeshMap& GetAllMutableMeshes() { return *allMeshes; }
            inline MeshConstPtr GetMesh(const ChunkID& chunkID) const
            {
              auto itr = allMeshes->find(chunkID);
              if (itr != allMeshes->end())
                return itr->second;
              return MeshConstPtr();
            }
            inline MeshPtr GetMutableMesh(const ChunkID& chunkID)
            {
              auto itr = allMeshes->find(chunkID);
              if (itr != allMeshes->end())
                return itr->second;
              return MeshPtr();
            }
            inline bool HasMesh(const ChunkID& chunkID) const { return allMeshes->find(chunkID) != allMeshes->end(); }

            inline bool GetUseColor() { return useColor; }
            inline bool GetUseColor() const { return useColor; }

            void RecomputeMesh(const ChunkID& chunkID, std::mutex& mutex);
            void RecomputeMeshes(const ChunkSet& chunks);
            void RecomputeMeshes(const ChunkMap& chunks);
            void ComputeNormalsFromGradients(Mesh* mesh);

            inline const Eigen::Vector3i& GetChunkSize() const { return chunkSize; }
            inline const Eigen::Vector3f& GetChunkSizeMeters() const { return chunkSizeMeters; }
            inline float GetResolution() const { return voxelResolutionMeters; }

            inline const Vec3List& GetCentroids() const { return centroids; }

            void PrintMemoryStatistics() const;

            void Reset();

            bool GetSDFAndGradient(const Eigen::Vector3f& pos, double* dist, Eigen::Vector3f* grad) const;
            bool GetSDF(const Eigen::Vector3f& pos, double* dist) const;

            inline void DeleteEmptyChunks() { DeleteEmptyChunks(incrementalChanges->getChangedChunks()); }
            void DeleteEmptyChunks(const ChunkMap& chunk_set);

            IncrementalChangesConstPtr getIncrementalChanges(){ return incrementalChanges; }
            void clearIncrementalChanges(){ incrementalChanges->clear(); }

            inline void RememberAddedChunk(ChunkPtr chunk) { incrementalChanges->RememberAddedChunk(chunk); }

            inline void RememberUpdatedChunk(ChunkPtr chunk) { incrementalChanges->RememberUpdatedChunk(chunk); }
            inline void RememberUpdatedChunk(ChunkPtr chunk, ChunkSet& meshes_to_update) { incrementalChanges->RememberUpdatedChunk(chunk, GetChunkSizeMeters(), meshes_to_update); }

            inline void RememberDeletedChunk(ChunkPtr chunk) { incrementalChanges->RememberDeletedChunk(chunk); }

            inline void RememberUpdatedVoxel(ChunkPtr chunk, const VoxelID& voxelID) { incrementalChanges->RememberUpdatedVoxel(chunk, voxelID); }
            inline void RememberCarvedVoxel(ChunkPtr chunk, const VoxelID& voxelID) { incrementalChanges->RememberCarvedVoxel(chunk, voxelID); }

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        protected:
            boost::shared_ptr<ChunkMap> chunks;
            Eigen::Vector3i chunkSize;
            Eigen::Vector3f chunkSizeMeters;
            float voxelResolutionMeters;
            Vec3List centroids;
            Eigen::Matrix<int, 3, 8> cubeIndexOffsets;
            boost::shared_ptr<MeshMap> allMeshes;
            bool useColor;
            unsigned int maxThreads;
            unsigned int threadTreshold;
            IncrementalChangesPtr incrementalChanges;

        private:
            Vec3 roundingFactor;
    };

    typedef boost::shared_ptr<ChunkManager> ChunkManagerPtr;
    typedef boost::shared_ptr<const ChunkManager> ChunkManagerConstPtr;

    typedef boost::shared_ptr<ChunkSet> ChunkSetPtr;
    typedef boost::shared_ptr<const ChunkSet> ChunkSetConstPtr;
} // namespace chisel 

#endif // CHUNKMANAGER_H_ 
