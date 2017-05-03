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
#include <open_chisel/geometry/AABB.h>
#include <open_chisel/threading/Threading.h>
#include <open_chisel/geometry/AABB.h>
#include <open_chisel/marching_cubes/MarchingCubes.h>
#include <open_chisel/geometry/Raycast.h>

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

    template<class VoxelType>
    std::size_t operator()(const std::pair<ChunkPtr<VoxelType>, VoxelID>& key) const
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

    template<class VoxelType>
    std::size_t operator()(const std::pair<ChunkPtr<VoxelType>, VoxelID>& key) const
    {
        return operator()(key.second);
    }
};

template<class VoxelType>
struct UpdatedVoxel
{

    UpdatedVoxel(ChunkPtr<VoxelType> chunk_, const int voxel_id_, const float weight_diff_, const float sdf_diff_):
        chunk(chunk_), voxel_id(voxel_id_), weight_diff(weight_diff_), sdf_diff(sdf_diff_){}

    UpdatedVoxel(ChunkPtr<VoxelType> chunk_, const int voxel_id_):
        chunk(chunk_), voxel_id(voxel_id_), weight_diff(0.0f), sdf_diff(0.0f){}

    bool operator==(const UpdatedVoxel& other) const
    {
        return (this->voxel_id == other.voxel_id && this->chunk->GetID() == other.chunk->GetID());
    }

    bool operator!=(const UpdatedVoxel &other) const
    {
        return !(*this == other);
    }

    ChunkPtr<VoxelType> chunk;
    int voxel_id;
    float weight_diff;
    float sdf_diff;
};

struct UpdatedVoxelHasher
{
    template<class ChunkPtrType>
    std::size_t operator()(const UpdatedVoxel<ChunkPtrType>& key) const
    {
        return key.voxel_id;
    }
};

template<class VoxelType>
using  ChunkMap = std::unordered_map<ChunkID, ChunkPtr<VoxelType>, ChunkHasher>;

template<class VoxelType>
using  UpdatedVoxelSet = std::unordered_set<UpdatedVoxel<VoxelType>, UpdatedVoxelHasher>;

template<class VoxelType>
using  VoxelSet = std::unordered_set<std::pair<ChunkPtr<VoxelType>, VoxelID>, VoxelHasher>;

template<class VoxelType>
using  ChunkVoxelMap = std::unordered_map<ChunkID, VoxelSet<VoxelType>, ChunkHasher>;

template<class VoxelType>
using  ChunkUpdatedVoxelMap = std::unordered_map<ChunkID, UpdatedVoxelSet<VoxelType>, ChunkHasher>;

typedef std::unordered_map<ChunkID, Vec3, ChunkHasher> ChunkSet;
typedef std::unordered_map<ChunkID, MeshPtr, ChunkHasher> MeshMap;

template<class VoxelType>
struct IncrementalChanges
{
    ChunkSet deletedChunks;
    ChunkMap<VoxelType> updatedChunks;
    ChunkMap<VoxelType> addedChunks;

    // for all chunks contained in the updated chunk set, the updated and carved voxel ids are stored
    ChunkUpdatedVoxelMap<VoxelType> updatedVoxels;
    ChunkVoxelMap<VoxelType> carvedVoxels;

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

    ChunkMap<VoxelType> merge(const ChunkMap<VoxelType>& a, const ChunkMap<VoxelType>& b) const
    {
        ChunkMap<VoxelType> temp(a);
        temp.insert(b.begin(), b.end());
        return temp;
    }

    ChunkSet mergeToSet(const ChunkMap<VoxelType>& a, const ChunkMap<VoxelType>& b) const
    {
        ChunkSet chunkset;

        for (const auto& chunk : a)
            chunkset.emplace(chunk.first, chunk.second->GetOrigin());

        for (const auto& chunk : b)
            chunkset.emplace(chunk.first, chunk.second->GetOrigin());

        return chunkset;
    }

    inline ChunkMap<VoxelType> getChangedChunks() const
    {
        return merge(addedChunks, updatedChunks);
    }

    inline ChunkMap<VoxelType> getAddedChunks() const
    {
        return addedChunks;
    }

    inline ChunkSet getChunkSet(const ChunkMap<VoxelType>& chunks) const
    {
        ChunkSet chunkset;

        for (auto& chunk: chunks)
        {
            chunkset.emplace(chunk.first, chunk.second->GetOrigin());
        }

        return chunkset;
    }

    void RememberAddedChunk(ChunkPtr<VoxelType> chunk)
    {
        ChunkID chunk_id = chunk->GetID();
        addedChunks.emplace(chunk_id, chunk);
        deletedChunks.erase(chunk_id);
    }

    void RememberUpdatedChunk(ChunkPtr<VoxelType> chunk)
    {
        ChunkID chunk_id = chunk->GetID();
        updatedChunks.emplace(chunk_id, chunk);
        deletedChunks.erase(chunk_id);
    }

    void RememberUpdatedChunk(ChunkPtr<VoxelType> chunk, const Vec3& chunk_size_meters, ChunkSet& meshes_to_update)
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

    void RememberDeletedChunk(const ChunkID& chunk_id, const Vec3& origin)
    {
        deletedChunks.emplace(chunk_id, origin);
        addedChunks.erase(chunk_id);
        updatedChunks.erase(chunk_id);
        updatedVoxels.erase(chunk_id);
    }

    void RememberDeletedChunk(ChunkPtr<VoxelType> chunk)
    {
        RememberDeletedChunk(chunk->GetID(), chunk->GetOrigin());

        for (size_t voxel_id = 0; voxel_id < chunk->GetTotalNumVoxels(); voxel_id++)
        {
            const auto& voxel = chunk->GetDistVoxel(voxel_id);
            if (voxel.IsValid())
                RememberCarvedVoxel(chunk, voxel_id);
        }
    }

    void RememberUpdatedVoxel(ChunkPtr<VoxelType> chunk, const VoxelID& voxelID, const float weight_diff, const float sdf_diff)
    {
        const ChunkID& chunk_id = chunk->GetID();
        updatedVoxels[chunk_id].emplace(UpdatedVoxel<VoxelType>(chunk, voxelID, weight_diff, sdf_diff));

        // delete any carve information
        if (RemoveFromChunkVoxelMap(carvedVoxels, chunk, voxelID))
            deletedChunks.erase(chunk_id);
    }

    void RememberCarvedVoxel(ChunkPtr<VoxelType> chunk, const VoxelID& voxelID)
    {
        const ChunkID& chunk_id = chunk->GetID();
        carvedVoxels[chunk_id].insert(std::make_pair(chunk, voxelID));

        // delete any add and update information
        if (RemoveFromChunkVoxelMap(updatedVoxels, chunk, voxelID))
            addedChunks.erase(chunk_id);
    }

    bool RemoveFromChunkVoxelMap(ChunkVoxelMap<VoxelType>& map, ChunkPtr<VoxelType> chunk, const VoxelID& voxelID)
    {
        // find voxel entry
        auto itr = map.find(chunk->GetID());
        if (itr != map.end())
        {
            VoxelSet<VoxelType>& voxel_set = itr->second;

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

    bool RemoveFromChunkVoxelMap(ChunkUpdatedVoxelMap<VoxelType>& map, ChunkPtr<VoxelType> chunk, const VoxelID& voxelID)
    {
        // find voxel entry
        auto itr = map.find(chunk->GetID());
        if (itr != map.end())
        {
            UpdatedVoxelSet<VoxelType>& changed_voxel_set = itr->second;

            // remove entry from voxelset
            changed_voxel_set.erase(UpdatedVoxel<VoxelType>(chunk, voxelID));

            // also delete voxel set itself if empty
            if (changed_voxel_set.empty())
            {
                map.erase(itr);
                return true;
            }
        }

        return false;
    }
};

template<class VoxelType>
using IncrementalChangesPtr = boost::shared_ptr<IncrementalChanges<VoxelType>>;

template<class VoxelType>
using IncrementalChangesConstPtr = boost::shared_ptr<const IncrementalChanges<VoxelType>>;

template<class VoxelType>
class ChunkManager
{
public:
    ChunkManager() :
        chunkSize(16, 16, 16), voxelResolutionMeters(0.03f), minimumWeight(0.0f),volumeBoundingBox()
    {
        chunkSizeMeters = chunkSize.cast<float>() * voxelResolutionMeters;
        CacheCentroids();
        maxThreads = 4;
        threadTreshold = 500;
        chunks = boost::make_shared<ChunkMap<VoxelType>>();
        allMeshes = boost::make_shared<MeshMap>();
        incrementalChanges = boost::make_shared<IncrementalChanges<VoxelType>>();
        roundingFactor = chunkSizeMeters.cwiseInverse();
    }

    ChunkManager(const Eigen::Vector3i& chunkSize, float voxelResolution, bool color, float minWeight, const Vec3& mapOrigin) :
        chunkSize(chunkSize), voxelResolutionMeters(voxelResolution), useColor(color), minimumWeight(minWeight), origin(mapOrigin), volumeBoundingBox()
    {
        chunkSizeMeters = chunkSize.cast<float>() * voxelResolutionMeters;
        CacheCentroids();
        maxThreads = 4;
        threadTreshold = 500;
        chunks = boost::make_shared<ChunkMap<VoxelType>>();
        allMeshes = boost::make_shared<MeshMap>();
        incrementalChanges = boost::make_shared<IncrementalChanges<VoxelType>>();
        roundingFactor = chunkSizeMeters.cwiseInverse();
    }

    virtual ~ChunkManager(){}

    inline const ChunkMap<VoxelType>& GetChunks() const { return *chunks; }
    inline ChunkMap<VoxelType>& GetMutableChunks() { return *chunks; }

    inline boost::shared_ptr<ChunkMap<VoxelType>> GetMutableChunksPointer() { return chunks; }
    inline boost::shared_ptr<const ChunkMap<VoxelType>> GetChunksPointer() const { return chunks; }
    inline void SetChunksPointer(boost::shared_ptr<ChunkMap<VoxelType>> data) {chunks = data; }

    inline const Vec3& GetRoundingFactor() const { return roundingFactor; }

    inline bool HasChunk(const ChunkID& chunk) const
    {
        return chunks->find(chunk) != chunks->end();
    }

    inline ChunkPtr<VoxelType> GetChunk(const ChunkID& chunk) const
    {
        auto itr = chunks->find(chunk);
        if (itr != chunks->end())
            return itr->second;

        return ChunkPtr<VoxelType>();
    }

    inline void AddChunk(const ChunkPtr<VoxelType>& chunk)
    {
        chunks->insert(std::make_pair(chunk->GetID(), chunk));
    }

    inline bool RemoveChunk(const ChunkID& chunk)
    {
        return RemoveChunk(GetChunk(chunk));
    }

    inline bool RemoveChunk(const ChunkPtr<VoxelType>& chunk)
    {
        if (chunk)
        {
            RememberDeletedChunk(chunk);
            chunks->erase(chunk->GetID());
            return true;
        }
        return false;
    }

    inline bool HasChunk(int x, int y, int z) const { return HasChunk(ChunkID(x, y, z)); }
    inline ChunkPtr<VoxelType> GetChunk(int x, int y, int z) const { return GetChunk(ChunkID(x, y, z)); }

    inline ChunkPtr<VoxelType> GetChunkAt(const Vec3& pos) const
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

    const VoxelType* GetDistanceVoxel(const Vec3& pos) const
    {
        const ChunkPtr<VoxelType> chunk = GetChunk(GetIDAt(pos));

        if (chunk)
        {
            Vec3 rel = (pos - chunk->GetOrigin());
            return &(chunk->GetDistVoxel(chunk->GetVoxelID(rel)));
        }
        else
            return nullptr;
    }

    VoxelType* GetDistanceVoxelMutable(const Vec3& pos)
    {
        ChunkPtr<VoxelType> chunk = GetChunkAt(pos);

        if (chunk)
        {
            Vec3 rel = (pos - chunk->GetOrigin());
            return &(chunk->GetDistVoxelMutable(chunk->GetVoxelID(rel)));
        }
        else
            return nullptr;
    }

    const VoxelType* GetDistanceVoxelGlobal(const Vec3& global_pos) const
    {
        Vec3 local_pos = global_pos - origin;
        const ChunkPtr<VoxelType> chunk = GetChunk(GetIDAt(local_pos));

        if (chunk)
        {
            Vec3 rel = (local_pos - chunk->GetOrigin());
            return &(chunk->GetDistVoxel(chunk->GetVoxelID(rel)));
        }
        else
            return nullptr;
    }

    const VoxelType* GetCoarsedDistanceVoxelGlobal(const Vec3& global_pos, float coarsening_factor) const
    {
        Vec3 local_pos = global_pos - origin;
        float scaled_res = voxelResolutionMeters * coarsening_factor;
        local_pos.x() = floor(local_pos.x() * (1 / scaled_res)) * scaled_res;
        local_pos.y() = floor(local_pos.y() * (1 / scaled_res)) * scaled_res;
        local_pos.z() = floor(local_pos.z() * (1 / scaled_res)) * scaled_res;

        const ChunkPtr<VoxelType> chunk = GetChunk(GetIDAt(local_pos));

        if (chunk)
        {
            Vec3 rel = (local_pos - chunk->GetOrigin());
            return &(chunk->GetDistVoxel(chunk->GetVoxelID(rel)));
        }
        else
            return nullptr;
    }

    const ColorVoxel* GetColorVoxel(const Vec3& pos) const
    {
        const ChunkPtr<VoxelType> chunk = GetChunk(GetIDAt(pos));

        if (chunk && chunk->HasColors())
        {
            Vec3 rel = (pos - chunk->GetOrigin());
            return &(chunk->GetColorVoxel(chunk->GetVoxelID(rel)));
        }
        else
            return nullptr;
    }

    ColorVoxel* GetColorVoxelMutable(const Vec3& pos)
    {
        ChunkPtr<VoxelType> chunk = GetChunkAt(pos);

        if (chunk && chunk->HasColors())
        {
            Vec3 rel = (pos - chunk->GetOrigin());
            return &(chunk->GetColorVoxelMutable(chunk->GetVoxelID(rel)));
        }
        else
            return nullptr;
    }

    bool GetClosestVoxelPosition(const Vec3& pos, Vec3& voxel_pos) const
    {
        ChunkPtr<VoxelType> chunk = GetChunkAt(pos);
        if (chunk)
        {
            voxel_pos = chunk->GetWorldCoords(chunk->GetVoxelCoords(pos - chunk->GetOrigin()));
            return true;
        }
        else
            return false;
    }


    void GetChunkIDsIntersecting(const AABB& box, ChunkIDList* chunkList)
    {
        assert(chunkList != nullptr);

        ChunkID minID = GetIDAt(box.min);
        ChunkID maxID = GetIDAt(box.max) + Eigen::Vector3i(1, 1, 1);

        for (int x = minID(0); x < maxID(0); x++)
        {
            for (int y = minID(1); y < maxID(1); y++)
            {
                for (int z = minID(2); z < maxID(2); z++)
                {
                    chunkList->push_back(ChunkID(x, y, z));
                }
            }
        }
    }

    void GetChunkIDsIntersecting(const Frustum& frustum, ChunkIDList* chunkList)
    {
        assert(chunkList != nullptr);

        AABB frustumAABB;
        frustum.ComputeBoundingBox(&frustumAABB);

        ChunkID minID = GetIDAt(frustumAABB.min);
        ChunkID maxID = GetIDAt(frustumAABB.max);

        //printf("FrustumAABB: %f %f %f %f %f %f\n", frustumAABB.min.x(), frustumAABB.min.y(), frustumAABB.min.z(), frustumAABB.max.x(), frustumAABB.max.y(), frustumAABB.max.z());
        //printf("Frustum min: %d %d %d max: %d %d %d\n", minID.x(), minID.y(), minID.z(), maxID.x(), maxID.y(), maxID.z());
        for (int x = minID(0) - 1; x <= maxID(0) + 1; x++)
        {
            for (int y = minID(1) - 1; y <= maxID(1) + 1; y++)
            {
                for (int z = minID(2) - 1; z <= maxID(2) + 1; z++)
                {
                    Vec3 min = Vec3(x * chunkSize(0), y * chunkSize(1), z * chunkSize(2)) * voxelResolutionMeters;
                    Vec3 max = min + chunkSize.cast<float>() * voxelResolutionMeters;
                    AABB chunkBox(min, max);
                    if(frustum.Intersects(chunkBox))
                    {
                        chunkList->push_back(ChunkID(x, y, z));
                    }
                }
            }
        }

        //printf("%lu chunks intersect frustum\n", chunkList->size());
    }

    void GetChunkIDsIntersecting(const PointCloud& cloud, const Transform& cameraTransform, float truncation, float minDist, float maxDist, ChunkIDList* chunkList)
    {
        assert(!!chunkList);
        chunkList->clear();
        ChunkMap<VoxelType> map;
        Point3 minVal(-std::numeric_limits<int>::max(), -std::numeric_limits<int>::max(), -std::numeric_limits<int>::max());
        Point3 maxVal(std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), std::numeric_limits<int>::max());
        //size_t numPoints = cloud.GetPoints().size();
        //size_t i = 0;
        for (const Vec3& point : cloud.GetPoints())
        {
            Vec3 end = cameraTransform * point;
            Vec3 start = cameraTransform.translation();
            float len = (end - start).norm();

            if(len > maxDist || len < minDist)
            {
                continue;
            }

            Vec3 dir = (end - start).normalized();
            Vec3 truncStart = end - dir * truncation;
            Vec3 truncEnd = end + dir * truncation;
            Vec3 startInt = Vec3(truncStart.x() * roundingFactor(0) , truncStart.y() * roundingFactor(1), truncStart.z() * roundingFactor(2));
            Vec3 endInt = Vec3(truncEnd.x() * roundingFactor(0), truncEnd.y() * roundingFactor(1), truncEnd.z() * roundingFactor(2));

            Point3List intersectingChunks;
            Raycast(startInt, endInt, minVal, maxVal, &intersectingChunks);

            for (const Point3& id : intersectingChunks)
            {
                if(map.find(id) == map.end())
                    map[id] = ChunkPtr<VoxelType>();
            }
        }

        for (const std::pair<ChunkID, ChunkPtr<VoxelType>>& it : map)
        {
            chunkList->push_back(it.first);
        }

    }

    ChunkPtr<VoxelType> CreateChunk(const ChunkID& id)
    {
        auto chunk = boost::allocate_shared<Chunk<VoxelType>>(Eigen::aligned_allocator<Chunk<VoxelType>>(), id, chunkSize, voxelResolutionMeters, useColor);
        AddChunk(chunk);
        RememberAddedChunk(chunk);
        volumeBoundingBox.ExtendToInclude(chunk->ComputeBoundingBox());
        return chunk;
    }
    void ClearPassedVoxels(const Vec3& start, const Vec3& end, float voxelCarvingResetTresh = std::numeric_limits<float>::max(), ChunkVoxelMap<VoxelType>* carvedVoxels = nullptr)
    {
        float roundingFactor = 1/voxelResolutionMeters;
        const Vec3 startRounded = start * roundingFactor;
        const Vec3 endRounded = end * roundingFactor;

        Point3List passedVoxels;
        Raycast(startRounded, endRounded, passedVoxels);

        const Vec3 voxelShift (0.5 * voxelResolutionMeters, 0.5 * voxelResolutionMeters, 0.5 * voxelResolutionMeters);

        for (const Point3& voxelCoords: passedVoxels)
        {
            Vec3 voxelPos = voxelCoords.cast<float>() * voxelResolutionMeters +  voxelShift;
            ChunkPtr<VoxelType> chunk = GetChunkAt(voxelPos);
            if(chunk)
            {
                Vec3 rel = (voxelPos - chunk->GetOrigin());
                VoxelID voxelID = chunk->GetVoxelID(rel);

                VoxelSet<VoxelType>* voxel_set;
                std::pair<ChunkPtr<VoxelType>, VoxelID> voxel_entry;

                if (carvedVoxels)
                {
                    voxel_set = &(*carvedVoxels)[chunk->GetID()];
                    voxel_entry = std::make_pair(chunk, voxelID);

                    // never carve recently updated voxels
                    if (voxel_set->find(voxel_entry) != voxel_set->end())
                        continue;
                }

                VoxelType& voxel = chunk->GetDistVoxelMutable(voxelID);
                if(voxel.IsValid())
                {
                    if (voxel.Carve(voxelCarvingResetTresh))
                        RememberCarvedVoxel(chunk, voxelID);
                    if (voxel_set)
                        voxel_set->insert(voxel_entry);
                }
            }
            else
            {
                continue;
            }
        }
    }

    void GenerateMesh(const ChunkPtr<VoxelType>& chunk, Mesh* mesh)
    {
        assert(mesh != nullptr);

        mesh->Clear();
        const int maxX = chunkSize(0);
        const int maxY = chunkSize(1);
        const int maxZ = chunkSize(2);


        Eigen::Vector3i index;
        VoxelID i = 0;
        VertIndex nextIndex = 0;

        // For voxels not bordering the outside, we can use a more efficient function.
        for (index.z() = 0; index.z() < maxZ - 1; index.z()++)
        {
            for (index.y() = 0; index.y() < maxY - 1; index.y()++)
            {
                for (index.x() = 0; index.x() < maxX - 1; index.x()++)
                {
                    i = chunk->GetVoxelID(index.x(), index.y(), index.z());
                    ExtractInsideVoxelMesh(chunk, index, centroids.at(i) + chunk->GetOrigin(), &nextIndex, mesh);
                }
            }
        }

        // Max X plane (takes care of max-Y corner as well).
        i = 0;
        index.x() = maxX - 1;
        for (index.z() = 0; index.z() < maxZ - 1; index.z()++)
        {
            for (index.y() = 0; index.y() < maxY; index.y()++)
            {
                i = chunk->GetVoxelID(index.x(), index.y(), index.z());
                ExtractBorderVoxelMesh(chunk, index, centroids.at(i) + chunk->GetOrigin(), &nextIndex, mesh);
            }
        }

        // Max Y plane.
        i = 0;
        index.y() = maxY - 1;
        for (index.z() = 0; index.z() < maxZ - 1; index.z()++)
        {
            for (index.x() = 0; index.x() < maxX - 1; index.x()++)
            {
                i = chunk->GetVoxelID(index.x(), index.y(), index.z());
                ExtractBorderVoxelMesh(chunk, index, centroids.at(i) + chunk->GetOrigin(), &nextIndex, mesh);
            }
        }

        // Max Z plane (also takes care of corners).
        i = 0;
        index.z() = maxZ - 1;
        for (index.y() = 0; index.y() < maxY; index.y()++)
        {
            for (index.x() = 0; index.x() < maxX; index.x()++)
            {
                i = chunk->GetVoxelID(index.x(), index.y(), index.z());
                ExtractBorderVoxelMesh(chunk, index, centroids.at(i) + chunk->GetOrigin(), &nextIndex, mesh);
            }
        }

        //printf("Generated a new mesh with %lu verts, %lu norm, and %lu idx\n", mesh->vertices.size(), mesh->normals.size(), mesh->indices.size());

        assert(mesh->vertices.size() == mesh->normals.size());
        assert(mesh->vertices.size() == mesh->indices.size());
    }

    void ColorizeMesh(Mesh* mesh)
    {
        assert(mesh != nullptr);

        mesh->colors.clear();
        mesh->colors.resize(mesh->vertices.size());
        for (size_t i = 0; i < mesh->vertices.size(); i++)
        {
            const Vec3& vertex = mesh->vertices.at(i);
            mesh->colors[i] = InterpolateColor(vertex);
        }
    }

    Vec3 InterpolateColor(const Vec3& colorPos)
    {
        const float& x = colorPos(0);
        const float& y = colorPos(1);
        const float& z = colorPos(2);
        const float round = 1/voxelResolutionMeters;
        const int x_0 = static_cast<int>(std::floor(x * round));
        const int y_0 = static_cast<int>(std::floor(y * round));
        const int z_0 = static_cast<int>(std::floor(z * round ));
        const int x_1 = x_0 + 1;
        const int y_1 = y_0 + 1;
        const int z_1 = z_0 + 1;


        const ColorVoxel* v_000 = GetColorVoxel(Vec3(x_0, y_0, z_0));
        const ColorVoxel* v_001 = GetColorVoxel(Vec3(x_0, y_0, z_1));
        const ColorVoxel* v_011 = GetColorVoxel(Vec3(x_0, y_1, z_1));
        const ColorVoxel* v_111 = GetColorVoxel(Vec3(x_1, y_1, z_1));
        const ColorVoxel* v_110 = GetColorVoxel(Vec3(x_1, y_1, z_0));
        const ColorVoxel* v_100 = GetColorVoxel(Vec3(x_1, y_0, z_0));
        const ColorVoxel* v_010 = GetColorVoxel(Vec3(x_0, y_1, z_0));
        const ColorVoxel* v_101 = GetColorVoxel(Vec3(x_1, y_0, z_1));

        if(!v_000 || !v_001 || !v_011 || !v_111 || !v_110 || !v_100 || !v_010 || !v_101)
        {
            const ChunkID& chunkID = GetIDAt(colorPos);

            if(!HasChunk(chunkID))
            {
                return Vec3(0, 0, 0);
            }
            else
            {
                const ChunkPtr<VoxelType>& chunk = GetChunk(chunkID);
                return chunk->GetColorAt(colorPos - chunk->GetOrigin());
            }
        }

        float xd = (x - x_0) / (x_1 - x_0);
        float yd = (y - y_0) / (y_1 - y_0);
        float zd = (z - z_0) / (z_1 - z_0);
        float red, green, blue = 0.0f;
        {
            float c_00 = v_000->GetRed() * (1 - xd) + v_100->GetRed() * xd;
            float c_10 = v_010->GetRed() * (1 - xd) + v_110->GetRed() * xd;
            float c_01 = v_001->GetRed() * (1 - xd) + v_101->GetRed() * xd;
            float c_11 = v_011->GetRed() * (1 - xd) + v_111->GetRed() * xd;
            float c_0 = c_00 * (1 - yd) + c_10 * yd;
            float c_1 = c_01 * (1 - yd) + c_11 * yd;
            float c = c_0 * (1 - zd) + c_1 * zd;
            red = c / 255.0f;
        }
        {
            float c_00 = v_000->GetGreen() * (1 - xd) + v_100->GetGreen() * xd;
            float c_10 = v_010->GetGreen() * (1 - xd) + v_110->GetGreen() * xd;
            float c_01 = v_001->GetGreen() * (1 - xd) + v_101->GetGreen() * xd;
            float c_11 = v_011->GetGreen() * (1 - xd) + v_111->GetGreen() * xd;
            float c_0 = c_00 * (1 - yd) + c_10 * yd;
            float c_1 = c_01 * (1 - yd) + c_11 * yd;
            float c = c_0 * (1 - zd) + c_1 * zd;
            green = c / 255.0f;
        }
        {
            float c_00 = v_000->GetBlue() * (1 - xd) + v_100->GetBlue()  * xd;
            float c_10 = v_010->GetBlue() * (1 - xd) + v_110->GetBlue()  * xd;
            float c_01 = v_001->GetBlue() * (1 - xd) + v_101->GetBlue()  * xd;
            float c_11 = v_011->GetBlue() * (1 - xd) + v_111->GetBlue()  * xd;
            float c_0 = c_00 * (1 - yd) + c_10 * yd;
            float c_1 = c_01 * (1 - yd) + c_11 * yd;
            float c = c_0 * (1 - zd) + c_1 * zd;
            blue = c / 255.0f;
        }

        return Vec3(red, green, blue);
    }


    void CacheCentroids()
    {
        Vec3 halfResolution = Vec3(voxelResolutionMeters, voxelResolutionMeters, voxelResolutionMeters) * 0.5f;
        centroids.resize(static_cast<size_t>(chunkSize(0) * chunkSize(1) * chunkSize(2)));
        int i = 0;
        for (int z = 0; z < chunkSize(2); z++)
        {
            for(int y = 0; y < chunkSize(1); y++)
            {
                for(int x = 0; x < chunkSize(0); x++)
                {
                    centroids[i] = Vec3(x, y, z) * voxelResolutionMeters + halfResolution;
                    i++;
                }
            }
        }

        cubeIndexOffsets << 0, 1, 1, 0, 0, 1, 1, 0,
                0, 0, 1, 1, 0, 0, 1, 1,
                0, 0, 0, 0, 1, 1, 1, 1;
    }

    void ExtractBorderVoxelMesh(const ChunkPtr<VoxelType>& chunk, const Eigen::Vector3i& index, const Eigen::Vector3f& coordinates, VertIndex* nextMeshIndex, Mesh* mesh)
    {
        const Eigen::Matrix<float, 3, 8> cubeCoordOffsets = cubeIndexOffsets.cast<float>() * voxelResolutionMeters;
        Eigen::Matrix<float, 3, 8> cornerCoords;
        Eigen::Matrix<float, 8, 1> cornerSDF;
        bool allNeighborsObserved = true;
        for (int i = 0; i < 8; ++i)
        {
            Eigen::Vector3i cornerIDX = index + cubeIndexOffsets.col(i);

            if (chunk->IsCoordValid(cornerIDX.x(), cornerIDX.y(), cornerIDX.z()))
            {
                const VoxelType& thisVoxel = chunk->GetDistVoxel(cornerIDX.x(), cornerIDX.y(), cornerIDX.z());
                // Do not extract a mesh here if one of the corners is unobserved
                // and outside the truncation region.
                if (!thisVoxel.IsValid(minimumWeight))
                {
                    allNeighborsObserved = false;
                    break;
                }
                cornerCoords.col(i) = coordinates + cubeCoordOffsets.col(i);
                cornerSDF(i) = thisVoxel.GetSDF();
            }
            else
            {
                Eigen::Vector3i chunkOffset = Eigen::Vector3i::Zero();


                for (int j = 0; j < 3; j++)
                {
                    if (cornerIDX(j) < 0)
                    {
                        chunkOffset(j) = -1;
                        cornerIDX(j) = chunkSize(j) - 1;
                    }
                    else if(cornerIDX(j) >= chunkSize(j))
                    {
                        chunkOffset(j) = 1;
                        cornerIDX(j) = 0;
                    }
                }

                ChunkID neighborID = chunkOffset + chunk->GetID();

                if (HasChunk(neighborID))
                {
                    const ChunkPtr<VoxelType>& neighborChunk = GetChunk(neighborID);
                    if(!neighborChunk->IsCoordValid(cornerIDX.x(), cornerIDX.y(), cornerIDX.z()))
                    {
                        allNeighborsObserved = false;
                        break;
                    }

                    const VoxelType& thisVoxel = neighborChunk->GetDistVoxel(cornerIDX.x(), cornerIDX.y(), cornerIDX.z());
                    // Do not extract a mesh here if one of the corners is unobserved
                    // and outside the truncation region.
                    if (!thisVoxel.IsValid(minimumWeight))
                    {
                        allNeighborsObserved = false;
                        break;
                    }
                    cornerCoords.col(i) = coordinates + cubeCoordOffsets.col(i);
                    cornerSDF(i) = thisVoxel.GetSDF();
                }
                else
                {
                    allNeighborsObserved = false;
                    break;
                }

            }

        }

        if (allNeighborsObserved)
        {
            MarchingCubes::MeshCube(cornerCoords, cornerSDF, nextMeshIndex, mesh);
        }
    }

    void ExtractInsideVoxelMesh(const ChunkPtr<VoxelType>& chunk, const Eigen::Vector3i& index, const Vec3& coords, VertIndex* nextMeshIndex, Mesh* mesh)
    {
        assert(mesh != nullptr);
        Eigen::Matrix<float, 3, 8> cubeCoordOffsets = cubeIndexOffsets.cast<float>() * voxelResolutionMeters;
        Eigen::Matrix<float, 3, 8> cornerCoords;
        Eigen::Matrix<float, 8, 1> cornerSDF;
        bool allNeighborsObserved = true;
        for (int i = 0; i < 8; ++i)
        {
            Eigen::Vector3i corner_index = index + cubeIndexOffsets.col(i);
            const VoxelType& thisVoxel = chunk->GetDistVoxel(corner_index.x(), corner_index.y(), corner_index.z());

            // Do not extract a mesh here if one of the corner is unobserved and
            // outside the truncation region.
            if (!thisVoxel.IsValid(minimumWeight))
            {
                allNeighborsObserved = false;
                break;
            }
            cornerCoords.col(i) = coords + cubeCoordOffsets.col(i);
            cornerSDF(i) = thisVoxel.GetSDF();
        }

        if (allNeighborsObserved)
        {
            MarchingCubes::MeshCube(cornerCoords, cornerSDF, nextMeshIndex, mesh);
        }
    }

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

    void RecomputeMesh(const ChunkID& chunkID, std::mutex& mutex)
    {
        mutex.lock();
        if (!HasChunk(chunkID))
        {
            mutex.unlock();
            return;
        }

        MeshPtr mesh;
        if (!HasMesh(chunkID))
        {
            mesh = boost::allocate_shared<Mesh>(Eigen::aligned_allocator<Mesh>());
        }
        else
        {
            mesh = GetMutableMesh(chunkID);
        }


        ChunkPtr<VoxelType> chunk = GetChunk(chunkID);
        mutex.unlock();

        GenerateMesh(chunk, mesh.get());

        if(useColor)
        {
            ColorizeMesh(mesh.get());
        }

        ComputeNormalsFromGradients(mesh.get());

        mutex.lock();
        if(!mesh->vertices.empty())
        {
            allMeshes->emplace(chunkID, mesh);
            chunk->SetMesh(mesh);
        }
        mutex.unlock();
    }

    void RecomputeMeshes(const ChunkSet& chunks)
    {

        if (chunks.empty())
        {
            return;
        }

        std::mutex mutex;

        for(auto iter=chunks.begin(); iter!=chunks.end(); iter++)
            //parallel_for(chunkMeshes.begin(), chunkMeshes.end(), [&](const ChunkID& chunkID)
        {
            RecomputeMesh(iter->first, mutex);
        }
        //);
    }

    void RecomputeMeshes(const ChunkMap<VoxelType>& chunks)
    {

        if (chunks.empty())
        {
            return;
        }

        std::mutex mutex;

        for(auto iter=chunks.begin(); iter!=chunks.end(); iter++)
            //parallel_for(chunkMeshes.begin(), chunkMeshes.end(), [&](const ChunkID& chunkID)
        {
            RecomputeMesh(iter->first, mutex);
        }
        //);
    }

    void ComputeNormalsFromGradients(Mesh* mesh)
    {
        assert(mesh != nullptr);
        double dist;
        Vec3 grad;
        for (size_t i = 0; i < mesh->vertices.size(); i++)
        {
            const Vec3& vertex = mesh->vertices.at(i);
            if(GetSDFAndGradient(vertex, &dist, &grad))
            {
                float mag = grad.norm();
                if(mag> 1e-12)
                {
                    mesh->normals[i] = grad * (1.0f / mag);
                }
            }
        }
    }

    inline const Eigen::Vector3i& GetChunkSize() const { return chunkSize; }
    inline const Eigen::Vector3f& GetChunkSizeMeters() const { return chunkSizeMeters; }
    inline float GetResolution() const { return voxelResolutionMeters; }
    inline const Vec3& GetOrigin() const { return origin; }
    inline const Vec3List& GetCentroids() const { return centroids; }
    inline const AABB& GetBoundingBox() const { return volumeBoundingBox; }

    void PrintMemoryStatistics() const
    {
        float bigFloat = std::numeric_limits<float>::max();

        chisel::AABB totalBounds;
        totalBounds.min = chisel::Vec3(bigFloat, bigFloat, bigFloat);
        totalBounds.max = chisel::Vec3(-bigFloat, -bigFloat, -bigFloat);

        ChunkStatistics stats;
        stats.numKnownInside = 0;
        stats.numKnownOutside = 0;
        stats.numUnknown = 0;
        stats.totalWeight = 0.0f;
        for (const std::pair<ChunkID, ChunkPtr<VoxelType>>& chunk : *chunks)
        {
            AABB bounds = chunk.second->ComputeBoundingBox();
            for (int i = 0; i < 3; i++)
            {
                totalBounds.min(i) = std::min(totalBounds.min(i), bounds.min(i));
                totalBounds.max(i) = std::max(totalBounds.max(i), bounds.max(i));
            }

            chunk.second->ComputeStatistics(&stats);
        }


        Vec3 ext = totalBounds.GetExtents();
        Vec3 numVoxels = ext * 2 / voxelResolutionMeters;
        float totalNum = numVoxels(0) * numVoxels(1) * numVoxels(2);

        float maxMemory = totalNum * sizeof(VoxelType) / 1000000.0f;

        size_t currentNum = chunks->size() * (chunkSize(0) * chunkSize(1) * chunkSize(2));
        float currentMemory = currentNum * sizeof(VoxelType) / 1000000.0f;

        printf("Num Unknown: %lu, Num KnownIn: %lu, Num KnownOut: %lu Weight: %f\n", stats.numUnknown, stats.numKnownInside, stats.numKnownOutside, stats.totalWeight);
        printf("Bounds: %f %f %f %f %f %f\n", totalBounds.min.x(), totalBounds.min.y(), totalBounds.min.z(), totalBounds.max.x(), totalBounds.max.y(), totalBounds.max.z());
        printf("Theoretical max (MB): %f, Current (MB): %f\n", maxMemory, currentMemory);

    }


    void Reset()
    {
        allMeshes->clear();
        chunks->clear();
        incrementalChanges->clear(); /// TODO: Add everything as deleted into IncrementalChanges
    }

    bool GetSDFAndGradient(const Eigen::Vector3f& pos, double* dist, Eigen::Vector3f* grad) const
    {
        Eigen::Vector3f posf = Eigen::Vector3f(std::floor(pos.x() / voxelResolutionMeters) * voxelResolutionMeters + voxelResolutionMeters / 2.0f,
                                               std::floor(pos.y() / voxelResolutionMeters) * voxelResolutionMeters + voxelResolutionMeters / 2.0f,
                                               std::floor(pos.z() / voxelResolutionMeters) * voxelResolutionMeters + voxelResolutionMeters / 2.0f);
        if (!GetSDF(posf, dist)) return false;
        double ddxplus, ddyplus, ddzplus = 0.0;
        double ddxminus, ddyminus, ddzminus = 0.0;
        if (!GetSDF(posf + Eigen::Vector3f(voxelResolutionMeters, 0, 0), &ddxplus)) return false;
        if (!GetSDF(posf + Eigen::Vector3f(0, voxelResolutionMeters, 0), &ddyplus)) return false;
        if (!GetSDF(posf + Eigen::Vector3f(0, 0, voxelResolutionMeters), &ddzplus)) return false;
        if (!GetSDF(posf - Eigen::Vector3f(voxelResolutionMeters, 0, 0), &ddxminus)) return false;
        if (!GetSDF(posf - Eigen::Vector3f(0, voxelResolutionMeters, 0), &ddyminus)) return false;
        if (!GetSDF(posf - Eigen::Vector3f(0, 0, voxelResolutionMeters), &ddzminus)) return false;

        *grad = Eigen::Vector3f(ddxplus - ddxminus, ddyplus - ddyminus, ddzplus - ddzminus);
        grad->normalize();
        return true;
    }

    bool GetSDF(const Eigen::Vector3f& posf, double* dist) const
    {
        chisel::ChunkConstPtr<VoxelType> chunk = GetChunkAt(posf);
        if(chunk)
        {
            Eigen::Vector3f relativePos = posf - chunk->GetOrigin();
            Eigen::Vector3i coords = chunk->GetVoxelCoords(relativePos);
            chisel::VoxelID id = chunk->GetVoxelID(coords);
            if(id >= 0 && id < chunk->GetTotalNumVoxels())
            {
                const VoxelType& voxel = chunk->GetDistVoxel(id);
                if(voxel.IsValid())
                {
                    *dist = voxel.GetSDF();
                    return true;
                }
            }
            return false;
        }
        else
        {
            return false;
        }
    }

    inline void DeleteEmptyChunks() { DeleteEmptyChunks(incrementalChanges->getChangedChunks()); }
    void DeleteEmptyChunks(const ChunkMap<VoxelType>& chunk_set)
    {
        for (const auto& chunkPair : chunk_set)
        {
            if (!HasChunk(chunkPair.first))
                continue;

            ChunkPtr<VoxelType> chunk = GetChunk(chunkPair.first);

            bool chunkContainsData = false;
            for (int i = 0; i < chunk->GetTotalNumVoxels(); i++)
            {
                if (chunk->GetDistVoxel(i).IsValid()) /// todo: also require weight > minweight?
                {
                    chunkContainsData = true;
                    break;
                }
            }
            if (!chunkContainsData)
            {
                RemoveChunk(chunk->GetID());
            }
        }
    }

    IncrementalChangesConstPtr<VoxelType> getIncrementalChanges(){ return incrementalChanges; }
    void clearIncrementalChanges(){ incrementalChanges->clear(); }

    inline void RememberAddedChunk(ChunkPtr<VoxelType> chunk) { incrementalChanges->RememberAddedChunk(chunk); }

    inline void RememberUpdatedChunk(ChunkPtr<VoxelType> chunk) { incrementalChanges->RememberUpdatedChunk(chunk); }
    inline void RememberUpdatedChunk(ChunkPtr<VoxelType> chunk, ChunkSet& meshes_to_update) { incrementalChanges->RememberUpdatedChunk(chunk, GetChunkSizeMeters(), meshes_to_update); }

    inline void RememberDeletedChunk(ChunkPtr<VoxelType> chunk) { incrementalChanges->RememberDeletedChunk(chunk); }

    inline void RememberUpdatedVoxel(ChunkPtr<VoxelType> chunk, const VoxelID& voxelID, const float weight_diff, const float sdf_diff) { incrementalChanges->RememberUpdatedVoxel(chunk, voxelID, weight_diff, sdf_diff); }
    inline void RememberCarvedVoxel(ChunkPtr<VoxelType> chunk, const VoxelID& voxelID) { incrementalChanges->RememberCarvedVoxel(chunk, voxelID); }

    void ComputeExpandedGrid(int n_level) //todo(kdaun) this function only works with MultDistVoxel, don't breal interface for DistVoxel
    {
        //todo(kdaun) iterate over all voxel and set size instead of relying on default constructor

        for(int level = 1; level < n_level; ++level)
        {
            printf("LEVEL %i/%i \n",level+1, n_level);
            for (auto& chunkTriple : *chunks) //ChunkID, ChunkPtr<VoxelType>, ChunkHasher
            {
                ChunkPtr<VoxelType> chunk = chunkTriple.second;
                for (int voxel_idx = 0; voxel_idx < chunk->GetTotalNumVoxels(); voxel_idx++)
                {
                    VoxelType voxel = chunk->GetDistVoxel(voxel_idx);
                    float sdf = voxel.GetExpandedSDF(level - 1);
                    if(sdf == 99999)
                        continue;
                    Point3 voxel_coords_local_int = chunk->GetLocalCoords(voxel_idx);
                    Point3 voxel_coords_local = voxel_coords_local_int;
                    Vec3 voxel_coords_global = chunk->GetWorldCoords(voxel_idx);
                    int level_range = level > 1 ? std::pow(2,level-2) : 1;
                    for(int dx = -level_range; dx <= level_range; ++dx)
                    {
                        //printf("current level: %f \n",(dx+level)*100./2.*level);
                        for(int dy = -level_range; dy <= level_range; ++dy)
                        {
                            for(int dz = -level_range; dz <= level_range; ++dz)
                            {
                                Point3 d = Point3(dx,dy,dz);
                                Vec3 d_global = Point3(dx,dy,dz).cast<float>()*voxelResolutionMeters;
                                //printf("d %i %i %i \n",dx,dy,dz);
                                if(chunk->IsCoordValid(voxel_coords_local.x() + dx,
                                                       voxel_coords_local.y() + dy, voxel_coords_local.z() + dz))
                                {
                                    Point3 shifted_voxel = voxel_coords_local + d;
                                    VoxelType& candidate_voxel= chunk->GetDistVoxelMutable(chunk->GetVoxelID(shifted_voxel));
                                    if(std::abs(candidate_voxel.GetExpandedSDF(level)) > std::abs(sdf))
                                    {
                                        candidate_voxel.SetExpandedSDF(level, sdf);
                                    }
                                }
                                else
                                {

                                    VoxelType* candidate_voxel_ptr = GetDistanceVoxelMutable(voxel_coords_global + d_global);
                                    if(candidate_voxel_ptr)
                                    {
                                        if(std::abs(candidate_voxel_ptr->GetExpandedSDF(level)) > std::abs(sdf))
                                        {
                                            candidate_voxel_ptr->SetExpandedSDF(level, sdf);
                                        }
                                    }
                                    else
                                    {
                                        const ChunkID& chunkID = GetIDAt(voxel_coords_global + d_global);
                                        CreateChunk(chunkID);
                                        candidate_voxel_ptr = GetDistanceVoxelMutable(voxel_coords_global + d_global);
                                        if(candidate_voxel_ptr)
                                        {
                                            if(std::abs(candidate_voxel_ptr->GetExpandedSDF(level)) > std::abs(sdf))
                                            {
                                                candidate_voxel_ptr->SetExpandedSDF(level, sdf);
                                            }
                                        }
                                        else
                                        {
                                            printf("Something wrong in expanded chunk creation\n");
                                            continue;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    protected:
        boost::shared_ptr<ChunkMap<VoxelType>> chunks;
    Eigen::Vector3i chunkSize;
    Eigen::Vector3f chunkSizeMeters;
    float voxelResolutionMeters;
    Vec3List centroids;
    Eigen::Matrix<int, 3, 8> cubeIndexOffsets;
    boost::shared_ptr<MeshMap> allMeshes;
    bool useColor;
    unsigned int maxThreads;
    unsigned int threadTreshold;
    IncrementalChangesPtr<VoxelType> incrementalChanges;
    float minimumWeight;
    Vec3 origin;
    AABB volumeBoundingBox;

private:
    Vec3 roundingFactor;
};


template<class VoxelType>
using ChunkManagerPtr = boost::shared_ptr<ChunkManager<VoxelType>>;
template<class VoxelType>
using ChunkManagerConstPtr = boost::shared_ptr<const ChunkManager<VoxelType>>;

typedef boost::shared_ptr<ChunkSet> ChunkSetPtr;
typedef boost::shared_ptr<const ChunkSet> ChunkSetConstPtr;
} // namespace chisel 

#endif // CHUNKMANAGER_H_ 
