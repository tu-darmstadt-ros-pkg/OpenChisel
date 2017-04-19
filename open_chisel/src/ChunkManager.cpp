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

#include <assert.h>
#include <open_chisel/threading/Threading.h>
#include <open_chisel/ChunkManager.h>
#include <open_chisel/geometry/Frustum.h>
#include <open_chisel/geometry/AABB.h>
#include <open_chisel/marching_cubes/MarchingCubes.h>
#include <open_chisel/geometry/Raycast.h>
#include <iostream>

namespace chisel
{

    ChunkManager::ChunkManager() :
            chunkSize(16, 16, 16, 0), voxelResolutionMeters(0.03f), minimumWeight(0.0f)
    {
        chunkSizeMeters = chunkSize.cast<float>() * voxelResolutionMeters;
        CacheCentroids();
        maxThreads = 4;
        threadTreshold = 500;
        chunks = boost::make_shared<ChunkMap>();
        allMeshes = boost::make_shared<MeshMap>();
        incrementalChanges = boost::make_shared<IncrementalChanges>();
        invChunkSizeMeters = chunkSizeMeters.cwiseInverse();
    }

    ChunkManager::~ChunkManager()
    {

    }

    ChunkManager::ChunkManager(const Point4& size, float res, bool color, float minWeight) :
            chunkSize(size), voxelResolutionMeters(res), useColor(color), minimumWeight(minWeight)
    {
        chunkSizeMeters = chunkSize.cast<float>() * voxelResolutionMeters;
        CacheCentroids();
        maxThreads = 4;
        threadTreshold = 500;
        chunks = boost::make_shared<ChunkMap>();
        allMeshes = boost::make_shared<MeshMap>();
        incrementalChanges = boost::make_shared<IncrementalChanges>();
        invChunkSizeMeters = chunkSizeMeters.cwiseInverse();
    }

    void ChunkManager::CacheCentroids()
    {
        Vec4 halfResolution = Vec4(voxelResolutionMeters, voxelResolutionMeters, voxelResolutionMeters, 0) * 0.5f;
        centroids.resize(static_cast<size_t>(chunkSize(0) * chunkSize(1) * chunkSize(2)));
        int i = 0;
        for (int z = 0; z < chunkSize(2); z++)
        {
            for(int y = 0; y < chunkSize(1); y++)
            {
                for(int x = 0; x < chunkSize(0); x++)
                {
                    centroids[i] = Vec4(x, y, z, 0) * voxelResolutionMeters + halfResolution;
                    i++;
                }
            }
        }

        cubeIndexOffsets << 0, 1, 1, 0, 0, 1, 1, 0,
                            0, 0, 1, 1, 0, 0, 1, 1,
                            0, 0, 0, 0, 1, 1, 1, 1;
    }

    void ChunkManager::GetChunkIDsIntersecting(const AABB& box, ChunkIDList* chunkList)
    {
        assert(chunkList != nullptr);

        ChunkID minID = GetIDAt(box.min);
        ChunkID maxID = GetIDAt(box.max) + chisel::Point4(1, 1, 1, 0);

        for (int x = minID(0); x < maxID(0); x++)
        {
            for (int y = minID(1); y < maxID(1); y++)
            {
                for (int z = minID(2); z < maxID(2); z++)
                {
                    chunkList->push_back(ChunkID(x, y, z, 0));
                }
            }
        }
    }


    void ChunkManager::RecomputeMesh(const ChunkID& chunkID, std::mutex& mutex)
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


        ChunkPtr chunk = GetChunk(chunkID);
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

    void ChunkManager::RecomputeMeshes(const ChunkSet& chunks)
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

    void ChunkManager::RecomputeMeshes(const ChunkMap& chunks)
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

    ChunkPtr ChunkManager::CreateChunk(const ChunkID& id)
    {
        ChunkPtr chunk = boost::allocate_shared<Chunk>(Eigen::aligned_allocator<Chunk>(), id, chunkSize, voxelResolutionMeters, useColor);
        AddChunk(chunk);
        RememberAddedChunk(chunk);
        return chunk;
    }

    void ChunkManager::Reset()
    {
        allMeshes->clear();
        chunks->clear();
        incrementalChanges->clear(); /// TODO: Add everything as deleted into IncrementalChanges
    }

    void ChunkManager::GetChunkIDsIntersecting(const Frustum& frustum, ChunkIDList* chunkList)
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
                    Vec4 min = Vec4(x * chunkSize(0), y * chunkSize(1), z * chunkSize(2), 0.0f) * voxelResolutionMeters;
                    Vec4 max = min + Vec4(chunkSize.x(), chunkSize.y(), chunkSize.z(), 0.0f) * voxelResolutionMeters;
                    AABB chunkBox(min, max);
                    if(frustum.Intersects(chunkBox))
                    {
                        chunkList->push_back(ChunkID(x, y, z, 0));
                    }
                }
            }
        }

        //printf("%lu chunks intersect frustum\n", chunkList->size());
    }

    void ChunkManager::GetChunkIDsIntersecting(const PointCloud& cloud, const Transform& cameraTransform,  float truncation, float minDist, float maxDist, ChunkIDList* chunkList)
    {
        assert(!!chunkList);
        chunkList->clear();
        ChunkMap map;
        Point4 minVal(-std::numeric_limits<int>::max(), -std::numeric_limits<int>::max(), -std::numeric_limits<int>::max(), 0);
        Point4 maxVal(std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), 0);
        //size_t numPoints = cloud.GetPoints().size();
        //size_t i = 0;
        for (const Vec4& point : cloud.GetPoints())
        {
            Vec4 end = cameraTransform * point;
            const Vec4 start(cameraTransform.translation().x(), cameraTransform.translation().y(), cameraTransform.translation().z(), 0.0f);

            float len = (end - start).norm();

            if(len > maxDist || len < minDist)
            {
                continue;
            }

            Vec4 dir = (end - start).normalized();
            Vec4 truncStart = end - dir * truncation;
            Vec4 truncEnd = end + dir * truncation;
            Vec4 startInt = Vec4(truncStart.x() * invChunkSizeMeters(0) , truncStart.y() * invChunkSizeMeters(1), truncStart.z() * invChunkSizeMeters(2), 0.0f);
            Vec4 endInt = Vec4(truncEnd.x() * invChunkSizeMeters(0), truncEnd.y() * invChunkSizeMeters(1), truncEnd.z() * invChunkSizeMeters(2), 0.0f);

            Point4List intersectingChunks;
            Raycast(startInt, endInt, minVal, maxVal, &intersectingChunks);

            for (const Point4& id : intersectingChunks)
            {
                if(map.find(id) == map.end())
                    map[id] = ChunkPtr();
            }
        }

        for (const std::pair<ChunkID, ChunkPtr>& it : map)
        {
            chunkList->push_back(it.first);
        }

    }

    void ChunkManager::ExtractInsideVoxelMesh(const ChunkPtr& chunk, const Point4& index, const Vec4& coords, VertIndex* nextMeshIndex, Mesh* mesh)
    {
        assert(mesh != nullptr);
        Eigen::Matrix<float, 4, 8> cubeCoordOffsets = cubeIndexOffsets.cast<float>() * voxelResolutionMeters;
        Eigen::Matrix<float, 4, 8> cornerCoords;
        Eigen::Matrix<float, 8, 1> cornerSDF;
        bool allNeighborsObserved = true;
        for (int i = 0; i < 8; ++i)
        {
            Point4 corner_index = index + cubeIndexOffsets.col(i);
            const DistVoxel& thisVoxel = chunk->GetDistVoxel(corner_index.x(), corner_index.y(), corner_index.z());

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

    void ChunkManager::ExtractBorderVoxelMesh(const ChunkPtr& chunk, const Point4& index, const Vec4& coordinates, VertIndex* nextMeshIndex, Mesh* mesh)
    {
        const Eigen::Matrix<float, 4, 8> cubeCoordOffsets = cubeIndexOffsets.cast<float>() * voxelResolutionMeters;
        Eigen::Matrix<float, 4, 8> cornerCoords;
        Eigen::Matrix<float, 8, 1> cornerSDF;
        bool allNeighborsObserved = true;
        for (int i = 0; i < 8; ++i)
        {
            Point4 cornerIDX = index + cubeIndexOffsets.col(i);

            if (chunk->IsCoordValid(cornerIDX.x(), cornerIDX.y(), cornerIDX.z()))
            {
                const DistVoxel& thisVoxel = chunk->GetDistVoxel(cornerIDX.x(), cornerIDX.y(), cornerIDX.z());
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
                chisel::Point4 chunkOffset = chisel::Point4::Zero();

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
                    const ChunkPtr& neighborChunk = GetChunk(neighborID);
                    if(!neighborChunk->IsCoordValid(cornerIDX.x(), cornerIDX.y(), cornerIDX.z()))
                    {
                        allNeighborsObserved = false;
                        break;
                    }

                    const DistVoxel& thisVoxel = neighborChunk->GetDistVoxel(cornerIDX.x(), cornerIDX.y(), cornerIDX.z());
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


    void ChunkManager::GenerateMesh(const ChunkPtr& chunk, Mesh* mesh)
    {
        assert(mesh != nullptr);

        mesh->Clear();
        const int maxX = chunkSize(0);
        const int maxY = chunkSize(1);
        const int maxZ = chunkSize(2);

        chisel::Point4 index = chisel::Point4::Zero();
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

    bool ChunkManager::GetSDFAndGradient(const Eigen::Vector4f& pos, double* dist, Eigen::Vector4f* grad) const
    {
        Eigen::Vector4f posf = Eigen::Vector4f(std::floor(pos.x() / voxelResolutionMeters) * voxelResolutionMeters + voxelResolutionMeters / 2.0f,
                                               std::floor(pos.y() / voxelResolutionMeters) * voxelResolutionMeters + voxelResolutionMeters / 2.0f,
                                               std::floor(pos.z() / voxelResolutionMeters) * voxelResolutionMeters + voxelResolutionMeters / 2.0f,
                                               0.0f);
        if (!GetSDF(posf, dist)) return false;
        double ddxplus, ddyplus, ddzplus = 0.0;
        double ddxminus, ddyminus, ddzminus = 0.0;
        if (!GetSDF(posf + Eigen::Vector4f(voxelResolutionMeters, 0, 0, 0), &ddxplus)) return false;
        if (!GetSDF(posf + Eigen::Vector4f(0, voxelResolutionMeters, 0, 0), &ddyplus)) return false;
        if (!GetSDF(posf + Eigen::Vector4f(0, 0, voxelResolutionMeters, 0), &ddzplus)) return false;
        if (!GetSDF(posf - Eigen::Vector4f(voxelResolutionMeters, 0, 0, 0), &ddxminus)) return false;
        if (!GetSDF(posf - Eigen::Vector4f(0, voxelResolutionMeters, 0, 0), &ddyminus)) return false;
        if (!GetSDF(posf - Eigen::Vector4f(0, 0, voxelResolutionMeters, 0), &ddzminus)) return false;

        *grad = Eigen::Vector4f(ddxplus - ddxminus, ddyplus - ddyminus, ddzplus - ddzminus, 0.0f);
        grad->normalize();
        return true;
    }

    bool ChunkManager::GetSDF(const Vec4& posf, double* dist) const
    {
        chisel::ChunkConstPtr chunk = GetChunkAt(posf);
        if(chunk)
        {
            Vec4 relativePos = posf - chunk->GetOrigin();
            Point4 coords = chunk->GetVoxelCoords(relativePos);
            chisel::VoxelID id = chunk->GetVoxelID(coords);
            if(id >= 0 && id < chunk->GetTotalNumVoxels())
            {
                const chisel::DistVoxel& voxel = chunk->GetDistVoxel(id);
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

    Vec3 ChunkManager::InterpolateColor(const Vec4& colorPos)
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

        const ColorVoxel* v_000 = GetColorVoxel(Vec4(x_0, y_0, z_0, 0));
        const ColorVoxel* v_001 = GetColorVoxel(Vec4(x_0, y_0, z_1, 0));
        const ColorVoxel* v_011 = GetColorVoxel(Vec4(x_0, y_1, z_1, 0));
        const ColorVoxel* v_111 = GetColorVoxel(Vec4(x_1, y_1, z_1, 0));
        const ColorVoxel* v_110 = GetColorVoxel(Vec4(x_1, y_1, z_0, 0));
        const ColorVoxel* v_100 = GetColorVoxel(Vec4(x_1, y_0, z_0, 0));
        const ColorVoxel* v_010 = GetColorVoxel(Vec4(x_0, y_1, z_0, 0));
        const ColorVoxel* v_101 = GetColorVoxel(Vec4(x_1, y_0, z_1, 0));

        if(!v_000 || !v_001 || !v_011 || !v_111 || !v_110 || !v_100 || !v_010 || !v_101)
        {
            const ChunkID& chunkID = GetIDAt(colorPos);

            if(!HasChunk(chunkID))
            {
                return Vec3(0, 0, 0);
            }
            else
            {
                const ChunkPtr& chunk = GetChunk(chunkID);
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

    const DistVoxel* ChunkManager::GetDistanceVoxel(const Vec4& pos) const
    {
        const ChunkPtr chunk = GetChunk(GetIDAt(pos));

        if (chunk)
        {
            Vec4 rel = (pos - chunk->GetOrigin());
            return &(chunk->GetDistVoxel(chunk->GetVoxelID(rel)));
        }
        else
          return nullptr;
    }

    DistVoxel* ChunkManager::GetDistanceVoxelMutable(const Vec4& pos)
    {
        ChunkPtr chunk = GetChunkAt(pos);

        if (chunk)
        {
            Vec4 rel = (pos - chunk->GetOrigin());
            return &(chunk->GetDistVoxelMutable(chunk->GetVoxelID(rel)));
        }
        else
          return nullptr;
    }




    const ColorVoxel* ChunkManager::GetColorVoxel(const Vec4& pos) const
    {
        const ChunkPtr chunk = GetChunk(GetIDAt(pos));

        if (chunk && chunk->HasColors())
        {
            Vec4 rel = (pos - chunk->GetOrigin());
            return &(chunk->GetColorVoxel(chunk->GetVoxelID(rel)));
        }
        else
          return nullptr;
    }

    ColorVoxel* ChunkManager::GetColorVoxelMutable(const Vec4& pos)
    {
        ChunkPtr chunk = GetChunkAt(pos);

        if (chunk && chunk->HasColors())
        {
            Vec4 rel = (pos - chunk->GetOrigin());
            return &(chunk->GetColorVoxelMutable(chunk->GetVoxelID(rel)));
        }
        else
          return nullptr;
    }

    bool ChunkManager::GetClosestVoxelPosition(const Vec4& pos, Vec4& voxel_pos) const
    {
      ChunkPtr chunk = GetChunkAt(pos);

      if (chunk)
      {
        voxel_pos = chunk->GetWorldCoords(chunk->GetVoxelCoords(pos - chunk->GetOrigin()));
        return true;
      }
      else
        return false;
    }


    void ChunkManager::ComputeNormalsFromGradients(Mesh* mesh)
    {
        assert(mesh != nullptr);
        double dist;
        Vec4 grad;
        for (size_t i = 0; i < mesh->vertices.size(); i++)
        {
            const Vec4& vertex = mesh->vertices.at(i);
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

    void ChunkManager::ColorizeMesh(Mesh* mesh)
    {
        assert(mesh != nullptr);

        mesh->colors.clear();
        mesh->colors.resize(mesh->vertices.size());
        for (size_t i = 0; i < mesh->vertices.size(); i++)
        {
            const Vec4 vertex(mesh->vertices.at(i).x(), mesh->vertices.at(i).y(), mesh->vertices.at(i).z(), 0);
            mesh->colors[i] = InterpolateColor(vertex);
        }
    }


    void ChunkManager::PrintMemoryStatistics() const
    {
        float bigFloat = std::numeric_limits<float>::max();

        chisel::AABB totalBounds;
        totalBounds.min = chisel::Vec4(bigFloat, bigFloat, bigFloat, 0.0f);
        totalBounds.max = chisel::Vec4(-bigFloat, -bigFloat, -bigFloat, 0.0f);

        ChunkStatistics stats;
        stats.numKnownInside = 0;
        stats.numKnownOutside = 0;
        stats.numUnknown = 0;
        stats.totalWeight = 0.0f;
        for (const std::pair<ChunkID, ChunkPtr>& chunk : *chunks)
        {
            AABB bounds = chunk.second->ComputeBoundingBox();
            for (int i = 0; i < 3; i++)
            {
                totalBounds.min(i) = std::min(totalBounds.min(i), bounds.min(i));
                totalBounds.max(i) = std::max(totalBounds.max(i), bounds.max(i));
            }

            chunk.second->ComputeStatistics(&stats);
        }


        Vec4 ext = totalBounds.GetExtents();
        Vec4 numVoxels = ext * 2 / voxelResolutionMeters;
        float totalNum = numVoxels(0) * numVoxels(1) * numVoxels(2);

        float maxMemory = totalNum * sizeof(DistVoxel) / 1000000.0f;

        size_t currentNum = chunks->size() * (chunkSize(0) * chunkSize(1) * chunkSize(2));
        float currentMemory = currentNum * sizeof(DistVoxel) / 1000000.0f;

        printf("Num Unknown: %lu, Num KnownIn: %lu, Num KnownOut: %lu Weight: %f\n", stats.numUnknown, stats.numKnownInside, stats.numKnownOutside, stats.totalWeight);
        printf("Bounds: %f %f %f %f %f %f\n", totalBounds.min.x(), totalBounds.min.y(), totalBounds.min.z(), totalBounds.max.x(), totalBounds.max.y(), totalBounds.max.z());
        printf("Theoretical max (MB): %f, Current (MB): %f\n", maxMemory, currentMemory);

    }

    void ChunkManager::ClearPassedVoxels(const Vec4& start, const Vec4& end, float voxelCarvingResetTresh, ChunkVoxelMap* carvedVoxels)
    {
        float roundingFactor = 1/voxelResolutionMeters;
        const Vec4 startRounded = start * roundingFactor;
        const Vec4 endRounded = end * roundingFactor;

        Point4List passedVoxels;
        Raycast(startRounded, endRounded, passedVoxels);

        const Vec4 voxelShift (0.5 * voxelResolutionMeters, 0.5 * voxelResolutionMeters, 0.5 * voxelResolutionMeters, 0.0f);

        for (const Point4& voxelCoords: passedVoxels)
        {
            Vec4 voxelPos(voxelCoords.cast<float>());
            voxelPos = voxelPos * voxelResolutionMeters + voxelShift;

            ChunkPtr chunk = GetChunkAt(voxelPos);
            if(chunk)
            {
                Vec4 rel = (voxelPos - chunk->GetOrigin());
                VoxelID voxelID = chunk->GetVoxelID(rel);

                VoxelSet* voxel_set;
                std::pair<ChunkPtr, VoxelID> voxel_entry;

                if (carvedVoxels)
                {
                  voxel_set = &(*carvedVoxels)[chunk->GetID()];
                  voxel_entry = std::make_pair(chunk, voxelID);

                  // never carve recently updated voxels
                  if (voxel_set->find(voxel_entry) != voxel_set->end())
                    continue;
                }

                DistVoxel& voxel = chunk->GetDistVoxelMutable(voxelID);
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

    void ChunkManager::DeleteEmptyChunks(const ChunkMap& chunk_set)
    {
      for (const auto& chunkPair : chunk_set)
      {
        if (!HasChunk(chunkPair.first))
          continue;

        ChunkPtr chunk = GetChunk(chunkPair.first);

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
} // namespace chisel 
