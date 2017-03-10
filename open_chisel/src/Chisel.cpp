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

#include <open_chisel/Chisel.h>
#include <open_chisel/io/PLY.h>
#include <open_chisel/geometry/Raycast.h>

namespace chisel
{

  Chisel::Chisel()
  {
    // TODO Auto-generated constructor stub
    maxThreads = 4;
    threadTreshold = 500;
    chunkManager = boost::make_shared<ChunkManager>();
  }

  Chisel::Chisel(const Eigen::Vector3i& chunkSize, float voxelResolution, bool useColor)
  {
    chunkManager = boost::make_shared<ChunkManager>(chunkSize, voxelResolution, useColor);

    maxThreads = 4;
    threadTreshold = 500;
  }

  Chisel::~Chisel()
  {
    // TODO Auto-generated destructor stub
  }

  void Chisel::Reset()
  {
    chunkManager->Reset();
    meshesToUpdate.clear();
  }

  void Chisel::UpdateMeshes()
  {
    chunkManager->RecomputeMeshes(meshesToUpdate);
    meshesToUpdate.clear();
  }

  void Chisel::GarbageCollect(const ChunkIDList& chunks)
  {
    for (const ChunkID& chunkID : chunks)
      {
        chunkManager->RemoveChunk(chunkID);
      }
  }

  bool Chisel::SaveAllMeshesToPLY(const std::string& filename)
  {
    printf("Saving all meshes to PLY file...\n");

    chisel::MeshPtr fullMesh(new chisel::Mesh());

    size_t v = 0;
    for (const std::pair<ChunkID, MeshPtr>& it : chunkManager->GetAllMeshes())
      {
        for (const Vec3& vert : it.second->vertices)
          {
            fullMesh->vertices.push_back(vert);
            fullMesh->indices.push_back(v);
            v++;
          }

        for (const Vec3& color : it.second->colors)
          {
            fullMesh->colors.push_back(color);
          }

        for (const Vec3& normal : it.second->normals)
          {
            fullMesh->normals.push_back(normal);
          }
      }

    printf("Full mesh has %lu verts\n", v);
    bool success = SaveMeshPLYASCII(filename, fullMesh);

    if (!success)
      {
        printf("Saving failed!\n");
      }

    return success;
  }

  //Integrate pointcloud after transforming to target frame using a given sensor pose
  void Chisel::IntegratePointCloud(const ProjectionIntegrator& integrator, const PointCloud& cloud, const Transform& extrinsic, const Vec3& sensorOrigin, float minDist, float maxDist)
  {
    ChunkVoxelMap updatedVoxels;

    //TODO: parallelize
    for (const Vec3& point : cloud.GetPoints())
    {
        Vec3 point_transformed = extrinsic * point;
        IntegrateRay(integrator, updatedVoxels, sensorOrigin, point_transformed, minDist, maxDist);
    }

    DetermineUpdatedChunks(updatedVoxels);
  }

  //Integrate pointcloud after transforming to target frame
  void Chisel::IntegratePointCloud(const ProjectionIntegrator& integrator, const PointCloud& cloud, const Transform& extrinsic, float minDist, float maxDist)
  {
    ChunkVoxelMap updatedVoxels;
    const Vec3& start = extrinsic.translation();

    //TODO: parallelize
    for (const Vec3& point : cloud.GetPoints())
    {
        Vec3 point_transformed = extrinsic * point;
        IntegrateRay(integrator, updatedVoxels, start, point_transformed, minDist, maxDist);
    }

    DetermineUpdatedChunks(updatedVoxels);
  }

  //Integrate pointcloud already transformed to target frame
  void Chisel::IntegratePointCloud(const ProjectionIntegrator& integrator, const PointCloud& cloud, const Vec3& sensorOrigin, float minDist, float maxDist)
  {
    ChunkVoxelMap updatedVoxels;

    //TODO: parallelize
    for (const Vec3& point : cloud.GetPoints())
    {
        IntegrateRay(integrator, updatedVoxels, sensorOrigin, point, minDist, maxDist);
    }

    DetermineUpdatedChunks(updatedVoxels);
  }

  //Integrate ray already transformed to target frame
  void Chisel::IntegrateRay(const ProjectionIntegrator& integrator, ChunkVoxelMap& updatedVoxels, const Vec3& startPoint, const Vec3& endPoint, float minDist, float maxDist)
  {
    const Vec3 difference = endPoint - startPoint;

    float distance = difference.norm();

    /// todo: set up as parameter
    bool carveAllRays = false;
    bool integrateRay = true;

    if(distance < minDist)
    {
        return;
    }
    else if(distance > maxDist)
    {
      if (carveAllRays)
        integrateRay = false;
      else
        return;
    }

    if (integrator.IsCarvingEnabled())
    {
        float truncation = integrator.ComputeTruncationDistance(distance);
        const Vec3 direction = difference.normalized();

        Vec3 truncatedPositiveEnd(endPoint);
        float truncationOffset = integrator.GetCarvingDist() + truncation;

        //apply truncation offset towards sensor origin
        truncatedPositiveEnd -= truncationOffset * direction;
        chunkManager->ClearPassedVoxels(startPoint, truncatedPositiveEnd, integrator.GetVoxelCarvingResetTresh(), &updatedVoxels);
    }

    if (integrateRay)
      integrator.IntegratePoint(startPoint, endPoint, difference, distance, *chunkManager, &updatedVoxels);
  }

  void Chisel::DetermineUpdatedChunks(ChunkVoxelMap& updatedVoxels)
  {
    for (const auto& entry : updatedVoxels)
    {
      ChunkPtr chunk = chunkManager->GetChunk(entry.first);
      chunkManager->RememberUpdatedChunk(chunk, meshesToUpdate);
    }
  }

} // namespace chisel 
