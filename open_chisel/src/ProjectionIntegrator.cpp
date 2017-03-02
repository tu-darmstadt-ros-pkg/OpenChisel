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

#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/geometry/Raycast.h>

namespace chisel
{
  ProjectionIntegrator::ProjectionIntegrator()
  {
    // TODO Auto-generated constructor stub

  }

  ProjectionIntegrator::ProjectionIntegrator(const TruncatorPtr& t, const WeighterPtr& w, const float maxWeight, float crvDist, bool enableCrv, const Vec3List& centers) :
    truncator(t), weighter(w), maximumWeight(maxWeight), carvingDist(crvDist), enableVoxelCarving(enableCrv), centroids(centers)
  {

  }

  bool ProjectionIntegrator::Integrate(const PointCloud& cloud, const Transform& cameraPose, Chunk* chunk) const
  {
    assert(!!chunk);

    if(cloud.HasColor() && chunk->HasColors())
      {
        return IntegrateColorPointCloud(cloud, cameraPose, chunk);
      }
    else
      {
        return IntegratePointCloud(cloud, cameraPose, chunk);
      }
  }


  bool ProjectionIntegrator::IntegratePointCloud(const PointCloud& cloud, const Transform& cameraPose, Chunk* chunk) const
  {
    const float roundingFactor = 1.0f / chunk->GetVoxelResolutionMeters();

    Point3List raycastVoxels;
    Point3 chunkMin = Point3::Zero();
    Point3 chunkMax = chunk->GetNumVoxels();
    bool updated = false;
    Vec3 startCamera = cameraPose.translation();
    for (const Vec3& point : cloud.GetPoints())
      {
        Vec3 worldPoint = cameraPose * point;
        const Vec3 distance = worldPoint - startCamera;
        float depth = distance.norm();
        Vec3 dir = distance.normalized();
        float truncation = truncator->GetTruncationDistance(depth);
        Vec3 start = worldPoint - dir * truncation - chunk->GetOrigin();
        Vec3 end = worldPoint + dir * truncation - chunk->GetOrigin();
        start.x() *= roundingFactor;
        start.y() *= roundingFactor;
        start.z() *= roundingFactor;
        end.x() *= roundingFactor;
        end.y() *= roundingFactor;
        end.z() *= roundingFactor;
        raycastVoxels.clear();
        Raycast(start, end, chunkMin, chunkMax, &raycastVoxels);

        for (const Point3& voxelCoords : raycastVoxels)
          {
            VoxelID id = chunk->GetVoxelID(voxelCoords);
            DistVoxel& distVoxel = chunk->GetDistVoxelMutable(id);
            const Vec3& centroid = centroids[id] + chunk->GetOrigin();
            float u = depth - (centroid - startCamera).norm();
            float weight = weighter->GetWeight(u, truncation);
            if (fabs(u) < truncation)
              {
                distVoxel.Integrate(u, weight, maximumWeight);
                updated = true;
              }
            else if (enableVoxelCarving && u > truncation + carvingDist)
              {
                if (distVoxel.IsValid())
                  {
                    distVoxel.Carve();
                    updated = true;
                  }
              }
          }
      }
    return updated;
  }

  void ProjectionIntegrator::IntegratePoint(const Vec3& sensorOrigin, const Vec3& point, const Vec3& direction, float distance, ChunkManager& chunkManager, ChunkVoxelMap* updatedChunks) const
  {
    const float resolution = chunkManager.GetResolution();
    const float roundingFactor = 1.0f / resolution;
    const float halfDiag = 0.5 * sqrt(3.0f) * resolution;

    const float truncation = truncator->GetTruncationDistance(distance);
    const Vec3 truncationOffset = direction.normalized() * truncation;
    const Vec3 start = (point - truncationOffset) * roundingFactor;
    const Vec3 end = (point + truncationOffset) * roundingFactor;

    const Vec3 voxelShift(0.5 * resolution, 0.5 * resolution, 0.5 * resolution);

    Point3List raycastVoxels;

    Raycast(start, end, raycastVoxels);

    for (const Point3& voxelCoords : raycastVoxels)
    {
        Vec3 voxelPos = voxelCoords.cast<float>() * resolution +  voxelShift;
        const ChunkID& chunkID = chunkManager.GetIDAt(voxelPos);

        if (!chunkManager.HasChunk(chunkID))
        {
          chunkManager.CreateChunk(chunkID);
        }

        ChunkPtr chunk = chunkManager.GetChunk(chunkID);
        const Vec3& origin = chunk->GetOrigin();

        voxelPos -= origin;
        VoxelID voxelID = chunk->GetVoxelID(voxelPos);

        DistVoxel& distVoxel = chunk->GetDistVoxelMutable(voxelID);
        const Vec3& centroid = centroids[voxelID] + origin;
        float u = distance - (centroid - sensorOrigin).norm();
        float weight = weighter->GetWeight(u, truncation);
        if (fabs(u) < truncation + halfDiag)
        {
          distVoxel.Integrate(u, weight, maximumWeight);
          chunkManager.RememberUpdatedVoxel(chunk, voxelID);
          if (updatedChunks)
            (*updatedChunks)[chunkID].insert(std::make_pair(chunk, voxelID));
        }
    }
  }

  bool ProjectionIntegrator::IntegrateColorPointCloud(const PointCloud& cloud, const Transform& cameraPose, Chunk* chunk) const
  {
    const float roundingFactor = 1.0f / chunk->GetVoxelResolutionMeters();

    Point3List raycastVoxels;
    Point3 chunkMin = Point3::Zero();
    Point3 chunkMax = chunk->GetNumVoxels();
    bool updated = false;
    size_t i = 0;
    Vec3 startCamera = cameraPose.translation();
    Transform inversePose = cameraPose.inverse();
    for (const Vec3& point : cloud.GetPoints())
      {
        const Vec3& color = cloud.GetColors()[i];
        Vec3 worldPoint = cameraPose * point;
        const Vec3 distance = worldPoint - startCamera;
        float depth = distance.norm();
        Vec3 dir = distance.normalized();
        float truncation = truncator->GetTruncationDistance(depth);
        Vec3 start = worldPoint - dir * truncation - chunk->GetOrigin();
        Vec3 end = worldPoint + dir * truncation - chunk->GetOrigin();
        start.x() *= roundingFactor;
        start.y() *= roundingFactor;
        start.z() *= roundingFactor;
        end.x() *= roundingFactor;
        end.y() *= roundingFactor;
        end.z() *= roundingFactor;
        raycastVoxels.clear();
        Raycast(start, end, chunkMin, chunkMax, &raycastVoxels);

        for (const Point3& voxelCoords : raycastVoxels)
          {
            VoxelID id = chunk->GetVoxelID(voxelCoords);
            ColorVoxel& voxel = chunk->GetColorVoxelMutable(id);
            DistVoxel& distVoxel = chunk->GetDistVoxelMutable(id);
            const Vec3& centroid = centroids[id] + chunk->GetOrigin();
            float u = depth - (inversePose * centroid - startCamera).norm();
            float weight = weighter->GetWeight(u, truncation);
            if (fabs(u) < truncation)
              {
                distVoxel.Integrate(u, weight, maximumWeight);
                voxel.Integrate((uint8_t)(color.x() * 255.0f), (uint8_t)(color.y() * 255.0f), (uint8_t)(color.z() * 255.0f), 2);
                updated = true;
              }
            else if (enableVoxelCarving && u > truncation + carvingDist)
              {
                if (distVoxel.IsValid())
                  {
                    distVoxel.Carve();
                    updated = true;
                  }
              }
          }
        i++;

      }
    return updated;
  }

  bool ProjectionIntegrator::IntegrateChunk(const Chunk* chunkToIntegrate, Chunk* chunk) const{

    assert(chunk != nullptr && chunkToIntegrate != nullptr);

    bool updated = false;

    for (size_t i = 0; i < centroids.size(); i++)
    {
        DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
        DistVoxel voxelToIntegrate = chunkToIntegrate->GetDistVoxel(i);

        if (voxelToIntegrate.GetWeight() > 0 && voxelToIntegrate.GetSDF() <99999)
        {
          voxel.Integrate(voxelToIntegrate.GetSDF(), voxelToIntegrate.GetWeight(), maximumWeight);
          updated = true;
        }

        if(enableVoxelCarving)
        {
          if (voxel.GetWeight() > 0 && voxel.GetSDF() < 1e-5)
          {
            voxel.Carve();
            updated = true;
          }
        }
    }
    return updated;
  }

  bool ProjectionIntegrator::IntegrateColorChunk(const Chunk* chunkToIntegrate, Chunk* chunk) const{

    assert(chunk != nullptr && chunkToIntegrate != nullptr);

    bool updated = false;

    for (size_t i = 0; i < centroids.size(); i++)
    {
        DistVoxel& distVoxel = chunk->GetDistVoxelMutable(i);
        ColorVoxel& colorVoxel = chunk->GetColorVoxelMutable(i);

        const DistVoxel& distVoxelToIntegrate = chunkToIntegrate->GetDistVoxel(i);
        const ColorVoxel& colorVoxelToIntegrate = chunkToIntegrate->GetColorVoxel(i);

        if (distVoxelToIntegrate.GetWeight() > 0 && distVoxelToIntegrate.GetSDF() <99999)
        {
          distVoxel.Integrate(distVoxelToIntegrate.GetSDF(), distVoxelToIntegrate.GetWeight(), maximumWeight);
          colorVoxel.Integrate((uint8_t) colorVoxelToIntegrate.GetRed(), (uint8_t) colorVoxelToIntegrate.GetGreen(), (uint8_t)  colorVoxelToIntegrate.GetBlue(), colorVoxelToIntegrate.GetWeight());

          updated = true;
        }

        if(enableVoxelCarving)
        {
          if (distVoxel.GetWeight() > 0 && distVoxel.GetSDF() < 1e-5)
          {
            distVoxel.Carve();
            colorVoxel.Reset();
            updated = true;
          }
        }
    }
    return updated;
  }


  ProjectionIntegrator::~ProjectionIntegrator()
  {
    // TODO Auto-generated destructor stub
  }

} // namespace chisel 
