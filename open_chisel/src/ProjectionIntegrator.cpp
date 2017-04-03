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
  ProjectionIntegrator::ProjectionIntegrator():
    minimumWeight(0.0f), maximumWeight(std::numeric_limits<float>::max()), carvingDist(0.1f), enableVoxelCarving(false), rememberAllUpdatedVoxels(true)
  {
    // TODO Auto-generated constructor stub

  }

  ProjectionIntegrator::ProjectionIntegrator(const TruncatorPtr& t, const WeighterPtr& w, float minWeight, float maxWeight, float crvDist, bool enableCrv, const Vec4List& centers, bool rememberAllUpdatedVoxels) :
    truncator(t), weighter(w), minimumWeight(minWeight), maximumWeight(maxWeight), carvingDist(crvDist), enableVoxelCarving(enableCrv), centroids(centers), rememberAllUpdatedVoxels(rememberAllUpdatedVoxels)
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

    Point4List raycastVoxels;
    Point3 chunkMin = Point3::Zero();
    Point3 chunkMax = chunk->GetNumVoxels();
    bool updated = false;
    Vec4 startCamera = Vec4::Zero();
    startCamera.head<3>() = cameraPose.translation();
    for (const Vec4& point : cloud.GetPoints())
      {
        Vec4 worldPoint = cameraPose * point;
        const Vec4 distance = worldPoint - startCamera;
        float depth = distance.norm();
        Vec4 dir = distance.normalized();
        float truncation = truncator->GetTruncationDistance(depth);
        Vec4 start = worldPoint - dir * truncation - chunk->GetOrigin();
        Vec4 end = worldPoint + dir * truncation - chunk->GetOrigin();
        start *= roundingFactor;
        end *= roundingFactor;

        raycastVoxels.clear();
        Raycast(start, end, chunkMin, chunkMax, &raycastVoxels);

        for (const Point4& voxelCoords : raycastVoxels)
          {
            VoxelID id = chunk->GetVoxelID(voxelCoords);
            DistVoxel& distVoxel = chunk->GetDistVoxelMutable(id);
            const Vec4& centroid = centroids[id] + chunk->GetOrigin();
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

  void ProjectionIntegrator::IntegratePoint(const Vec4& sensorOrigin, const Vec4& point, const Vec4& direction, float distance, ChunkManager& chunkManager, ChunkVoxelMap* updatedChunks) const
  {
    const float resolution = chunkManager.GetResolution();
    const float roundingFactor = 1.0f / resolution;
    const float halfDiag = 0.5 * sqrt(3.0f) * resolution;

    const float truncation = truncator->GetTruncationDistance(distance);
    const Vec4 truncationOffset = direction.normalized() * truncation;
    const Vec4 start = (point - truncationOffset) * roundingFactor;
    const Vec4 end = (point + truncationOffset) * roundingFactor;

    const Vec4 voxelShift(0.5 * resolution, 0.5 * resolution, 0.5 * resolution, 0.0f);

    Point4List raycastVoxels;
    Raycast(start, end, raycastVoxels);

    for (const Point4& voxelCoords : raycastVoxels)
    {
        Vec4 voxelPos(voxelCoords.x(), voxelCoords.y(), voxelCoords.z(), 0.0f);
        voxelPos = voxelPos * resolution +  voxelShift;

        const ChunkID& chunkID = chunkManager.GetIDAt(voxelPos);

        if (!chunkManager.HasChunk(chunkID))
        {
          chunkManager.CreateChunk(chunkID);
        }

        ChunkPtr chunk = chunkManager.GetChunk(chunkID);
        const Vec4& origin = chunk->GetOrigin();

        voxelPos -= origin;
        VoxelID voxelID = chunk->GetVoxelID(voxelPos);

        DistVoxel& distVoxel = chunk->GetDistVoxelMutable(voxelID);
        const Vec4& centroid = centroids[voxelID] + origin;
        float u = distance - (centroid - sensorOrigin).norm();
        float weight = weighter->GetWeight(u, truncation);
        if (fabs(u) < truncation + halfDiag)
        {
          float weight_diff = - distVoxel.GetWeight();
          float sdf_diff = - distVoxel.GetSDF();
          distVoxel.Integrate(u, weight, maximumWeight);
          weight_diff += distVoxel.GetWeight();
          sdf_diff += distVoxel.GetSDF();

          if(rememberAllUpdatedVoxels || distVoxel.IsValid(minimumWeight))
            chunkManager.RememberUpdatedVoxel(chunk, voxelID, weight_diff, sdf_diff);

          if (updatedChunks)
            (*updatedChunks)[chunkID].insert(std::make_pair(chunk, voxelID));
        }
    }
  }

  bool ProjectionIntegrator::IntegrateColorPointCloud(const PointCloud& cloud, const Transform& cameraPose, Chunk* chunk) const
  {
    const float roundingFactor = 1.0f / chunk->GetVoxelResolutionMeters();

    Point4List raycastVoxels;
    Point3 chunkMin = Point3::Zero();
    Point3 chunkMax = chunk->GetNumVoxels();
    bool updated = false;
    size_t i = 0;
    Vec4 startCamera = Vec4::Zero();
    startCamera.head<3>() = cameraPose.translation();
    Transform inversePose = cameraPose.inverse();
    for (const Vec4& point : cloud.GetPoints())
      {
        const Vec3& color = cloud.GetColors()[i];
        Vec4 worldPoint = cameraPose * point;
        const Vec4 distance = worldPoint - startCamera;
        float depth = distance.norm();
        Vec4 dir = distance.normalized();
        float truncation = truncator->GetTruncationDistance(depth);
        Vec4 start = worldPoint - dir * truncation - chunk->GetOrigin();
        Vec4 end = worldPoint + dir * truncation - chunk->GetOrigin();
        start *= roundingFactor;
        end *= roundingFactor;

        raycastVoxels.clear();
        Raycast(start, end, chunkMin, chunkMax, &raycastVoxels);

        for (const Point4& voxelCoords : raycastVoxels)
          {
            VoxelID id = chunk->GetVoxelID(voxelCoords);
            ColorVoxel& voxel = chunk->GetColorVoxelMutable(id);
            DistVoxel& distVoxel = chunk->GetDistVoxelMutable(id);
            const Vec4& centroid = centroids[id] + chunk->GetOrigin();
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

        if (voxelToIntegrate.IsValid())
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

        if (distVoxelToIntegrate.IsValid())
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
