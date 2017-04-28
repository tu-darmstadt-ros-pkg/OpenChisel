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

#ifndef PROJECTIONINTEGRATOR_H_
#define PROJECTIONINTEGRATOR_H_

#include <open_chisel/pointcloud/PointCloud.h>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/geometry/Frustum.h>
#include <open_chisel/geometry/AABB.h>
#include <open_chisel/geometry/Raycast.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/camera/ColorImage.h>
#include <open_chisel/Chunk.h>

#include <open_chisel/truncation/Truncator.h>
#include <open_chisel/weighting/Weighter.h>
#include <open_chisel/ChunkManager.h>

namespace chisel
{

  class ProjectionIntegrator
  {
  public:
    ProjectionIntegrator();
    ProjectionIntegrator(const TruncatorPtr& t, const WeighterPtr& w, const float minWeight, const float maxWeight, float carvingDist, bool enableCarving, const Vec3List& centroids, bool rememberAllUpdatedVoxels);
    virtual ~ProjectionIntegrator();

    template<class VoxelType = DistVoxel>
    bool Integrate(const PointCloud& cloud, const Transform& cameraPose, Chunk<VoxelType>* chunk) const
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

    template<class VoxelType = DistVoxel>
    bool IntegratePointCloud(const PointCloud& cloud, const Transform& cameraPose, Chunk<VoxelType>* chunk) const
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

    void IntegratePoint(const Vec3& sensorOrigin, const Vec3& point, const Vec3& direction, float distance, ChunkManager& chunkManager, ChunkVoxelMap* updatedChunks) const;

    template<class VoxelType = DistVoxel>
    bool IntegrateColorPointCloud(const PointCloud& cloud, const Transform& cameraPose, Chunk<VoxelType>* chunk) const
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


    template<class VoxelType = DistVoxel>
    bool IntegrateChunk(const Chunk<VoxelType>* chunkToIntegrate, Chunk<VoxelType>* chunk) const{

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

    template<class VoxelType = DistVoxel>
    bool IntegrateColorChunk(const Chunk<VoxelType>* chunkToIntegrate, Chunk<VoxelType>* chunk) const{

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

    float ComputeTruncationDistance(const float depth) const
    {
        return truncator->GetTruncationDistance(depth);
    }

    template<class DataType>
    bool Integrate(const boost::shared_ptr<const DepthImage<DataType> >& depthImage, const PinholeCamera& camera, const Transform& cameraPose, ChunkManager& chunkManager, const ChunkID& chunkID) const
    {
      float resolution = chunkManager.GetResolution();
      const Point3& numVoxels = chunkManager.GetChunkSize();

      const Vec3 origin(numVoxels(0) * chunkID(0) * resolution, numVoxels(1) * chunkID(1) * resolution, numVoxels(2) * chunkID(2) * resolution);

      ChunkPtr chunk;
      bool gotChunkPointer = false;

      float diag = 2.0 * sqrt(3.0f) * resolution;
      Vec3 voxelCenter;
      bool updated = false;
      for (size_t i = 0; i < centroids.size(); i++)
        {
          voxelCenter = centroids[i] + origin;
          Vec3 voxelCenterInCamera = cameraPose * voxelCenter;
          Vec3 cameraPos = camera.ProjectPoint(voxelCenterInCamera);

          if (!camera.IsPointOnImage(cameraPos) || voxelCenterInCamera.z() < 0)
            continue;

          float voxelDist = voxelCenterInCamera.z();
          float depth = depthImage->DepthAt((int)cameraPos(1), (int)cameraPos(0)); //depthImage->BilinearInterpolateDepth(cameraPos(0), cameraPos(1));

          if(std::isnan(depth))
            {
              continue;
            }

          float truncation = truncator->GetTruncationDistance(depth);
          float surfaceDist = depth - voxelDist;

          if (fabs(surfaceDist) < truncation + diag)
            {

            if (!gotChunkPointer)
            {
              if (!chunkManager.HasChunk(chunkID))
                chunkManager.CreateChunk(chunkID);

              chunk = chunkManager.GetChunk(chunkID);
              gotChunkPointer = true;
            }

            //mutex.unlock();
              DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
              float weight_diff = - voxel.GetWeight();
              float sdf_diff = - voxel.GetSDF();
              voxel.Integrate(surfaceDist, weighter->GetWeight(surfaceDist, truncation), maximumWeight);
              weight_diff += voxel.GetWeight();
              sdf_diff += voxel.GetSDF();

              if(rememberAllUpdatedVoxels || voxel.IsValid(minimumWeight))
                chunkManager.RememberUpdatedVoxel(chunk, i, weight_diff, sdf_diff);

              updated = true;
            }
          else if (enableVoxelCarving && surfaceDist > truncation + carvingDist)
            {

            if (!gotChunkPointer)
            {
              if (!chunkManager.HasChunk(chunkID))
                chunkManager.CreateChunk(chunkID);

              chunk = chunkManager.GetChunk(chunkID);
              gotChunkPointer = true;
            }
              DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
              if (voxel.GetWeight() > 0)
                {
                  if (voxel.Carve(voxelCarvingResetTresh))
                    chunkManager.RememberCarvedVoxel(chunk, i);
                  updated = true;
                }
            }
        }
      return updated;
    }

    template<class DataType, class ColorType>
    bool IntegrateColor(const boost::shared_ptr<const DepthImage<DataType> >& depthImage, const PinholeCamera& depthCamera, const Transform& depthCameraPose, const boost::shared_ptr<const ColorImage<ColorType> >& colorImage, const PinholeCamera& colorCamera, const Transform& colorCameraPose, ChunkManager& chunkManager, const ChunkID& chunkID) const
    {
      float resolution = chunkManager.GetResolution();
      const Point3& numVoxels = chunkManager.GetChunkSize();

      const Vec3 origin(numVoxels(0) * chunkID(0) * resolution, numVoxels(1) * chunkID(1) * resolution, numVoxels(2) * chunkID(2) * resolution);

      ChunkPtr chunk;
      bool gotChunkPointer = false;

      float resolutionDiagonal = 2.0 * sqrt(3.0f) * resolution;
      bool updated = false;

      Color<ColorType> color;

      for (size_t i = 0; i < centroids.size(); i++)
        //parallel_for(centroids.begin(), centroids.end(), [&](const size_t& i)
        {
          Vec3 voxelCenter = centroids[i] + origin;
          Vec3 voxelCenterInCamera = depthCameraPose * voxelCenter;
          Vec3 cameraPos = depthCamera.ProjectPoint(voxelCenterInCamera);

          if (!depthCamera.IsPointOnImage(cameraPos) || voxelCenterInCamera.z() < 0)
            {
              continue;
            }

          float voxelDist = voxelCenterInCamera.z();
          float depth = depthImage->DepthAt((int)cameraPos(1), (int)cameraPos(0)); //depthImage->BilinearInterpolateDepth(cameraPos(0), cameraPos(1));

          if(std::isnan(depth))
            {
              continue;
            }

          float truncation = truncator->GetTruncationDistance(depth);
          float surfaceDist = depth - voxelDist;

          if (std::abs(surfaceDist) < truncation + resolutionDiagonal)
            {
              Vec3 voxelCenterInColorCamera = colorCameraPose* voxelCenter;
              Vec3 colorCameraPos = colorCamera.ProjectPoint(voxelCenterInColorCamera);

              if (!gotChunkPointer)
              {
                if (!chunkManager.HasChunk(chunkID))
                  chunkManager.CreateChunk(chunkID);

                chunk = chunkManager.GetChunk(chunkID);
                gotChunkPointer = true;
              }

              if(colorCamera.IsPointOnImage(colorCameraPos))
                {
                  ColorVoxel& colorVoxel = chunk->GetColorVoxelMutable(i);

                  if (colorVoxel.GetWeight() < 5)
                    {
                      int r = static_cast<int>(colorCameraPos(1));
                      int c = static_cast<int>(colorCameraPos(0));
                      colorImage->At(r, c, &color);

                      colorVoxel.Integrate(color.red, color.green, color.blue, 2);
                    }
                }

              DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
              float weight_diff = - voxel.GetWeight();
              float sdf_diff = - voxel.GetSDF();
              voxel.Integrate(surfaceDist, weighter->GetWeight(surfaceDist, truncation), maximumWeight);

              weight_diff += voxel.GetWeight();
              sdf_diff += voxel.GetSDF();

              if(rememberAllUpdatedVoxels || voxel.IsValid(minimumWeight))
                chunkManager.RememberUpdatedVoxel(chunk, i, weight_diff, sdf_diff);

              updated = true;
            }
          else if (enableVoxelCarving && surfaceDist > truncation + carvingDist)
            {
              if (!gotChunkPointer)
              {
                if (!chunkManager.HasChunk(chunkID))
                  chunkManager.CreateChunk(chunkID);

                chunk = chunkManager.GetChunk(chunkID);
                gotChunkPointer = true;
              }
              DistVoxel& voxel = chunk->GetDistVoxelMutable(i);
              if (voxel.GetWeight() > 0)
                {
                  if (voxel.Carve(voxelCarvingResetTresh))
                    chunkManager.RememberCarvedVoxel(chunk, i);
                  updated = true;
                }
            }


        }
      //);

      return updated;
    }

    inline const TruncatorPtr& GetTruncator() { return truncator; }
    inline void SetTruncator(const TruncatorPtr& value) { truncator = value; }
    inline const WeighterPtr& GetWeighter() { return weighter; }
    inline void SetWeighter(const WeighterPtr& value) { weighter = value; }

    inline float GetCarvingDist() { return carvingDist; }
    inline float GetCarvingDist() const { return carvingDist; }

    inline bool IsCarvingEnabled() { return enableVoxelCarving; }
    inline bool IsCarvingEnabled() const { return enableVoxelCarving; }

    inline void SetCarvingDist(float dist) { carvingDist = dist; }
    inline void SetCarvingEnabled(bool enabled) { enableVoxelCarving = enabled; }

    inline void SetCentroids(const Vec3List& c) { centroids = c; }

    inline void SetMaximumWeight(const float maxWeight){ maximumWeight = maxWeight; }
    inline float GetMaxVoxelWeight() const { return maximumWeight; }

    inline void SetMinimumWeight(const float minWeight){ minimumWeight = minWeight; }
    inline float GetMinVoxelWeight() const { return minimumWeight; }

    inline void SetVoxelCarvingResetTresh(const float tresh){ voxelCarvingResetTresh = tresh; }
    inline float GetVoxelCarvingResetTresh() const { return voxelCarvingResetTresh; }

    inline float RememberAllUpdatedVoxels(bool remember){ rememberAllUpdatedVoxels = remember; }

  protected:
    TruncatorPtr truncator;
    WeighterPtr weighter;
    float carvingDist;
    bool enableVoxelCarving;
    Vec3List centroids;
    float minimumWeight;
    float maximumWeight;
    float voxelCarvingResetTresh;
    bool rememberAllUpdatedVoxels;
  };

} // namespace chisel 

#endif // PROJECTIONINTEGRATOR_H_ 
