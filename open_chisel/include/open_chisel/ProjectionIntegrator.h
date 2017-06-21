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

    float ComputeTruncationDistance(const float depth) const
    {
        return truncator->GetTruncationDistance(depth);
    }

    template<class VoxelType>
    void Integrate(const Vec3& sensorOrigin, const Vec3& point, const Vec3& direction, float distance, Vec3 color, ChunkManager<VoxelType>& chunkManager, ChunkVoxelMap<VoxelType>* updatedChunks) const
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
            chunkManager.CreateChunk(chunkID);

          ChunkPtr<VoxelType> chunk = chunkManager.GetChunk(chunkID);
          const Vec3& origin = chunk->GetOrigin();

          voxelPos -= origin;
          VoxelID voxelID = chunk->GetVoxelID(voxelPos);

          VoxelType& distVoxel = chunk->GetDistVoxelMutable(voxelID);
          const Vec3& centroid = centroids[voxelID] + origin;
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

            if (chunk->HasColors())
            {
              ColorVoxel& colorVoxel = chunk->GetColorVoxelMutable(voxelID);
              colorVoxel.Integrate((uint8_t)(color.x() * 255.0f), (uint8_t)(color.y() * 255.0f), (uint8_t)(color.z() * 255.0f), 2);
            }

            if (updatedChunks)
              (*updatedChunks)[chunkID].insert(std::make_pair(chunk, voxelID));
          }
      }
    }

    template<class DataType, class VoxelType>
    bool Integrate(const boost::shared_ptr<const DepthImage<DataType> >& depthImage, const PinholeCamera& camera, const Transform& cameraPose, ChunkManager<VoxelType>& chunkManager, const ChunkID& chunkID) const
    {
      float resolution = chunkManager.GetResolution();
      const Point3& numVoxels = chunkManager.GetChunkSize();

      const Vec3 origin(numVoxels(0) * chunkID(0) * resolution, numVoxels(1) * chunkID(1) * resolution, numVoxels(2) * chunkID(2) * resolution);

      ChunkPtr<VoxelType> chunk;
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
              VoxelType& voxel = chunk->GetDistVoxelMutable(i);
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
              VoxelType& voxel = chunk->GetDistVoxelMutable(i);
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

    template<class DataType, class ColorType, class VoxelType>
    bool IntegrateColor(const boost::shared_ptr<const DepthImage<DataType> >& depthImage, const PinholeCamera& depthCamera, const Transform& depthCameraPose, const boost::shared_ptr<const ColorImage<ColorType> >& colorImage, const PinholeCamera& colorCamera, const Transform& colorCameraPose, ChunkManager<VoxelType>& chunkManager, const ChunkID& chunkID) const
    {
      float resolution = chunkManager.GetResolution();
      const Point3& numVoxels = chunkManager.GetChunkSize();

      const Vec3 origin(numVoxels(0) * chunkID(0) * resolution, numVoxels(1) * chunkID(1) * resolution, numVoxels(2) * chunkID(2) * resolution);

      ChunkPtr<VoxelType> chunk;
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

              VoxelType& voxel = chunk->GetDistVoxelMutable(i);
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
              VoxelType& voxel = chunk->GetDistVoxelMutable(i);
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
