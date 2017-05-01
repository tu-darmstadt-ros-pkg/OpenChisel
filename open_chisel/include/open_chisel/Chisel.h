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

#ifndef CHISEL_H_
#define CHISEL_H_

#include <open_chisel/threading/Threading.h>
#include <open_chisel/ChunkManager.h>
#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/geometry/Frustum.h>
#include <open_chisel/pointcloud/PointCloud.h>
#include <open_chisel/io/PLY.h>
#include <open_chisel/geometry/Raycast.h>

namespace chisel
{
    template<class VoxelType = DistVoxel>
    class Chisel
    {
        public:
            Chisel()
            {
              // TODO Auto-generated constructor stub
              maxThreads = 4;
              threadTreshold = 500;
            }

            Chisel(const Eigen::Vector3i& chunkSize, float voxelResolution, bool useColor, float minimumWeight) :
              chunkManager(chunkSize, voxelResolution, useColor, minimumWeight)
            {
              maxThreads = 4;
              threadTreshold = 500;
            }

            Chisel(const Eigen::Vector3i& chunkSize, float voxelResolution, bool useColor) :
              chunkManager(chunkSize, voxelResolution, useColor, 0.0f)
            {
              maxThreads = 4;
              threadTreshold = 500;
            }
            virtual ~Chisel(){}

            inline const ChunkManager<VoxelType>& GetChunkManager() const { return chunkManager; }
            inline ChunkManager<VoxelType>& GetMutableChunkManager() { return chunkManager; }
            inline void SetChunkManager(const ChunkManager<VoxelType>& manager) { chunkManager = manager; }

            //Integrate pointcloud after transforming to target frame using a given sensor pose
            void IntegratePointCloud(const ProjectionIntegrator& integrator, const PointCloud& cloud, const Transform& extrinsic, const Vec3& sensorOrigin, float minDist, float maxDist)
            {
              ChunkVoxelMap<VoxelType> updatedVoxels;

              //TODO: parallelize
              for (const Vec3& point : cloud.GetPoints())
              {
                  Vec3 point_transformed = extrinsic * point;
                  IntegrateRay(integrator, updatedVoxels, sensorOrigin, point_transformed, minDist, maxDist);
              }

              DetermineUpdatedChunks(updatedVoxels);
            }

            //Integrate pointcloud after transforming to target frame
            void IntegratePointCloud(const ProjectionIntegrator& integrator, const PointCloud& cloud, const Transform& extrinsic, float minDist, float maxDist)
            {
              ChunkVoxelMap<VoxelType> updatedVoxels;
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
            void IntegratePointCloud(const ProjectionIntegrator& integrator, const PointCloud& cloud, const Vec3& sensorOrigin, float minDist, float maxDist)
            {
              ChunkVoxelMap<VoxelType> updatedVoxels;

              //TODO: parallelize
              for (const Vec3& point : cloud.GetPoints())
              {
                  IntegrateRay(integrator, updatedVoxels, sensorOrigin, point, minDist, maxDist);
              }

              DetermineUpdatedChunks(updatedVoxels);
            }

            //Integrate ray already transformed to target frame
            void IntegrateRay(const ProjectionIntegrator& integrator, ChunkVoxelMap<VoxelType>& updatedVoxels, const Vec3& startPoint, const Vec3& endPoint, float minDist, float maxDist)
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
                  chunkManager.ClearPassedVoxels(startPoint, truncatedPositiveEnd, integrator.GetVoxelCarvingResetTresh(), &updatedVoxels);
              }

              if (integrateRay)
                integrator.IntegratePoint(startPoint, endPoint, difference, distance, chunkManager, &updatedVoxels);
            }

            template <class DataType> void IntegrateDepthScan(const ProjectionIntegrator& integrator, const boost::shared_ptr<const DepthImage<DataType> >& depthImage, const Transform& extrinsic, const PinholeCamera& camera)
            {
                    Frustum frustum;
                    camera.SetupFrustum(extrinsic, &frustum);

                    ChunkIDList chunksIntersecting;
                    chunkManager.GetChunkIDsIntersecting(frustum, &chunksIntersecting);

                    Transform inverseExtrinsic = extrinsic.inverse();

                    std::mutex mutex;

                    for(const ChunkID& chunkID : chunksIntersecting)
                    //parallel_for(chunksIntersecting.begin(), chunksIntersecting.end(), [&](const ChunkID& chunkID)
                    {
                        bool needsUpdate = integrator.Integrate(depthImage, camera, inverseExtrinsic, chunkManager, chunkID);

                        mutex.lock();
                        if (needsUpdate)
                        {
                            ChunkPtr<VoxelType> chunk = chunkManager.GetChunk(chunkID);
                            chunkManager.RememberUpdatedChunk(chunk, meshesToUpdate);
                        }

                        mutex.unlock();
                    }//, maxThreads, threadTreshold
                    //);
            }

            template <class DataType, class ColorType>
            void IntegrateDepthScanColor(const ProjectionIntegrator& integrator, const boost::shared_ptr<const DepthImage<DataType> >& depthImage,  const Transform& depthExtrinsic, const PinholeCamera& depthCamera, const boost::shared_ptr<const ColorImage<ColorType> >& colorImage, const Transform& colorExtrinsic, const PinholeCamera& colorCamera)
            {
                    Frustum frustum;
                    depthCamera.SetupFrustum(depthExtrinsic, &frustum);

                    Transform inverseDepthExtrinsic = depthExtrinsic.inverse();
                    Transform inverseColorExtrinsic = colorExtrinsic.inverse();

                    ChunkIDList chunksIntersecting;
                    chunkManager.GetChunkIDsIntersecting(frustum, &chunksIntersecting);
                    std::mutex mutex;
                    //ChunkIDList garbageChunks;
                    for ( const ChunkID& chunkID : chunksIntersecting)
                    //parallel_for(chunksIntersecting.begin(), chunksIntersecting.end(), [&](const ChunkID& chunkID)
                    {
                        bool needsUpdate = integrator.IntegrateColor(depthImage, depthCamera, inverseDepthExtrinsic, colorImage, colorCamera, inverseColorExtrinsic, chunkManager, chunkID);

                        mutex.lock();
                        if (needsUpdate)
                        {
                            ChunkPtr<VoxelType> chunk = chunkManager.GetChunk(chunkID);
                            chunkManager.RememberUpdatedChunk(chunk, meshesToUpdate);
                        }
                        mutex.unlock();
                    }//, maxThreads, threadTreshold
                    //);
                    //chunkManager.PrintMemoryStatistics();
            }

            inline void SetThreadingParameters(unsigned int maxThreads, unsigned int threadTreshold)
            {
              this->maxThreads = maxThreads;
              this->threadTreshold = threadTreshold;

              chunkManager.SetThreadingParameters(maxThreads, threadTreshold);
            }

            void GarbageCollect(const ChunkIDList& chunks)
            {
              for (const ChunkID& chunkID : chunks)
                {
                  chunkManager.RemoveChunk(chunkID);
                }
            }

            void UpdateMeshes()
            {
              chunkManager.RecomputeMeshes(meshesToUpdate);
              meshesToUpdate.clear();
            }

            bool SaveAllMeshesToPLY(const std::string& filename)
            {
              printf("Saving all meshes to PLY file...\n");

              chisel::MeshPtr fullMesh(new chisel::Mesh());

              size_t v = 0;
              for (const std::pair<ChunkID, MeshPtr>& it : chunkManager.GetAllMeshes())
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

            void Reset()
            {
              chunkManager.Reset();
              meshesToUpdate.clear();
            }

            const ChunkSet& GetMeshesToUpdate() const { return meshesToUpdate; }

        protected:
            ChunkManager<VoxelType> chunkManager;
            ChunkSet meshesToUpdate;
            unsigned int maxThreads;
            unsigned int threadTreshold;

        private:
            void DetermineUpdatedChunks(ChunkVoxelMap<VoxelType>& updatedVoxels)
            {
              for (const auto& entry : updatedVoxels)
              {
                ChunkPtr<VoxelType> chunk = chunkManager.GetChunk(entry.first);
                chunkManager.RememberUpdatedChunk(chunk, meshesToUpdate);
              }
            }


    };

    template<class VoxelType>
    using ChiselPtr = boost::shared_ptr<Chisel<VoxelType>>;

    template<class VoxelType>
    using ChiselConstPtr = boost::shared_ptr<const Chisel<VoxelType>>;


} // namespace chisel 

#endif // CHISEL_H_ 
