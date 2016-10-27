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

namespace chisel
{

    class Chisel
    {
        public:
            Chisel();
            Chisel(const Eigen::Vector3i& chunkSize, float voxelResolution, bool useColor);
            virtual ~Chisel();

            inline const ChunkManager& GetChunkManager() const { return chunkManager; }
            inline ChunkManager& GetMutableChunkManager() { return chunkManager; }
            inline void SetChunkManager(const ChunkManager& manager) { chunkManager = manager; }

            void IntegratePointCloud(const ProjectionIntegrator& integrator, const PointCloud& cloud, const Transform& extrinsic, const Vec3& sensorOrigin, float minDist, float maxDist);
            void IntegratePointCloud(const ProjectionIntegrator& integrator, const PointCloud& cloud, const Transform& extrinsic, float minDist, float maxDist);
            void IntegratePointCloud(const ProjectionIntegrator& integrator, const PointCloud& cloud, const Vec3& sensorOrigin, float minDist, float maxDist);
            void IntegrateRay(const ProjectionIntegrator& integrator, ChunkSet& updatedChunks, const Vec3& startPoint, const Vec3& endPoint, float minDist, float maxDist);

            void IntegrateChunks(const ProjectionIntegrator& integrator, ChunkManager& sourceChunkManager, ChunkSet& changedChunks);
            void DeleteChunks(ChunkSet &chunks);

            template <class DataType> void IntegrateDepthScan(const ProjectionIntegrator& integrator, const boost::shared_ptr<const DepthImage<DataType> >& depthImage, const Transform& extrinsic, const PinholeCamera& camera)
            {
                    Frustum frustum;
                    camera.SetupFrustum(extrinsic, &frustum);

                    ChunkIDList chunksIntersecting;
                    chunkManager.GetChunkIDsIntersecting(frustum, &chunksIntersecting);

                    Transform inverseExtrinsic = extrinsic.inverse();

                    std::mutex mutex;
                    ChunkIDList garbageChunks;

                    for(const ChunkID& chunkID : chunksIntersecting)
                    //parallel_for(chunksIntersecting.begin(), chunksIntersecting.end(), [&](const ChunkID& chunkID)
                    {
                        bool chunkNew = false;

                        mutex.lock();
                        if (!chunkManager.HasChunk(chunkID))
                        {
                           chunkNew = true;
                           chunkManager.CreateChunk(chunkID);
                        }

                        ChunkPtr chunk = chunkManager.GetChunk(chunkID);
                        mutex.unlock();

                        bool needsUpdate = integrator.Integrate(depthImage, camera, inverseExtrinsic, chunk.get());

                        mutex.lock();
                        if (needsUpdate)
                        {
                            chunkManager.RememberChangedChunk(chunkID);
                            for (int dx = -1; dx <= 1; dx++)
                            {
                                for (int dy = -1; dy <= 1; dy++)
                                {
                                    for (int dz = -1; dz <= 1; dz++)
                                    {
                                        meshesToUpdate[chunkID + ChunkID(dx, dy, dz)] = true;
                                    }
                                }
                            }
                        }
                        else if(chunkNew)
                        {
                            garbageChunks.push_back(chunkID);
                        }
                        mutex.unlock();
                    }//, maxThreads, threadTreshold
                    //);
                    GarbageCollect(garbageChunks);
                    //chunkManager.PrintMemoryStatistics();
            }

            template <class DataType, class ColorType> void IntegrateDepthScanColor(const ProjectionIntegrator& integrator, const boost::shared_ptr<const DepthImage<DataType> >& depthImage,  const Transform& depthExtrinsic, const PinholeCamera& depthCamera, const boost::shared_ptr<const ColorImage<ColorType> >& colorImage, const Transform& colorExtrinsic, const PinholeCamera& colorCamera)
            {
                    Frustum frustum;
                    depthCamera.SetupFrustum(depthExtrinsic, &frustum);

                    Transform inverseDepthExtrinsic = depthExtrinsic.inverse();
                    Transform inverseColorExtrinsic = colorExtrinsic.inverse();

                    ChunkIDList chunksIntersecting;
                    chunkManager.GetChunkIDsIntersecting(frustum, &chunksIntersecting);
                    std::mutex mutex;
                    ChunkIDList garbageChunks;
                    for ( const ChunkID& chunkID : chunksIntersecting)
                    //parallel_for(chunksIntersecting.begin(), chunksIntersecting.end(), [&](const ChunkID& chunkID)
                    {

                        mutex.lock();
                        bool chunkNew = false;
                        if (!chunkManager.HasChunk(chunkID))
                        {
                           chunkNew = true;
                           chunkManager.CreateChunk(chunkID);
                        }

                        ChunkPtr chunk = chunkManager.GetChunk(chunkID);
                        mutex.unlock();


                        bool needsUpdate = integrator.IntegrateColor(depthImage, depthCamera, inverseDepthExtrinsic, colorImage, colorCamera, inverseColorExtrinsic, chunk.get());

                        mutex.lock();
                        if (needsUpdate)
                        {
                            chunkManager.RememberChangedChunk(chunkID);

                            for (int dx = -1; dx <= 1; dx++)
                            {
                                for (int dy = -1; dy <= 1; dy++)
                                {
                                    for (int dz = -1; dz <= 1; dz++)
                                    {
                                        meshesToUpdate[chunkID + ChunkID(dx, dy, dz)] = true;
                                    }
                                }
                            }
                        }
                        else if(chunkNew)
                        {
                            garbageChunks.push_back(chunkID);
                        }
                        mutex.unlock();
                    }//, maxThreads, threadTreshold
                    //);

                    GarbageCollect(garbageChunks);
                    //chunkManager.PrintMemoryStatistics();
            }

            inline void SetThreadingParameters(unsigned int maxThreads, unsigned int threadTreshold)
            {
              this->maxThreads = maxThreads;
              this->threadTreshold = threadTreshold;

              chunkManager.SetThreadingParameters(maxThreads, threadTreshold);
            }

            void GarbageCollect(const ChunkIDList& chunks);
            void UpdateMeshes();

            bool SaveAllMeshesToPLY(const std::string& filename);
            void Reset();

            const ChunkSet& GetMeshesToUpdate() const { return meshesToUpdate; }

        protected:
            ChunkManager chunkManager;
            ChunkSet meshesToUpdate;
            unsigned int maxThreads;
            unsigned int threadTreshold;

        private:
            Point3 getVoxelCoordinates(VoxelID id, Eigen::Vector3i chunkSize);
            bool interpolateDistVoxel(const Vec3& voxelPos, ChunkManager& sourceChunkManager, DistVoxel* voxel);
            bool interpolateGridTrilinear(const Vec3& startPos, const Vec3& endPos, const float& resolution, ChunkManager& sourceChunkManager, ChunkManager& targetChunkManager, const bool& useColor);
            bool interpolateGridNearestNeighbour(const Vec3& startPos, const Vec3& endPos, const float& resolution, ChunkManager& sourceChunkManager, ChunkManager& stargetChunkManager, const bool& useColor);
            void DetermineMeshesToUpdate(ChunkSet& updatedChunks);


    };
    typedef boost::shared_ptr<Chisel> ChiselPtr;
    typedef boost::shared_ptr<const Chisel> ChiselConstPtr;

} // namespace chisel 

#endif // CHISEL_H_ 
