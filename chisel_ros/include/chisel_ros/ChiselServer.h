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

#ifndef CHISELSERVER_H_
#define CHISELSERVER_H_

#include <boost/shared_ptr.hpp>

#include <chisel_msgs/ResetService.h>
#include <chisel_msgs/PauseService.h>
#include <chisel_msgs/SaveMeshService.h>
#include <chisel_msgs/GetAllChunksService.h>
#include <chisel_msgs/GetLatestChunksService.h>
#include <chisel_msgs/GetDeletedChunksService.h>

#include <chisel_ros/Conversions.h>
#include <chisel_ros/Serialization.h>

#include <open_chisel/Chisel.h>
#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/camera/ColorImage.h>
#include <open_chisel/pointcloud/PointCloud.h>
#include <open_chisel/DistVoxel.h>
#include <open_chisel/truncation/QuadraticTruncator.h>
#include <open_chisel/weighting/ConstantWeighter.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace chisel_ros
{

    typedef float DepthData;
    typedef uint8_t ColorData;

    enum class FusionMode
    {
        DepthImage,
        PointCloud
    };

    struct RosCameraTopic
    {
        std::string imageTopic;
        std::string infoTopic;
        std::string transform;
        chisel::PinholeCamera cameraModel;
        ros::Subscriber imageSubscriber;
        ros::Subscriber infoSubscriber;
        ros::Publisher lastPosePublisher;
        ros::Publisher frustumPublisher;
        chisel::Transform lastPose;
        ros::Time lastImageTimestamp;
        bool gotPose;
        bool gotInfo;
        bool gotImage;
    };

    struct RosPointCloudTopic
    {
        std::string cloudTopic;
        std::string transform;
        ros::Subscriber cloudSubscriber;
        chisel::Transform lastPose;
        ros::Time lastTimestamp;
        bool gotPose;
        bool gotCloud;
    };

    template<class VoxelType = chisel::DistVoxel>
    class ChiselServer
    {
        public:

            ChiselServer() :
                useColor(false), hasNewData(false), nearPlaneDist(0.05), farPlaneDist(5), isPaused(false), mode(FusionMode::DepthImage)
            {
                maxThreads = 4;
                threadTreshold = 500;
            }

            ChiselServer(const ros::NodeHandle& nodeHanlde, int chunkSizeX, int chunkSizeY, int chunkSizeZ, float resolution, bool color, FusionMode fusionMode) :
                nh(nodeHanlde), useColor(color), hasNewData(false), isPaused(false), mode(fusionMode)
            {
                chiselMap.reset(new chisel::Chisel<>(Eigen::Vector3i(chunkSizeX, chunkSizeY, chunkSizeZ), resolution, color));
                maxThreads = 4;
                threadTreshold = 500;
            }
            virtual ~ChiselServer(){}



            inline chisel::ChiselPtr<VoxelType> GetChiselMap() { return chiselMap; }
            inline void SetChiselMap(const chisel::ChiselPtr<VoxelType> value) { chiselMap = value; }

            inline const std::string& GetBaseTransform() const { return baseTransform; }
            inline const std::string& GetMeshTopic() const { return meshTopic; }

            inline void SetThreadingParameters(unsigned int maxThreads, unsigned int threadTreshold)
            {
              this->maxThreads = maxThreads;
              this->threadTreshold = threadTreshold;
              chiselMap->SetThreadingParameters(maxThreads, threadTreshold);

            }

            void SetupProjectionIntegrator(const chisel::Vec4& truncation, uint16_t weight, bool useCarving, float carvingDist)
            {
                projectionIntegrator.SetCentroids(GetChiselMap()->GetChunkManager().GetCentroids());
                projectionIntegrator.SetTruncator(chisel::TruncatorPtr(new chisel::QuadraticTruncator(truncation(0), truncation(1), truncation(2), truncation(3))));
                projectionIntegrator.SetWeighter(chisel::WeighterPtr(new chisel::ConstantWeighter(weight)));
                projectionIntegrator.SetCarvingDist(carvingDist);
                projectionIntegrator.SetCarvingEnabled(useCarving);
            }

            void SetupMeshPublisher(const std::string& meshTopic)
            {
                this->meshTopic = meshTopic;
                meshPublisher = nh.advertise<visualization_msgs::Marker>(meshTopic, 1);
                normalPublisher = nh.advertise<visualization_msgs::Marker>("chisel_normals", 1);
                tsdfPublisher = nh.advertise<sensor_msgs::PointCloud2>("chisel_tsdf", 1);
            }

            void SetupChunkBoxPublisher(const std::string& boxTopic)
            {
                chunkBoxTopic = boxTopic;
                chunkBoxPublisher = nh.advertise<visualization_msgs::Marker>(chunkBoxTopic, 1);
                latestChunkPublisher = nh.advertise<visualization_msgs::Marker>(chunkBoxTopic + "/latest", 1);
            }

            void SetupDepthPosePublisher(const std::string& depthPoseTopic)
            {
                depthCamera.lastPosePublisher = nh.advertise<geometry_msgs::PoseStamped>(depthPoseTopic, 1);
            }

            void SetupColorPosePublisher(const std::string& colorPoseTopic)
            {
                colorCamera.lastPosePublisher = nh.advertise<geometry_msgs::PoseStamped>(colorPoseTopic, 1);
            }

            void SetupDepthFrustumPublisher(const std::string& frustumTopic)
            {
                depthCamera.frustumPublisher = nh.advertise<visualization_msgs::Marker>(frustumTopic, 1);
            }

            void SetupColorFrustumPublisher(const std::string& frustumTopic)
            {
                colorCamera.frustumPublisher = nh.advertise<visualization_msgs::Marker>(frustumTopic, 1);
            }


            void PublishMeshes()
            {
                visualization_msgs::Marker marker;
                FillMarkerTopicWithMeshes(&marker);

                if(!marker.points.empty())
                    meshPublisher.publish(marker);

               /* visualization_msgs::Marker normalMarker;
                FillNormalMarkerTopicWithMeshes(&normalMarker);

                if(!normalMarker.points.empty())
                    normalPublisher.publish(normalMarker);*/

            }
            void PublishChunkBoxes()
            {
                const chisel::ChunkManager<VoxelType>& chunkManager = chiselMap->GetChunkManager();
                visualization_msgs::Marker marker;
                marker.header.stamp = ros::Time::now();
                marker.header.frame_id = baseTransform;
                marker.ns = "chunk_box";
                marker.type = visualization_msgs::Marker::CUBE_LIST;
                marker.scale.x = chunkManager.GetChunkSize()(0) * chunkManager.GetResolution();
                marker.scale.y = chunkManager.GetChunkSize()(1) * chunkManager.GetResolution();
                marker.scale.z = chunkManager.GetChunkSize()(2) * chunkManager.GetResolution();
                marker.pose.position.x = 0;
                marker.pose.position.y = 0;
                marker.pose.position.z = 0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.color.r = 0.95f;
                marker.color.g = 0.3f;
                marker.color.b = 0.3f;
                marker.color.a = 0.6f;
                for (const std::pair<chisel::ChunkID, chisel::ChunkPtr<VoxelType>>& pair : chunkManager.GetChunks())
                {
                    chisel::AABB aabb = pair.second->ComputeBoundingBox();
                    chisel::Vec3 center = aabb.GetCenter();
                    geometry_msgs::Point pt;
                    pt.x = center.x();
                    pt.y = center.y();
                    pt.z = center.z();
                    marker.points.push_back(pt);
                }

                chunkBoxPublisher.publish(marker);
            }

            void PublishLatestChunkBoxes()
            {
                if (!latestChunkPublisher) return;
                const chisel::ChunkManager<VoxelType>& chunkManager = chiselMap->GetChunkManager();
                visualization_msgs::Marker marker;
                marker.header.stamp = ros::Time::now();
                marker.header.frame_id = baseTransform;
                marker.ns = "chunk_box";
                marker.type = visualization_msgs::Marker::CUBE_LIST;
                marker.scale.x = chunkManager.GetChunkSize()(0) * chunkManager.GetResolution();
                marker.scale.y = chunkManager.GetChunkSize()(1) * chunkManager.GetResolution();
                marker.scale.z = chunkManager.GetChunkSize()(2) * chunkManager.GetResolution();
                marker.pose.position.x = 0;
                marker.pose.position.y = 0;
                marker.pose.position.z = 0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.color.r = 0.3f;
                marker.color.g = 0.95f;
                marker.color.b = 0.3f;
                marker.color.a = 0.6f;
                const chisel::ChunkSet& latest = chiselMap->GetMeshesToUpdate();

                for (auto id : latest)
                {
                    if(chunkManager.HasChunk(id.first))
                    {
                        chisel::AABB aabb = chunkManager.GetChunk(id.first)->ComputeBoundingBox();
                        chisel::Vec3 center = aabb.GetCenter();
                        geometry_msgs::Point pt;
                        pt.x = center.x();
                        pt.y = center.y();
                        pt.z = center.z();
                        marker.points.push_back(pt);
                    }
                }

                latestChunkPublisher.publish(marker);
            }

            void PublishDepthPose()
            {
                chisel::Transform lastPose = depthCamera.lastPose;

                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = baseTransform;
                pose.header.stamp = depthCamera.lastImageTimestamp;
                pose.pose.position.x = lastPose.translation()(0);
                pose.pose.position.y = lastPose.translation()(1);
                pose.pose.position.z = lastPose.translation()(2);

                chisel::Quaternion quat(lastPose.rotation());
                pose.pose.orientation.x = quat.x();
                pose.pose.orientation.y = quat.y();
                pose.pose.orientation.z = quat.z();
                pose.pose.orientation.w = quat.w();

                depthCamera.lastPosePublisher.publish(pose);
            }

            void PublishColorPose()
            {
                chisel::Transform lastPose = colorCamera.lastPose;

                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = baseTransform;
                pose.header.stamp = colorCamera.lastImageTimestamp;
                pose.pose.position.x = lastPose.translation()(0);
                pose.pose.position.y = lastPose.translation()(1);
                pose.pose.position.z = lastPose.translation()(2);

                chisel::Quaternion quat(lastPose.rotation());
                pose.pose.orientation.x = quat.x();
                pose.pose.orientation.y = quat.y();
                pose.pose.orientation.z = quat.z();
                pose.pose.orientation.w = quat.w();

                colorCamera.lastPosePublisher.publish(pose);
            }

            void PublishDepthFrustum()
            {
                chisel::Frustum frustum;
                depthCamera.cameraModel.SetupFrustum(depthCamera.lastPose, &frustum);
                visualization_msgs::Marker marker = CreateFrustumMarker(frustum);
                depthCamera.frustumPublisher.publish(marker);
            }

            void PublishColorFrustum()
            {
                chisel::Frustum frustum;
                colorCamera.cameraModel.SetupFrustum(colorCamera.lastPose, &frustum);
                visualization_msgs::Marker marker = CreateFrustumMarker(frustum);
                colorCamera.frustumPublisher.publish(marker);
            }

            void PublishTSDFMarkers()
            {
              const chisel::ChunkManager<VoxelType>& chunkManager = chiselMap->GetChunkManager();
              const float resolution = chunkManager.GetResolution();
              pcl::PointCloud<pcl::PointXYZRGB> cloud;
              cloud.clear();

              int stepSize=1;

              for (const std::pair<chisel::ChunkID, chisel::ChunkPtr<VoxelType>>& pair : chunkManager.GetChunks())
              {
                const std::vector<VoxelType>&  voxels = pair.second->GetVoxels();
                chisel::Vec3 origin = pair.second->GetOrigin();

                int voxelID = 0;

                for (int z = 0; z < chunkManager.GetChunkSize()(2); z+=stepSize)
                {
                  for (int y = 0; y < chunkManager.GetChunkSize()(1); y+=stepSize)
                  {
                    for (int x = 0; x < chunkManager.GetChunkSize()(0); x+=stepSize)
                    {
                      if(voxels[voxelID].GetWeight() > 0)
                      {

                          float sdf = voxels[voxelID].GetSDF();

                          if(sdf>0)
                          {
                            pcl::PointXYZRGB point = pcl::PointXYZRGB(0, 0, 255);
                            point.x = origin.x() + x *resolution;
                            point.y = origin.y() + y *resolution;
                            point.z = origin.z() + z *resolution;
                            cloud.points.insert(cloud.end(), point);
                          }
                          else
                          {
                            pcl::PointXYZRGB point = pcl::PointXYZRGB(255, 0, 0);
                            point.x = origin.x() + x *resolution;
                            point.y = origin.y() + y *resolution;
                            point.z = origin.z() + z *resolution;
                            cloud.points.insert(cloud.end(), point);
                          }
                      }

                      voxelID+=stepSize;
                    }
                  }
                }
              }
              sensor_msgs::PointCloud2 pc;

              pcl::toROSMsg(cloud, pc);
              pc.header.frame_id = baseTransform;
              pc.header.stamp = ros::Time::now();
              tsdfPublisher.publish(pc);
            }

            void SubscribeDepthImage(const std::string& depthImageTopic, const std::string& cameraInfoTopic, const std::string& transform)
            {
                depthCamera.imageTopic = depthImageTopic;
                depthCamera.transform = transform;
                depthCamera.infoTopic = cameraInfoTopic;
                depthCamera.imageSubscriber = nh.subscribe(depthCamera.imageTopic, 1, &ChiselServer::DepthImageCallback, this);
                depthCamera.infoSubscriber = nh.subscribe(depthCamera.infoTopic, 1, &ChiselServer::DepthCameraInfoCallback, this);
            }

            void DepthCameraInfoCallback(sensor_msgs::CameraInfoConstPtr cameraInfo)
            {
                SetDepthCameraInfo(cameraInfo);
            }

            void DepthImageCallback(sensor_msgs::ImageConstPtr depthImage)
            {
                if (IsPaused()) return;
                SetDepthImage(depthImage);

                bool gotTransform = false;
                tf::StampedTransform tf;

                int tries = 0;
                int maxTries = 10;

                while(!gotTransform && tries < maxTries)
                {
                    tries++;
                    try
                    {
                        transformListener.waitForTransform(baseTransform, depthImage->header.frame_id, depthImage->header.stamp, ros::Duration(0.5));
                        transformListener.lookupTransform(baseTransform, depthImage->header.frame_id, depthImage->header.stamp, tf);
                        depthCamera.gotPose = true;
                        gotTransform = true;
                    }
                    catch (std::exception& e)
                    {
                        ros::Rate lookupRate(0.5f);
                        ROS_WARN("%s\n", e.what());
                    }
                }

                depthCamera.lastPose = RosTfToChiselTf(tf);

                hasNewData = true;
            }

            void SubscribeColorImage(const std::string& colorImageTopic, const std::string& cameraInfoTopic, const std::string& transform)
            {
                colorCamera.imageTopic = cameraInfoTopic;
                colorCamera.transform = transform;
                colorCamera.infoTopic = cameraInfoTopic;
                colorCamera.imageSubscriber = nh.subscribe(colorCamera.imageTopic, 1, &ChiselServer::ColorImageCallback, this);
                colorCamera.infoSubscriber = nh.subscribe(colorCamera.infoTopic, 1, &ChiselServer::ColorCameraInfoCallback, this);
            }

            void ColorCameraInfoCallback(sensor_msgs::CameraInfoConstPtr cameraInfo)
            {
                if (IsPaused()) return;
                SetColorCameraInfo(cameraInfo);
            }
            void ColorImageCallback(sensor_msgs::ImageConstPtr colorImage)
            {
                if (IsPaused()) return;
                SetColorImage(colorImage);

                bool gotTransform = false;
                tf::StampedTransform tf;

                int tries = 0;
                int maxTries = 10;

                while(!gotTransform && tries < maxTries)
                {
                    tries++;
                    try
                    {
                        transformListener.waitForTransform(baseTransform, colorImage->header.frame_id, colorImage->header.stamp, ros::Duration(0.5));
                        transformListener.lookupTransform(baseTransform, colorImage->header.frame_id, colorImage->header.stamp, tf);
                        colorCamera.gotPose = true;
                        gotTransform = true;
                    }
                    catch (std::exception& e)
                    {
                        ros::Rate lookupRate(0.5f);
                        ROS_WARN("%s\n", e.what());
                    }
                }

                colorCamera.lastPose = RosTfToChiselTf(tf);
            }

            void SubscribePointCloud(const std::string& topic)
            {
                pointcloudTopic.cloudTopic = topic;
                pointcloudTopic.gotCloud = false;
                pointcloudTopic.gotPose = false;
                pointcloudTopic.cloudSubscriber = nh.subscribe(pointcloudTopic.cloudTopic, 1, &ChiselServer::PointCloudCallback, this);
            }

            void PointCloudCallback(sensor_msgs::PointCloud2ConstPtr pointcloud)
            {
                if (IsPaused()) return;
                if (!lastPointCloud.get())
                {
                    lastPointCloud.reset(new chisel::PointCloud());
                }
                ROSPointCloudToChisel(pointcloud, lastPointCloud.get());
                pointcloudTopic.transform = pointcloud->header.frame_id;
                bool gotTransform = false;
                tf::StampedTransform tf;

                int tries = 0;
                int maxTries = 10;

                while(!gotTransform && tries < maxTries)
                {
                    tries++;
                    try
                    {
                        transformListener.waitForTransform(baseTransform, pointcloudTopic.transform, pointcloud->header.stamp, ros::Duration(0.5));
                        transformListener.lookupTransform(baseTransform, pointcloudTopic.transform, pointcloud->header.stamp, tf);
                        pointcloudTopic.gotPose = true;
                        gotTransform = true;
                    }
                    catch (std::exception& e)
                    {
                        ros::Rate lookupRate(0.5f);
                        ROS_WARN("%s\n", e.what());
                    }
                }

                pointcloudTopic.lastPose = RosTfToChiselTf(tf);
                pointcloudTopic.lastTimestamp = pointcloud->header.stamp;
                hasNewData = true;
            }

            void IntegrateLastDepthImage()
            {
                if (!IsPaused() && depthCamera.gotInfo && depthCamera.gotPose && lastDepthImage.get())
                {
                    chiselMap->GetMutableChunkManager().clearIncrementalChanges();

                    if(useColor)
                    {
                        chiselMap->template IntegrateDepthScanColor<DepthData, ColorData>(projectionIntegrator,  lastDepthImage, depthCamera.lastPose, depthCamera.cameraModel, lastColorImage, colorCamera.lastPose, colorCamera.cameraModel);
                    }
                    else
                    {
                        chiselMap->template IntegrateDepthScan<DepthData>(projectionIntegrator, lastDepthImage, depthCamera.lastPose, depthCamera.cameraModel);
                    }
                    //PublishLatestChunkBoxes();
                    //PublishDepthFrustum();

                    chiselMap->UpdateMeshes();
                    hasNewData = false;
                }
            }

            void IntegrateLastPointCloud()
            {
                if (!IsPaused()  && pointcloudTopic.gotPose && lastPointCloud.get())
                {
                    chiselMap->GetMutableChunkManager().clearIncrementalChanges();

                    chiselMap->IntegratePointCloud(projectionIntegrator, *lastPointCloud, pointcloudTopic.lastPose, nearPlaneDist, farPlaneDist);
                    //chiselMap->IntegratePointCloud(projectionIntegrator, *lastPointCloud, pointcloudTopic.lastPose, 0.1f, nearPlaneDist, farPlaneDist);
                    //PublishLatestChunkBoxes();
                    chiselMap->UpdateMeshes();;
                    hasNewData = false;
                }
            }

            void FillMarkerTopicWithMeshes(visualization_msgs::Marker* marker)
            {
                assert(marker != nullptr);
                marker->header.stamp = ros::Time::now();
                marker->header.frame_id = baseTransform;
                marker->scale.x = 1;
                marker->scale.y = 1;
                marker->scale.z = 1;
                marker->pose.orientation.x = 0;
                marker->pose.orientation.y = 0;
                marker->pose.orientation.z = 0;
                marker->pose.orientation.w = 1;
                marker->type = visualization_msgs::Marker::TRIANGLE_LIST;
                const chisel::MeshMap& meshMap = chiselMap->GetChunkManager().GetAllMeshes();

                FillMarkerTopicWithMeshes(meshMap, marker);
            }

            void FillMarkerTopicWithMeshes(const chisel::MeshMap& meshMap, visualization_msgs::Marker* marker)
            {
                if(meshMap.size() == 0)
                {
                    return;
                }

                chisel::Vec3 lightDir(0.8f, -0.2f, 0.7f);
                lightDir.normalize();
                chisel::Vec3 lightDir1(-0.5f, 0.2f, 0.2f);
                lightDir.normalize();
                const chisel::Vec3 ambient(0.2f, 0.2f, 0.2f);
                //int idx = 0;
                for (const std::pair<chisel::ChunkID, chisel::MeshPtr>& meshes : meshMap)
                {
                    const chisel::MeshPtr& mesh = meshes.second;
                    for (size_t i = 0; i < mesh->vertices.size(); i++)
                    {
                        const chisel::Vec3& vec = mesh->vertices[i];
                        geometry_msgs::Point pt;
                        pt.x = vec[0];
                        pt.y = vec[1];
                        pt.z = vec[2];
                        marker->points.push_back(pt);

                        if(mesh->HasColors())
                        {
                            const chisel::Vec3& meshCol = mesh->colors[i];
                            std_msgs::ColorRGBA color;
                            color.r = meshCol[0];
                            color.g = meshCol[1];
                            color.b = meshCol[2];
                            color.a = 1.0;
                            marker->colors.push_back(color);
                        }
                        else
                        {
                          if(mesh->HasNormals())
                          {
                              const chisel::Vec3 normal = mesh->normals[i];
                              std_msgs::ColorRGBA color;
                              chisel::Vec3 lambert = LAMBERT(normal, lightDir) + LAMBERT(normal, lightDir1) + ambient;
                              color.r = fmin(lambert[0], 1.0);
                              color.g = fmin(lambert[1], 1.0);
                              color.b = fmin(lambert[2], 1.0);
                              color.a = 1.0;
                              marker->colors.push_back(color);
                          }
                          else
                          {
                            std_msgs::ColorRGBA color;
                            color.r = vec[0] * 0.25 + 0.5;
                            color.g = vec[1] * 0.25 + 0.5;
                            color.b = vec[2] * 0.25 + 0.5;
                            color.a = 1.0;
                            marker->colors.push_back(color);
                          }
                        }
                        //marker->indicies.push_back(idx);
                        //idx++;
                    }
                }
            }

            void FillNormalMarkerTopicWithMeshes(visualization_msgs::Marker* marker)
            {
                assert(marker != nullptr);
                marker->header.stamp = ros::Time::now();
                marker->header.frame_id = baseTransform;
                marker->action = visualization_msgs::Marker::ADD;
                marker->scale.x = 0.04;
                marker->pose.orientation.x = 0;
                marker->pose.orientation.y = 0;
                marker->pose.orientation.z = 0;
                marker->pose.orientation.w = 1;

                marker->color.r=1;
                marker->color.g=0;
                marker->color.b=0;
                marker->color.a=1;
                marker->type = visualization_msgs::Marker::LINE_LIST;
                const chisel::MeshMap& meshMap = chiselMap->GetChunkManager().GetAllMeshes();

                if(meshMap.size() == 0)
                {
                    return;
                }

                for (const std::pair<chisel::ChunkID, chisel::MeshPtr>& meshes : meshMap)
                {
                    const chisel::MeshPtr& mesh = meshes.second;
                    for (size_t i = 0; i < mesh->vertices.size(); i+=300)
                    {
                        if(mesh->HasNormals())
                        {
                        const chisel::Vec3& vec = mesh->vertices[i];
                        geometry_msgs::Point pt;
                        pt.x = vec[0];
                        pt.y = vec[1];
                        pt.z = vec[2];
                        marker->points.push_back(pt);

                        const chisel::Vec3 endPoint = 0.3*mesh->normals[i] + vec;
                        geometry_msgs::Point pt2;
                        pt2.x = endPoint[0];
                        pt2.y = endPoint[1];
                        pt2.z = endPoint[2];
                        marker->points.push_back(pt2);
                        }
                    }
                }
            }

            inline void SetBaseTransform(const std::string& frameName) { baseTransform = frameName; }

            inline bool HasNewData() { return hasNewData; }

            inline float GetNearPlaneDist() const { return nearPlaneDist; }
            inline float GetFarPlaneDist() const { return farPlaneDist; }
            inline void SetNearPlaneDist(float dist) { nearPlaneDist = dist; }
            inline void SetFarPlaneDist(float dist) { farPlaneDist = dist; }

            bool Reset(chisel_msgs::ResetService::Request& request, chisel_msgs::ResetService::Response& response)
            {
                chiselMap->Reset();
                return true;
            }

            bool TogglePaused(chisel_msgs::PauseService::Request& request, chisel_msgs::PauseService::Response& response)
            {
                SetPaused(!IsPaused());
                return true;
            }

            bool SaveMesh(chisel_msgs::SaveMeshService::Request& request, chisel_msgs::SaveMeshService::Response& response)
            {
                bool saveSuccess = chiselMap->SaveAllMeshesToPLY(request.file_name);
                return saveSuccess;
            }

            bool GetAllChunks(chisel_msgs::GetAllChunksService::Request& request, chisel_msgs::GetAllChunksService::Response& response)
            {
                const chisel::ChunkMap<VoxelType>& chunkmap = chiselMap->GetChunkManager().GetChunks();
                response.chunks.chunks.resize(chunkmap.size());
                size_t i = 0;
                for (const std::pair<chisel::ChunkID, chisel::ChunkPtr<VoxelType>>& chunkPair : chiselMap->GetChunkManager().GetChunks())
                {
                    chisel_msgs::ChunkMessage& msg = response.chunks.chunks.at(i);
                    FillChunkMessage<VoxelType>(chunkPair.second, &msg);
                    i++;
                }

                return true;
            }

            bool GetLatestChunks(chisel_msgs::GetLatestChunksService::Request& request, chisel_msgs::GetLatestChunksService::Response& response)
            {

                chisel::ChunkManager<VoxelType>& chunkManager = chiselMap->GetMutableChunkManager();
                const chisel::ChunkSet& latestChunks = chunkManager.getIncrementalChanges()->getChunkSet(chunkManager.getIncrementalChanges()->getChangedChunks());

                int i = 0;

                response.chunks.chunks.resize(latestChunks.size());

                for (auto id : latestChunks)
                {
                    if(chunkManager.HasChunk(id.first))
                    {
                      chisel_msgs::ChunkMessage& msg = response.chunks.chunks.at(i);
                      FillChunkMessage<VoxelType>(chunkManager.GetChunk(id.first), &msg);
                      i++;
                    }
                }

                return true;
            }

            bool GetDeletedChunks(chisel_msgs::GetDeletedChunksService::Request& request, chisel_msgs::GetDeletedChunksService::Response& response)
            {
                chisel::ChunkManager<VoxelType>& chunkManager = chiselMap->GetMutableChunkManager();
                const chisel::ChunkSet& deletedChunks =  chunkManager.getIncrementalChanges()->deletedChunks;

                int i = 0;

                response.id_x.resize(deletedChunks.size());
                response.id_y.resize(deletedChunks.size());
                response.id_z.resize(deletedChunks.size());
                response.header.stamp = ros::Time::now();
                ROS_INFO_STREAM("Size of deleted chunks: " << deletedChunks.size());


                for (auto id : deletedChunks)
                {
                      chisel::ChunkID chunkID = id.first;

                      response.id_x.at(i) = chunkID.x();
                      response.id_y.at(i) = chunkID.y();
                      response.id_z.at(i) = chunkID.z();

                      i++;
                }

                return true;
            }

            inline bool IsPaused() { return isPaused; }
            inline void SetPaused(bool paused) { isPaused = paused; }

            void AdvertiseServices()
            {
                resetServer = nh.advertiseService("Reset", &ChiselServer::Reset, this);
                pauseServer = nh.advertiseService("TogglePaused", &ChiselServer::TogglePaused, this);
                saveMeshServer = nh.advertiseService("SaveMesh", &ChiselServer::SaveMesh, this);
                getAllChunksServer = nh.advertiseService("GetAllChunks", &ChiselServer::GetAllChunks, this);
                getLatestChunksServer = nh.advertiseService("GetLatestChunks", &ChiselServer::GetLatestChunks, this);
                getDeletedChunksServer = nh.advertiseService("get_deleted_chunks", &ChiselServer::GetDeletedChunks, this);
            }

            inline FusionMode GetMode() { return mode; }
            inline void SetMode(const FusionMode& m) { mode = m; }

            void SetDepthImage(const sensor_msgs::ImageConstPtr& img)
            {
                if (!lastDepthImage.get())
                {
                  lastDepthImage.reset(new chisel::DepthImage<DepthData>(img->width, img->height));
                }

                ROSImgToDepthImg(img, lastDepthImage.get());
                depthCamera.lastImageTimestamp = img->header.stamp;
                depthCamera.gotImage = true;
            }

            void SetDepthPose(const Eigen::Affine3f& tf)
            {
                depthCamera.lastPose = tf;
                depthCamera.gotPose = true;
            }

            void SetColorImage(const sensor_msgs::ImageConstPtr& img)
            {
                if (!lastColorImage.get())
                {
                    lastColorImage.reset(ROSImgToColorImg<ColorData>(img));
                }

                ROSImgToColorImg(img, lastColorImage.get());

                colorCamera.lastImageTimestamp = img->header.stamp;
                colorCamera.gotImage = true;
            }

            void SetColorPose(const Eigen::Affine3f& tf)
            {
                colorCamera.lastPose = tf;
                colorCamera.gotPose  = true;
            }

            void SetColorCameraInfo(const sensor_msgs::CameraInfoConstPtr& info)
            {
                colorCamera.cameraModel = RosCameraToChiselCamera(info);
                colorCamera.cameraModel.SetNearPlane(GetNearPlaneDist());
                colorCamera.cameraModel.SetFarPlane(GetFarPlaneDist());
                colorCamera.gotInfo = true;
            }

            void SetDepthCameraInfo(const sensor_msgs::CameraInfoConstPtr& info)
            {
                depthCamera.cameraModel = RosCameraToChiselCamera(info);
                depthCamera.cameraModel.SetNearPlane(GetNearPlaneDist());
                depthCamera.cameraModel.SetFarPlane(GetFarPlaneDist());
                depthCamera.gotInfo = true;
            }

        protected:
            visualization_msgs::Marker CreateFrustumMarker(const chisel::Frustum& frustum)
            {
                visualization_msgs::Marker marker;
                marker.id = 0;
                marker.header.frame_id = baseTransform;
                marker.color.r = 1.;
                marker.color.g = 1.;
                marker.color.b = 1.;
                marker.color.a = 1.;
                marker.pose.position.x = 0;
                marker.pose.position.y = 0;
                marker.pose.position.z = 0;
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 0.01;
                marker.scale.y = 0.01;
                marker.scale.z = 0.01;

                marker.type = visualization_msgs::Marker::LINE_LIST;
                const chisel::Vec3* lines = frustum.GetLines();
                for (int i = 0; i < 24; i++)
                {
                    const chisel::Vec3& linePoint = lines[i];
                    geometry_msgs::Point pt;
                    pt.x = linePoint.x();
                    pt.y = linePoint.y();
                    pt.z = linePoint.z();
                    marker.points.push_back(pt);
                }

                return marker;
            }

            chisel::Vec3 LAMBERT(const chisel::Vec3& n, const chisel::Vec3& light)
            {
                return fmax(n.dot(light), 0.0f) * chisel::Vec3(0.5, 0.5, 0.5);
            }


            ros::NodeHandle nh;
            chisel::ChiselPtr<VoxelType> chiselMap;
            tf::TransformListener transformListener;
            boost::shared_ptr<chisel::DepthImage<DepthData> > lastDepthImage;
            boost::shared_ptr<chisel::ColorImage<ColorData> > lastColorImage;
            chisel::PointCloudPtr lastPointCloud;
            chisel::ProjectionIntegrator projectionIntegrator;
            std::string baseTransform;            std::string meshTopic;
            std::string chunkBoxTopic;
            ros::Publisher meshPublisher;
            ros::Publisher normalPublisher;
            ros::Publisher tsdfPublisher;
            ros::Publisher chunkBoxPublisher;
            ros::Publisher latestChunkPublisher;
            ros::ServiceServer resetServer;
            ros::ServiceServer pauseServer;
            ros::ServiceServer saveMeshServer;
            ros::ServiceServer getAllChunksServer;
            ros::ServiceServer getLatestChunksServer;
            ros::ServiceServer getDeletedChunksServer;

            RosCameraTopic depthCamera;
            RosCameraTopic colorCamera;
            RosPointCloudTopic pointcloudTopic;
            bool useColor;
            bool hasNewData;
            float nearPlaneDist;
            float farPlaneDist;
            bool isPaused;
            FusionMode mode;
            unsigned int maxThreads;
            unsigned int threadTreshold;

    };

    template<class VoxelType = chisel::DistVoxel>
    using ChiselServerPtr = boost::shared_ptr<ChiselServer<VoxelType>>;

    template<class VoxelType = chisel::DistVoxel>
    using ChiselServerConstPtr = boost::shared_ptr<const ChiselServer<VoxelType>>;

} // namespace chisel 

#endif // CHISELSERVER_H_ 
