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

namespace chisel
{
  ProjectionIntegrator::ProjectionIntegrator():
    minimumWeight(0.0f), maximumWeight(std::numeric_limits<float>::max()), carvingDist(0.1f), enableVoxelCarving(false), rememberAllUpdatedVoxels(true)
  {
    // TODO Auto-generated constructor stub

  }

  ProjectionIntegrator::ProjectionIntegrator(const TruncatorPtr& t, const WeighterPtr& w, const float minWeight, const float maxWeight, float crvDist, bool enableCrv, const Vec3List& centers, bool rememberAllUpdatedVoxels) :
    truncator(t), weighter(w), minimumWeight(minWeight), maximumWeight(maxWeight), carvingDist(crvDist), enableVoxelCarving(enableCrv), centroids(centers), rememberAllUpdatedVoxels(rememberAllUpdatedVoxels)
  {

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




  ProjectionIntegrator::~ProjectionIntegrator()
  {
    // TODO Auto-generated destructor stub
  }

} // namespace chisel 
