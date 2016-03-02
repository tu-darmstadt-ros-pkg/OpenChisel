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

namespace chisel
{

  Chisel::Chisel()
  {
    // TODO Auto-generated constructor stub
    maxThreads = 4;
    threadTreshold = 500;
  }

  Chisel::Chisel(const Eigen::Vector3i& chunkSize, float voxelResolution, bool useColor) :
    chunkManager(chunkSize, voxelResolution, useColor)
  {
    maxThreads = 4;
    threadTreshold = 500;
  }

  Chisel::~Chisel()
  {
    // TODO Auto-generated destructor stub
  }

  void Chisel::Reset()
  {
    chunkManager.Reset();
    meshesToUpdate.clear();
  }

  void Chisel::UpdateMeshes()
  {
    chunkManager.RecomputeMeshes(meshesToUpdate);
    meshesToUpdate.clear();
  }

  void Chisel::GarbageCollect(const ChunkIDList& chunks)
  {
    for (const ChunkID& chunkID : chunks)
      {
        chunkManager.RemoveChunk(chunkID);
      }
  }

  bool Chisel::SaveAllMeshesToPLY(const std::string& filename)
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

  void Chisel::IntegratePointCloud(const ProjectionIntegrator& integrator, const PointCloud& cloud, const Transform& extrinsic, float truncation, float maxDist)
  {
    clock_t begin = clock();
    ChunkIDList chunksIntersecting;
    chunkManager.GetChunkIDsIntersecting(cloud, extrinsic, truncation, maxDist, &chunksIntersecting);
    printf("There are %lu chunks intersecting\n", chunksIntersecting.size());
    std::mutex mutex;
    ChunkIDList garbageChunks;
    //for(const ChunkID& chunkID : chunksIntersecting)
      parallel_for(chunksIntersecting.begin(), chunksIntersecting.end(), [&](const ChunkID& chunkID)
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

        bool needsUpdate = integrator.Integrate(cloud, extrinsic, chunk.get());

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
              };

          }
        else if(chunkNew)
          {
            garbageChunks.push_back(chunkID);
          }
        mutex.unlock();
      }, maxThreads);

    GarbageCollect(garbageChunks);
    //  chunkManager.PrintMemoryStatistics();

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    printf("\n \n  ~ %f HZ     \n \n", 1/elapsed_secs);
  }

  void Chisel::IntegrateChunks(const ProjectionIntegrator& integrator,  ChunkManager& sourceChunkManager, ChunkSet& changedChunks)
  {
    clock_t begin = clock();
    printf("CHISEL: Integrating chunks \n");

    ChunkIDList chunksIntersecting;
    std::vector<chisel::ChunkPtr> chunksToIntegrate;

    float mapResolution = GetChunkManager().GetResolution();
    float chunkResolution = sourceChunkManager.GetResolution();
    Eigen::Vector3i mapChunksize = GetChunkManager().GetChunkSize();
    Eigen::Vector3i chunkChunksize = sourceChunkManager.GetChunkSize();

    const bool useColor = sourceChunkManager.GetUseColor();
    ChunkManager tempTargetChunkManager(mapChunksize, mapResolution, useColor);

    if(std::fabs(mapResolution - chunkResolution) < 0.0001 && mapChunksize == chunkChunksize)
    {
      printf("Chunk Size and chunk Resolution fit for source and target chunks! \n");
      for (const std::pair<chisel::ChunkID, bool>& c : changedChunks)
      {
        chunksIntersecting.push_back(c.first);
        chunksToIntegrate.push_back(sourceChunkManager.GetChunk(c.first));
      }
    }
    else
    {
      if (mapResolution != chunkResolution)
      {
        printf("Interpolating chunks! \n");

        for (const std::pair<chisel::ChunkID, ChunkPtr>& c : sourceChunkManager.GetChunks())
        {
          Vec3 origin = c.second->GetOrigin();
          Vec3 maxChunkPos = origin + chunkResolution * chunkChunksize.cast<float>();

          //Get all to the chunk belonging grid vertices as cuboid
          Vec3 minPosition(origin(0) - std::fmod(origin(0), mapResolution), origin(1) - std::fmod(origin(1), mapResolution), origin(2) - std::fmod(origin(2), mapResolution));
          Vec3 maxPosition(maxChunkPos(0) - std::fmod(maxChunkPos(0), mapResolution), maxChunkPos(1) - std::fmod(maxChunkPos(1), mapResolution), maxChunkPos(2) - std::fmod(maxChunkPos(2), mapResolution));

          //interpolateGridTrilinear(minPosition, maxPosition, mapResolution, sourceChunkManager, tempTargetChunkManager, useColor);
          interpolateGridNearestNeighbour(minPosition, maxPosition, mapResolution, sourceChunkManager, tempTargetChunkManager, useColor);
        }
      }
      else
      {
        //Resulution is fitting, but because of different chunksizes, all data goes into other chunks
        for (const std::pair<chisel::ChunkID, ChunkPtr>& c : sourceChunkManager.GetChunks())
        {
          ChunkPtr chunk;

          for (int i = 0; i < c.second->GetTotalNumVoxels(); i++)
          {
            Vec3 worldPosition = c.second->GetOrigin() + chunkResolution * getVoxelCoordinates(i, chunkChunksize).cast<float>() + 0.5* chunkResolution * Vec3::Ones();
            ChunkID chunkID = tempTargetChunkManager.GetIDAt(worldPosition);

            if (tempTargetChunkManager.HasChunk(chunkID))
            {
              chunk = tempTargetChunkManager.GetChunk(chunkID);
            }
            else
            {
              tempTargetChunkManager.CreateChunk(chunkID);
              chunk = tempTargetChunkManager.GetChunk(chunkID);
            }

            chisel::Vec3 relPosition = (worldPosition - chunk->GetOrigin());
            VoxelID voxelID = chunk->GetVoxelID(relPosition);

            //Apply the old data to the fitting voxel
            const DistVoxel& distVoxel = c.second->GetDistVoxel(i);
            chunk->GetDistVoxelMutable(voxelID).SetSDF(distVoxel.GetSDF());
            chunk->GetDistVoxelMutable(voxelID).SetWeight(distVoxel.GetWeight());

            if (useColor)
            {
              const ColorVoxel& colorVoxel = c.second->GetColorVoxel(i);
              chunk->GetColorVoxelMutable(voxelID).SetRed(colorVoxel.GetRed());
              chunk->GetColorVoxelMutable(voxelID).SetGreen(colorVoxel.GetGreen());
              chunk->GetColorVoxelMutable(voxelID).SetBlue(colorVoxel.GetBlue());
              chunk->GetColorVoxelMutable(voxelID).SetWeight(colorVoxel.GetWeight());
            }
          }
        }
      }

      ChunkMap tempChunks = tempTargetChunkManager.GetChunks();

        for (auto it : tempChunks)
        {
          chunksIntersecting.push_back(it.first);
          chunksToIntegrate.push_back(it.second);
        }

        printf("Finished converting");
    }

    std::mutex mutex;
    ChunkIDList garbageChunks;
    for(int i = 0; i< chunksIntersecting.size();i++ )
    //parallel_for(chunksIntersecting.begin(), chunksIntersecting.end(), [&](const ChunkID& chunkID)
    {
      ChunkID chunkID = chunksIntersecting.at(i);
      bool chunkNew = false;

      mutex.lock();
      if (!chunkManager.HasChunk(chunkID))
        {
          chunkNew = true;
          chunkManager.CreateChunk(chunkID);
        }

      ChunkPtr chunk = chunkManager.GetChunk(chunkID);
      mutex.unlock();

      bool needsUpdate = false;

      if(useColor)
        needsUpdate = integrator.IntegrateColorChunk(chunksToIntegrate.at(i).get(), chunk.get());
      else
        needsUpdate = integrator.IntegrateChunk(chunksToIntegrate.at(i).get(), chunk.get());

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
    }
    //, maxThreads);
    printf("CHISEL: Done with chunks \n");
    GarbageCollect(garbageChunks);
    //chunkManager.PrintMemoryStatistics();
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    printf("\n \n  Time needed: %f  ms  \n \n", elapsed_secs*1000);
  }

 const DistVoxel* getValidDistVoxel(const DistVoxel* voxel, const DistVoxel* defaultVoxel)
 {
   if (!voxel)
    return defaultVoxel;
   else
    return voxel;
 }

 const ColorVoxel* getValidColorVoxel(const ColorVoxel* voxel, const ColorVoxel* defaultVoxel)
 {
   if (!voxel)
     return defaultVoxel;
   else
     return voxel;
 }

  bool Chisel::interpolateGridTrilinear(const Vec3& startPos, const Vec3& endPos, const float& resolution, ChunkManager& sourceChunkManager, ChunkManager& targetChunkManager, const bool &useColor)
  {
    const float sourceResolution = sourceChunkManager.GetResolution();
    DistVoxel defaultVoxel;
    defaultVoxel.SetSDF(0.0f);
    defaultVoxel.SetWeight(0.0f);

    ColorVoxel defaultColorVoxel;
    defaultColorVoxel.SetBlue(150);
    defaultColorVoxel.SetGreen(150);
    defaultColorVoxel.SetRed(150);
    defaultColorVoxel.SetWeight(0);

    for (float x = startPos(0) + 0.5*resolution; x < endPos(0); x+=resolution)
    {
      for (float y = startPos(1) + 0.5*resolution; y < endPos(1); y+=resolution)
      {
        for (float z = startPos(2) + 0.5*resolution; z < endPos(2); z+=resolution)
        {
          Vec3 voxelPosition(x, y, z);

          if(!targetChunkManager.HasChunk(targetChunkManager.GetIDAt(voxelPosition)))
            targetChunkManager.CreateChunk(targetChunkManager.GetIDAt(voxelPosition));

          DistVoxel* voxel = targetChunkManager.GetDistanceVoxelMutable(voxelPosition);
          if(voxel == nullptr)
            continue;

          Vec3 firstCorner(x - std::fmod(x, sourceResolution), y - std::fmod(y, sourceResolution), z - std::fmod(z, sourceResolution));
          firstCorner += 0.5* sourceResolution * Vec3::Ones();

          const Vec3 lastCorner = firstCorner + sourceResolution * Vec3::Ones();

          const float& x_0 = firstCorner(0);
          const float& y_0 = firstCorner(1);
          const float& z_0 = firstCorner(2);
          const float& x_1 = lastCorner(0);
          const float& y_1 = lastCorner(1);
          const float& z_1 = lastCorner(2);

          const DistVoxel* v_000 = getValidDistVoxel(sourceChunkManager.GetDistanceVoxel(firstCorner), &defaultVoxel);
          const DistVoxel* v_001 = getValidDistVoxel(sourceChunkManager.GetDistanceVoxel(firstCorner + Vec3(0, 0, sourceResolution)), &defaultVoxel);
          const DistVoxel* v_011 = getValidDistVoxel(sourceChunkManager.GetDistanceVoxel(firstCorner + Vec3(0, sourceResolution, sourceResolution)), &defaultVoxel);
          const DistVoxel* v_111 = getValidDistVoxel(sourceChunkManager.GetDistanceVoxel(lastCorner), &defaultVoxel);
          const DistVoxel* v_110 = getValidDistVoxel(sourceChunkManager.GetDistanceVoxel(firstCorner + Vec3(sourceResolution, sourceResolution, 0)), &defaultVoxel);
          const DistVoxel* v_100 = getValidDistVoxel(sourceChunkManager.GetDistanceVoxel(firstCorner + Vec3(sourceResolution, 0, 0)), &defaultVoxel);
          const DistVoxel* v_010 = getValidDistVoxel(sourceChunkManager.GetDistanceVoxel(firstCorner + Vec3(0, sourceResolution, 0)), &defaultVoxel);
          const DistVoxel* v_101 = getValidDistVoxel(sourceChunkManager.GetDistanceVoxel(firstCorner + Vec3(sourceResolution, 0, sourceResolution)), &defaultVoxel);

          float xd = (x - x_0) / (x_1 - x_0);
          float yd = (y - y_0) / (y_1 - y_0);
          float zd = (z - z_0) / (z_1 - z_0);

          {
            float sdf_00 = v_000->GetSDF() * (1 - xd) + v_100->GetSDF() * xd;
            float sdf_10 = v_010->GetSDF() * (1 - xd) + v_110->GetSDF() * xd;
            float sdf_01 = v_001->GetSDF() * (1 - xd) + v_101->GetSDF() * xd;
            float sdf_11 = v_011->GetSDF() * (1 - xd) + v_111->GetSDF() * xd;
            float sdf_0 = sdf_00 * (1 - yd) + sdf_10 * yd;
            float sdf_1 = sdf_01 * (1 - yd) + sdf_11 * yd;
            voxel->SetSDF(sdf_0 * (1 - zd) + sdf_1 * zd);
          }

          {
            float weight_00 = v_000->GetWeight() * (1 - xd) + v_100->GetWeight() * xd;
            float weight_10 = v_010->GetWeight() * (1 - xd) + v_110->GetWeight() * xd;
            float weight_01 = v_001->GetWeight() * (1 - xd) + v_101->GetWeight() * xd;
            float weight_11 = v_011->GetWeight() * (1 - xd) + v_111->GetWeight() * xd;
            float weight_0 = weight_00 * (1 - yd) + weight_10 * yd;
            float weight_1 = weight_01 * (1 - yd) + weight_11 * yd;
            voxel->SetWeight(weight_0 * (1 - zd) + weight_1 * zd);
          }

          if (useColor)
          {
            ColorVoxel* colorVoxel = targetChunkManager.GetColorVoxelMutable(voxelPosition);
            if(colorVoxel == nullptr)
              continue;

            const ColorVoxel* clrVxl_000 = getValidColorVoxel(sourceChunkManager.GetColorVoxel(firstCorner), &defaultColorVoxel);
            const ColorVoxel* clrVxl_001 = getValidColorVoxel(sourceChunkManager.GetColorVoxel(firstCorner + Vec3(0, 0, sourceResolution)), &defaultColorVoxel);
            const ColorVoxel* clrVxl_011 = getValidColorVoxel(sourceChunkManager.GetColorVoxel(firstCorner + Vec3(0, sourceResolution, sourceResolution)), &defaultColorVoxel);
            const ColorVoxel* clrVxl_111 = getValidColorVoxel(sourceChunkManager.GetColorVoxel(lastCorner), &defaultColorVoxel);
            const ColorVoxel* clrVxl_110 = getValidColorVoxel(sourceChunkManager.GetColorVoxel(firstCorner + Vec3(sourceResolution, sourceResolution, 0)), &defaultColorVoxel);
            const ColorVoxel* clrVxl_100 = getValidColorVoxel(sourceChunkManager.GetColorVoxel(firstCorner + Vec3(sourceResolution, 0, 0)), &defaultColorVoxel);
            const ColorVoxel* clrVxl_010 = getValidColorVoxel(sourceChunkManager.GetColorVoxel(firstCorner + Vec3(0, sourceResolution, 0)), &defaultColorVoxel);
            const ColorVoxel* clrVxl_101 = getValidColorVoxel(sourceChunkManager.GetColorVoxel(firstCorner + Vec3(sourceResolution, 0, sourceResolution)), &defaultColorVoxel);

            float red_00 = clrVxl_000->GetRed() * (1 - xd) + clrVxl_100->GetRed() * xd;
            float red_10 = clrVxl_010->GetRed() * (1 - xd) + clrVxl_110->GetRed() * xd;
            float red_01 = clrVxl_001->GetRed() * (1 - xd) + clrVxl_101->GetRed() * xd;
            float red_11 = clrVxl_011->GetRed() * (1 - xd) + clrVxl_111->GetRed() * xd;
            float red_0 = red_00 * (1 - yd) + red_10 * yd;
            float red_1 = red_01 * (1 - yd) + red_11 * yd;
            colorVoxel->SetRed(red_0 * (1 - zd) + red_1 * zd);

            float green_00 = clrVxl_000->GetGreen() * (1 - xd) + clrVxl_100->GetGreen() * xd;
            float green_10 = clrVxl_010->GetGreen() * (1 - xd) + clrVxl_110->GetGreen() * xd;
            float green_01 = clrVxl_001->GetGreen() * (1 - xd) + clrVxl_101->GetGreen() * xd;
            float green_11 = clrVxl_011->GetGreen() * (1 - xd) + clrVxl_111->GetGreen() * xd;
            float green_0 = green_00 * (1 - yd) + green_10 * yd;
            float green_1 = green_01 * (1 - yd) + green_11 * yd;
            colorVoxel->SetGreen(green_0 * (1 - zd) + green_1 * zd);

            float blue_00 = clrVxl_000->GetBlue() * (1 - xd) + clrVxl_100->GetBlue() * xd;
            float blue_10 = clrVxl_010->GetBlue() * (1 - xd) + clrVxl_110->GetBlue() * xd;
            float blue_01 = clrVxl_001->GetBlue() * (1 - xd) + clrVxl_101->GetBlue() * xd;
            float blue_11 = clrVxl_011->GetBlue() * (1 - xd) + clrVxl_111->GetBlue() * xd;
            float blue_0 = blue_00 * (1 - yd) + blue_10 * yd;
            float blue_1 = blue_01 * (1 - yd) + blue_11 * yd;
            colorVoxel->SetBlue(blue_0 * (1 - zd) + blue_1 * zd);

            float cweight_00 = clrVxl_000->GetWeight() * (1 - xd) + clrVxl_100->GetWeight() * xd;
            float cweight_10 = clrVxl_010->GetWeight() * (1 - xd) + clrVxl_110->GetWeight() * xd;
            float cweight_01 = clrVxl_001->GetWeight() * (1 - xd) + clrVxl_101->GetWeight() * xd;
            float cweight_11 = clrVxl_011->GetWeight() * (1 - xd) + clrVxl_111->GetWeight() * xd;
            float cweight_0 = cweight_00 * (1 - yd) + cweight_10 * yd;
            float cweight_1 = cweight_01 * (1 - yd) + cweight_11 * yd;
            colorVoxel->SetWeight(cweight_0 * (1 - zd) + cweight_1 * zd);
          }
        }
      }
    }
    return true;
  }

  bool Chisel::interpolateGridNearestNeighbour(const Vec3 &startPos, const Vec3 &endPos, const float &resolution, ChunkManager &sourceChunkManager, ChunkManager &targetChunkManager, const bool &useColor)
  {
    const float sourceResolution = sourceChunkManager.GetResolution();

    for (float x = startPos(0) + 0.5*resolution; x < endPos(0); x+=resolution)
    {
      for (float y = startPos(1) + 0.5*resolution; y < endPos(1); y+=resolution)
      {
        for (float z = startPos(2) + 0.5*resolution; z < endPos(2); z+=resolution)
        {
          Vec3 voxelPosition(x, y, z);

          if(!targetChunkManager.HasChunk(targetChunkManager.GetIDAt(voxelPosition)))
            targetChunkManager.CreateChunk(targetChunkManager.GetIDAt(voxelPosition));

          DistVoxel* targetVoxel = targetChunkManager.GetDistanceVoxelMutable(voxelPosition);
          if(targetVoxel == nullptr)
            continue;


          Vec3 sourceVoxelPosition;

          //search nearest neighbour
          for (int i = 0; i<3; i++)
          {
            Vec3 voxelPosition(x,y,z);
            float remainder = std::fmod(voxelPosition(i), sourceResolution);
            if (remainder < 0.5 * sourceResolution)
              sourceVoxelPosition(i) = voxelPosition(i)-remainder;
            else
              sourceVoxelPosition(i) = voxelPosition(i) +remainder;
          }

          sourceVoxelPosition += 0.5* sourceResolution * Vec3::Ones();

          const DistVoxel* sourceVoxel = sourceChunkManager.GetDistanceVoxel(sourceVoxelPosition);

          if(sourceVoxel == nullptr)
              continue;

          targetVoxel->SetSDF(sourceVoxel->GetSDF());
          targetVoxel->SetWeight(sourceVoxel->GetWeight());

          if (useColor)
          {
              const ColorVoxel* sourceColorVoxel = sourceChunkManager.GetColorVoxel(sourceVoxelPosition);
              ColorVoxel* targetColorVoxel = targetChunkManager.GetColorVoxelMutable(voxelPosition);

              targetColorVoxel->SetRed(sourceColorVoxel->GetRed());
              targetColorVoxel->SetGreen(sourceColorVoxel->GetGreen());
              targetColorVoxel->SetBlue(sourceColorVoxel->GetBlue());
              targetColorVoxel->SetWeight(sourceColorVoxel->GetWeight());
          }
        }
      }
    }
    return true;
  }

  Point3 Chisel::getVoxelCoordinates(VoxelID id, Eigen::Vector3i chunkSize)
  {
    int x = id % chunkSize(0);

    int fraction = ((id-x)/chunkSize(0));

    int y = fraction % chunkSize(2);
    int z = (fraction-y)/chunkSize(2);

    return Point3(x,y,z);
  }

  void Chisel::DeleteChunks(ChunkSet &chunks)
  {
    for (std::pair<ChunkID,bool> chunk: chunks)
    {
      chunkManager.RemoveChunk(chunk.first);
      chunkManager.GetAllMutableMeshes().erase(chunk.first);
    }
  }
} // namespace chisel 
