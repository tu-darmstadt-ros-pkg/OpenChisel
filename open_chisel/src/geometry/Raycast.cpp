#include <open_chisel/geometry/Raycast.h>
using namespace chisel;

inline int signum(int x)
{
    return x == 0 ? 0 : x < 0 ? -1 : 1;
}

inline float computeTMax(int ds)
{
    return (ds == 0 ? std::numeric_limits<float>::max() : 0.0f);
}

void Raycast(const chisel::Vec3& start, const chisel::Vec3& end, chisel::Point3List& output)
{
  assert(!!output);
  // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
  // by John Amanatides and Andrew Woo, 1987
  // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
  // <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
  // Extensions to the described algorithm:
  //   • Imposed a distance limit.
  //   • The face passed through to reach the current cube is provided to
  //     the callback.

  // The foundation of this algorithm is a parameterized representation of
  // the provided ray,
  //                    origin + t * direction,
  // except that t is not actually stored; rather, at any given point in the
  // traversal, we keep track of the *greater* t values which we would have
  // if we took a step sufficient to cross a cube boundary along that axis
  // (i.e. change the integer part of the coordinate) in the variables
  // tMaxX, tMaxY, and tMaxZ.

  // Cube containing origin point.
  int x =  static_cast<int>(std::floor(start.x()));
  int y = static_cast<int>(std::floor(start.y()));
  int z = static_cast<int>(std::floor(start.z()));
  const int endX = static_cast<int>(std::floor(end.x()));
  const int endY = static_cast<int>(std::floor(end.y()));
  const int endZ = static_cast<int>(std::floor(end.z()));

  // Break out direction vector.
  const int dx = endX - x;
  const int dy = endY - y;
  const int dz = endZ - z;

  // Direction to increment x,y,z when stepping.
  const int stepX = signum(dx);
  const int stepY = signum(dy);
  const int stepZ = signum(dz);

  // See description above. The initial values depend on the fractional
  // part of the origin.
  float tMaxX = computeTMax(dx);
  float tMaxY = computeTMax(dy);
  float tMaxZ = computeTMax(dz);

  // The change in t when taking a step (always positive).
  const float tDeltaX = static_cast<float>(stepX) / dx;
  const float tDeltaY = static_cast<float>(stepY) / dy;
  const float tDeltaZ = static_cast<float>(stepZ) / dz;

  // Avoids an infinite loop.
  if (stepX == 0 && stepY == 0 && stepZ == 0)
  {
      return;
  }

  Point3 point = Point3(x,y,z);

  while (true)
  {
      output.push_back(point);

      if(point(0) == endX && point(1) == endY && point(2) == endZ)
      {
        break;
      }

      // tMaxX stores the t-value at which we cross a cube boundary along the
      // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
      // chooses the closest cube boundary. Only the first case of the four
      // has been commented in detail.
      if (tMaxX < tMaxY)
      {
          if (tMaxX < tMaxZ)
          {
              // Update which cube we are now in.
              point(0) += stepX;
              // Adjust tMaxX to the next X-oriented boundary crossing.
              tMaxX += tDeltaX;
          }
          else
          {
              point(2) += stepZ;
              tMaxZ += tDeltaZ;
          }
      }
      else
      {
          if (tMaxY < tMaxZ)
          {
              point(1) += stepY;
              tMaxY += tDeltaY;
          }
          else
          {
              point(2) += stepZ;
              tMaxZ += tDeltaZ;
          }
      }
  }
}
