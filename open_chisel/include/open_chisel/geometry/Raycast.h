#ifndef RAYCAST_H_
#define RAYCAST_H_

#include "Geometry.h"

int signum(int x);
float mod(float value, float modulus);
float intbound(float s, int ds);
void Raycast(const chisel::Vec3& start, const chisel::Vec3& end, const chisel::Point3& min, const chisel::Point3& max, chisel::Point3List* output);
void Raycast(const chisel::Vec3& start, const chisel::Vec3& end, chisel::Point3List& output);

void GetRayChunkIntersect(const chisel::Vec3& rayOrigin, const chisel::Vec3& rayDir,  const chisel::Vec3& chunkMin, const chisel::Vec3& chunkMax, float& tmin, float& tmax);

#endif // RAYCAST_H_ 
