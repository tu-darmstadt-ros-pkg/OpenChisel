#ifndef RAYCAST_H_
#define RAYCAST_H_

#include "Geometry.h"

int signum(int x);
float computeTMax(int ds);
void Raycast(const chisel::Vec3& start, const chisel::Vec3& end, chisel::Point3List& output);

#endif // RAYCAST_H_ 
