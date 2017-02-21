#ifndef RAYCAST_H_
#define RAYCAST_H_

#include "Geometry.h"

int signum(int x);
float mod(float value, float modulus);
float intbound(float s, int ds);
void Raycast(const chisel::Vec4& start, const chisel::Vec4& end, const chisel::Point3& min, const chisel::Point3& max, chisel::Point4List* output);
void Raycast(const chisel::Vec4& start, const chisel::Vec4& end, chisel::Point4List& output);

#endif // RAYCAST_H_ 
