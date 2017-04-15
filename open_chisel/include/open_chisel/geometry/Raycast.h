#ifndef RAYCAST_H_
#define RAYCAST_H_

#include "Geometry.h"

int signum(int x);
float mod(float value, float modulus);
float intbound(float s, int ds);
float computeTMax(int ds);
void Raycast(const chisel::Vec4& start, const chisel::Vec4& end, chisel::Point4List& output);

#endif // RAYCAST_H_ 
