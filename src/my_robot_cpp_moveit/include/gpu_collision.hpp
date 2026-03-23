#pragma once

#include <vector>

struct Sphere
{
    float x, y, z;
    float radius;
};

extern "C"
bool gpuCollisionCheck(
    const std::vector<Sphere>& robot,
    const std::vector<Sphere>& obstacles);