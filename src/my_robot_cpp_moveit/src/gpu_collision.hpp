#pragma once
#include <vector>

struct Sphere {
    float x, y, z;
    float radius;
};

// GPU lifecycle
void initGPU();
void freeGPU();

// kernel launcher
void launchCollisionKernel(
    Sphere* d_states,
    Sphere* d_obs,
    bool* d_results,
    int N_states,
    int N_links,
    int N_obs);

// batch API
bool gpuCheckBatch(
    const std::vector<Sphere>& states,
    const std::vector<Sphere>& obstacles,
   std::vector<char>& results,
    int N_states,
    int N_links);