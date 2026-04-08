#include "gpu_collision.hpp"
#include <cuda_runtime.h>
#include <iostream>

static Sphere* d_states = nullptr;
static Sphere* d_obs = nullptr;
static bool* d_results = nullptr;
static bool initialized = false;

#define CUDA_CHECK(call) \
{ \
    cudaError_t err = call; \
    if (err != cudaSuccess) { \
        std::cerr << "CUDA ERROR: " << cudaGetErrorString(err) << std::endl; \
        exit(EXIT_FAILURE); \
    } \
}

// ============================================================
// INIT
// ============================================================

void initGPU()
{
    if (initialized) return;

    CUDA_CHECK(cudaMalloc(&d_states, 2048 * 16 * sizeof(Sphere)));
    CUDA_CHECK(cudaMalloc(&d_obs, 64 * sizeof(Sphere)));
    CUDA_CHECK(cudaMalloc(&d_results, 2048 * sizeof(bool)));

    initialized = true;
}

// ============================================================
// FREE
// ============================================================

void freeGPU()
{
    if (!initialized) return;

    cudaFree(d_states);
    cudaFree(d_obs);
    cudaFree(d_results);

    initialized = false;
}

// ============================================================
// BATCH CHECK
// ============================================================

bool gpuCheckBatch(
    const std::vector<Sphere>& states,
    const std::vector<Sphere>& obstacles,
    std::vector<char>& results,
    int N_states,
    int N_links)
{
    if (!initialized) {
        std::cerr << "GPU not initialized!\n";
        return false;
    }

    int total = N_states * N_links;

    CUDA_CHECK(cudaMemcpy(
        d_states,
        states.data(),
        total * sizeof(Sphere),
        cudaMemcpyHostToDevice));

    CUDA_CHECK(cudaMemcpy(
        d_obs,
        obstacles.data(),
        obstacles.size() * sizeof(Sphere),
        cudaMemcpyHostToDevice));

    launchCollisionKernel(
        d_states,
        d_obs,
        d_results,
        N_states,
        N_links,
        obstacles.size()
    );

    CUDA_CHECK(cudaDeviceSynchronize());

    results.resize(N_states);

    CUDA_CHECK(cudaMemcpy(
        results.data(),
        d_results,
        N_states * sizeof(bool),
        cudaMemcpyDeviceToHost));

    return true;
}