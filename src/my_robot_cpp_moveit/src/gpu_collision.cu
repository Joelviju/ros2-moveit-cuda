/*#include <cuda_runtime.h>
#include <vector>

struct Sphere
{
    float x, y, z;
    float radius;
};

__device__
bool checkCollision(const Sphere& a, const Sphere& b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;

    float dist2 = dx*dx + dy*dy + dz*dz;
    float r = a.radius + b.radius;

    return dist2 <= r*r;
}

__global__
void collisionKernel(
    Sphere* robot,
    Sphere* obstacles,
    int num_links,
    int num_obs,
    int* results)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;

    if (i >= num_links) return;

    for (int j = 0; j < num_obs; j++)
    {
        if (checkCollision(robot[i], obstacles[j]))
        {
            results[i] = 1;
            return;
        }
    }

    results[i] = 0;
}

// ======================================================
// HOST FUNCTION (EXPORTED)
// ======================================================
extern "C"
bool gpuCollisionCheck(
    const std::vector<Sphere>& robot,
    const std::vector<Sphere>& obstacles)
{
    int n = robot.size();
    int m = obstacles.size();

    if (n == 0 || m == 0)
        return false;

    Sphere* d_robot;
    Sphere* d_obs;
    int* d_results;

    cudaMalloc(&d_robot, n * sizeof(Sphere));
    cudaMalloc(&d_obs, m * sizeof(Sphere));
    cudaMalloc(&d_results, n * sizeof(int));

    cudaMemcpy(d_robot, robot.data(), n*sizeof(Sphere), cudaMemcpyHostToDevice);
    cudaMemcpy(d_obs, obstacles.data(), m*sizeof(Sphere), cudaMemcpyHostToDevice);

    int blockSize = 256;
    int gridSize = (n + blockSize - 1) / blockSize;

    collisionKernel<<<gridSize, blockSize>>>(
        d_robot, d_obs, n, m, d_results);

    std::vector<int> results(n);
    cudaMemcpy(results.data(), d_results, n*sizeof(int), cudaMemcpyDeviceToHost);

    cudaFree(d_robot);
    cudaFree(d_obs);
    cudaFree(d_results);

    for (int r : results)
        if (r == 1) return true;

    return false;
} */

// gpu_collision.cu
// gpu_collision.cu

#include "gpu_collision.hpp"
#include <cuda_runtime.h>

// ============================================================
// 🔥 KERNEL
// ============================================================

__global__
void collisionKernelBatch(
    Sphere* states,
    Sphere* obstacles,
    bool* results,
    int N_states,
    int N_links,
    int N_obs)
{
    int state_id = blockIdx.x;
    int link_id  = threadIdx.x;

    if (state_id >= N_states || link_id >= N_links) return;

    Sphere link = states[state_id * N_links + link_id];

    for (int o = 0; o < N_obs; o++) {
        Sphere obs = obstacles[o];

        float dx = link.x - obs.x;
        float dy = link.y - obs.y;
        float dz = link.z - obs.z;

        float dist2 = dx*dx + dy*dy + dz*dz;
        float rad   = link.radius + obs.radius;

        if (dist2 < rad * rad) {
            results[state_id] = true;
            return;
        }
    }
}

// ============================================================
// 🚀 LAUNCHER (IMPORTANT)
// ============================================================

void launchCollisionKernel(
    Sphere* d_states,
    Sphere* d_obs,
    bool* d_results,
    int N_states,
    int N_links,
    int N_obs)
{
    dim3 blocks(N_states);
    dim3 threads(N_links);

    collisionKernelBatch<<<blocks, threads>>>(
        d_states,
        d_obs,
        d_results,
        N_states,
        N_links,
        N_obs
    );
}