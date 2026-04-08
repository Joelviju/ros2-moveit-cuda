#pragma once

#include "gpu_collision.hpp"
#include <vector>

class BatchManager
{
public:
    BatchManager(int batch_size, int num_links);

    void addState(const std::vector<Sphere>& state);
    bool isReady() const;
    void processBatch(const std::vector<Sphere>& obstacles);

    const std::vector<char>& getResults() const;
    void clear();

private:
    int batch_size_;
    int num_links_;

    std::vector<Sphere> state_buffer_;
    std::vector<char> results_;
};