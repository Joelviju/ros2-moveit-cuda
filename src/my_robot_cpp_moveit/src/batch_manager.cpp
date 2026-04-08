#include "batch_manager.hpp"

// constructor
BatchManager::BatchManager(int batch_size, int num_links)
    : batch_size_(batch_size), num_links_(num_links) {}

// add state
void BatchManager::addState(const std::vector<Sphere>& state)
{
    for (const auto& s : state)
        state_buffer_.push_back(s);
}

// ready check
bool BatchManager::isReady() const
{
    int N_states = state_buffer_.size() / num_links_;
    return N_states >= batch_size_;
}

// process batch
void BatchManager::processBatch(const std::vector<Sphere>& obstacles)
{
    int N_states = state_buffer_.size() / num_links_;

    gpuCheckBatch(state_buffer_, obstacles, results_, N_states, num_links_);
}

// get results
const std::vector<char>& BatchManager::getResults() const
{
    return results_;
}

// clear
void BatchManager::clear()
{
    state_buffer_.clear();
    results_.clear();
}