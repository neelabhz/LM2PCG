#pragma once

#include <string>

namespace pcg {

struct ParamsConfig {
    double radius = 0.02;
    int min_cluster_size = 50;
    int max_neighbors = 100;
    double filter_factor = 0.30; // 30% of average
    // If max_cluster_size / min_cluster_size <= no_filter_ratio, skip filtering entirely
    double no_filter_ratio = 2.0;
};

// Loads a very small subset of YAML/INI: lines like `key: value` or `key=value`
// Supported keys: radius (double), min_cluster_size (int), max_neighbors (int), filter_factor (double)
// Returns true on success (file opened). Unknown keys are ignored.
bool load_params_from_file(const std::string& filepath, ParamsConfig& out, std::string* error = nullptr);

} // namespace pcg
