#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

namespace pcg {

// Minimal PLY loader for the provided header schema. Reads only vertex positions (x,y,z).
// Supports binary_little_endian 1.0 with the specific fields listed, ignores color and scalars.
bool load_ply_xyz(const std::string& filepath, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                  std::string* error = nullptr);

} // namespace pcg
