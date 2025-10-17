#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

namespace pcg { namespace geom {

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point3 = Kernel::Point_3;
using Mesh   = CGAL::Surface_mesh<Point3>;

// Compute signed volume of a closed, outward-oriented triangle mesh.
// Returns positive volume on success, 0.0 if mesh is empty or not suitable.
double mesh_signed_volume(const Mesh& mesh);

// Compute convex hull volume of the given point cloud. Returns 0.0 if fewer than 4 points or degenerate.
double convex_hull_volume(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

} } // namespace pcg::geom
