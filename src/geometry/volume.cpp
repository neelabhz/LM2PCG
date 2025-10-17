#include "pcg/geometry/volume.hpp"

#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/convex_hull_3.h>

namespace pcg { namespace geom {

double mesh_signed_volume(const Mesh& mesh) {
    if (mesh.is_empty()) return 0.0;
    namespace PMP = CGAL::Polygon_mesh_processing;
    // volume() returns a signed volume; requires closed mesh for meaningful result
    try {
        double v = PMP::volume(mesh);
        return v >= 0.0 ? v : -v;
    } catch (...) {
        return 0.0;
    }
}

double convex_hull_volume(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
    if (!cloud || cloud->size() < 4) return 0.0;
    std::vector<Point3> pts; pts.reserve(cloud->size());
    for (const auto& p : cloud->points) {
        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
            pts.emplace_back(p.x, p.y, p.z);
    }
    if (pts.size() < 4) return 0.0;
    Mesh hull;
    try {
        CGAL::convex_hull_3(pts.begin(), pts.end(), hull);
    } catch (...) {
        return 0.0;
    }
    return mesh_signed_volume(hull);
}

} } // namespace pcg::geom
