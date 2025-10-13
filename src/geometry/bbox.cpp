#include "pcg/bbox.hpp"

#include <Eigen/Eigenvalues>
#include <fstream>
#include <cmath>

namespace pcg {

static inline void minmax_z(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                            const std::vector<int>& idx,
                            float& zmin, float& zmax) {
    zmin = std::numeric_limits<float>::infinity();
    zmax = -std::numeric_limits<float>::infinity();
    for (int i : idx) {
        const float z = cloud->points[static_cast<std::size_t>(i)].z;
        if (z < zmin) zmin = z;
        if (z > zmax) zmax = z;
    }
}

// Quick PCA-based UOBB on XY plane: use principal axis for yaw, then compute extents.
// For production, replace with convex hull + rotating calipers for minimum-area rectangle.
static inline void pca_xy(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                          const std::vector<int>& idx,
                          Eigen::Vector2f& mean,
                          Eigen::Matrix2f& R2) {
    // Compute mean
    mean.setZero();
    for (int i : idx) {
        const auto& p = cloud->points[static_cast<std::size_t>(i)];
        mean.x() += p.x; mean.y() += p.y;
    }
    mean /= static_cast<float>(idx.size());

    // Covariance
    float c00=0.f, c01=0.f, c11=0.f;
    for (int i : idx) {
        const auto& p = cloud->points[static_cast<std::size_t>(i)];
        const float dx = p.x - mean.x();
        const float dy = p.y - mean.y();
        c00 += dx*dx; c01 += dx*dy; c11 += dy*dy;
    }
    c00 /= idx.size(); c01 /= idx.size(); c11 /= idx.size();
    Eigen::Matrix2f C; C << c00, c01, c01, c11;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es(C);
    // Largest eigenvector as e1
    Eigen::Vector2f e1 = es.eigenvectors().col(1).normalized();
    Eigen::Vector2f e2(-e1.y(), e1.x());
    R2.col(0) = e1; R2.col(1) = e2;
}

UOBB compute_uobb(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                  const std::vector<int>& indices) {
    UOBB box{};
    if (!cloud || indices.empty()) return box;

    // XY PCA for orientation
    Eigen::Vector2f mu; Eigen::Matrix2f R2;
    pca_xy(cloud, indices, mu, R2);
    const float yaw = std::atan2(R2(1,0), R2(0,0));

    // Project to e1,e2 to get extents
    float min_u= std::numeric_limits<float>::infinity();
    float max_u=-std::numeric_limits<float>::infinity();
    float min_v= std::numeric_limits<float>::infinity();
    float max_v=-std::numeric_limits<float>::infinity();
    for (int i : indices) {
        const auto& p = cloud->points[static_cast<std::size_t>(i)];
        const Eigen::Vector2f q(p.x - mu.x(), p.y - mu.y());
        const float u = R2.col(0).dot(q);
        const float v = R2.col(1).dot(q);
        if (u < min_u) min_u = u; if (u > max_u) max_u = u;
        if (v < min_v) min_v = v; if (v > max_v) max_v = v;
    }
    const float lx = (max_u - min_u);
    const float ly = (max_v - min_v);

    // Height from z-range
    float zmin, zmax; minmax_z(cloud, indices, zmin, zmax);
    const float lz = (zmax - zmin);

    // Center in world
    const float cx = mu.x() + R2.col(0).x() * (min_u + max_u)/2.f + R2.col(1).x() * (min_v + max_v)/2.f;
    const float cy = mu.y() + R2.col(0).y() * (min_u + max_u)/2.f + R2.col(1).y() * (min_v + max_v)/2.f;
    const float cz = (zmin + zmax) * 0.5f;

    box.center = Eigen::Vector3f(cx, cy, cz);
    box.size   = Eigen::Vector3f(lx, ly, lz);
    box.yaw    = yaw;
    // base area and volume are no longer computed per user request

    // Rotation matrix with up = Z
    box.R.setIdentity();
    box.R(0,0) =  std::cos(yaw); box.R(1,0) = std::sin(yaw);
    box.R(0,1) = -std::sin(yaw); box.R(1,1) = std::cos(yaw);
    // R.col(2) remains (0,0,1)

    // Corners
    const Eigen::Vector3f e1 = box.R.col(0);
    const Eigen::Vector3f e2 = box.R.col(1);
    const Eigen::Vector3f ez(0.f,0.f,1.f);
    const Eigen::Vector3f h = 0.5f * box.size;
    std::array<Eigen::Vector3f,4> base{
        box.center + e1*h.x() + e2*h.y() - ez*h.z(),
        box.center - e1*h.x() + e2*h.y() - ez*h.z(),
        box.center - e1*h.x() - e2*h.y() - ez*h.z(),
        box.center + e1*h.x() - e2*h.y() - ez*h.z()
    };
    std::array<Eigen::Vector3f,4> top{
        base[0] + ez*box.size.z(),
        base[1] + ez*box.size.z(),
        base[2] + ez*box.size.z(),
        base[3] + ez*box.size.z()
    };
    box.corners = { base[0], base[1], base[2], base[3], top[0], top[1], top[2], top[3] };

    return box;
}

// Minimal PLY writer for a box mesh (8 vertices, 12 triangles)
static bool write_box_ply_binary(const std::string& path, const std::array<Eigen::Vector3f,8>& V) {
    struct Tri { uint8_t n; uint32_t a,b,c; };
    const uint32_t nverts = 8;
    const Tri F[12] = {
        {3,0,1,2},{3,0,2,3}, // bottom
        {3,4,5,6},{3,4,6,7}, // top
        {3,0,1,5},{3,0,5,4},
        {3,1,2,6},{3,1,6,5},
        {3,2,3,7},{3,2,7,6},
        {3,3,0,4},{3,3,4,7}
    };
    const uint32_t nfaces = 12;

    std::ofstream os(path, std::ios::binary);
    if (!os) return false;
    os << "ply\n";
    os << "format binary_little_endian 1.0\n";
    os << "element vertex " << nverts << "\n";
    os << "property float x\nproperty float y\nproperty float z\n";
    os << "element face " << nfaces << "\n";
    os << "property list uchar int vertex_indices\n";
    os << "end_header\n";
    // verts
    for (const auto& v : V) {
        float x=v.x(), y=v.y(), z=v.z();
        os.write(reinterpret_cast<const char*>(&x), sizeof(float));
        os.write(reinterpret_cast<const char*>(&y), sizeof(float));
        os.write(reinterpret_cast<const char*>(&z), sizeof(float));
    }
    // faces
    for (const auto& t : F) {
        os.write(reinterpret_cast<const char*>(&t.n), sizeof(uint8_t));
        os.write(reinterpret_cast<const char*>(&t.a), sizeof(uint32_t));
        os.write(reinterpret_cast<const char*>(&t.b), sizeof(uint32_t));
        os.write(reinterpret_cast<const char*>(&t.c), sizeof(uint32_t));
    }
    return true;
}

bool save_uobb_ply(const std::string& filepath, const UOBB& box) {
    return write_box_ply_binary(filepath, box.corners);
}

} // namespace pcg
