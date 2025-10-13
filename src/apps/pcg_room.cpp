#include "pcg/ply_io.hpp"
#include "pcg/clustering.h"

#include <filesystem>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

namespace fs = std::filesystem;

static bool is_ply(const fs::path& p) {
    return p.has_extension() && p.extension() == ".ply";
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: pcg_room <input_room_dir> <output_room_dir> [radius] [min_cluster_size]\n";
        return 1;
    }

    fs::path room_in = argv[1];
    fs::path room_out = argv[2];
    // Defaults aligned to triangulate.cpp reference: min_cluster_size=50, radius=0.02, max_neighbors=100
    double radius = (argc >= 4) ? std::stod(argv[3]) : 0.02;
    int min_cluster = (argc >= 5) ? std::stoi(argv[4]) : 50;
    const int max_neighbors = 100;

    fs::path diag_dir = room_out / "diagnostics";
    std::error_code ec;
    fs::create_directories(diag_dir, ec);

    std::ofstream log(diag_dir / (room_in.filename().string() + std::string{".log"}));
    if (!log) {
        std::cerr << "Failed to open diagnostics log for writing: " << diag_dir << "\n";
    }

    size_t files_processed = 0;

    for (auto& entry : fs::directory_iterator(room_in)) {
        if (!entry.is_regular_file()) continue;
        if (!is_ply(entry.path())) continue;

        std::string err;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        if (!pcg::load_ply_xyz(entry.path().string(), cloud, &err)) {
            std::cerr << "[ERROR] Load PLY failed: " << entry.path() << " => " << err << "\n";
            if (log) log << "ERROR: " << entry.path().string() << " => " << err << "\n";
            continue;
        }

        pcg::ClusteringParams params;
        params.radius = radius;
        params.min_cluster_size = min_cluster;
        params.max_neighbors = max_neighbors;

        auto clusters = pcg::cluster_cloud(cloud, params);

        // Filtering by 10% of average cluster size (mirrors triangulate.cpp)
    auto filtered = pcg::filter_clusters_by_average(clusters, 0.10);

        std::cout << "File: " << entry.path().filename().string()
                  << ", points: " << cloud->size()
                  << ", clusters(raw/filtered): " << clusters.size() << "/" << filtered.size() << "\n";
        if (log) {
            log << "File: " << entry.path().filename().string()
                << ", points: " << cloud->size()
                << ", clusters(raw/filtered): " << clusters.size() << "/" << filtered.size() << "\n";
            std::size_t total_clustered_points = 0;
            for (const auto& c : clusters) total_clustered_points += c.indices.size();
            const double avg = clusters.empty() ? 0.0 : static_cast<double>(total_clustered_points) / clusters.size();
            const double thr = avg * 0.10;
            log << "  Avg cluster size: " << avg << ", filter threshold (10%): " << thr << "\n";
            for (size_t ci = 0; ci < clusters.size(); ++ci) {
                log << "  - raw cluster " << ci << ": size=" << clusters[ci].indices.size() << "\n";
            }
            for (size_t ci = 0; ci < filtered.size(); ++ci) {
                log << "  - kept cluster " << ci << ": size=" << filtered[ci].indices.size() << "\n";
            }
        }

        // Export filtered clusters to diagnostics/filtered_clusters/<file_stem>/ as binary PLY
        try {
            const std::string stem = entry.path().stem().string();
            fs::path clusters_dir = diag_dir / "filtered_clusters" / stem;
            fs::create_directories(clusters_dir, ec);
            std::size_t saved = 0;
            for (std::size_t ci = 0; ci < filtered.size(); ++ci) {
                const auto& inds = filtered[ci].indices;
                pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>);
                c->points.reserve(inds.size());
                for (int idx : inds) c->points.push_back(cloud->points[static_cast<std::size_t>(idx)]);
                c->width = static_cast<uint32_t>(c->points.size());
                c->height = 1; c->is_dense = true;

                fs::path fn = clusters_dir / (stem + "_cluster_" + std::to_string(ci) + "_" + std::to_string(c->points.size()) + "pts.ply");
                if (pcl::io::savePLYFileBinary(fn.string(), *c) == 0) {
                    ++saved;
                    if (log) log << "  ✓ saved: " << fn.string() << "\n";
                } else {
                    if (log) log << "  ✗ failed to save: " << fn.string() << "\n";
                }
            }
            if (log) log << "Saved " << saved << "/" << filtered.size() << " filtered clusters to: " << (diag_dir / "filtered_clusters" / stem).string() << "\n";
        } catch (const std::exception& e) {
            if (log) log << "  Warning: failed exporting clusters: " << e.what() << "\n";
        }
        ++files_processed;
    }

    if (files_processed == 0) {
        std::cerr << "No .ply files found under: " << room_in << "\n";
        return 2;
    }

    return 0;
}
