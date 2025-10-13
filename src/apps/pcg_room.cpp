#include "pcg/ply_io.hpp"
#include "pcg/clustering.hpp"
#include "pcg/params.hpp"
#include "pcg/bbox.hpp"
#include "pcg/csv_writer.hpp"

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

    // Load params from config file if present
    pcg::ParamsConfig cfg; // defaults: radius=0.02, min_cluster=50, max_neighbors=100, filter_factor=0.30
    std::string cfgErr;
    fs::path cfg_path = room_out.parent_path().parent_path() / "data" / "configs" / "default.yaml";
    // Also try relative to executable working dir if above fails
    if (!std::filesystem::exists(cfg_path)) {
        cfg_path = fs::path("../data/configs/default.yaml");
    }
    pcg::load_params_from_file(cfg_path.string(), cfg, &cfgErr);

    // CLI overrides radius and min_cluster if provided
    double radius = (argc >= 4) ? std::stod(argv[3]) : cfg.radius;
    int min_cluster = (argc >= 5) ? std::stoi(argv[4]) : cfg.min_cluster_size;
    const int max_neighbors = cfg.max_neighbors;

    fs::path diag_dir = room_out / "diagnostics";
    std::error_code ec;
    fs::create_directories(diag_dir, ec);

    std::ofstream log(diag_dir / (room_in.filename().string() + std::string{".log"}));
    // Prepare room CSV
    const std::string csv_path = (room_out / (room_in.filename().string() + std::string{".csv"})).string();
    pcg::CsvWriter csv(csv_path);
    if (csv.good()) {
        csv.writeHeader({
            "file","cluster_id",
            "center_x","center_y","center_z",
            "size_x","size_y","size_z",
            "yaw_rad"
        });
    } else {
        if (log) log << "WARNING: cannot open CSV: " << csv_path << "\n";
    }
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

        // If the file is room.ply, treat all points as a single cluster (walls/ceiling), skip clustering and filtering
        const std::string stem = entry.path().stem().string();
        std::vector<pcl::PointIndices> clusters;
        std::vector<pcl::PointIndices> filtered;
        if (stem == "room") {
            pcl::PointIndices one;
            one.indices.reserve(cloud->size());
            for (std::size_t i = 0; i < cloud->size(); ++i) one.indices.push_back(static_cast<int>(i));
            clusters.push_back(std::move(one));
            filtered = clusters; // keep all
            if (log) {
                log << "  Detected room.ply -> treat as single cluster, skip clustering and filtering\n";
            }
        } else {
            pcg::ClusteringParams params;
            params.radius = radius;
            params.min_cluster_size = min_cluster;
            params.max_neighbors = max_neighbors;

            clusters = pcg::cluster_cloud(cloud, params);

            // Decide whether to skip filtering based on max/min ratio
            if (!clusters.empty()) {
                std::size_t min_sz = clusters[0].indices.size();
                std::size_t max_sz = clusters[0].indices.size();
                std::size_t total_pts = 0;
                for (const auto& c : clusters) {
                    const std::size_t s = c.indices.size();
                    total_pts += s;
                    if (s < min_sz) min_sz = s;
                    if (s > max_sz) max_sz = s;
                }
                const double ratio = (min_sz == 0) ? std::numeric_limits<double>::infinity()
                                                   : static_cast<double>(max_sz) / static_cast<double>(min_sz);
                const bool skip_filter = std::isfinite(ratio) && (ratio <= cfg.no_filter_ratio);
                if (skip_filter) {
                    filtered = clusters; // keep all
                } else {
                    // Filtering by factor of average cluster size
                    filtered = pcg::filter_clusters_by_average(clusters, cfg.filter_factor);
                }
                if (log) {
                    log << "  Cluster size stats: min=" << min_sz << ", max=" << max_sz
                        << ", ratio=" << (min_sz == 0 ? 0.0 : static_cast<double>(max_sz)/static_cast<double>(min_sz))
                        << ", no_filter_ratio=" << cfg.no_filter_ratio
                        << (skip_filter ? " -> skip filtering" : " -> apply filtering")
                        << "\n";
                }
            }
        }

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
            const double thr = avg * cfg.filter_factor;
            log << "  Avg cluster size: " << avg << ", filter threshold (factor=" << cfg.filter_factor << "): " << thr << "\n";
            for (size_t ci = 0; ci < clusters.size(); ++ci) {
                log << "  - raw cluster " << ci << ": size=" << clusters[ci].indices.size() << "\n";
            }
            for (size_t ci = 0; ci < filtered.size(); ++ci) {
                log << "  - kept cluster " << ci << ": size=" << filtered[ci].indices.size() << "\n";
            }
        }

        // Export filtered clusters' UOBB to diagnostics and write CSV
        try {
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

                // Compute UOBB from contiguous cluster cloud
                std::vector<int> contig(c->size());
                for (std::size_t k=0;k<c->size();++k) contig[k]=static_cast<int>(k);
                pcg::UOBB box = pcg::compute_uobb(c, contig);

                // Save UOBB mesh
                fs::path uobb_name = clusters_dir / (stem + "_cluster_" + std::to_string(ci) + "_uobb.ply");
                if (pcg::save_uobb_ply(uobb_name.string(), box)) {
                    ++saved;
                    if (log) log << "  ✓ saved UOBB: " << uobb_name.string() << "\n";
                } else {
                    if (log) log << "  ✗ failed to save UOBB: " << uobb_name.string() << "\n";
                }

                // CSV row for this cluster
                if (csv.good()) {
                    csv.writeRow({
                        entry.path().filename().string(), std::to_string(ci),
                        std::to_string(box.center.x()), std::to_string(box.center.y()), std::to_string(box.center.z()),
                        std::to_string(box.size.x()), std::to_string(box.size.y()), std::to_string(box.size.z()),
                        std::to_string(box.yaw)
                    });
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
