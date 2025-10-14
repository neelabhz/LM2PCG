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
#include <algorithm>
#include <cctype>
#include <limits>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

namespace fs = std::filesystem;

static bool is_ply(const fs::path& p) { return p.has_extension() && p.extension() == ".ply"; }

static bool has_any_ply(const fs::path& dir) {
    std::error_code ec;
    if (!fs::exists(dir, ec) || !fs::is_directory(dir, ec)) return false;
    for (auto& e : fs::directory_iterator(dir)) {
        if (e.is_regular_file() && is_ply(e.path())) return true;
    }
    return false;
}

static bool name_contains_shell(const std::string& name) {
    std::string s = name;
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return s.find("shell") != std::string::npos;
}

static fs::path find_config_path_from_cwd_or_repo() {
    // Try common locations relative to current working dir
    fs::path candidates[] = {
        fs::path("data/configs/default.yaml"),
        fs::path("../data/configs/default.yaml"),
        fs::path("../../data/configs/default.yaml")
    };
    for (const auto& p : candidates) {
        if (fs::exists(p)) return p;
    }
    return fs::path();
}

static void process_one_room(const fs::path& room_in,
                             const fs::path& room_out,
                             const pcg::ParamsConfig& cfg,
                             double radius,
                             int min_cluster,
                             int max_neighbors)
{
    std::error_code ec;
    fs::create_directories(room_out, ec);
    fs::path diag_dir = room_out / "diagnostics";
    fs::create_directories(diag_dir, ec);

    std::ofstream log(diag_dir / (room_in.filename().string() + std::string{".log"}));
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

        const std::string stem = entry.path().stem().string();
        std::vector<pcl::PointIndices> clusters;
        std::vector<pcl::PointIndices> filtered;

        // New rule: filenames containing "shell" are treated as whole cloud (no clustering/filtering)
        if (name_contains_shell(stem)) {
            pcl::PointIndices one;
            one.indices.reserve(cloud->size());
            for (std::size_t i = 0; i < cloud->size(); ++i) one.indices.push_back(static_cast<int>(i));
            clusters.push_back(std::move(one));
            filtered = clusters;
            if (log) {
                log << "  Detected *shell* -> treat as single cluster, skip clustering and filtering\n";
            }
        } else {
            pcg::ClusteringParams params;
            params.radius = radius;
            params.min_cluster_size = min_cluster;
            params.max_neighbors = max_neighbors;
            clusters = pcg::cluster_cloud(cloud, params);

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
                    filtered = clusters;
                } else {
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

                std::vector<int> contig(c->size());
                for (std::size_t k=0;k<c->size();++k) contig[k]=static_cast<int>(k);
                pcg::UOBB box = pcg::compute_uobb(c, contig);

                fs::path uobb_name = clusters_dir / (stem + "_cluster_" + std::to_string(ci) + "_uobb.ply");
                if (pcg::save_uobb_ply(uobb_name.string(), box)) {
                    ++saved;
                    if (log) log << "  ✓ saved UOBB: " << uobb_name.string() << "\n";
                } else {
                    if (log) log << "  ✗ failed to save UOBB: " << uobb_name.string() << "\n";
                }

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
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: pcg_room <input_root_dir> <output_root_dir> [radius] [min_cluster_size]\n";
        return 1;
    }

    fs::path input_root = argv[1];
    fs::path output_root = argv[2];

    // Load params from config file if present
    pcg::ParamsConfig cfg; // defaults
    std::string cfgErr;
    fs::path cfg_path = find_config_path_from_cwd_or_repo();
    if (cfg_path.empty()) {
        // last resort: try relative to output root's ancestors
        fs::path p = output_root;
        for (int i=0;i<4 && !fs::exists(cfg_path);++i) {
            cfg_path = p / "data/configs/default.yaml";
            p = p.parent_path();
        }
    }
    pcg::load_params_from_file(cfg_path.string(), cfg, &cfgErr);

    // CLI overrides
    double radius = (argc >= 4) ? std::stod(argv[3]) : cfg.radius;
    int min_cluster = (argc >= 5) ? std::stoi(argv[4]) : cfg.min_cluster_size;
    const int max_neighbors = cfg.max_neighbors;

    std::error_code ec;
    fs::create_directories(output_root, ec);

    // Case A: input_root is a leaf room folder (contains .ply files)
    if (has_any_ply(input_root)) {
        process_one_room(input_root, output_root, cfg, radius, min_cluster, max_neighbors);
        return 0;
    }

    // Case B: input_root contains floors -> rooms
    size_t rooms_processed = 0;
    for (auto& floor_entry : fs::directory_iterator(input_root)) {
        if (!floor_entry.is_directory()) continue;
        const fs::path floor_dir = floor_entry.path();
        for (auto& room_entry : fs::directory_iterator(floor_dir)) {
            if (!room_entry.is_directory()) continue;
            const fs::path room_dir = room_entry.path();
            if (!has_any_ply(room_dir)) continue;
            const fs::path room_out = output_root / floor_dir.filename() / room_dir.filename();
            process_one_room(room_dir, room_out, cfg, radius, min_cluster, max_neighbors);
            ++rooms_processed;
        }
    }

    if (rooms_processed == 0) {
        std::cerr << "No rooms with .ply files found under: " << input_root << "\n";
        return 2;
    }
    return 0;
}
