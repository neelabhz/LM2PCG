#include "pcg/params.hpp"
#include "pcg/recon/poisson.hpp"
#include "pcg/recon/af.hpp"
#include "pcg/geometry/volume.hpp"

#include <CGAL/IO/polygon_mesh_io.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>

#include <filesystem>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

namespace fs = std::filesystem;

static bool is_ply(const fs::path& p) { return p.has_extension() && p.extension() == ".ply"; }
static bool has_suffix(const std::string& s, const std::string& suf) {
    return s.size() >= suf.size() && std::equal(suf.rbegin(), suf.rend(), s.rbegin());
}

static fs::path find_config_path() {
    const fs::path candidates[] = {
        fs::path("data/configs/default.yaml"),
        fs::path("../data/configs/default.yaml"),
        fs::path("../../data/configs/default.yaml")
    };
    for (const auto& p : candidates) if (fs::exists(p)) return p; return {};
}

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: pcg_reconstruct <input_root_or_room_dir> <output_root_dir>\n";
        std::cerr << "  Input can be: site root (rooms/<site>), floor dir, or a single room dir containing diagnostics/filtered_clusters\n";
        return 1;
    }
    fs::path input_root = argv[1];
    fs::path output_root = argv[2];

    // Reduce PCL console noise: hide non-critical PLYReader warnings (e.g., unhandled 'camera' properties)
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    // Load params
    pcg::ParamsConfig cfg; std::string err;
    pcg::load_params_from_file(find_config_path().string(), cfg, &err);
    pcg::recon::PoissonParams pparams{};
    pparams.spacing_neighbors = cfg.poisson_spacing_neighbors;
    pparams.normal_neighbors  = cfg.poisson_normal_neighbors;
    pparams.min_points = 0;                 // no minimum point count gate
    pparams.min_oriented_fraction = cfg.poisson_min_oriented_fraction;
    pparams.require_closed_output = cfg.poisson_require_closed;
    pcg::recon::AFParams aparams{};
    aparams.min_points = cfg.af_min_points;
    aparams.require_closed_output = cfg.af_require_closed;

    auto reconstruct_one = [&](const fs::path& ply_path, const fs::path& out_dir){
        if (!fs::exists(out_dir)) { std::error_code ec; fs::create_directories(out_dir, ec); }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(ply_path.string(), *cloud) != 0) {
            std::cerr << "  ✗ Read failed: " << ply_path << "\n"; return; }

        pcg::recon::Mesh mesh;
        // Always compute convex hull volume of input for validation
    const double hull_vol = pcg::geom::convex_hull_volume(cloud);
    const double invalid_ratio = cfg.poisson_invalid_ratio_vs_hull; // Poisson volume > ratio * hull => invalid
        bool reconstructed = false;
        try {
            if (pcg::recon::poisson_reconstruct(cloud, mesh, pparams)) {
                // Validate Poisson by volume vs convex hull
                double mesh_vol = pcg::geom::mesh_signed_volume(mesh);
                bool valid = mesh_vol > 0.0 && (hull_vol <= 0.0 || mesh_vol <= invalid_ratio * hull_vol);
                if (valid) {
                    const fs::path out_ply_poisson = out_dir / (ply_path.stem().string() + std::string{"_poisson.ply"});
                    if (!CGAL::IO::write_polygon_mesh(out_ply_poisson.string(), mesh, CGAL::parameters::stream_precision(17)))
                        std::cerr << "  ✗ Write failed: " << out_ply_poisson << "\n";
                    else {
                        std::cout << "  ✓ Poisson: " << out_ply_poisson << "\n";
                        reconstructed = true;
                    }
                } else {
                    // Invalid Poisson by volume check -> force AF fallback
                    std::cout << "  ! Poisson invalid by volume (mesh=" << mesh_vol
                              << ", hull=" << hull_vol << ") -> fallback AF\n";
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "  ! Poisson threw exception, will fallback to AF: " << e.what() << "\n";
        } catch (...) {
            std::cerr << "  ! Poisson threw unknown exception, will fallback to AF\n";
        }
        if (reconstructed) return; // Poisson accepted; do not run AF

        // Fallback to AF (also guarded)
        try {
            if (pcg::recon::af_reconstruct(cloud, mesh, aparams)) {
                const fs::path out_ply_af = out_dir / (ply_path.stem().string() + std::string{"_af.ply"});
                if (!CGAL::IO::write_polygon_mesh(out_ply_af.string(), mesh, CGAL::parameters::stream_precision(17)))
                    std::cerr << "  ✗ Write failed: " << out_ply_af << "\n";
                else {
                    std::cout << "  ✓ AF: " << out_ply_af << "\n";
                    reconstructed = true;
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "  ! AF threw exception: " << e.what() << "\n";
        } catch (...) {
            std::cerr << "  ! AF threw unknown exception\n";
        }
        if (!reconstructed) {
            std::cout << "  ✗ Reconstruction failed: " << ply_path << "\n";
        }
    };

    auto process_room = [&](const fs::path& room_dir){
        const fs::path diag = room_dir / "diagnostics" / "filtered_clusters";
        if (!fs::exists(diag)) return;
        // For each object stem folder, traverse *.ply excluding *_uobb.ply
        for (auto& obj_dir : fs::directory_iterator(diag)) {
            if (!obj_dir.is_directory()) continue;
            for (auto& ply_entry : fs::directory_iterator(obj_dir.path())) {
                if (!ply_entry.is_regular_file()) continue;
                if (!is_ply(ply_entry.path())) continue;
                const std::string name = ply_entry.path().filename().string();
                if (has_suffix(name, "_uobb.ply")) continue;
                // Output next to diagnostics as diagnostics/recon/<stem>/
                const fs::path out_dir = room_dir / "diagnostics" / "recon" / obj_dir.path().filename();
                reconstruct_one(ply_entry.path(), out_dir);
            }
        }
    };

    // input_root can be site -> floors -> rooms, or directly a room
    if (fs::exists(input_root / "diagnostics" / "filtered_clusters")) {
        process_room(input_root);
        return 0;
    }

    auto has_room_signature = [](const fs::path& p){ return fs::exists(p / "diagnostics" / "filtered_clusters"); };

    // Iterate floors → rooms
    for (auto& floor_entry : fs::directory_iterator(input_root)) {
        if (!floor_entry.is_directory()) continue;
        for (auto& room_entry : fs::directory_iterator(floor_entry.path())) {
            if (!room_entry.is_directory()) continue;
            if (!has_room_signature(room_entry.path())) continue;
            process_room(room_entry.path());
        }
    }
    return 0;
}
