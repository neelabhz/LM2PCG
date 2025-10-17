#include "pcg/geometry/volume.hpp"

#include <CGAL/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <iostream>
#include <filesystem>
#include <iomanip>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: pcg_volume <mesh_file_1> [mesh_file_2 ...]\n";
        std::cerr << "  Supported formats (by CGAL): OFF/OBJ/PLY/STL, etc.\n";
        return 1;
    }

    std::cout.setf(std::ios::fixed); std::cout << std::setprecision(10);
    std::cerr.setf(std::ios::fixed); std::cerr << std::setprecision(10);

    for (int i = 1; i < argc; ++i) {
        const fs::path path = argv[i];
        if (!fs::exists(path)) {
            std::cerr << "✗ Not found: " << path << "\n";
            continue;
        }
        pcg::geom::Mesh mesh;
        if (!CGAL::IO::read_polygon_mesh(path.string(), mesh) || mesh.is_empty()) {
            std::cerr << "✗ Read failed or empty: " << path << "\n";
            continue;
        }

        bool is_closed = CGAL::is_closed(mesh);
        const double vol = pcg::geom::mesh_signed_volume(mesh);

        std::cout << path << "\n"
                  << "  closed: " << (is_closed ? "true" : "false") << "\n"
                  << "  volume: " << vol << "\n";
    }
    return 0;
}
