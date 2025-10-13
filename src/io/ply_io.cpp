#include "pcg/ply_io.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cstdint>
#include <cstring>

namespace pcg {

namespace {
struct PlyHeader {
    bool binary_little_endian = false;
    uint32_t vertex_count = 0;
};

bool parse_header(std::istream& is, PlyHeader& hdr, std::string& err) {
    std::string line;
    if (!std::getline(is, line)) { err = "Empty file"; return false; }
    if (line != "ply") { err = "Not a PLY file"; return false; }

    bool got_format = false;
    bool in_vertex = false;

    while (std::getline(is, line)) {
        if (line == "end_header") break;
        std::istringstream ls(line);
        std::string tok;
        if (!(ls >> tok)) continue;
        if (tok == "format") {
            std::string fmt; double ver;
            ls >> fmt >> ver;
            if (fmt == "binary_little_endian" && (ver == 1.0 || ver == 1)) {
                hdr.binary_little_endian = true;
                got_format = true;
            } else {
                err = "Unsupported PLY format (need binary_little_endian 1.0)";
                return false;
            }
        } else if (tok == "element") {
            std::string name; uint32_t count;
            ls >> name >> count;
            if (name == "vertex") { hdr.vertex_count = count; in_vertex = true; }
            else { in_vertex = false; }
        } else if (tok == "property") {
            // ignore properties; we'll read based on the known schema order
            (void)in_vertex;
        } else if (tok == "comment" || tok == "obj_info") {
            // ignore
        }
    }

    if (!is || !hdr.binary_little_endian) {
        err = "Failed to parse PLY header or wrong endian";
        return false;
    }
    return true;
}

// Helper to read little-endian types regardless of host
template <typename T>
bool read_le(std::istream& is, T& out) {
    char buf[sizeof(T)];
    if (!is.read(buf, sizeof(T))) return false;
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    std::memcpy(&out, buf, sizeof(T));
#else
    // swap bytes
    T tmp{};
    for (size_t i = 0; i < sizeof(T); ++i) {
        reinterpret_cast<unsigned char*>(&tmp)[i] = reinterpret_cast<unsigned char*>(buf)[sizeof(T)-1-i];
    }
    out = tmp;
#endif
    return true;
}

} // namespace

bool load_ply_xyz(const std::string& filepath, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                  std::string* error) {
    std::ifstream ifs(filepath, std::ios::binary);
    if (!ifs) { if (error) *error = "Cannot open file: " + filepath; return false; }

    PlyHeader hdr;
    std::string err;
    if (!parse_header(ifs, hdr, err)) {
        if (error) *error = err; return false;
    }

    // The provided schema lists properties in order: x,y,z, uchar r,g,b, float scalar_Classification, float scalar_User_Data
    // We'll read position + skip the rest per vertex.
    const size_t bytes_per_vertex = sizeof(float)*3 + sizeof(uint8_t)*3 + sizeof(float)*2; // 12 + 3 + 8 = 23? but alignment: PLY stores tightly -> 3*4 + 3*1 + 2*4 = 23 bytes
    // However many tools align to 4 bytes; but per PLY spec, properties are packed without padding. We'll read field-by-field.

    cloud_out.reset(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_out->points.resize(hdr.vertex_count);

    for (uint32_t i = 0; i < hdr.vertex_count; ++i) {
        float x=0, y=0, z=0;
        if (!read_le(ifs, x) || !read_le(ifs, y) || !read_le(ifs, z)) { if (error) *error = "Unexpected EOF reading xyz"; return false; }

        // skip colors (3 bytes)
        uint8_t c;
        for (int k = 0; k < 3; ++k) { if (!ifs.read(reinterpret_cast<char*>(&c), 1)) { if (error) *error = "Unexpected EOF reading color"; return false; } }

        // skip scalar_Classification and scalar_User_Data (2 floats)
        float s;
        if (!read_le(ifs, s) || !read_le(ifs, s)) { if (error) *error = "Unexpected EOF reading scalars"; return false; }

        cloud_out->points[i].x = x;
        cloud_out->points[i].y = y;
        cloud_out->points[i].z = z;
    }

    cloud_out->width = hdr.vertex_count;
    cloud_out->height = 1;
    cloud_out->is_dense = true;
    return true;
}

} // namespace pcg
