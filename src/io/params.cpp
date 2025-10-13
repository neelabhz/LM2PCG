#include "pcg/params.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
#include <cctype>

namespace pcg {

namespace {

inline void ltrim(std::string& s) { s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch){ return !std::isspace(ch); })); }
inline void rtrim(std::string& s) { s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch){ return !std::isspace(ch); }).base(), s.end()); }
inline void trim(std::string& s) { ltrim(s); rtrim(s); }

inline bool starts_with(const std::string& s, const std::string& p) {
    return s.rfind(p, 0) == 0;
}

inline bool parse_kv_line(const std::string& line, std::string& key, std::string& value) {
    std::size_t pos = line.find(':');
    std::size_t pos_eq = line.find('=');
    std::size_t use = std::string::npos;
    if (pos != std::string::npos) use = pos;
    else if (pos_eq != std::string::npos) use = pos_eq;
    if (use == std::string::npos) return false;
    key = line.substr(0, use);
    value = line.substr(use + 1);
    trim(key); trim(value);
    // strip inline comments starting with '#'
    auto cpos = value.find('#');
    if (cpos != std::string::npos) {
        value = value.substr(0, cpos);
        trim(value);
    }
    return !key.empty();
}

} // namespace

bool load_params_from_file(const std::string& filepath, ParamsConfig& out, std::string* error) {
    std::ifstream ifs(filepath);
    if (!ifs) {
        if (error) *error = "Cannot open config: " + filepath;
        return false;
    }
    std::string line;
    while (std::getline(ifs, line)) {
        trim(line);
        if (line.empty()) continue;
        if (starts_with(line, "#")) continue;
        std::string key, value;
        if (!parse_kv_line(line, key, value)) continue;
        try {
            if (key == "radius") out.radius = std::stod(value);
            else if (key == "min_cluster_size") out.min_cluster_size = std::stoi(value);
            else if (key == "max_neighbors") out.max_neighbors = std::stoi(value);
            else if (key == "filter_factor") out.filter_factor = std::stod(value);
            else if (key == "no_filter_ratio") out.no_filter_ratio = std::stod(value);
        } catch (...) {
            // ignore individual parse errors
        }
    }
    return true;
}

} // namespace pcg
