#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace taskplan {

struct GeoTarget {
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude_m = 0.0;
};

struct Task {
    std::string task_id;
    std::string incident_id;
    std::string kind;
    std::string priority;
    long revision = 1;
    std::string required_capability;
    GeoTarget target_wgs84;
    std::string predecessor_task_id;
    double payload_kg = 0.0;
    long deadline_s = 0;
    std::string frame_id;
    std::string map_version;
};

struct Plan {
    std::vector<Task> tasks;
};

Plan parse_json(const std::string& json);
Plan load_file(const std::filesystem::path& path);

} // namespace taskplan
