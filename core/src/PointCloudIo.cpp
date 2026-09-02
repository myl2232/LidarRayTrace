#include "lidar_core/PointCloudIo.h"

#include <fstream>
#include <iomanip>

namespace lidar {

bool writeAsciiPly(const PointCloud& cloud, const std::string& path) {
    std::ofstream out(path);
    if (!out) {
        return false;
    }
    out << "ply\nformat ascii 1.0\n";
    out << "comment device " << cloud.device_name << "\n";
    out << "comment stamp " << std::fixed << std::setprecision(6) << cloud.stamp_s << "\n";
    out << "element vertex " << cloud.points.size() << "\n";
    out << "property float x\nproperty float y\nproperty float z\n";
    out << "property float intensity\nproperty uchar ring\n";
    out << "end_header\n";
    out << std::setprecision(6);
    for (const LidarPoint& p : cloud.points) {
        out << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.intensity << ' '
            << static_cast<int>(p.ring) << '\n';
    }
    return static_cast<bool>(out);
}

bool writeXyzCsv(const PointCloud& cloud, const std::string& path) {
    std::ofstream out(path);
    if (!out) {
        return false;
    }
    out << "x,y,z,intensity,timestamp,ring\n";
    out << std::setprecision(7);
    for (const LidarPoint& p : cloud.points) {
        out << p.x << ',' << p.y << ',' << p.z << ',' << p.intensity << ',' << p.timestamp_s << ','
            << p.ring << '\n';
    }
    return static_cast<bool>(out);
}

}  // namespace lidar
