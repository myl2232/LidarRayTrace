#pragma once

#include "lidar_core/Types.h"

#include <string>

namespace lidar {

bool writeAsciiPly(const PointCloud& cloud, const std::string& path);
bool writeXyzCsv(const PointCloud& cloud, const std::string& path);

}  // namespace lidar
