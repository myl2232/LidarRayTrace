#include "lidar_core/CpuSceneBackend.h"
#include "lidar_core/DeviceCatalog.h"
#include "lidar_core/PointCloudIo.h"
#include "lidar_core/ScanPattern.h"
#include "lidar_core/Simulator.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void printUsage() {
    std::cout
        << "lidar_sim — standalone Livox-style scan (no Unreal required)\n"
        << "Usage: lidar_sim [--device mid360] [--csv path] [--out cloud.ply] [--frames N]\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string device_name = "mid360";
    std::string csv_path;
    std::string out_path = "mid360_room.ply";
    int frames = 1;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printUsage();
            return 0;
        }
        if (arg == "--device" && i + 1 < argc) {
            device_name = argv[++i];
        } else if (arg == "--csv" && i + 1 < argc) {
            csv_path = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            out_path = argv[++i];
        } else if (arg == "--frames" && i + 1 < argc) {
            frames = std::max(1, std::atoi(argv[++i]));
        }
    }

    const auto spec_opt = lidar::findDevice(device_name);
    if (!spec_opt) {
        std::cerr << "Unknown device: " << device_name << '\n';
        return 1;
    }
    lidar::DeviceSpec spec = *spec_opt;

    lidar::ScanPattern pattern = lidar::ScanPattern::generateMid360Rose(50000);
    if (!csv_path.empty()) {
        pattern = lidar::ScanPattern::fromLivoxCsv(csv_path);
    } else if (spec.id != lidar::DeviceId::Mid360) {
        pattern = lidar::ScanPattern::generateRepetitiveSpin(spec, 1800, spec.beams);
    }

    lidar::CpuSceneBackend scene;
    scene.addBoxRoom(15.0f, 5.0f, 0.32f);
    scene.addSphere({{8.0f, 3.0f, 1.4f}, 1.0f, 0.85f});
    scene.addAabb({{-4.0f, -6.0f, 0.0f}, {-2.0f, -4.0f, 1.8f}, 0.55f});

    lidar::SimConfig cfg;
    cfg.device = spec;
    cfg.apply_range_noise = true;
    lidar::Simulator sim(cfg, std::move(pattern), scene);

    lidar::Transform pose;
    pose.translation = {0.0f, 0.0f, 1.7f};

    lidar::PointCloud merged;
    merged.device_name = spec.name;
    merged.frame_id = "livox_frame";
    for (int f = 0; f < frames; ++f) {
        const lidar::PointCloud cloud =
            sim.simulateFrame(pose, static_cast<double>(f) / spec.frame_rate_hz);
        merged.points.insert(merged.points.end(), cloud.points.begin(), cloud.points.end());
        merged.missed_rays += cloud.missed_rays;
        std::cout << "frame " << f << " hits=" << cloud.points.size()
                  << " misses=" << cloud.missed_rays << " backend=" << scene.name()
                  << " pattern=" << sim.pattern().source() << '\n';
    }
    merged.stamp_s = 0.0;
    if (!lidar::writeAsciiPly(merged, out_path)) {
        std::cerr << "Failed to write " << out_path << '\n';
        return 1;
    }
    std::cout << "wrote " << merged.points.size() << " points to " << out_path << '\n';
    return 0;
}
