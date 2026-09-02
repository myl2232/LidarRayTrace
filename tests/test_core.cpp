#include "lidar_core/CpuSceneBackend.h"
#include "lidar_core/DeviceCatalog.h"
#include "lidar_core/ScanPattern.h"
#include "lidar_core/Simulator.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

int g_failed = 0;

void expect(bool cond, const char* msg) {
    if (!cond) {
        std::cerr << "FAIL: " << msg << '\n';
        ++g_failed;
    } else {
        std::cout << "ok   " << msg << '\n';
    }
}

void test_mid360_spec() {
    const lidar::DeviceSpec mid = lidar::makeMid360();
    expect(mid.horizontal_fov_deg == 360.0f, "Mid360 horizontal FOV is 360");
    expect(mid.vertical_min_deg == -7.0f && mid.vertical_max_deg == 52.0f, "Mid360 vertical FOV");
    expect(mid.point_rate_hz == 200000.0f, "Mid360 point rate 200k");
    expect(std::fabs(mid.point_rate_hz / mid.frame_rate_hz - 20000.0f) < 1e-3f,
           "Mid360 20k points per 10 Hz frame");
}

void test_direction_convention() {
    // Horizon, azimuth 0 -> +X
    const lidar::Vec3 fwd = lidar::directionFromLivoxAzimuthZenith(0.0f, lidar::kPi * 0.5f);
    expect(std::fabs(fwd.x - 1.0f) < 1e-5f && std::fabs(fwd.y) < 1e-5f && std::fabs(fwd.z) < 1e-5f,
           "zenith 90 azimuth 0 is +X");

    // Official first Mid360 sample-like: zenith 37.838 deg -> looking up
    const lidar::Vec3 upish =
        lidar::directionFromLivoxAzimuthZenith(268.99f * lidar::kDeg2Rad, 37.838f * lidar::kDeg2Rad);
    expect(upish.z > 0.7f, "small zenith looks upward");
}

void test_csv_and_rose(const std::string& csv_path) {
    const lidar::ScanPattern rose = lidar::ScanPattern::generateMid360Rose(4000);
    expect(rose.size() == 4000, "procedural rose has requested samples");
    expect(!rose.samples().empty(), "rose samples exist");

    if (!csv_path.empty()) {
        const lidar::ScanPattern csv = lidar::ScanPattern::fromLivoxCsv(csv_path);
        expect(csv.size() > 1000, "official mid360.csv has a dense table");
        float min_el = 1e9f;
        float max_el = -1e9f;
        for (const lidar::ScanSample& s : csv.samples()) {
            const float el = (lidar::kPi * 0.5f - s.zenith_rad) * lidar::kRad2Deg;
            min_el = std::min(min_el, el);
            max_el = std::max(max_el, el);
        }
        expect(min_el > -12.0f && max_el < 58.0f, "CSV elevations stay near Mid360 FOV");
    }
}

void test_room_simulation() {
    lidar::CpuSceneBackend scene;
    scene.addBoxRoom(12.0f, 4.0f, 0.4f);
    scene.addSphere({{6.0f, 0.0f, 1.2f}, 0.8f, 0.9f});

    lidar::SimConfig cfg;
    cfg.device = lidar::makeMid360();
    cfg.apply_range_noise = false;
    lidar::Simulator sim(cfg, lidar::ScanPattern::generateMid360Rose(50000), scene);

    lidar::Transform pose;
    pose.translation = {0.0f, 0.0f, 1.6f};
    const lidar::PointCloud cloud = sim.simulateFrame(pose, 0.0);

    expect(sim.pointsPerFrame() == 20000, "frame budget is 20000 rays");
    expect(cloud.points.size() > 15000, "most Mid360 rays hit the room");
    expect(cloud.missed_rays < 5000, "misses stay bounded in a closed room");

    float min_z = 1e9f;
    float max_r = 0.0f;
    for (const lidar::LidarPoint& p : cloud.points) {
        min_z = std::min(min_z, p.z);
        max_r = std::max(max_r, std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z));
    }
    expect(min_z < -1.0f, "points exist below the sensor (floor)");
    expect(max_r < cfg.device.max_range_m + 0.1f, "ranges stay within device max");
}

void test_backend_port() {
    lidar::CpuSceneBackend scene;
    scene.addPlane({{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, 1.0f});
    expect(std::string(scene.name()) == "CpuScene", "CPU backend name");
    expect(scene.kind() == lidar::TraceBackendKind::CpuScene, "CPU backend kind");
    expect(!scene.isAsync(), "CPU backend is synchronous");
}

}  // namespace

int main(int argc, char** argv) {
    std::string csv;
    if (argc > 1) {
        csv = argv[1];
    }
    test_mid360_spec();
    test_direction_convention();
    test_csv_and_rose(csv);
    test_room_simulation();
    test_backend_port();
    if (g_failed != 0) {
        std::cerr << g_failed << " test(s) failed\n";
        return 1;
    }
    std::cout << "all tests passed\n";
    return 0;
}
