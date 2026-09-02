# LidarRayTrace

在 Unreal Engine 5 里仿真 Livox 系列激光雷达（默认 **Mid-360**），扫描模型与官方 [livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation) 的 `scan_mode` CSV 一致。

**批量异步射线适合 Mid-360。** 硬件 RayTrace 是可选加速/精度路径。扫描核可以完全脱离 UE5 编译；真正打到关卡几何时再进 UE5 插件。

## 三个问题的直接回答

### 1. 批量异步射线适不适合？

适合，作为默认路径。

Livox `scan_mode`（`mid360.csv`、`avia.csv`、`HAP.csv` …）不是机械雷达那种规则栅格，而是 `(Time, Azimuth, Zenith)` 的非重复方向表。Mid-360：200 000 点/秒，典型 10 Hz → **每帧 20 000 条任意方向射线**。

这和官方 Gazebo 插件的 MultiRay 是同一件事。在 UE5 里对应：

- 推荐：`ParallelFor` + `UWorld::LineTraceSingleByChannel`（Chaos 批量查询）
- 不推荐：为每条射线挂一个 `AsyncLineTraceByChannel` 委托（20k 回调开销更大）
- 游戏线程可以丢到 worker，自然带 1 帧延迟，和真雷达一致

限制：打的是**碰撞体**，不是 Nanite / 纯渲染网格。HAP 或同场景多颗雷达时 CPU 会先顶满。

### 2. 硬件 RayTrace 怎么做？

对着同一批 Livox 方向做 **1D GPU dispatch**，不要用深度立方体去套花瓣扫描。

1. 打开工程 Hardware Ray Tracing，`r.RayTracing=1`
2. `FSceneViewExtensionBase` 在 TLAS 构建之后取 `GetLayerView(ERayTracingSceneLayer::Base)`
3. 上传 `StructuredBuffer`（origin / direction / tmin / tmax）
4. `LidarHardwareRT.usf` 里 `TraceRay` / `TraceRayInline`，dispatch 尺寸 = 射线数
5. 回读 hit t、法线，交给 `lidar_core::Simulator` 拼点云

逐步说明：[`docs/UE5_HARDWARE_RT.md`](docs/UE5_HARDWARE_RT.md)。插件里的 `FHardwareRayTraceBackend` 已留好端口；RDG 接通前 `CanDispatch()` 为 false，组件自动回退 Chaos，避免空点云。

### 3. 能不能做成不依赖 UE5 的独立插件？最终又要在 UE5 里仿真？

能拆，不能把相交也搬出引擎。

| 部分 | 依赖 | 产物 |
|---|---|---|
| `core/` 扫描核 | 仅 C++17 | 设备参数、CSV / 花瓣、噪声、`IRayTraceBackend`、CPU 房间 |
| `unreal/LidarRayTrace` | UE5 插件，**不改引擎** | Chaos / Hardware 后端 + `ULidarSensorComponent` |
| 硬件 RT | UE RHI + 当前关卡 TLAS | 不能在 UE 进程外打 UE 场景 |

本仓库用 CMake 在无 Unreal 环境下编译并单测扫描核；把插件拷进 UE 工程即可在关卡里仿真。

架构图：[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)。

## 目录

```
core/                         独立 C++ 库
scan_mode/mid360.csv          官方 Livox 扫描表
unreal/LidarRayTrace/         UE5 插件
docs/                         架构与硬件 RT 步骤
tests/  tools/                无引擎测试与 CLI
```

## 无 Unreal 构建

```bash
cmake -S . -B build
cmake --build build
./build/lidar_core_tests scan_mode/mid360.csv
./build/lidar_sim --device mid360 --csv scan_mode/mid360.csv --out mid360_room.ply --frames 1
```

`lidar_sim` 用 CPU 后端打一间盒子房间，用来验证扫描表，不需要 GPU，也不需要 UE5。

## 接入 UE5

1. 复制 `unreal/LidarRayTrace` 到工程 `Plugins/`
2. 生成工程文件并编译
3. 在 Actor 上添加 `LidarSensorComponent`，设备选 Mid-360
4. `ScanModeCsv` 指向 `scan_mode/mid360.csv`（留空则用程序化花瓣）
5. 后端默认 `ChaosAsync`；订阅 `OnCloudReady` 取点云

其它 Livox 型号用同一套 CSV 格式，把官方 `scan_mode` 文件放进 `scan_mode/` 即可。可用 `scripts/fetch_livox_scan_mode.sh` 拉取 Avia / Horizon / HAP 等。

## 坐标系

Core 与 Livox / ROS 一致：右手 FLU，米。UE 适配器转为虚幻左手厘米（Y 取反）。方向变换与官方插件相同：`elevation = zenith - π/2`，再 `Rz(azimuth) * Ry(elevation) * +X`。
