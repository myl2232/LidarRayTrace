# 架构：独立扫描核 + UE5 相交后端

## 结论先说

1. **Livox `scan_mode` + 批量异步射线适合 Mid-360。** 官方 CSV 是 `(Time, Azimuth, Zenith)` 序列，每帧取 `point_rate / frame_rate` 条（Mid-360：200 k/s ÷ 10 Hz = **20 000 射线**）。这和 Gazebo 官方插件的 MultiRay 是同一模型，只是把 ODE 换成 Chaos `LineTrace`。
2. **硬件 RayTrace 适合「要渲染网格精度 / 多雷达 / 强度」的路径**，用 1D RayGen 对着 TLAS 发射同一批方向，不要用深度立方体贴图硬套非重复扫描。
3. **可以做成独立插件，但不能让相交也脱离 UE5。** `core/` 零引擎依赖；`unreal/LidarRayTrace` 只实现 `IRayTraceBackend`。最终仿真必须在 UE5 里跑，因为场景在 UE5。

## 分层

```
                 ┌──────────────────────────┐
                 │  Livox scan_mode CSV     │
                 │  mid360 / avia / HAP …   │
                 └────────────┬─────────────┘
                              │
                 ┌────────────▼─────────────┐
                 │  lidar_core (C++17)      │
                 │  DeviceSpec              │
                 │  ScanPattern             │
                 │  Simulator               │
                 │  IRayTraceBackend        │
                 └────────────┬─────────────┘
            ┌─────────────────┼──────────────────┐
            ▼                 ▼                  ▼
     CpuSceneBackend   ChaosAsyncBackend   HardwareRTBackend
     (CMake / 单测)     (UE5 物理)          (UE5 DXR/Vulkan)
```

`IRayTraceBackend::traceBatch(rays, hits)` 是唯一引擎边界。Core 输出 ROS/Livox 右手 FLU 米制；UE 适配器负责厘米和 Y 轴翻转。

## 为什么批量射线适合 Mid-360

Livox 不是机械 64 线栅格。`scan_mode/mid360.csv` 约 5 万行，4 束一组，天顶角约 38°–94°（对应仰角约 +52°～-4°），方位 0–360° 非重复花瓣。每帧只是在这张表上往前切 20 000 个样本并循环。

因此后端只需要「任意方向的批量射线」，不需要 2D 图像式 LiDAR。

- **Chaos `ParallelFor` + `LineTraceSingleByChannel`**：20k 查询在现代 CPU 上通常数毫秒级，适合 10 Hz。比 20k 个 `AsyncLineTraceByChannel` 委托开销小。
- 从游戏线程看可以丢到 worker，自然带 **1 帧传感器延迟**（和真雷达一致）。
- 限制：打的是**碰撞网格**。视觉-only、植被、Nanite 细节会漏。多雷达或 HAP（45 万点/秒）会顶满 CPU，应切硬件 RT。

## 独立插件怎么理解

| 模块 | 依赖 UE5？ | 作用 |
|---|---|---|
| `core/` | 否 | 设备目录、CSV / 花瓣扫描、噪声、点云、CPU 房间自测 |
| `unreal/LidarRayTrace` | 是，但是独立 *UE 插件*，不改引擎 | Chaos / Hardware 后端 + `ULidarSensorComponent` |
| 硬件 RT usf | 是（RHI） | 1D RayGen，见 `docs/UE5_HARDWARE_RT.md` |

CMake 目标 `lidar_core` / `lidar_sim` / `lidar_core_tests` 在没有 Unreal 的机器上即可编译运行。把 `unreal/LidarRayTrace` 拷进工程 `Plugins/` 即可在 UE5 里挂组件仿真。

不能做的是：在 UE 进程外对 UE 关卡做硬件 RT。没有 TLAS 就没有那条路径。
