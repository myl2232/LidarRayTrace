# UE5 硬件 RayTrace 操作步骤

硬件 RT 走的是渲染线程上的 TLAS，不是 Chaos 物理场景。Livox / Mid-360 的射线来自 `scan_mode` CSV，是一维不规则方向，**不要**按屏幕像素做 2D dispatch。

## 1. 工程开关

- 项目设置启用 Hardware Ray Tracing / Support Compute Skin Cache。
- `r.RayTracing=1`，RHI 为 D3D12 或 Vulkan（需 `GRHISupportsRayTracing`）。
- 插件模块已依赖 `RenderCore`、`RHI`、`Renderer`。启动时把着色器目录映射为 `/Plugin/LidarRayTrace`。

## 2. 数据流

```
lidar_core::ScanPattern
        │  20k rays / 10 Hz (Mid-360)
        ▼
StructuredBuffer<FLidarGpuRay>   origin / direction / tmin / tmax
        │
        ▼
RayGen 1D dispatch (N = ray count)
  TraceRay(TLAS, FIRST_HIT, ...)
        │
        ▼
RWStructuredBuffer<FLidarGpuHit>  t / hit / normal
        │
        ▼
lidar_core::Simulator 组装传感器坐标系点云
```

对应着色器：`unreal/LidarRayTrace/Shaders/Private/LidarHardwareRT.usf`。

## 3. 在插件里接线（无需改引擎）

1. 写一个 `FSceneViewExtensionBase`，挂 `PostTLASBuild` 或 `PrePostProcessPass_RenderThread`。
2. 取场景加速结构（UE 5.3+ 必须在 pass 内 `GetRHI()`）：

```cpp
FRDGBufferSRVRef LayerView =
    Scene->GetRayTracingScene().GetLayerView(ERayTracingSceneLayer::Base);
PassParameters->TLAS = LayerView->GetRHI();
```

3. 把本帧 `RayRequest` 上传为 `FRDGBuffer`（StructuredBuffer）。
4. `RayTraceDispatch(Pipeline, RayGen, Scene, GlobalResources, RayCount, 1)`。
5. 用 `FRHIGPUBufferReadback` 或 GPU 上直接转成点云，再回填 `HitResult`。

完整 PSO 绑定（RayGen / ClosestHit / Miss）见 Epic 社区教程 *Custom Ray Tracing Shader as a Plugin*。Inline `TraceRayInline` 计算着色器往往比完整 RayGen 管线更适合雷达这种「任意方向、只要首击」的负载。

## 4. 和 Chaos 批量射线的取舍

| | Chaos 批量 LineTrace | 硬件 RT |
|---|---|---|
| 几何 | 碰撞体 | 渲染网格 / Nanite 代理 |
| 依赖 | 任意 Chaos 场景 | RT 硬件 + 工程开关 |
| Mid-360 20k/帧 | CPU ParallelFor 可承受 | GPU 更轻松，可上多雷达 / HAP |
| 强度 | 近似（自定义材质回调） | 可靠近材质 / 反射率 |
| 无头仿真 | 适合 | 通常需要渲染线程 |

默认后端保持 Chaos。`FHardwareRayTraceBackend::CanDispatch()` 在 RDG 接通前返回 false，组件会自动回退，避免空点云。

## 5. 仍然不能做成「完全脱离 UE 的硬件 RT」

TLAS 属于 UE 渲染器。独立的是扫描表、设备参数和点云组装（`core/`）。硬件相交必须在 UE5 进程内、对着当前关卡的加速结构发射。
