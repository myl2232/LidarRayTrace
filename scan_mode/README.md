# Livox scan_mode

官方表来自 [Livox-SDK/livox_laser_simulation](https://github.com/Livox-SDK/livox_laser_simulation/tree/main/scan_mode)。

格式：

```
Time/s,Azimuth/deg,Zenith/deg
1,268.99,37.838
```

- `Time` 在官方 Mid-360 表里是序号，不是墙钟。
- `Azimuth`：绕 Z，度，0–360。
- `Zenith`：从 +Z 向下的天顶角。Gazebo 插件里写成 `zenith_rad - π/2` 再当 pitch。

本目录自带 `mid360.csv`。其余型号：

```bash
./scripts/fetch_livox_scan_mode.sh
```
