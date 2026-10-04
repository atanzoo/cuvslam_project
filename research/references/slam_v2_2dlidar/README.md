# `/Applications/slam_v2` 早期 2D LiDAR 參照包

來源日期：2026-09-03 盤點

這個資料夾是從 `/Applications/slam_v2` 複製出的最小研究參照包，不是
目前專案的 ROS runtime，也不會被任何 `colcon` package 自動安裝或啟動。
原始資料保留在 `/Applications/slam_v2`，本次沒有移除或修改來源檔案。

## 複製內容

| 參照檔 | 原始來源 | 用途 |
|---|---|---|
| `rplidar_a2m12_launch.py` | `mac_slam_ws/src/rplidar_ros/launch/` | A2M12 serial、baud、scan mode 參數參照 |
| `mapping.launch.py` | `mac_slam_ws/src/slam_bringup/launch/` | `slam_toolbox` 啟動與 frame/topic 參數參照 |
| `slam_toolbox.yaml` | `mac_slam_ws/src/slam_bringup/config/` | 2D 建圖參數參照 |
| `robot.urdf.xacro` | `mac_slam_ws/src/robot_description/urdf/` | `base_link -> laser_frame` 與 LiDAR 外參參照 |

## 與目前實機整合的對照

目前實機仍使用：

```text
D435i/cuVSLAM -> odom -> base_link
A2M12         -> /scan, frame_id=rplidar_link
slam_toolbox  -> map -> odom
```

可參照的部分：

- A2M12 使用 `256000` baud、`Sensitivity`、`angle_compensate=true`。
- `slam_toolbox` 使用 `map -> odom -> base_link` 與 `/scan`。
- LiDAR 必須有明確的 `base_link -> sensor frame` TF。
- scan、odom、TF 必須使用可對應的 timestamp。

不可直接搬用的部分：

- 舊版 `rplidar_ros/rplidar_node` 不取代目前 Jetson 已部署的
  `sllidar_ros2/sllidar_node`。
- 舊版 `mapping.launch.py` 不可直接啟動，因為它是舊工作區入口。
- 舊版 LiDAR ICP odometry 不可與 cuVSLAM 同時發布 `odom -> base_link`。
- `laser_frame`、預設 `laser_z=0.10` 只是歷史模型值，不是目前 A2M12 實際
  安裝外參。
- 舊版 Nav2、GUI、Gazebo、map、build/install/log 與 Conda 環境刻意沒有複製。

## 來源 checksum

checksum 只用來確認參照檔未被意外改寫；若要修改目前 runtime，應修改
`real_robot/cuvslam/` 下的正式檔案並同步研究紀錄，不要把本資料夾當成
runtime source of truth。

```text
rplidar_a2m12_launch.py  44af01938c3efe3d7e851fbdba73946a375470e22da6a62ca1d62e87fad01973
mapping.launch.py         c5240c360829123698753ebbdfc5c366e0fd6447ef537cd29df1bcbc88d6ba46
slam_toolbox.yaml         dc60b5464673fca4b6472f55013a59f03de862523bb405eeb3e3fafb011b2bd8
robot.urdf.xacro          f39cd5611df3931f92ef51d2c70c79de151be6a9a32b7464d51b264b1dea1cd4
```
