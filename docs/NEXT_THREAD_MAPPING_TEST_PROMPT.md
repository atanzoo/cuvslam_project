# Prompt: Locate the Gazebo Stereo Image-Freeze Boundary

請先閱讀並遵循：

- `/Users/tsengpochien/Desktop/cuvslam_project/ENGINEERING_GUIDELINES.md`
- `/Users/tsengpochien/Desktop/cuvslam_project/PROJECT_HANDOFF.md`
- `/Users/tsengpochien/Desktop/cuvslam_project/docs/SIMULATION_HANDOFF.md`
- `/Users/tsengpochien/Desktop/cuvslam_project/docs/sensor_and_frame_contract.md`
- `/Users/tsengpochien/Desktop/cuvslam_project/reports/nvidia_reference_and_gazebo_1m_debug_20260730.md`

## 已確認狀態

- Jetson AGX Orin，JetPack 5.1.3。
- Host ROS 2 Foxy；Isaac ROS container 使用 Humble。
- 模擬使用 `ROS_DOMAIN_ID=43`。
- Isaac ROS Visual SLAM 2.1.0 / cuVSLAM 11.4。
- `enable_imu_fusion=false`。
- NVIDIA release-2.1 官方 `small_pol_test` 已通過 package-level 控制：
  725 張左影像產生 725 筆 odometry、725 筆 status，全部
  `vo_state=1`，輸出為 `odom -> base_link`。
- `cuvslam_mapping_simple.sdf` 水平底盤 1 m 測試：
  native truth `1.0176 m`，cuVSLAM `0.1914 m`，低估 `81.2%`，
  假旋轉 `5.05 deg`。
- NVIDIA debug dump 中，車體仍在移動時，frames 2750–3250 的
  501 組左右影像各自只有一個唯一 SHA-256；時間戳仍持續增加。
- TF、CameraInfo baseline、release-2.1 physical camera-frame contract、
  `odom -> base_link` ownership 已完成稽核，本階段不得再修改。
- 行動電源已通過短時間 CPU 與 Gazebo + cuVSLAM `MODE_30W` 壓測，
  但尚未做續航與低電量關機驗證。

## 本對話唯一主要假設

影像在以下三個邊界之一開始重複 stale pixels：

1. Gazebo Transport camera payload；
2. Gazebo-to-ROS bridge 的 ROS image；
3. NVIDIA debug dump。

不要同時研究轉向、場景材質、IMU、Nav2 或 odometry scale。

## 第一個最小實驗

1. 盤點 Jetson 開機後 active process、Docker、ROS domain、transient
   services 與殘留 `/clock`。
2. 使用：

```text
/home/tseng/jetson_slam_ws/install/slam_gazebo/share/slam_gazebo/worlds/cuvslam_mapping_simple.sdf
```

3. 使用獨立 Ignition partition，不與 GUI baseline 同時啟動。
4. 讓車輛低速直行約 `0.5 m`，ground truth 只能控制測試終點與離線評估，
   不得輸入 cuVSLAM。
5. 在同一時間窗記錄：
   - Gazebo Transport 左右 camera payload；
   - ROS `/d435i/infra1/image_rect_raw`；
   - ROS `/d435i/infra2/image_rect_raw`；
   - CameraInfo；
   - NVIDIA debug dump；
   - native Gazebo model pose；
   - cuVSLAM odometry/status。
6. 對每一層計算：
   - message timestamp；
   - content SHA-256；
   - unique-frame count；
   - consecutive duplicate run length；
   - first stale timestamp；
   - left/right timestamp delta。
7. 只在找到第一個 stale-pixel owner 後，提出至少兩個最小修正方案。
   C2 變更需先取得使用者核准。
8. 修正後重跑 NVIDIA 1 m 驗收：
   - translation `0.95–1.05 m`；
   - orientation error不超過 `15 deg`；
   - 不允許 scale multiplier 或 ground-truth correction。

## 禁止事項

- 不得修改 `/home/tseng/slam_ws` 或 `/Applications/slam_v2`。
- 不得啟動真機馬達、production Nav2 或 AMCL。
- 不得修改 TF、CameraInfo、baseline 或 cuVSLAM frame ownership。
- 不得以 `vo_state=1`、topic 存在或 process active 宣稱軌跡合格。
- 不得用 `force_planar_mode`、IMU fusion 或後處理比例掩蓋影像凍結。
- 不得讓兩個 Gazebo world 在 ROS domain 43 同時發布相同 topics。

## 收尾

- 發布零速度。
- 停止 recorder、cuVSLAM、bridge、Gazebo 與臨時 monitor。
- 驗證所有測試 services inactive、無殘留 process、`/clock` 無資料。
- 更新 report 與 handoff，列出證據、未完成項目與下一個單一假設。
