# 專案文件總入口

> 文件統一規則：先讀根目錄的 `PROJECT_HANDOFF.md`，再依本索引進入
> cuVSLAM、導航模擬或部署細節。其他 `README.md` 只負責說明所屬資料夾，
> 不取代主交接文件。

## 唯一閱讀順序

1. [`../AGENTS.md`](../AGENTS.md) — AI／協作者的專案層固定工作規則；要求先讀
   工程規範與主交接，但不取代它們。
2. [`../ENGINEERING_GUIDELINES.md`](../ENGINEERING_GUIDELINES.md) — 強制工程政策；
   變更、測試、安全與 Git 的規則。
3. [`../PROJECT_HANDOFF.md`](../PROJECT_HANDOFF.md) — 唯一主交接；目前架構、
   里程碑、D1 Max 可行性與下一步。
4. [`CUVSLAM_CURRENT_STATUS_20260825.md`](CUVSLAM_CURRENT_STATUS_20260825.md) —
   實機 D435i/cuVSLAM 狀態、品質門檻與測試證據。
5. [`PROJECT_CONTENT_MAP.md`](PROJECT_CONTENT_MAP.md) —
   專案內容分類、cuVSLAM 主線與路徑規劃/導航模擬支線的邊界。
6. [`PROJECT_LAYOUT.md`](PROJECT_LAYOUT.md) — 實機、cuVSLAM 模擬、路徑規劃、研究記錄
   與 shared 工具的實體資料夾邊界。
7. [`sensor_and_frame_contract.md`](sensor_and_frame_contract.md) —
   frame、TF、CameraInfo 與 publisher ownership 契約。
8. [`CUVSLAM_NVIDIA_REFERENCE_DEBUG_DESIGN.md`](CUVSLAM_NVIDIA_REFERENCE_DEBUG_DESIGN.md) —
   NVIDIA 官方輸入契約與 debug 原則。

## 文件角色

| 類型 | 唯一入口 | 內容定位 |
|---|---|---|
| 主交接 | `../PROJECT_HANDOFF.md` | 專案目前狀態、架構、決策與下一步 |
| 實機狀態 | `CUVSLAM_CURRENT_STATUS_20260825.md` | D435i/cuVSLAM 數據與限制 |
| 文件索引 | `docs/README.md` | 本頁，負責導覽所有細節 |
| 內容分類 | `PROJECT_CONTENT_MAP.md` | 主線、導航模擬與證據資料夾邊界 |
| 詳細流程 | 各工作流文件 | 只在執行對應工作時閱讀 |
| 資料夾規則 | 各資料夾的 `README.md` | 只說明該資料夾的用途與保存規則 |

歷史或子系統文件仍然保留，但不再作為下一個對話的主入口：

- `REAL_D435I_MAPPING_HANDOFF_20260817.md`：實機流程參考文件。
- `SIMULATION_HANDOFF.md`：模擬／回放流程參考文件。
- `../research/path_planning/IMM_TRAINING_HANDOFF_2026-08-07.md`：IMM 訓練研究參考文件。

## 實機 D435i

| 文件 | 用途 |
|---|---|
| `REAL_D435I_MAPPING_HANDOFF_20260817.md` | 實機建圖流程與操作邊界 |
| `../real_robot/cuvslam/evidence/reports/real_d435i_r1_static_contract_20260818.md` | R1 靜態 stereo/IMU 契約證據 |
| `../real_robot/cuvslam/evidence/reports/real_d435i_r1p5_imu_qualification_20260818.md` | R1.5 IMU fusion qualification |
| `../real_robot/cuvslam/evidence/reports/real_d435i_r2_stereo_imu_ab_20260818.md` | 初始 stereo / stereo+IMU A/B |
| `../real_robot/cuvslam/evidence/reports/real_d435i_lighting_robustness_20260818.md` | official、emitter-off、曝光診斷 |
| `../real_robot/cuvslam/evidence/logs/real_d435i_quality/20260825_r2_correction_log.md` | 最新 R2 路線分析與修正紀錄 |

## 輸入、frame 與 estimator 設計

- `CUVSLAM_STAGE1_INTRINSICS_AUDIT_DESIGN.md`
- `CUVSLAM_STAGE2_STEREO_GEOMETRY_DESIGN.md`
- `CUVSLAM_STAGE3_IMAGE_TIMING_DESIGN.md`
- `CUVSLAM_STAGE4_OFFICIAL_INPUT_CONTRACT_DESIGN.md`
- `CUVSLAM_TF_CONTRACT_FIX_DESIGN.md`
- `STATIC_TF_TARGET_VALIDATION_DESIGN.md`
- `CUVSLAM_NVIDIA_REFERENCE_DEBUG_DESIGN.md`
- `CUVSLAM_ESTIMATOR_ONLY_LEVEL0_DESIGN.md`

這些文件主要記錄模擬、輸入契約與設計決策；不應直接覆蓋已確認的實機
RealSense factory CameraInfo 或 TF。

## 模擬與回放

- `SIMULATION_HANDOFF.md` — 模擬/回放入口與 ROS domain 隔離。
- `../research/cuvslam/` — cuVSLAM 研究假設、實驗計畫與跨次結果解讀。
- `CUVSLAM_MAPPING_SIMPLE_WORLD_DESIGN.md` — simple world 與 mapping 設計。
- `CUVSLAM_STRAIGHT_MAPPING_DESIGN.md` — 直線 mapping gate。
- `CUVSLAM_NEAR_FIELD_OBSERVABILITY_DESIGN.md` — 近場視覺觀測性。
- `CUVSLAM_SIM_CAMERA_FRAME_FIX_DESIGN.md` — 模擬 camera frame 修正。
- `FRAME_AWARE_MOTION_DIAGNOSTIC_DESIGN.md` — frame-aware motion diagnostic。
- `FOXGLOVE_WORLD_OUTLINE_DESIGN.md` — Foxglove/world visualization。
- `GAZEBO_CASTER_CONTACT_PLANE_FIX_DESIGN.md` — Gazebo 接觸面修正。

模擬報告只能支援 simulation/replay 結論，不可當作 D435i 實機 accuracy。

## Jetson 部署整理

- `../research/cuvslam/jetson_workspace_organization_20260831.md` — Jetson
  工具、設定、實機/模擬資料分區、清理與回滾證據。
- `../research/cuvslam/real_d435i_rplidar_integration_20260831.md` — A2M12
  整合設計、Jetson 盤點、暫存外參與驗收邊界。
- `../research/cuvslam/real_d435i_lidar_odom_ekf_shadow_20260903.md` — LiDAR
  scan-matching odometry、EKF shadow、TF 所有權與下一階段硬體驗證門檻。
- `../research/references/slam_v2_2dlidar/` — 從 `/Applications/slam_v2`
  複製的最小 2D LiDAR 參照包；只供研究比對，不是 runtime source。

## D1 Edu / 導航研究（獨立支線）

- `../research/path_planning/NAV2_MPPI_BOUNDARY_GATE_COMMAND_HEADROOM_20261002.md` —
  MPPI 邊界路徑的超速停用證據、診斷修正與已選速度餘量方案。
- `../research/path_planning/NAV2_MPPI_PPO_IMM_INTERFACE_DESIGN_20261002.md` —
  已選架構下 PPO／IMM 與 Nav2 MPPI 的兩種輸入接口方案與驗收門檻。
- `../research/path_planning/NAV2_IN_LOOP_MAP3_PPO_DESIGN_20261001.md` — D1 Max
  3× 候選圖、已知固定物與行人的 Nav2 隔離模擬／訓練前門檻。
- `../simulation/path_planning/evidence/20261001_nav2_map3_jetson_path_and_proxy_gate.md`
  — Jetson 真 Nav2 單場景 map/path 通過，但本機 proxy 閉迴路超時，未重訓。
- `../research/path_planning/RESEARCH_PROGRESS_2026-08-06.md`
- `../research/path_planning/PPO_BALANCED_SAFE_300K_SUMMARY.md`
- `../research/path_planning/IMM_TRAINING_HANDOFF_2026-08-07.md`

這些文件與目前實機 odom 主線分開；目前可作為 D1 Max／Nav2 模擬整合的
參考，但不能把 D1 Edu 訓練結果直接當成 D1 Max 實機驗證。

完整的資料夾分類與後續整合邊界請看
[`PROJECT_CONTENT_MAP.md`](PROJECT_CONTENT_MAP.md)。

## 文件規則

- [`GITHUB_UPDATE_PREPARATION_20261003.md`](GITHUB_UPDATE_PREPARATION_20261003.md)
  — GitHub 更新準備、排除資料、離線檢查與尚未完成的上傳步驟；不是實機驗收。

- `PROJECT_HANDOFF.md` 是唯一主交接；目前實機數據以
  `CUVSLAM_CURRENT_STATUS_20260825.md` 為準。
- 詳細 handoff 是工作流參考，不可覆蓋主交接的目前架構決策。
- `real_robot/cuvslam/evidence/` 保存實機原始證據與報告，失敗報告不可覆蓋。
- `simulation/cuvslam/evidence/` 保存模擬／回放證據，不可當作實機 accuracy。
- `shared/deployment/` 保存跨工作流部署基礎；各工作流的 runtime deployment
  留在自己的資料夾。
- 各工作流的 `tools/` 保存可重複執行的測試與分析工具。
