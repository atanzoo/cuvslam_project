# cuVSLAM 文件索引

## 先讀這些

1. [`CUVSLAM_CURRENT_STATUS_20260825.md`](CUVSLAM_CURRENT_STATUS_20260825.md) —
   目前唯一的實機建圖主索引、已知問題、品質門檻與下一步。
2. [`PROJECT_CONTENT_MAP.md`](PROJECT_CONTENT_MAP.md) —
   專案內容分類、cuVSLAM 主線與路徑規劃/導航模擬支線的邊界。
3. [`REAL_D435I_MAPPING_HANDOFF_20260817.md`](REAL_D435I_MAPPING_HANDOFF_20260817.md) —
   實機 R0/R1/R2/R3 流程、資料契約與限制。
4. [`sensor_and_frame_contract.md`](sensor_and_frame_contract.md) —
   frame、TF、CameraInfo 與 publisher ownership 契約。
5. [`CUVSLAM_NVIDIA_REFERENCE_DEBUG_DESIGN.md`](CUVSLAM_NVIDIA_REFERENCE_DEBUG_DESIGN.md) —
   NVIDIA 官方輸入契約與 debug 原則。

## 實機 D435i

| 文件 | 用途 |
|---|---|
| `REAL_D435I_MAPPING_HANDOFF_20260817.md` | 實機建圖流程與操作邊界 |
| `../reports/real_d435i_r1_static_contract_20260818.md` | R1 靜態 stereo/IMU 契約證據 |
| `../reports/real_d435i_r1p5_imu_qualification_20260818.md` | R1.5 IMU fusion qualification |
| `../reports/real_d435i_r2_stereo_imu_ab_20260818.md` | 初始 stereo / stereo+IMU A/B |
| `../reports/real_d435i_lighting_robustness_20260818.md` | official、emitter-off、曝光診斷 |
| `../logs/real_d435i_quality/20260825_r2_correction_log.md` | 最新 R2 路線分析與修正紀錄 |

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
- `CUVSLAM_MAPPING_SIMPLE_WORLD_DESIGN.md` — simple world 與 mapping 設計。
- `CUVSLAM_STRAIGHT_MAPPING_DESIGN.md` — 直線 mapping gate。
- `CUVSLAM_NEAR_FIELD_OBSERVABILITY_DESIGN.md` — 近場視覺觀測性。
- `CUVSLAM_SIM_CAMERA_FRAME_FIX_DESIGN.md` — 模擬 camera frame 修正。
- `FRAME_AWARE_MOTION_DIAGNOSTIC_DESIGN.md` — frame-aware motion diagnostic。
- `FOXGLOVE_WORLD_OUTLINE_DESIGN.md` — Foxglove/world visualization。
- `GAZEBO_CASTER_CONTACT_PLANE_FIX_DESIGN.md` — Gazebo 接觸面修正。

模擬報告只能支援 simulation/replay 結論，不可當作 D435i 實機 accuracy。

## D1 Edu / 導航研究（獨立支線）

- `RESEARCH_PROGRESS_2026-08-06.md`
- `PPO_BALANCED_SAFE_300K_SUMMARY.md`
- `IMM_TRAINING_HANDOFF_2026-08-07.md`

這些文件與目前實機建圖主線分開，不要在 R2 建圖尚未驗收前混入 D1 Max、
Nav2 或避障決策。

完整的資料夾分類與後續整合邊界請看
[`PROJECT_CONTENT_MAP.md`](PROJECT_CONTENT_MAP.md)。

## 文件規則

- `PROJECT_HANDOFF.md` 是完整專案/模擬歷史；目前實機狀態以
  `CUVSLAM_CURRENT_STATUS_20260825.md` 為準。
- `reports/` 保存實驗結論，失敗報告不可覆蓋。
- `logs/` 保存原始證據，不當作程式 source。
- `deployment/` 保存 Jetson/Isaac ROS 部署邊界。
- `tools/` 保存可重複執行的測試與分析工具。
