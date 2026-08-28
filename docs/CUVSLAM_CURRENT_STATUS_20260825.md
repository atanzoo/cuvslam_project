# cuVSLAM 目前狀態與實機建圖主索引

最後更新：2026-08-28（Asia/Taipei）  
專案根目錄：`<repository-root>`

> 這是目前 cuVSLAM 實機工作的第一入口。歷史設計、實驗報告與原始 logs
> 仍然保留，但若和本文件衝突，以較新的實機證據與本文件的「目前決策」為準。

## 1. 專案目標

目前目標是完成：

```text
RealSense D435i stereo + IMU fusion
        ↓
Jetson AGX Orin / Isaac ROS Visual SLAM / cuVSLAM
        ↓
穩定 odom、直線與轉彎驗證、可重複的 3D visual mapping
        ↓
後續才評估 localization 與 Nav2 整合
```

cuVSLAM 3D landmark/visual map 不等於 Nav2 2D occupancy map；目前不把
導航、避障或馬達控制混入建圖驗證。

## 2. 已確認的環境

| 項目 | 目前值 |
|---|---|
| 硬體 | Jetson AGX Orin + Intel RealSense D435i |
| D435i serial | `261222077990` |
| 韌體 | `5.15.1.55` |
| JetPack / L4T | JetPack 5.1.3 / R35.5.0 |
| Isaac ROS / cuVSLAM | Isaac ROS Visual SLAM 2.1.0 / cuVSLAM 11.4 |
| RealSense ROS / librealsense | 4.54.1 / 2.54.2 |
| ROS domain | 實機 42；模擬/回放 43 |
| 實機影像 | 左右 IR、mono8、官方 30 Hz profile |
| IMU | accel + gyro，約 200 Hz，`camera_imu_optical_frame` |
| Mac ↔ Jetson | USB SSH，通常為 `192.168.55.1` |

## 3. 目前正式實機設定

正式 R2 建圖基準固定為：

```text
IMU fusion          = true
mapping             = true
profile             = 640x360x30
denoise images      = true
auto exposure       = true
IR projector        = on
lighting profile    = official
```

手動曝光、高對比與 emitter-off 只作診斷比較，不作正式建圖預設。
目前證據不支持用固定 odom scale multiplier、手動 TF 或手動 CameraInfo
修正來掩蓋追蹤問題。

## 4. 實機階段狀態

### R1：資料與硬體契約 — 已完成

- 左右 IR 約 30 Hz，左右 timestamp 在 R1 bag 中同步。
- IMU 約 199.5 Hz，timestamp 單調遞增。
- 工廠 CameraInfo baseline 約 0.050072 m。
- 靜止 gyro、加速度大小、相機/IMU timing 與 TF tree 已記錄。

詳細證據：
`reports/real_d435i_r1_static_contract_20260818.md`

### R1.5：IMU fusion readiness — 已完成

- 直接讀取 librealsense motion calibration records。
- IMU-to-camera TF 與 frame convention 已確認。
- 靜止 fusion 測試穩定，沒有 pose jump。
- X/Y/Z 軸向與正負號測試已由 GUI 完成，IMU 可納入正式 R2。

`IMU Calibration is not available` 仍可在 driver 出現，原因是 factory
motion bias/noise 欄位為零；這不等同於 IMU 沒有資料或 fusion 不能使用。

詳細證據：
`reports/real_d435i_r1p5_imu_qualification_20260818.md`

### R2：實機建圖與 odom — 已可執行，仍在改善追蹤品質

2026-08-25 最新矩形路線結果：

| 指標 | 結果 | 判讀 |
|---|---:|---|
| 完整路徑點 | 3,216 | 路徑保留正常 |
| 累積路徑 | 5.612 m | 有完整移動紀錄 |
| 影像 rate L/R | 29.8 / 29.8 Hz | 正常 |
| image gap P95 | 33.3 / 33.3 ms | 符合 30 Hz 週期 |
| clipped L/R | 1.6% / 1.7% | 不是本次主要失敗原因 |
| odom rate | 26.0 Hz | 高於 GUI 25 Hz pass，但低於約 28 Hz 工程目標 |
| GUI 分數 | 70 / 100 | 主要被 VO/追蹤項目扣分 |
| 最後 yaw | +4.92° | 若是完整閉環，接近 5° pass 門檻 |
| 起點到終點 XY | 0.529 m | 若確實回到起點，超過 0.20 m review 門檻 |

注意：本次舊版 Jetson collector 曾以 GUI telemetry snapshot 估算 VO
有效率，因此原本顯示的 86.2% 是 provisional，不是正式 raw status ratio。
已修正 collector，下一次必須確認 GUI 顯示：
`raw /visual_slam/status samples`。

本次判斷是：影像輸入基本正常，主要待處理的是 VO 追蹤連續性、odom 速率
與路線閉環品質；目前不先把問題歸因於 IMU 或曝光。

2026-08-26 在影像品質良好的環境完成多組純旋轉與「前進一小段再轉彎」測試。
純旋轉可穩定累積約 75–90° yaw；但特殊場景的左轉仍觀察到短暫的側向假平移，
即使 raw VO、影像 rate、clipped 與 odom rate 都通過即時門檻。因此目前把它列為
「場景相關的 stereo tracking limitation」，不以即時分數 PASS 取代路徑幾何檢查。
GUI 現在會在停止 R2 後保存完整追蹤紀錄與轉向片段資訊，供後續和同一路線重測比對。

詳細修正紀錄：
`logs/real_d435i_quality/20260825_r2_correction_log.md`

## 5. 目前已完成的軟體修正

- GUI 完整保留本次 R2 路徑，不再隨時間只顯示短窗口。
- GUI 工作紀錄加入建圖分析摘要與複製按鈕。
- 摘要包含影像 rate/gap/clipped、VO、odom rate、完整路徑、累積路徑、
  起終點距離、yaw、警告與統計來源。
- VO collector 改為直接計算 `/visual_slam/status` 訊息的有效比例，並輸出
  `vo_status_samples`、`vo_valid_samples`。
- GUI 區分 raw status 統計與舊版 fallback；fallback 只標示為暫定值。
- 修正版 collector 已同步至 Jetson；遠端舊版備份為
  `collect_real_d435i_odom_live.py.bak_20260825`。
- 工作記錄簿現在保存帶時間戳的過程事件；建圖摘要另行顯示與複製。
- odom rate 使用 ROS timestamp span；轉向抖動另以 yaw-rate P95 與 sign flips
  記錄，不直接混入總分。
- GUI 可在移動前啟動 rosbag 錄製，並下載 IMU、TF、odom、VO pose、SLAM path 與
  landmarks cloud 到 `logs/real_d435i_r2_downloads/`。錄製預設為輕量模式，不含
  左右原始 IR 影像；需要完整影像回放時才切換到 full 模式。兩種模式都使用
  Jetson ROS 2 rosbag 支援的 zstd file-level compression，完整模式仍可能很大。

主要程式：

- `tools/real_d435i_axis_web_gui.py`
- `tools/collect_real_d435i_odom_live.py`
- `tools/run_real_d435i_r2.sh`
- `tools/open_real_d435i_rviz_view.sh`

## 6. 下一個正確測試順序

1. 啟動 GUI 與 R2，確認 IMU fusion、official profile 與 mapping 設定；若要保存
   建圖資料，先選「輕量建圖（推薦）」並按開始錄製。
2. 靜止 5–10 秒，確認影像 rate、odom rate、raw VO samples 開始累積。
3. 以相同起點、相同方向、相同矩形路線重跑一次，完成後停止錄製並下載。
4. 先判斷 raw VO ratio 與 odom rate，再判斷 60 cm、90° turn 與閉環誤差。
5. 若 raw VO ratio 仍低於 95%，優先檢查轉彎速度、場景紋理、反光/飽和
   區域與相機視角；不要先改 IMU noise、TF、CameraInfo 或 odom scale。
6. 若轉向抖動診斷進入 REVIEW，對照過程事件與轉彎段 yaw-rate sign flips，
   先降低轉向速度並重跑同一路線。
7. R2 穩定後才進入正式長路徑 mapping、map save/load 與 localization。
8. Nav2、D1 Max、避障與馬達控制全部留到 visual map/localization 介面
   量測完成後。

## 7. 建圖品質判定標準

| 測試 | Pass | Review |
|---|---:|---:|
| 60 cm 直線距離誤差 | ≤10% | ≤15% |
| 90° 轉彎 yaw 誤差 | ≤5° | ≤10° |
| 閉環終點位置誤差 | ≤10 cm | ≤20 cm |
| 閉環 yaw 誤差 | ≤5° | ≤10° |
| 影像 rate | ≥28 Hz | ≥25 Hz |
| clipped | ≤5% | ≤10% |
| odom rate | ≥25 Hz GUI pass | 20–25 Hz 注意 |
| VO raw 有效率 | ≥98% | 95–98% |

即時分數是工程監控，不取代完成路線後的距離、轉彎與閉環量測。

## 8. 文件與資料夾導覽

### 目前應先讀

1. 本文件：目前狀態與下一步。
2. `docs/REAL_D435I_MAPPING_HANDOFF_20260817.md`：實機 R0–R3 流程與限制。
3. `docs/sensor_and_frame_contract.md`：frame、TF 與 CameraInfo 契約。
4. `docs/CUVSLAM_NVIDIA_REFERENCE_DEBUG_DESIGN.md`：NVIDIA 輸入契約與 debug 原則。
5. `logs/real_d435i_quality/20260825_r2_correction_log.md`：最新修正與實測判讀。

### 各資料夾用途

| 資料夾 | 用途 | 規則 |
|---|---|---|
| `docs/` | 設計、契約、handoff、目前決策 | 以本索引找到有效文件 |
| `reports/` | 實驗結果與 acceptance report | 歷史失敗報告不可刪除或覆蓋 |
| `logs/` | 原始 runtime、bag metadata、GUI JSON | 只作證據，不當作 source |
| `calibration/` | 相機、IMU、extrinsic、時間紀錄 | 實機校正資料集中保存 |
| `deployment/` | Jetson、Isaac ROS、容器與網路部署 | 不混入 production Nav2 |
| `tools/` | 可重複執行的收集、分析與 GUI 工具 | 程式修改要附驗證 |
| `datasets/` | 原始/處理後 recording 與 provenance | 大檔不複製到 reports |

### 歷史或不同主題文件

- `docs/CUVSLAM_STAGE1_*` 至 `STAGE4_*`：模擬與輸入契約的設計歷史。
- `docs/CUVSLAM_*_DESIGN.md`：模擬、frame、觀測性與 estimator 設計。
- `docs/RESEARCH_PROGRESS_2026-08-06.md`、`PPO_*`、`IMM_*`：D1 Edu/導航研究，
  與目前實機建圖主線分開閱讀。
- `PROJECT_HANDOFF.md`：完整專案與模擬歷史；目前實機狀態以本文件為準。
