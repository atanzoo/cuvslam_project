# Experimental candidate-map localization + read-only monitor

日期：2026-09-24（Asia/Taipei）
變更類別：C2 interface/launch + C3 localization experiment
狀態：第一階段 local review artifact；未部署、未啟動硬體

## 目標與非目標

目標是以既有 `map.pgm/map.yaml` 建立隔離的候選地圖定位路徑，讓
`nav2_map_server` 載入地圖、AMCL 成為唯一 `map -> odom` owner，並提供
唯讀的 AMCL pose、平面 covariance、TF freshness、狀態轉換與 map-frame
trajectory 監控。

非目標包括：修改 production Nav2/AMCL、改動 `/Applications/slam_v2`、
啟動 planner/controller/behavior tree/velocity path、發布任何速度命令、
啟動 Jetson/D435i/A2M12、校正外參、宣稱地圖或定位 accuracy PASS，以及
執行 stationary、人工初始定位、低速直線、labelled turn 或回到起點驗證。

## 方案比較與決策

### 方案 A（採用）— `nav2_map_server + AMCL`

`experimental_candidate_map_localization.launch.py` 只啟動 map server、
AMCL 與 localization lifecycle manager。外部 cuVSLAM/A2M12 路徑提供
`odom -> base_link` 與 `/scan`；AMCL 單獨發布 `map -> odom`。這保留了既有
候選 2D map 的資料型態，也把全域修正與 cuVSLAM local odometry 分層。

優點：Nav2 相容、TF ownership 清楚、可先做 offline/static contract review，
不需要修改既有 slam_toolbox source。代價：AMCL 初始 pose 仍需人工操作，
候選 map 品質與 scan/map 時間對齊未證明，AMCL 參數沒有被 accuracy-qualified。

### 替代方案 — `slam_toolbox` localization mode

以既有 slam_toolbox localization runtime 直接載入同一張 map。

優點：沿用 2026-09-11 產圖路徑與既有 `/scan` 觀測工具。代價：會把既有
experimental mapping/scan-timing 歷史與本次候選 localization 混合，且目前
交接中 slam_toolbox 是另一條已知實驗線的 `map -> odom` owner；不利於把
AMCL 這個獨立假設單獨驗證。因此本階段不採用。

## Ownership 與 public telemetry contract

```text
candidate map.pgm/yaml -> candidate_map_server -> /map
external /scan + odom -> candidate_amcl -> map -> odom
external cuVSLAM -> odom -> base_link
experimental_localization_monitor -> subscribe/display only
```

本階段唯一新增 public contract 是
`experimental_localization_telemetry.v1`（stdout 與 `GET /api/telemetry`）。
主要欄位為：

- `state`: `WAIT | OK | REVIEW | LOST`；
- `pose`: map-frame `x_m`, `y_m`, `yaw_rad` 與 `covariance.xy_m2/yaw_rad2`；
- `map_to_odom`: availability、age、parent/child frame；
- `state_transition`: 前一狀態到目前狀態；
- `trajectory`: bounded map-frame pose samples；
- `candidate_map_warning` 與 `extrinsic_candidate.status`。

monitor 不建立 publisher、TF broadcaster、ROS service client 或 command
path。`/initialpose` 只透過畫面說明交由人以 RViz/terminal 操作。

狀態規則是：新鮮 pose + 新鮮 `map -> odom` + 平面 covariance 在 review
門檻內為 `OK`；缺資料或 covariance 不完整為 `REVIEW/WAIT`；資料最老超過
3 秒為 `LOST`。這些是 telemetry health gates，不是定位精度 gates。

## 已知風險與失敗模式

- 候選 map 保存時約只有 `1%` endpoint alignment within `0.30 m`；載入 map、
  出現 pose 或 `OK` 狀態都不能提升為 map/accuracy PASS。
- 候選 `base_link -> rplidar_link` 為 `x=0.0, y=-0.10, z=0.0,
  yaw=2.334 rad`，是未驗證 candidate，不可當作 calibrated extrinsic。
- 沒有 `/scan`、沒有 `odom -> base_link`、沒有初始 pose 或 timestamp 過期時，
  monitor 必須顯示 WAIT/REVIEW/LOST，而不是填入假 pose。
- duplicate `map -> odom` owner、誤啟動另一條 slam_toolbox、或加入 navigation
  command path 都屬 launch contract failure。
- Mac 沒有 ROS 2 runtime 時只能執行 static/contract tests；不能虛構 live pass。

## Rollback 與部署界線

本輪只新增隔離檔案，不修改既有 launch/config/map/evidence。rollback 是在
review tree 移除 `real_robot/cuvslam/experimental_localization/` 與本設計記錄；
候選 map 保留不動。沒有 commit、push、Jetson sync 或 production deployment。

## 驗證計畫與本輪證據

本輪只做：

1. candidate map YAML/PGM existence、PGM dimensions、byte count 與 SHA-256；
2. launch static ownership/forbidden-node negative checks；
3. telemetry schema/read-only source checks；
4. state machine positive、missing-data、stale/lost、recovery、quaternion tests；
5. Python compilation 與新增檔案 whitespace check。

後續需另行人工批准並保留 evidence 的順序為：stationary、人工 initial pose、
低速直線、labelled turn、回到起點。這些不屬於本輪結果。

## 參考證據

- `real_robot/cuvslam/evidence/maps/20260911_162933_cuvslam_a2m12_emitter_off/README.md`
- `docs/sensor_and_frame_contract.md`
- `PROJECT_HANDOFF.md` 的 candidate-map 段落與 A2M12 ownership 記錄
