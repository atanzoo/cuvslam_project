# Map-3 Nav2 MPPI 邊界路徑診斷與速度餘量設計

日期：2026-10-02（Asia/Taipei）

狀態：**使用者選定的 C2 方案 A 已完成：v3 速度餘量設定通過一條邊界路徑與原始固定物路徑的閉環驗收。使用者於 2026-10-02 決定略過額外固定物鏡射／不同配置，直接往 IMM／PPO 接口驗收；略過項目仍未驗證，未啟動新 PPO 訓練。**

## 目標、範圍與環境

完成既有 3× Map-3 場景的一條約 10 m 邊界接近路徑，釐清控制器中止的
原因，作為 IMM／PPO 接入前的靜態門檻。本次沒有訓練 PPO，沒有啟用行人。

- Jetson：既有隔離容器 `nav2_mppi_map3_20261002_v4`，映像
  `isaac_ros_dev-aarch64:fixed`；ROS 2 Humble、MPPI 套件
  `1.1.12-1focal.20231005.231956`。
- ROS domain `178`，`ROS_LOCALHOST_ONLY=1`；橋接 HTTP 只監聽
  `127.0.0.1:8987`，經 USB SSH 通道連接本機 `127.0.0.1:28471`。
- Mac：專案 `.venv`，MuJoCo `3.11.0`，D1 Max kinematic proxy。
- 地圖、固定物與 reset seed `10042` 保持一致；scene ID
  `reset10042_scene1369567760`。行人 `appear_time=inf`。
- 實際 Nav2 路徑起點 `(-1.4199998473, 1.0800002548)`、終點
  `(-8.445, -5.545)`，初始朝向 `pi`。
- 代理半徑 `0.49 m`；固定物邊緣距離門檻 `0.61 m`。
- 橋接硬上限保持 `|vx| <= 0.30 m/s`、`|vy| <= 0.02 m/s`、
  `|wz| <= 0.70 rad/s`，姿態與命令 watchdog 均為 `0.30 s`。

## 觀察結果

離線路徑長 `10.1741236579 m`、407 點；602 個密集幾何樣本全部可通行，
路徑與固定物的最小中心到邊緣距離 `1.2214543231 m`。全域 costmap 的
1,007 個密集樣本全部通過，source Occupied／Unknown 禁行檢查通過。
這只驗證規劃幾何，尚未完成整條路徑的實際追蹤。

兩次初期啟動嘗試未讓代理移動：第一次本機 socket 被沙箱拒絕，第二次
代理等待 active goal 超時。Nav2 日誌在目標啟動約 10 秒後記錄
`Failed to make progress`。調整為同一次操作內依序啟動姿態迴圈與動作後，
代理能收到控制命令，排除了持續性的 ROS domain／狀態 topic 接線錯誤。

| 同步啟動嘗試 | 模擬時間 | 已消費命令 | 剩餘目標距離 | 最小固定物邊緣距離 | 碰撞／地圖違規／70 s timeout |
|---|---:|---:|---:|---:|---|
| retry2 | 0.576 s | 6 | 9.618709 m | 1.730358 m | 無／無／無 |
| retry3（補足診斷欄位） | 0.672 s | 7 | 9.611698 m | 1.723795 m | 無／無／無 |

retry3 保存的停止原因是 `bridge_disarmed_while_goal_active`；當時橋接
`fault_reason=invalid_or_overspeed_nav2_command`、`fresh_pose=true`、
`goal_active=true`、動作狀態為 `2`，返回速度 `[0, 0, 0]`。

同時，以唯讀 ROS subscriber 在 domain 178 觀察 `/cmd_vel` 20 秒，共取得
110 筆命令。最大絕對值是 `[0.3115339875, 0.0, 0.3477657437]`。
第一筆超限樣本（observer sample 8）為：

```json
{
  "vx_mps": 0.31153398752212524,
  "vy_mps": 0.0,
  "wz_radps": -0.28992852568626404
}
```

後續樣本的 `vx` 包括 `0.3089031577`、`0.3042638600`、`0.3010109961`，
也有 float 精度下略大於 `0.30` 的值。observer 包含橋接停用後 Nav2
繼續輸出的命令；代理未消費這些超限命令。

**確認的原因：** MPPI 設定的 `vx_max=0.30` 並未讓本次實際發布的每筆
命令都滿足橋接的 `0.30` 硬上限，因此橋接停用；其後 Nav2 因代理停止而
進度不足並回報 `ABORTED`（狀態 `6`）。本次尚未追查 MPPI 內部哪一個
運算階段造成超出設定值，不能把平滑濾波當成已驗證的根因。

這些是門檻／啟動診斷，不能併入 PPO 的成功、timeout 或 collision
比例。短時間沒有碰撞也不足以認定邊界路徑合格。

## 已完成的 C1 修正與驗證

只修改 `simulation/path_planning/nav2_map3/run_nav2_mujoco_gate.py`：

- 保存 `episode_stop_reason` 與明確 disarm 前的 `bridge_state_at_stop`，
  避免原始 fault 被後續清理操作覆蓋。
- `initial_yaw_rad` 改記真正的起始朝向；retry2 舊檔曾誤記終止朝向，
  原始檔保留，retry3 的值為 `pi`。
- 不改橋接、碰撞判斷、速度／watchdog 門檻或控制參數。

驗證完成：Python 編譯檢查通過；既有橋接 6 個單元檢查通過；retry3
整合診斷確認原始 fault、active goal、新鮮姿態、零速度與初始朝向均寫入
證據。整條路徑、鏡射、行人、IMM、PPO 與硬體測試尚未完成。

## C2／C3 修正方案比較

| 方案 | 優點與相容性 | 代價、失效方式與維護 | 回滾與部署影響 |
|---|---|---|---|
| **A：MPPI 內部速度目標留餘量（推薦，C2）** | 在新的隔離試驗 profile 中只把 `vx_max` 從 `0.30` 降到 `0.28`。橋接硬上限與單一 `/cmd_vel` 所有權不變；維持已安裝的 MPPI。 | 較慢，可能提高 timeout；`0.02 m/s` 餘量比本次觀察的超量大，但仍需實測，不能當成形式保證。維護成本小，後續可做配對參數搜尋。 | 停止新試驗容器，回到保留的 v2 profile；不覆寫 v2、來源地圖或實機設定。 |
| B：控制器內部在最終輸出前施加硬裁切（C3） | 可直接限制發布命令，保留 controller_server 為唯一發布者。 | 需要維護／建置 Nav2 插件改版並重新驗證運動模型、平滑度及 critic 行為；裁切可能改變已評估軌跡。維護成本較大，擴充與版本升級需再次驗收。 | 回到既有發布版 MPPI 及 v2 profile；保存自訂版本、建置資訊與失敗試驗。 |

**選定 A。** 使用者在兩方案說明及是否採 A 的確認後回覆「好 開始」，
授權先執行速度餘量修正與靜態驗證。B 暫不採用，因目前沒有必要維護
控制器改版。維持橋接速度上限及數值容差、watchdog。新 profile 為
`simulation/path_planning/nav2_map3/map3_mppi_speed_tuning_v3_headroom.yaml`，
複製 v2 的其餘設定，唯一差異是：

```diff
 controller_server:
   ros__parameters:
     FollowPath:
-      vx_max: 0.30
+      vx_max: 0.28
```

上述設定已建立；v2 保留不變，v3 的兩條指定路徑已用新的實際命令與
閉環結果判定通過。此選擇只授權隔離模擬試驗，沒有更動真機配置或橋接保護行為。
對應 `ENGINEERING_GUIDELINES.md` §3.2 的兩方案與使用者選擇已記錄。

## 選定方案後的驗收

1. 新建隔離試驗 profile／runtime，保存 v2 與地圖，讀回實際參數。
2. 使用相同 seed／路徑／固定物，連續保存完整 `/cmd_vel`、橋接停止
   原因與 action 結果。所有命令有限且符合現有三軸硬上限，不得因超限
   或 stale watchdog 停用。
3. 本邊界路徑與原本 10.962 m 基準路徑各須於 70 秒內收到 Nav2 成功
   結果；零接觸、零幾何碰撞、零 map violation、最小固定物邊緣距離
   至少 `0.61 m`。故障／timeout 必須分開記錄。
4. 使用者於 2026-10-02 明確要求不做額外固定物鏡射／不同配置，因此
   移除這項 pilot 前置門檻，保留未驗證限制。下一步直接接入 IMM 動態
   預測與 PPO 偏好接口；接口方案及實作範圍另見
   `NAV2_MPPI_PPO_IMM_INTERFACE_DESIGN_20261002.md`。逐回合碰撞／邊界／
   clearance／速度保護維持，動態與接口驗收通過後才開始已同意的 25k pilot。

review trigger：仍有超限、watchdog、固定物／邊界違規，或速度餘量導致
70 秒 timeout 時，停止並審查命令產生／控制成本，保留所有失敗證據。

## v3 驗收結果

新 runtime 為 `/home/tseng/jetson_slam_ws/simulation/path_planning/nav2_map3_mppi_20261002_v5_headroom`，
容器 `nav2_mppi_map3_20261002_v5_headroom`。沿用上述 Jetson、Humble MPPI、
domain 178、Mac MuJoCo 與 kinematic proxy；容器 root filesystem 唯讀、
無硬體 device mount，maps／config 唯讀，results／logs 可寫。
映像 ID 為 `sha256:934771b0dd2ad60b7bad76e97c2dcf25fc71a0b0a2db09ee009951f5123adbe6`。

啟動後讀回 `FollowPath.vx_max=0.28`、`wz_max=0.70`、`DiffDrive`、
30 步 × 0.10 s horizon、controller 10 Hz；地圖、planner、controller lifecycle
均 active。姿態橋接獨佔 odom／TF，controller_server 為唯一 `/cmd_vel`
publisher；命令紀錄器只新增唯讀 subscriber。
本機／Jetson v3 profile SHA-256 均為
`efdf0cad6717f8d9df09e44795e4f79a095b47c0d5aa5b8c364d447a571a07fa`。
與 v2 比對有效設定行，唯一差異為上述 `vx_max`。

本次保存新的全域 costmap，再於動作啟動前驗證兩條路徑：602／654 個
幾何樣本與 1,007／1,089 個 costmap 樣本全部通過。新的 costmap 與前次
boundary costmap 只差一格（raw index 204379，0 → 217）；Occupied／Unknown
禁行仍通過。不能只因 costmap hash 不同便宣稱控制參數或地圖改變。

| v3 指定路徑 | 路徑長 | 模擬到達時間 | 最小中心到固定物邊緣距離 | 終點誤差 | ROS 命令筆數 | 最大 \|vx\| | 結果 |
|---|---:|---:|---:|---:|---:|---:|---|
| 邊界接近，初始 yaw = pi | 10.174 m | 37.568 s | 1.178471 m | 0.291321 m | 378 | 0.284927 m/s | 通過 |
| 原始繞固定物，初始 yaw = pi/2 | 10.962 m | 40.808 s | 0.932213 m | 0.288401 m | 411 | 0.284281 m/s | 通過 |

兩個 Jetson `FollowPath` 結果均為 `succeeded`（4），local runner 均記錄
`goal_reached_nav2_succeeded`；零 physical contact、geometric collision、
map violation、timeout、非有限命令及超限命令。三軸命令上限保持
`[0.30, 0.02, 0.70]`；最大 yaw 分別為 `0.442293`／`0.451254 rad/s`，vy 均為 0。
兩次 `/cmd_vel` receipt 間隔中位數均約 0.100 s，最大約 0.111 s；這是
subscriber 收到命令的間隔，並非 MPPI 計算時間剖析或期限的形式保證。
local proxy 消費次數 396／432 與 ROS 發布筆數不同，因橋接讀取週期會
重複消費上一筆有效命令；不能相加當成 ROS 獨立命令數。

控制器實際輸出仍曾略高於設定目標 0.28，但本次每筆均低於既有 0.30
硬上限。兩次試驗的餘量修正有效，不表示其他場景必然不會超限。
表中時間是模擬行走時間；包含初始化／握手的 runner wall time 分別為
55.651／59.860 s，不是訓練吞吐量證據。

新增 `record_nav2_commands.py` 保存完整命令與 paired action UUID／terminal
status，最長執行 120 s，遇新 goal terminal 後停止；拒絕覆寫既有結果。
新增 5 個命令彙整單元檢查，連同既有 bridge 6 項共 11 項通過；Python
編譯檢查通過。未安裝 PyYAML；YAML 的實際解析以 ROS 啟動／參數讀回驗證。
未做鏡射／不同固定物配置、行人、IMM、PPO preference、critic timing、
gait 或真機測試。這是同一個 scene 的兩條靜態路徑，不是訓練成功率估計。

## 證據與清理

證據根目錄：
[`../../simulation/path_planning/evidence/nav2_mppi_map3_seed10042_20261002/`](../../simulation/path_planning/evidence/nav2_mppi_map3_seed10042_20261002/)

- `20261002_nav2_mppi_boundary_path_seed10042.json`
- `20261002_nav2_mppi_boundary_global_costmap_seed10042.json` 及 `.bin`
- `20261002_nav2_mppi_follow_path_boundary_seed10042_no_fresh_pose_abort.json`
- `20261002_nav2_mppi_follow_path_boundary_seed10042_fresh_pose_abort.json`
- `20261002_nav2_mppi_mujoco_boundary_gate_seed10042_retry2.json`
- `20261002_nav2_mppi_mujoco_boundary_gate_seed10042_retry3.json`
- `20261002_nav2_mppi_follow_path_boundary_seed10042_retry2.json`
- `20261002_nav2_mppi_follow_path_boundary_seed10042_retry3.json`

診斷期間查到的目標啟動／進度中止時間對包括 `15728.240/15738.342`、
`15871.937/15881.939`、`16260.469/16271.171`（Jetson ROS 時間），保留於
容器 stdout／既有 logs mount。20 秒唯讀命令觀察結果保留於本次工具
輸出，上述最大值與首筆超限值逐字轉錄。

v3 新證據（同一根目錄，舊失敗檔案保留）：

- `20261002_nav2_mppi_headroom_v3_readiness_seed10042.json`
- `20261002_nav2_mppi_headroom_v3_global_costmap_seed10042.json` 及 `.bin`
- `20261002_nav2_mppi_headroom_v3_boundary_seed10042_mujoco_gate.json`
- `20261002_nav2_mppi_headroom_v3_boundary_seed10042_follow_gate.json`
- `20261002_nav2_mppi_headroom_v3_boundary_seed10042_commands.json`
- `20261002_nav2_mppi_headroom_v3_baseline_seed10042_mujoco_gate.json`
- `20261002_nav2_mppi_headroom_v3_baseline_seed10042_follow_gate.json`
- `20261002_nav2_mppi_headroom_v3_baseline_seed10042_commands.json`

v3 清理完成：橋接先停用，health 確認無 active goal 且返回零速度；容器
停止且保留，讀回 `exited`／`Running=false`。Jetson `8987` 和 Mac `28471`
監聽均已消失，無執行中 Docker container。未刪除任何舊 evidence、安裝
套件、拉取映像、提交 Git 或部署真機。`PROJECT_HANDOFF.md` 與架構紀錄
只更新此模擬里程碑／剩餘門檻，不改真機驗收狀態或把舊失敗改列通過。
