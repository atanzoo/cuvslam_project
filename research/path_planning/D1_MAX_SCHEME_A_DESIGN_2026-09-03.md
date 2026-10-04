# D1 Max 方案 A：分階段導航模擬設計

日期：2026-09-03

## 目標與邊界

本階段把「使用 2D LiDAR 地圖訓練導航決策」與「D1 Max 步態／機構模型」拆成
可驗證的兩個邊界。第一階段先建立 D1 Max 的明確 robot profile 契約，保留
D1 Edu 為可重現的相容基線；在完整步態與 footprint 仍未完成時，先以明確
標示的 D1 Max kinematic proxy 驗證地圖、PPO 與 MPPI 資料流，完整 D1 Max
執行 profile 仍維持停用。

本階段不修改實機 cuVSLAM、LiDAR、Nav2、TF 或 `/cmd_vel`，不部署 Jetson，
也不把 D1 Edu 的網格或碰撞尺寸宣稱成 D1 Max 真實幾何。

## 方案比較與決策

| 方案 | 優點 | 代價／風險 | 回滾方式 |
|---|---|---|---|
| A：先建立 D1 Max profile 與 2D 地圖訓練邊界，再接步態 | 可先驗證地圖、全域參考路徑、PPO、MPPI 的資料流；步態完成後只替換執行層 | 在官方模型到位前不能宣稱 D1 Max 幾何／動力學結果 | 保留 `d1_edu` 預設與原有 replay JSON；停用 `d1_max_proxy` |
| B：等待完整 D1 Max 步態與模型後才開始導航訓練 | 機構一致性最高 | 等待時間長，無法先隔離地圖／決策問題 | 不需回滾，但研究進度被模型依賴鎖住 |
| C：直接把 D1 Edu 改名成 D1 Max | 最快看到一個可運行畫面 | 會把錯誤機構、footprint 和速度限制帶入訓練，結果不可追溯 | 只能刪除誤導性結果，會破壞研究證據，不採用 |

決策：採用方案 A。D1 Edu 是目前唯一具完整 runnable 狀態的 profile；
`d1_max_proxy` 只可透過 `--kinematic-proxy` 啟動模擬 episode，不代表完整
D1 Max 步態或實機可運行。

## 目標資料流

```text
2D LiDAR map / Nav2-style global reference
                    |
                    v
            PPO high-level decision
                    |
                    v
       MPPI local velocity optimization
                    |
                    v
         SimulatedD1SDK / gait adapter
                    |
                    v
              D1 Max model
```

在步態模型完成前，最後兩層只能使用明確標示的 simulation proxy；目前
repository 內的可運行模型仍是 `simulation/path_planning/models/d1_edu/`。

## 分階段驗收

### A0：profile 契約（已完成）

- `d1_edu` 維持既有預設行為。
- `d1_max_proxy` 明確顯示模型來源已取得，但尚未具備完成的 planner
  footprint、gait 執行層與 D1 Max 專用驗收；完整 profile 仍不可直接運行。
- 未明確指定 simulation-only proxy 時，誤選 D1 Max 會在載入 MuJoCo 前失敗。
- profile 名稱會進入可選的 episode trace，便於後續證據追溯。

### A1：D1 Max 模型輸入（已完成模型輸入門檻）

- 已從官方 repository 的 `urdf/max_description` 取得 `max.urdf` 與 29 個
  mesh，來源 revision 為 `83bdebb04ae140f751e7c5e9e2c8be44aad62473`。
- URDF 靜態檢查通過：29 links、28 joints、29 個 mesh references，缺失 mesh
  為 0；關節包含 12 fixed、12 revolute、4 continuous。
- 已產生 `external/Agibot_D1_Max/d1_max.xml`，MuJoCo 可載入為
  `nq=23, nv=22, nu=16`，並完成 100-step finite smoke。
- 官方 `BASE_LINK.STL` 約 1.32M faces，超過 MuJoCo 單一 mesh 的實務限制；
  目前 MJCF 僅將 base visual/collision 改成由官方 bounds 推導的 box proxy，
  其餘 limb mesh 保留官方幾何。這是轉換用 proxy，不是已驗證的 D1 Max
  碰撞模型。
- 尚未完成 planner footprint、質量／接觸校準、站立／步態測試，也未接真實
  馬達或實機 `/cmd_vel`。

官方套件的 `package.xml` 目前仍標示為 `TODO: License declaration`；因此
模型資產暫存於被 Git 忽略的 `external/`，不作為已核准的 repository 研究
證據或可提交資產。

### A2：2D LiDAR 地圖訓練

- 明確標示目前是 map-driven simulation；現有 in-process global planner 是
  Nav2-style 參考，不等同完整 ROS 2 Nav2 runtime。
- A2 的第一個幾何子階段已完成：`inspect_d1_max_geometry.py` 量測官方
  mesh／目前 base proxy 在 MuJoCo 預設姿態的平面範圍，報告保存於
  `research/path_planning/D1_MAX_GEOMETRY_AUDIT_2026-09-03.json`。
- 量測結果為 base `0.922156 x 0.237112 m`、納入目前 29 個 robot collision
  geoms 的外接範圍 `0.922156 x 0.476008 m`；以 base 原點計算的目前姿態
  planar radius 為 `0.480335 m`，`0.49 m` 只是四捨五入後的候選值。
- 這個候選值尚未成為訓練參數；必須先以站立與步態資料量測 articulated
  footprint envelope，再重新產生 global path、PPO observation 與 MPPI
  collision cost。
- 同一工具另以 16 個受限 hinge joint 的兩端點加固定 seed 隨機範圍掃描，
  使用 MuJoCo `geom_rbound` 得到保守 joint-limit envelope：半徑約
  `1.031931 m`、四捨五入候選值 `1.04 m`。這不是完整 reachable-set proof，
  也不是行走 footprint；它只用來證明不能以關節極限直接取代 gait envelope。
- 以固定 seeds、碰撞／goal／timeout／path error 與學習曲線驗收。

### A3：步態執行層替換

- 將 `SimulatedD1SDK` 的底層由 kinematic base 替換為已驗證的 gait adapter。
- 重新量測命令延遲、速度追蹤、加速度與 yaw response。
- 先重播固定場景，再重新訓練或 fine-tune PPO；不沿用未校準模型的效能宣稱。

### A2-kinematic：步態延期的明確替代路徑（已啟用）

- 以 `--robot-profile d1_max_proxy --kinematic-proxy` 明確 opt-in，使用官方
  D1 Max MJCF、目前 `0.49 m` default-pose planner 候選 footprint、pose-hold
  關節與 kinematic base。
- 若官方 MJCF 沒有 floor，scene builder 會補上 simulation floor；這只服務
  模型預覽、LiDAR、MPPI 與 episode loop，不改官方資產本身。
- 未指定 `--kinematic-proxy` 時，`d1_max_proxy` 仍在 episode 建構前被阻擋。
- 此模式可驗證 D1 Max 幾何接入、動態障礙、LiDAR、Decision Layer、MPPI 與
  SimulatedD1SDK 的資料流，但不代表四足站立、行走、接觸穩定或真機控制。
- Stable-Baselines3 environment 已切換到此 proxy，但必須以
  `--robot-profile d1_max_proxy --kinematic-proxy` 明確啟用；其 observation／
  action 語義與 D1 Edu 相同，checkpoint 仍視為不同 robot profile，不混用
  既有 D1 Edu policy。

### A2-SB3：D1 Max proxy 高層決策訓練（已完成介面整合）

- `d1_edu_sb3_env.py` 會實際載入 D1 Max MJCF、pose-hold 關節與 kinematic
  base；global path、tracker、MPPI、幾何 clearance 會使用目前校正候選
  `0.49 m` planner radius。
- `run_d1_edu_decision_layer_sb3.py` 提供 profile／proxy CLI，並在模型旁邊
  產生 `<policy>.metadata.json`，記錄 robot profile、模型 revision、
  observation/action contract 與 timesteps。
- 載入或續訓 D1 Max 時若 metadata 缺失或 profile 不一致會拒絕，避免把
  D1 Edu checkpoint 誤當成 D1 Max policy。
- 最小 smoke 已完成：16 environment steps、1 筆評估可跑完並輸出模型與
  metadata；這只證明資料流與介面可執行，不代表 16 steps 的性能或足以供
  實機使用。
- 範例：

  ```bash
  PYTHONPATH=simulation/path_planning/tools \\
  ./.venv/bin/python simulation/path_planning/tools/run_d1_edu_decision_layer_sb3.py \\
    --robot-profile d1_max_proxy --kinematic-proxy \\
    --scenario-mode people_random_multi_target \\
    --timesteps 50000 --eval-trials 100
  ```

## 風險、回滾與驗證

- 最大風險是把 D1 Edu 結果誤讀為 D1 Max 結果；以 profile readiness 閘門防止。
- 最大未知已從「模型是否存在」收斂為 base proxy 的幾何誤差、官方碰撞 geom
  的實際意義、受步態約束的 D1 Max footprint，以及步態／命令執行層的可控性。
- 回滾只需維持 `robot_profile="d1_edu"`，不需移動或刪除既有模型與研究結果。
- 本階段驗證包含 Python 編譯、profile 單元測試、D1 Edu 預設 episode 的
  headless smoke、D1 Max URDF/MJCF 靜態檢查、MuJoCo 載入與 100-step finite
  smoke、D1 Max 預設姿態幾何 audit 與 audit 單元測試，以及 D1 Edu／D1 Max
  proxy 的 SB3 reset/step 與短訓練 smoke；不包含 D1 Max 站立／步態、Nav2
  ROS runtime 或實機運動驗收。
