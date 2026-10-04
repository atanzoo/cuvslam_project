# Map-3 雙目標：真正 Nav2 參與的訓練前驗收設計

日期：2026-10-01（Asia/Taipei）  
狀態：**使用者已選 A；Jetson 隔離 Nav2 單場景 map/path 門檻通過，閉迴路試跑超時失敗，未開始新訓練**  
變更級別：C5 架構／新 ROS 依賴；控制介面與成本地圖亦涉及 C2–C4。

## 目標、範圍與現況

目標：在與後續 PPO 訓練相同的 3× 候選地圖、10 m 路線、D1 Max
kinematic proxy、固定物與行人場景中，先由**實際 ROS 2 Nav2** 產生並執行路徑；
證明受測場景的固定物及地圖邊界沒有被路徑或閉迴路機器人侵犯，然後才啟動
新的 70 s／25k PPO 訓練。這是有明確場景和覆蓋率的模擬驗收，並非對所有
未知環境「永遠不碰撞」的證明。

非目標：不啟動實機、馬達或 Jetson 正式工作區；不部署到實機 `/cmd_vel`；
不宣稱步態、定位或硬體安全性；不覆寫原始 YAML/PGM、舊模型或停止的 10k
checkpoint；不把目前的 `Nav2GlobalPath` 名稱當作真 Nav2 測試。

目前事實：

- `D1DecisionEnv` 使用 `CandidateMapPlanner` 產生已膨脹地圖上的路線，再由
  `Nav2GlobalPath.set_path()` 消費；場景固定物在路線選取後才加入。現有
  `Nav2GlobalPath` 是進程內介面，不是 Nav2 ROS 節點。
- 2026-10-01 停止的訓練記錄為 11,520 steps；兩個已完成的訓練視窗中
  21/22 回合以碰撞結束。三個重播場景的首次 MuJoCo 接觸均為固定物，
  不是行人或地圖牆。這些樣本不代表所有訓練碰撞的歸因。
- 現有 `experimental_candidate_map_localization.launch.py` 只啟動
  `nav2_map_server`、`nav2_amcl` 和 lifecycle manager；契約測試明確排除
  planner/controller/`/cmd_vel`。本機 Mac 沒有 `ros2`、`/opt/ros`、Docker、
  Podman 或已發現的 VM 執行器；目前約 26 GiB 可用空間。**沒有可運行的
  本機 Nav2-in-the-loop 測試環境。**
- 原始地圖 SHA-256：YAML
  `be42f16b9730106c54f0bb75e83b04ec48b7c2c69f49e12c4fb4183ca21feb5d`，
  PGM `41e913c872bd118fc971b313024339f78e23247aec94fde00a18e51aa62efd74`。
  現有 3× 模擬以最近鄰複製像素，在記憶體中保持 0.05 m/cell；若輸出
  Nav2 專用衍生圖，預期為 597×924 cells、origin (-11.67, -16.02, 0)。
  輸出與來源的方向、閾值、unknown 語義須做逐格驗證。後續真 Nav2
  測試發現原 YAML 的 `free_thresh: 0.25` 會讓 Foxy 把灰 205 當 Free；
  另存的 Nav2 專用 YAML 改用 0.19，來源與 PGM 像素維持唯讀不變。

## 兩個真正使用 Nav2 的選項

兩者都必須在隔離的 ROS 2 環境運行 Nav2 map server、costmap、planner
與控制鏈，讓 Nav2 回傳的路徑進入同一個 MuJoCo proxy 閉迴路；PPO 仍只做
高階決策，局部 MPPI 不可另立第二個最終速度指令所有者。模擬 TF 可由
MuJoCo truth/odom adapter 單獨提供，但必須明確標示為模擬真值，不能
當作 AMCL 或實機定位驗收。ROS domain、namespace 和 `/cmd_vel` 必須
與任何實機完全隔離。

| 項目 | A：固定物先寫入場景衍生靜態地圖（建議先做） | B：固定物由模擬 LiDAR 寫入 Nav2 障礙層 |
|---|---|---|
| 優點 | Nav2 在首次規劃前已知道固定物；最直接驗證已知固定物與 unknown/地圖邊界；場景可重現 | 驗證感測、成本地圖更新和重新規劃；更接近未知但靜止物體的流程 |
| 代價／相容性 | 固定物被視為事先已知，不能證明即時感測鏈；必須為每場景保存衍生地圖身分。原始地圖與訓練 action contract 保持不變 | 需同步 MuJoCo 時鐘、LaserScan、TF、障礙層及重規劃；場景身分增加 sensor/latency 參數；舊訓練結果不可直接相比 |
| 失敗模式 | 衍生圖轉置、origin 或 inflation 錯誤；路徑安全但局部控制仍撞物；若人也被烘焙進靜態圖會污染任務 | 掃描漏檢／延遲、成本地圖更新不及、障礙清除過早、TF 時戳不一致、控制在重規劃前已接觸 |
| 部署影響 | 僅 Jetson 的隔離 Linux/ROS 模擬目錄，不同步到正式工作區或實機；需要受批准的 ROS/Nav2 執行環境 | 同左，另需 ROS–MuJoCo 感測橋與 QoS/時序契約；更高整合風險 |
| 維護／擴充 | 衍生圖產生器和身分驗證較簡單；後續仍需 B 才能驗證感測式避障 | 橋接與資料流較多，但可延伸到動態障礙、遮蔽與時間延遲實驗 |
| 回滾 | 停止隔離的 Nav2 節點、停用新增 adapter；保留衍生圖和失敗日誌，原圖／舊模型不變 | 同左，另停用 sensor bridge；不清除失敗證據 |

方案 A 是目前「已知固定物 + 邊界先通過，再重訓」的最小**真 Nav2**
選項；不是現有 Nav2-style Python A*。方案 B 可作後續感測整合驗收。
2026-10-01 使用者選擇 A，並允許在手邊 Jetson 執行；B 因需要額外的
即時感測橋、成本地圖時序與重規劃驗證，先列為後續工作，不拿來代替
本次已知固定物驗收。Jetson 只能使用 `/home/tseng/jetson_slam_ws` 下的
全新隔離試驗目錄；不得碰 `/home/tseng/slam_ws` 或任何實機輸出。

Jetson 唯讀盤點顯示 Ubuntu 20.04 / ROS Foxy，已裝 Nav2 planner 0.4.7，
模擬工作區存在，約 7.8 GiB 可用；目標試驗目錄原先不存在。既有
`navigation_bringup` 參數為 robot radius 0.16 m、`allow_unknown: true`，
且 limiter 輸出 `/cmd_vel`，不符合本次 proxy／unknown 不可通行契約，
不可原樣啟動或覆寫。Jetson host Python 尚無 MuJoCo/Gymnasium/SB3；
因此先建立真正 Nav2 planner 的純規劃門檻，閉迴路需隔離的 Mac–Jetson
模擬橋或另經驗證的 Jetson runtime，**不能用 planner PASS 代替**。

## 實作前介面契約

1. 來源 YAML/PGM 唯讀；另建 3× 衍生 OccupancyGrid，逐格對照
   free/occupied/unknown、resolution、origin、翻轉方向與來源雜湊。
   Unknown 和圖外一律不可通行。
2. 固定物在 Nav2 **規劃前**進入 global/local costmap；其 0.5×0.5 m
   實體幾何、候選 robot 半徑 0.49 m 和至少 0.12 m margin 必須一致。
   行人保持動態，不烘焙進靜態圖。
3. 模擬 `map -> odom -> base_link` 每條 TF 僅一個所有者；Nav2 path
   與 MuJoCo、PPO/MPPI 使用相同座標系。最終速度只有一個模擬 owner；
   無任何輸出路由到實機或正式 `/cmd_vel`。
4. 對 Nav2 planner 回傳的每段路徑做密集 footprint/成本地圖檢查；
   不允許 direct-fallback、未知區、圖外、固定物膨脹區的路徑。
   只檢查路徑頂點或僅見到 `SUCCEEDED` 都不足以驗收。

## 訓練前驗收與停止條件

1. **環境門檻**：記錄 ROS 2/Nav2 版本、執行主機與 ROS domain；
   lifecycle 節點 active、真 Nav2 planner action 回應、TF/時鐘有效、
   costmap 包含固定物；確認無連到實機。缺一則停止。
2. **幾何門檻**：固定種子與左右鏡射場景各保存 Nav2 path、成本地圖
   快照、scene/map/route hash；所有路徑段在已知 free、遠離膨脹固定物與
   邊界。任何違規即停止，不啟動 PPO。
3. **閉迴路門檻**：先在無行人、僅固定物的 10 m／70 s 場景測試
   基準決策控制，至少原固定種子及左右鏡射；受測集全部到達目標、
   MuJoCo 固定物 contact 為零、map violation 為零、無 timeout。
   僅停在起點的「零碰撞」不算通過。
4. **雙目標門檻**：再測五種行人運動及左右鏡射。固定物 contact 和
   map violation 仍須為零；行人 contact、成功、timeout 分開記錄，
   不把行人失敗誤判為固定物修復失敗。受測條件與未知區策略不得放寬。
5. **訓練**：只有 1–4 通過並由使用者接受新架構身分後，才用新的
   output/checkpoint/metadata 身分開始 fresh 70 s／25k PPO；不載入舊
   10k checkpoint。每 5k 視窗回報成功、固定物碰撞、行人碰撞、
   timeout、map violation。若固定物或邊界碰撞重現，立刻停訓並保存證據。

目前 [單場景實測報告](../../simulation/path_planning/evidence/20261001_nav2_map3_jetson_path_and_proxy_gate.md)
只證明 Jetson 上真 Nav2 的 map/costmap/path 幾何門檻；把此 path 交給本機
MuJoCo 控制鏈後，固定物 contact 與地圖越界為零，但 70 s 超時、沒有到達目標。
因此閉迴路門檻明確 **FAIL**，更不是 Nav2 controller、行人、訓練或實機安全
已驗收的證據。

## 後續控制接法：使用者已選 A2 真 Nav2 controller

已核准的「A」是**已知固定物寫入場景衍生靜態地圖**，不是對新的最終速度
所有者、控制器演算法或跨主機橋接的自動核准。單場景路徑雖安全，現有
MuJoCo/MPPI 追蹤在 70 s 超時；在 AVOID 加入簡單轉向的隔離對照又於
46.288 s 碰到固定物。下一個變更涉及 C2 介面、C3 控制器與 C4 速度
安全邊界，須另選接法後才實作。目標仍是「固定物及邊界無侵犯、70 s 內到達」；
非目標仍是實機、馬達、正式 `/cmd_vel`、改原始圖或拿局部模擬當硬體驗收。

| 比較 | A1：真 Nav2 planner/costmap + 成本地圖約束的現有 MPPI | A2：真 Nav2 controller_server 作模擬最終速度所有者（推薦作完整 Nav2 驗收） |
|---|---|---|
| 優點 | 保留 PPO 的五個高階動作與現有側移 MPPI，較容易沿用目前 25k 訓練身分；Nav2 路徑與成本地圖成為每個候選動作的硬約束 | 真正測到 Nav2 的規劃、local costmap、控制、goal checker 與命令停止；不再把「有真 Nav2 路徑」誤當完整導航鏈 |
| 代價／相容性 | 必須新增經過驗證的軌跡掃掠／footprint 安全濾網和終點控制；**不是** Nav2 controller 驗收，不能使用原設計的完整 Nav2-in-the-loop 名稱 | Mac MuJoCo ↔ Jetson Foxy 需隔離的 pose/clock/command 橋；現有 Jetson DWB 設定半徑 0.16 m、側移速度 0，不符 D1 proxy；PPO 五動作如何影響 Nav2 控制需另定版本化契約，不能直接延用舊 checkpoint |
| 失敗模式／安全 | MPPI 離散預測漏掉兩步間接觸、濾網過於保守而原地停住、終點修正繞過避障、成本地圖與場景不同步 | ROS 時鐘 1970／跨主機延遲、TF 單一所有者失效、controller 命令在斷線後殘留、local costmap 漏固定物、Nav2 非側移預設導致不可達或與 PPO 動作衝突 |
| 部署／維護／擴充 | Mac 模擬 + Jetson 隔離 planner，保留自訂 MPPI 維護成本；可逐步擴到動態行人，但須自負控制安全證據 | 僅 Jetson 隔離模擬工作區，無實機路由；橋接與 ROS 契約維護較重，但可延伸到感測式 B、重規劃及標準 Nav2 行為驗收 |
| 回滾 | 停用新的實驗濾網／終點控制選項，回到已保存的路徑與失敗重播；不得刪失敗資料 | 停止指定的 ROS domain 與本次橋接程序，確認零速度／無程序殘留；原圖、現有 Jetson config、Mac PPO 源碼與 checkpoint 不變 |
| 驗證 | 單元測試拒絕 Unknown、障礙及圖外候選速度；固定物原 seed／鏡射 10 m、70 s 必須到達且 contact/map violation/timeout 皆零；再跑行人五型鏡射 | 先只用模擬 2D proxy 驗證 lifecycle、TF、clock、goal checker、local costmap、斷線零速度；再用**同一 MuJoCo proxy** 和固定物／鏡射做完整閉迴路接觸驗收；PPO 動作映射獨立驗證後才能訓練 |

兩案都不能拿本次「一條 Nav2 安全路徑」直接啟動 25k PPO。若選 A2，
第一個 reviewable unit 只驗證隔離控制鏈與安全停機，不同時改 PPO／安全層；
通過後再審 PPO 五動作映射。若選 A1，成果名稱必須明示「Nav2 planner +
自訂 MPPI」，不可稱已驗證 Nav2 controller。2026-10-01 使用者明確
選擇 A2；A1 保留作比較方案，不拿來宣稱完整 Nav2 controller 驗收。
A2 的第一個可審查單位是隔離的控制鏈／斷線零速度，**不是** PPO 訓練，
也不是實機安全驗收。

### A2 第一單位：介面、所有權與停止條件

- Jetson 模擬專用 ROS domain 43 且 `ROS_LOCALHOST_ONLY=1`。已驗證的
  場景衍生圖是唯一靜態圖；`map_server` 管理 map，static TF 單一發布
  `map -> odom`，模擬 bridge 單一發布 `odom -> base_link` 與 `/odom`。
  不啟用 AMCL、實機 driver 或正式 safety stack。
- Jetson `controller_server` 是本隔離 domain 唯一 `/cmd_vel` 發布者。
  bridge 只訂閱該 topic，對 Mac MuJoCo 的唯一模型命令輸入執行時間戳、
  scene、序號及速度範圍檢查。無新鮮姿態、無新鮮 controller 命令、
  FollowPath 取消或連線失效時輸出精確零速度；不得 hold last command。
- Mac 不需安裝 ROS：經 USB SSH loopback tunnel 傳送 MuJoCo 真值 pose，
  Jetson 用自身單一 ROS clock 加時間戳。Jetson 的 1970 wall date 只允許
  作相對時序實驗，不能拿 log 日期當 2026 證據；使用 monotonic watchdog。
  bridge HTTP 只綁 loopback、只接受匹配 scene identity 的數據。
- 本單位不讓 PPO 五動作介入控制，亦不把既有 MPPI 作第二個速度 owner。
  controller 使用全新模擬專用參數，半徑至少 0.61 m；local/global
  costmap 對原始 Unknown 及固定物都不可通行。不能複用 Jetson 既有
  半徑 0.16 m、側移 0 的 config 而稱 D1 proxy 驗收。
- 僅在單 seed 完成 TF/QoS/clock/lifecycle、map/local costmap、
  FollowPath 與零速度 watchdog 之後，才執行 10 m／70 s MuJoCo 固定物
  閉迴路；需要到達、無固定物接觸、無地圖越界、無 timeout。
  任何接觸或停止條件失敗立即留存證據並停止，不進鏡射／行人／PPO。

第一單位的回滾是停止指定 bridge、tunnel、launch/action 程序，確認
ROS domain 43 無殘留 controller，保留失敗 log；原本的 planner-only
設定、Jetson 正式工作區、原始圖、舊模型和訓練入口皆不修改。
