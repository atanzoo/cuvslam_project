# 2026-09-10 fused odom 與 cuVSLAM 改善目標方針

## 目前執行基線（2026-09-10 A/B 後更新）

在目前測試空間與既定 D435i 安裝條件下，下一階段先採用單純
cuVSLAM odom 作為導航開發基線：

```text
D435i + IMU -> cuVSLAM -> odom -> base_link -> Nav2
```

emitter A/B 測試後，`Emitter OFF` 是目前觀察到較穩定、較適合這個空間的
設定。RPLIDAR A2M12 仍可用於建圖與障礙物觀察，但 LiDAR scan-matching
odom、RF2O 與 EKF 暫時不接管正式 odom，也不作為目前導航必要條件。

這不是刪除融合方案，而是將它降為未來增強方案。既有 shadow runtime、輸入
契約 guard、設定、測試與 rollback evidence 全部保留；只有在 cuVSLAM-only
導航暴露出可重現且值得由第二來源處理的失效模式後，才重新啟動融合驗證。

本決策只適用於目前空間、光源、相機安裝、動作路線與
`640x360x30 + official` 設定，不能外推為所有環境的 emitter 或 odom 結論。

## 原始融合規劃（暫緩執行）

以下保留原本規劃的可驗證、可降級、適合 Nav2 的 fused odom 路線，作為
未來增強方案，不是目前空間的立即執行目標：

```text
D435i + IMU -> cuVSLAM -> 輸入整理 ─┐
                                   ├-> 品質管理 -> EKF -> fused odom -> Nav2
RPLIDAR A2M12 -> RF2O -> 輸入整理 ─┘
```

目前 `ekf_shadow` 仍然只是實機比較分支。正式模式的目標所有權是：

- fused EKF 唯一發布 `odom -> base_link`；
- `slam_toolbox` 唯一發布 `map -> odom`；
- `base_link -> camera/lidar` 由固定外參與 robot description 發布；
- Nav2 只消費通過品質管理的 fused odom，不直接在來源之間自行切換。

這個架構在室內、低速、平面幾何足夠的條件下具有工程可行性；但目前證據尚不足以稱為可靠的正式導航來源。可靠性必須由故障注入、重複路線、轉向與閉環測試建立，不能由 EKF 軌跡看起來平滑推論。

## 範圍與非目標

### 目標

1. 統一 cuVSLAM 與 RF2O 的座標、時間、旋轉方向、旋轉中心與協方差語義。
2. 修正或隔離 RF2O 目前不可信的欄位；特別是固定為零的側向速度與缺失協方差。
3. 建立品質管理狀態：`BOTH_OK`、`VISUAL_DEGRADED`、`LIDAR_DEGRADED`、`BOTH_UNRELIABLE`、`RECOVERING`。
4. 在 shadow 階段記錄每筆觀測是接受、拒絕、延遲或隔離，以及原因與資料年齡。
5. 經過 shadow 驗證後，才建立獨立的 fused 實測模式，並由 EKF 成為唯一 `odom -> base_link` 發布者。
6. 以實際 Jetson 負載、延遲、溫度與資源餘裕確認導航可用性。

### 非目標

- 今日不修改正式 TF 所有權、不接管 Nav2、不修改 `/cmd_vel` 或安全行為。
- 不把 `ekf_shadow` 的幾何投影百分比解讀為 Kalman gain。
- 不以 `vo_state=1`、30 Hz、地圖外觀或平滑軌跡單獨宣稱 odom 正確。
- 暫不處理 slam_toolbox 地圖逐漸變厚與地圖更新率問題。
- 不用影像處理製造人工紋理，也不直接替換 cuVSLAM 內部特徵提取器。

## 分階段工作順序

### Phase 1：輸入契約與基準

- 固定目前 D435i 與 A2M12 的安裝位置，以目前硬體姿態為基準。
- 驗證 `base_link`、相機、LiDAR 的軸向、yaw、lever arm 與旋轉中心。
- 保留原始 sensor timestamp；分開量測感測時間、發布時間、處理延遲與 ROS graph 延遲。
- 確認兩個 odom 的 pose 與 twist 是否真的代表同一個機體參考點。

### Phase 2：修正融合輸入

- 每個來源只選擇語義清楚且互不重複的 pose 或 twist 欄位。
- 修正 zero／invalid covariance，不以小數值自動掩蓋未知不確定性。
- 核對 RF2O 實際輸出；若 `vy=0` 只是節點固定填值，不得把它當成四足機器人的真實無側移觀測。
- 處理來源重啟、時間倒退、長時間空窗與起點重新初始化。

### Phase 3：品質管理與故障處理

- 視覺品質使用 tracking status、影像時間、實際曝光、模糊／飽和、觀測分布與幾何結果共同判定。
- LiDAR 品質使用有效點比例、scan matching 收斂、幾何退化、資料年齡與連續性判定。
- 對異常觀測採用 gating、協方差膨脹或暫停輸入，並加入恢復等待與 hysteresis，避免反覆重連與跳換。
- 來源失效時，EKF 只預測並標示不確定性增加；兩者都不可信時，輸出狀態必須通知導航／安全層。

### Phase 4：shadow 驗證

測試順序固定為：靜止、直線、原地旋轉、一般轉彎、閉環、低紋理黑箱、單一來源失效、延遲／丟包與雙來源失效。每組至少保留設定、版本、硬體、TF ownership、原始 bag、結果與操作備註。

初始工程門檻先作為待確認目標：

- 1 m 直線誤差不超過 5%；
- 固定旋轉中心的 90° 旋轉額外 XY 位移不超過 0.05 m；
- 約 10 m 閉環終點 XY 誤差不超過 0.20 m、yaw 不超過 5°；
- 來源失效時不產生位置跳變，恢復時不反覆切換；
- Jetson 上 EKF、RF2O、品質管理加入後仍符合導航延遲與資源門檻。

以上是本專案的初始驗收目標，不是通用安全標準，也不是目前已達成的結果。

### Phase 5：正式 fused 實測模式

只有在 Phase 1–4 通過後，才建立獨立的正式 fused launch。切換前先停止目前 odom TF publisher，確認單一 `odom -> base_link` owner，再做低速空載導航測試。回滾時恢復 cuVSLAM 原有 owner，保留所有 fused evidence。

## cuVSLAM 的改善方向

單顆 D435i 還有改善空間，但改善順序是「先控制輸入，再處理影像」：

1. **Emitter／曝光 A/B**：沿用同一路線分別比較 emitter on/off、automatic exposure 與受控曝光。短曝光可減少轉動模糊，但過短會降低有效訊號；不能以單次結果決定。NVIDIA Isaac ROS 2.1 文件也提醒 RealSense emitter 可能造成近物漂移，應納入受控比較。
2. **低成本對比增強**：對實際送入 cuVSLAM 的左右 IR 影像測試溫和 gamma 或 CLAHE；左右影像與連續影格需保持一致，並同時記錄原始影像、延遲、飽和與追蹤結果。CLAHE 可能同時放大雜訊，因此只能以轉角與低紋理 A/B 結果決定。
3. **去噪與模糊控制**：既有 `denoise_input_images` 不宜再疊加重濾波；應優先檢查曝光、機械振動、IMU 時間與影像幀率。
4. **幾何與環境觀測性**：低紋理黑箱、純色牆面與反光表面不是單靠演算法參數就能可靠解決。若允許改變環境，應加入固定、非重複且有深度的真實表面細節；這是環境改善，不是製造假特徵。
5. **不直接替換 cuVSLAM 特徵器**：目前 release-2.1 的 ROS 介面沒有簡單替換內部 feature extractor 的入口；改用 learned feature、direct method 或緊耦合視覺—LiDAR估測器屬於另一個 C3/C5 專案。

四足機器人還需額外驗證機身俯仰、上下振動與掃描平面變化。RF2O 是平面 LiDAR odometry，通過手持或平地測試不能直接代表四足行走通過。

## 今日結論與下一步

今日方針更新為：先以 emitter OFF 的 cuVSLAM-only odom 進行導航基線驗證，優先完成 TF、直行、轉彎、閉環與 Nav2 odometry/costmap 檢查。LiDAR odom 與 EKF 暫緩接入正式導航，但保留既有 shadow 實作、輸入契約檢查與離線 fault-injection 路線，作為未來針對可重現失效的增強方案。CLAHE 與其他紋理處理排在導航基線之後。

本次 A/B 結果與限制另記於
`research/cuvslam/real_d435i_cuvslam_baseline_emitter_ab_20260910.md`。

## Phase 1 輸入契約修正已完成（本機）

本次已完成方案 A 的第一個本機實作單元：

- `robot_localization` shadow 改接 `/fusion/visual_odom` 與 `/fusion/lidar_odom`。
- 兩個來源均停用 pose 融合與 differential pose；cuVSLAM 只選用 `vx、vy、wz`，RF2O 只選用 `vx、wz`。
- 新增 `odom_contract_guard.py`：檢查 `frame_id`、`child_frame_id`、timestamp 單調性、twist 有限值與選定欄位 covariance；不符合者拒絕送入 EKF，並發布 `/fusion/odom_contract_diagnostics`。
- guard 不自行填入 covariance，避免把未知誤差偽裝成高可信度。
- raw `/lidar/odom` 與 raw cuVSLAM odom 仍保留給比較、GUI 與 bag recording；正式 TF ownership 沒有改變。
- telemetry collector 會轉送 guard diagnostics，GUI 的可複製工作紀錄會記下每個來源的 accepted／isolated 狀態與原因；raw、guarded 與 diagnostics topic 都保留在 recording topic list。

本機驗證已通過：Python 編譯、guard 單元測試（4 tests）、整合 contract test、Bash syntax 與 `git diff --check`。尚未同步 Jetson、尚未啟動 ROS、尚未取得 RF2O 實機 covariance，因此目前不能宣稱 LiDAR 已經進入 fused odom；若 RF2O covariance 仍為零，guard 會按設計隔離它，下一步是用實測資料校準來源 covariance。

## 方案 A Jetson 同步紀錄（2026-09-10）

已將方案 A 的 runtime 檔案同步到 Jetson `/home/tseng/isaac_ros_ws/real_robot/cuvslam/`；launch 同時更新 canonical `launch/` 與 ROS package source copy。同步前的遠端版本保存在：

`/home/tseng/isaac_ros_ws/backups/odom_contract_phase1_20260910_154300`

同步後所有部署檔案的 SHA-256 與本機一致。Jetson 離線驗證通過：Python compile、guard 單元測試 4/4、Bash syntax，以及 launch/config/recording 關鍵契約檢查。尚未啟動 Docker、ROS、相機、LiDAR 或馬達；本次結果只是部署與介面驗證，不是 fused odom 精度驗收。

## D435i emitter A/B GUI 開關（2026-09-10）

GUI 新增獨立的 `IR projector A/B` 選擇：A=`ON`、B=`OFF`。它會覆寫 emitter 狀態，但保留所選 lighting profile 的曝光、gain、自動曝光與去噪設定，因此 `official + A` 與 `official + B` 只改變投射器開關。Requested runtime、工作紀錄、trace 與 session 都會保存 `emitter_override`。

Jetson 的 R2 與 A2M12 mapping runner 已加入 `emitter_override=profile|on|off` 參數並完成同步；既有執行中的 container 不受影響，下一次由 GUI 啟動時才生效。遠端 runner SHA-256、Bash syntax 與 emitter 參數檢查通過。A/B 實驗尚未開始，仍需使用同一路線、同一速度與 `cuVSLAM` estimator 逐次比較。

## 參考

- [NVIDIA Isaac ROS Visual SLAM 2.1](https://nvidia-isaac-ros.github.io/v/release-2.1/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html)
- [robot_localization sensor configuration](https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/8696ee5a9e4f959fcaae37835dcf2ed12ead581b/doc/configuring_robot_localization.rst)
- [robot_localization state estimation nodes](https://raw.githubusercontent.com/cra-ros-pkg/robot_localization/rolling-devel/doc/state_estimation_nodes.rst)
- [Nav2 odometry setup](https://docs.nav2.org/rolling/configuration_and_development/first_time_robot_setup_guide/odom/setup_odom/)
- [RF2O laser odometry](https://github.com/MAPIRlab/rf2o_laser_odometry)
