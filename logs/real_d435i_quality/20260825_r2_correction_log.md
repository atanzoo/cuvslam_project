# cuVSLAM R2 建圖修正紀錄

日期：2026-08-25  
硬體：Intel RealSense D435i + Jetson
測試模式：IMU fusion · mapping · 640×360×30 · official lighting

## 本次實測摘要

- 開始時間：2026-08-25 09:57:31
- 完整路徑點：3,216
- 累積路徑：5.612 m
- 影像 rate L/R：29.8 / 29.8 Hz
- 影像 gap P95 L/R：33.3 / 33.3 ms
- clipped L/R：1.6% / 1.7%
- odom rate：26.0 Hz
- VO 有效率：86.2%（當時 GUI 使用舊版 telemetry fallback，並非 raw status formal ratio）
- 最後位置：x=-0.020 m、y=-0.529 m、z=-0.019 m
- 最後 yaw：+4.92°
- 起點到終點 XY：0.529 m

## 判讀

1. 影像輸入不是這次失敗主因。左右影像接近 30 Hz、gap P95 為一個 30 Hz 週期，clipped 約 1.6–1.7%，符合目前官方光照策略的可用範圍。
2. 主要瓶頸是 VO 追蹤有效率偏低，以及 odom rate 只有 26 Hz，低於本專案希望的約 28 Hz 穩定目標。
3. 路徑保留正常：3,216 點與 5.612 m 累積路徑已被完整保留，問題不是 GUI 隨時間刪除軌跡。
4. 若本次路線確實是完整回到起點，XY 閉環誤差 0.529 m，超過 Review 門檻 0.20 m；yaw +4.92°則接近且落在 Pass 門檻內。若實際尚未回到起點，這個位置數字只能視為目前終點，不應當作正式閉環誤差。
5. 原本顯示的 86.2% VO 有效率需要保留，但必須標示為 provisional，因為當時 Jetson 端仍可能是舊版 collector；不能把週期性 GUI snapshot 當成 raw `/visual_slam/status` 樣本。

## 已完成的程式修正

### 1. 修正 VO 有效率計數方式

檔案：`tools/collect_real_d435i_odom_live.py`

- 改為每收到一筆 `/visual_slam/status` 就計數。
- 不再用每筆 odom callback 套用「最近一次 vo_state」來估算 status 樣本。
- telemetry 新增 `vo_status_samples` 與 `vo_valid_samples`，讓比例可追溯。

### 2. GUI 明確區分正式與暫定統計

檔案：`tools/real_d435i_axis_web_gui.py`

- 新增 VO 統計來源、樣本數與有效樣本數。
- 使用新版 collector 時顯示 `raw /visual_slam/status samples`。
- 使用舊版 collector 時顯示 `GUI telemetry fallback`，並在警告與摘要中註明「分數僅供參考」。
- 建圖分析摘要新增 VO 統計來源，避免把 fallback 數字誤當成正式驗收結果。
- 位移品質 JSON 同步保存 VO 統計來源、樣本數、odom rate、gap P95 與品質判定。

### 3. 保留既有建圖分析功能

- 完整路徑仍會保留到按下「停止 R2 並清除」為止。
- 工作紀錄保留可複製的建圖分析摘要。
- 本次未改動 official lighting、auto exposure、denoise 或 emitter；因為本次影像指標沒有顯示需要先改光照。

### 4. 2026-08-25 後續監測修正

- 工作記錄改為帶時間戳的過程事件時間線；建圖分析摘要獨立顯示與複製。
- R2 每次開始時清空上一回合的即時事件，避免不同測試混在同一個工作記錄簿。
- odom rate 改用已接受 odometry 的 ROS timestamp span 計算，避免把 container/ROS
  discovery warm-up 算進平均值。
- 新增轉向抖動診斷：turn yaw-rate P95、turn samples、yaw-rate sign flips 與
  REVIEW 事件。此診斷先作為獨立警示，不直接改變 R2 總分。

### 5. 建圖資料下載功能

- GUI 新增「開始錄製建圖資料」、「停止錄製」與「下載本次建圖資料」。
- 錄製內容包含左右 IR、CameraInfo、IMU、TF、cuVSLAM status、odometry、
  VO pose、SLAM path 與 landmarks cloud。
- 下載後保存到本機 `logs/real_d435i_r2_downloads/<run_name>/`，並附上
  `gui_mapping_summary.txt`。
- 本次已完成的 R2 是在下載功能加入前執行，沒有可回溯的 rosbag；下一次必須
  在實際移動前先按「開始錄製建圖資料」。

### 6. 2026-08-25 10:52 R2 建圖基準與錄製容量修正

- 本次 official lighting、IMU fusion、640x360x30、mapping 測試通過：100/100。
- raw `/visual_slam/status` 有效率 100%，odom rate 30.0 Hz，影像左右 29.8 Hz，
  clipped 左右 1.2%，轉向 yaw-rate P95 13.2°/s、sign flips 6，未觸發抖動警告。
- 路徑累積 5.020 m；若本次確實回到起點，終點 XY 誤差約 0.020 m、yaw 誤差
  3.78°，可作為目前建圖品質基準。
- 原本錄製器會把左右原始 IR 影像以 30 Hz 寫入 rosbag，檔案大小會快速增加。
- 已將 GUI 錄製模式改為「輕量建圖（預設）」與「完整回放」：輕量模式排除原始
  IR image，只保留 CameraInfo、IMU、TF、cuVSLAM status、odometry、VO pose、
  SLAM path 與 landmarks cloud；完整模式仍保留給需要影像回放的除錯情境。
- 已確認 Jetson 的 ROS 2 rosbag 支援 zstd，兩種模式都改用 file-level zstd 壓縮；
  完整模式雖仍可能很大，但會比未壓縮版本小。
- 錄製資料夾名稱會包含 `light` 或 `full`，避免之後誤判內容；現有舊資料不刪除，
  下一次建圖前請使用輕量模式開始錄製。

## 驗證與後續必要測試

- 已完成 Python syntax compile：GUI 與 odom collector 均通過。
- 已將修正版 collector 同步到 Jetson；遠端原版已備份為 `collect_real_d435i_odom_live.py.bak_20260825`，同步後 SHA-256 與本機修正版一致。
- 本次新增的 timestamp-rate 與 turn-jitter 版本也已同步到 Jetson；同步前版本備份為
  `collect_real_d435i_odom_live.py.bak_20260825_rate_fix`。
- 下一次 R2 啟動後，確認摘要顯示 `raw /visual_slam/status samples`，且樣本數會隨時間增加。
- 確認摘要顯示 `odom rate 計算基準：ROS odometry timestamp span`，並觀察轉向段的
  `轉向抖動診斷` 與工作記錄 REVIEW 事件。
- 以相同 official lighting 重跑：先靜止 5–10 秒，再走同一個矩形路線。
- 正式判定前檢查：VO raw ratio、odom rate、完整路徑、回到起點的 XY 誤差與 yaw 誤差。
- 若 raw VO ratio 仍低於 95%，再針對轉彎速度、場景紋理與特徵可見性處理；目前不先把問題歸因於 IMU 或曝光。

## 結論

本次不是「影像完全不可用」，而是「輸入品質可用，但追蹤連續性與 VO 統計可信度不足」。已先修正統計方法與 GUI 的判讀透明度；raw collector 尚需在 Jetson 端實際跑起後，再對下一次路線做正式比較。

## 2026-08-26 旋轉假平移診斷修正

### 問題確認

- 09:48:49 測試為近乎純旋轉；影像 29.6 Hz、odom 30.0 Hz、VO raw 有效率 100%、vo_state=1，輸入健康分數 100/100。
- 但 odom 顯示目前 XY 位移約 0.336 m。這個數字與實際「原地旋轉」不一致，應先標示為旋轉期間的估計假平移風險，不能解釋成相機真的移動了 26–33 cm。
- 既有模擬報告也顯示：低速純旋轉在場景可觀測性不足時，cuVSLAM 可能維持 vo_state=1，卻產生明顯 XY 假平移；因此 `vo_state` 與輸入健康分數不能單獨代表幾何精度。

### 已完成修正

- `tools/collect_real_d435i_odom_live.py` 新增「轉向平移診斷」：以連續 yaw-rate 段為單位，記錄 yaw 變化、旋轉期間最大 XY 估計位移與轉向段累積路徑。
- `tools/real_d435i_axis_web_gui.py` 新增獨立 GUI 指標、警告、摘要欄位與工作紀錄事件。總分仍代表影像/追蹤串流健康；轉向平移另行判定，避免 100/100 掩蓋幾何異常。
- 判定門檻：yaw 變化至少 30° 才結算；最大 XY ≤0.10 m 為 PASS、≤0.20 m 為 REVIEW、>0.20 m 為 FAIL。這是「原地旋轉診斷」門檻，不適用於有真實平移的轉彎路線。
- 已通過 Python syntax compile。下一次啟動 R2 後，摘要應出現：`轉向平移診斷：... · yaw Δ=... · max XY=...`。

### 後續測試

- 在同一環境做一次「原地旋轉 90°」：先靜止 5–10 秒，再以穩定低速旋轉，旋轉前後保持相同位置，不要邊轉邊走。
- 若 `max XY` 仍 >0.20 m，將它正式記為純旋轉假平移；再比較場景紋理、旋轉速度與影像曝光/眩光，而不是調整 odom 尺度。
- 本次 09:48:49 沒有對應的原始 rosbag，因此目前能確認的是「數值與物理動作不一致」及其風險，無法回溯到每一幀影像的特徵匹配失敗時刻。

### 2026-08-26 10:10–10:13 實機重複確認

- 測試一：實際執行近乎原地 90° 旋轉；odom 顯示 yaw -89.75°、起點到目前 XY 0.402 m、累積路徑 0.689 m。
- 測試二：實際執行 90° 旋轉；odom 顯示 yaw +37.59°、起點到目前 XY 0.359 m、累積路徑 0.627 m。這次連旋轉角也沒有正確恢復。
- 兩次影像約 29.4–29.5 Hz、odom 30 Hz、VO raw 有效率 100%、vo_state=1，表示「串流健康」與「幾何位姿正確」在本案例中明確分離。
- 兩次估計位移大小與估計 yaw 不符合固定旋轉半徑；不能以相機固定外參或 GUI 起點解釋。實機即時 topic 亦確認 `frame_id=odom`、`child_frame_id=camera_link`，且 `camera_link → camera_infra1_frame` 固定平移為 `[0, 0, 0]`。
- 結論更新：26 公分級位移不是實體移動，而是旋轉期間 cuVSLAM 的錯誤平移估計；第二筆同時顯示旋轉角低估，屬於 tracking / pose decomposition failure。

### 原始 rosbag 離線驗證

- `gui_20260826_093822_imu_mapping_light`：odom 3,260 筆、VO status 全部 `1`、IMU 21,699 筆。偵測到多段旋轉；其中 IMU 積分約 86.08° 時，odom yaw 約 87.65°，但旋轉段最大 XY 假平移達 0.656 m；另一段 IMU 約 61.43°、odom 約 62.26°，最大 XY 假平移 0.359 m。
- `gui_20260826_094108_imu_mapping_light`：odom 2,325 筆、VO status 全部 `1`、IMU 15,441 筆。第一段 IMU 積分約 62.67°、odom 約 63.45°，最大 XY 假平移 0.407 m；另一路段 IMU 積分約 75.40°、odom 約 76.53°，最大 XY 假平移 0.491 m。
- 這證明 IMU 確實量到旋轉，而且 cuVSLAM 的 yaw 多數可跟上；錯誤主要出現在旋轉時的平移分量，不是「真的走了 26 公分」，也不是單純 `vo_state` 失效。
- 輕量 bag 不含原始 IR image，因此可以確認 odom/IMU/status 的時序與幾何矛盾，但無法由本次 bag 回溯哪一幀影像的特徵匹配造成平移漂移。

### 動作分段更正

- 使用者確認第二個 bag 中最大約 `1.188 m` 的段落包含真實平移，因此該段不得列入「原地旋轉假平移」證據；它是合理的實際移動，不是 cuVSLAM 錯誤。
- 同理，離線工具看到的每個 yaw-rate 段只能表示「旋轉期間的估計 XY 變化」，不能自動宣告為假平移。只有在動作紀錄確認該段為原地旋轉時，才套用 ≤0.10 m / ≤0.20 m 門檻。
- 目前仍保留第一筆測試 `起點到目前 XY=0.402 m` 作為候選異常；第二個 bag 的 `0.407 m` 與 `0.491 m` 需先和實際動作時間對齊，不能在沒有動作標記下定案。
- 已修正 collector、離線分析工具與 GUI 的語意：旋轉段現在顯示「觀測值」及「原地旋轉候選」；不再把任何含平移的旋轉段自動記為 FAIL 或加入正式警告。

### 局部場景敏感性假設

- 使用者多次測試後觀察到：不是整段路線都容易失敗，而是固定場景區段特別容易出錯。
- 目前最合理的工作假設改為「局部視覺可觀測性不足」：該區段可能有低/重複紋理、遮擋、視角切換、反光或深度結構不足；暫不全域修改 IMU fusion、曝光或 odom 尺度。
- 下一次應對同一區段與正常區段各錄一段 `full` rosbag，逐幀比較 IR mean/std、clipped、gradient、左右影像時間差、VO 狀態與旋轉平移候選，才能把場景因素與 cuVSLAM 演算法因素分開。

### NVIDIA 官方處理方式對照（2026-08-26）

- NVIDIA 的 cuVSLAM 不是用固定比例或「把旋轉時的 XY 強制歸零」來修正局部錯位；核心仍是 2D 特徵、landmark 與 pose graph。
- 當影像因暗光、長直線/低紋理表面或嚴重 motion blur 而退化時，cuVSLAM 可短時間改由 IMU 維持姿態；官方說明這是短時緩衝，不是長時間替代視覺幾何。
- 重新看到已知 landmark 後，loop closure 與 pose-graph optimization 才能修正目前及歷史姿態；這屬於全域/後續修正，不能保證當下每一幀 odom 的局部平移立即正確。
- NVIDIA 對缺乏 distinctive features 的場景，方向是增加感測器方法的多樣性（例如多組 stereo camera）並保持影像輸入時序穩定，而不是靜默改 odom 尺度。
- 本專案對應措施：保留原始 IR 與完整 rosbag、比較正常/異常區段、檢查 observations/landmarks/pose graph；若同一區段仍反覆發生純旋轉候選異常，再加入補光、場景紋理改善或第二種定位來源，並維持現有 IMU fusion。

### 特殊場景完整記錄調整（2026-08-26）

- GUI 新增場景標籤與動作標籤，讓「正常場景」和「特殊場景」的相同動作可以直接對照。
- collector 現在保留整段 yaw 變化、累積 yaw、完整轉向片段歷史與每段時間戳；GUI 不再把最後一個短片段的 yaw Δ 誤當成整段旋轉角度。
- 每次 R2 執行會保存不截斷的 JSONL trace，包含 odom telemetry、影像品質快照與接收時間；停止 R2 時另保存 session JSON，包含完整 XY 路徑、場景/動作標籤、轉向歷史、品質評估與工作記錄。
- 輕量與完整 rosbag 均加入 observations、PoseGraph nodes/edges 與 loop-closure cloud topic；完整模式仍額外包含左右原始 IR 影像。

### 朝向顯示、stereo 同步與官方處理界線（2026-08-26）

- 原 GUI 藍線只畫 `x/y`，沒有畫 pose orientation，因此摘要的 `yaw=74.9°` 不會自然顯示成藍線轉了 74.9°；兩者不是互相矛盾，而是顯示了不同狀態量。
- GUI 已增加起點與目前朝向箭頭：紫色為起始 yaw、橘色為目前 yaw；藍線仍保留為原始 XY 平移，不對 cuVSLAM odom 做平滑、縮放或旋轉時強制歸零。
- 新增左右 IR 影像 header timestamp 配對，記錄 stereo offset 的 latest/P95/max（µs）。原本的 `image gap P95≈33.3 ms` 只是單一串流的 30 Hz 幀週期，不能證明左右 stereo 同步。
- GUI、品質分數、摘要與 JSONL trace 已加入「左右同步 P95」；P95 ≤100 µs 為 PASS、≤500 µs 為 REVIEW、>500 µs 為 FAIL，超過 100 µs 同時標記 `stereo:desync`。
- GUI 新增「特徵診斷」啟動選項：`standard` 維持正式建圖的較低負載，`features` 才開啟 cuVSLAM `enable_observations_view`，讓特殊場景 bag 實際取得 observations cloud。先前錄製清單雖含該 topic，但 launch 關閉 publisher 時可能沒有資料；GUI 說明已同步修正。
- NVIDIA 官方說明指出：IMU fusion 主要補償短時間 motion blur／曝光瞬變，不會提升低特徵 stereo 影像本身的追蹤品質。缺乏 distinctive features 時，官方方向是增加相機視角（multi-camera），或以 wheel+IMU odometry 提供另一種運動估計；不是用參數把旋轉期間 XY 直接歸零。
- 本專案的處理順序：先驗證左右同步、原始 rectified IR、30 Hz 與 IMU；再以特殊場景 full bag 對齊影像、IMU、odom 與特徵觀測。若同步合格但同一場景仍穩定重現，判定為單一 D435i 視覺可觀測性限制，導航端需增加 wheel/IMU 或第二視角，不把異常 odom 直接當可靠真值。
- 2026-08-26 本機修改已通過 Python 與 shell syntax 檢查；同步 Jetson 時 `192.168.55.1:22` 逾時，Mac 亦未出現 `192.168.55.x` USB 網路介面。Jetson 重新建立 USB 網路後仍需同步 `tools/collect_real_d435i_image_quality.py` 與 `tools/run_real_d435i_r2.sh`，再重啟 R2 才會產生 stereo sync 指標與 features observations。

### Jetson 部署與特殊場景重測（2026-08-26 16:42）

- USB 網路已恢復：Mac `192.168.55.100`，Jetson `192.168.55.1`；SSH 與 D435i `8086:0b3a` 枚舉確認正常。
- 部署前已備份 Jetson 版本為 `collect_real_d435i_image_quality.py.bak_20260826_stereo_sync` 與 `run_real_d435i_r2.sh.bak_20260826_feature_diag`。
- 部署後 SHA-256 與本機一致：collector `ebedc64c...eb16`，R2 script `5b85fb85...7b4c`。
- 修正 GUI 啟動初期 `run_start_yaw_deg=null` 造成的解析錯誤；修正左右 callback 邊界幀被誤算成 33 ms stereo max 的問題，另外保留 unmatched ratio。
- 實測靜止輸入：stereo sync P95/max `0/0 µs`，unmatched 約 `0.42%`；odom 約 30 Hz、VO raw 100%、影像 clipped 約 2%，目前輸入健康。
- `features` 模式實測 `/visual_slam/vis/observations_cloud` 約 30 Hz，確認 publisher 已真正啟用。
- 曾開始完整特殊場景錄製：`gui_20260826_164511_imu_mapping_special_turn_scene_forward_left_turn_full`；使用者隨後取消本次測試，未執行指定移動，因此本 bag 只視為靜止初始化資料，不用於假平移結論。
- GUI 已送出停止錄製與停止 R2，並保存本機 session：`20260826_165022_r2_session.json`。停止當下 Jetson `192.168.55.1` USB SSH、`172.20.10.2` Wi-Fi SSH 與 ping 均無回應，因此遠端 rosbag/container 清理狀態尚待重新連線後確認；不得標記為已完成遠端清理。
