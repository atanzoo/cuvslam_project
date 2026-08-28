# IMM 動態障礙物預測訓練研究日誌

日期：2026-08-07  
狀態：準備開始 IMM 單獨訓練  
研究目標：先提升動態障礙物軌跡預測，再接回 PPO／MPPI

## 1. 研究決策

目前先把問題拆成兩個獨立模組：

```text
LiDAR track history
        ↓
IMM / motion predictor       ← 這一階段先訓練與評估
        ↓
預測軌跡、turn rate、不確定性
        ↓
PPO 高層決策
        ↓
MPPI 局部控制
```

這一階段固定 PPO、MPPI、SimulatedD1SDK、goal 與 global path，不把 PPO reward 混進 IMM 的訓練目標。這樣可以清楚回答：

> collision 改善是否真的來自更好的障礙物預測？

## 2. 目前 IMM 實作狀態

現有程式：

- [d1_edu_tracker_models.py](/Users/tsengpochien/Desktop/cuvslam_project/tools/d1_edu_tracker_models.py)
- [run_d1_edu_decision_layer_viewer.py](/Users/tsengpochien/Desktop/cuvslam_project/tools/run_d1_edu_decision_layer_viewer.py)
- [d1_edu_scenario.py](/Users/tsengpochien/Desktop/cuvslam_project/tools/d1_edu_scenario.py)

目前是 heuristic IMM-style predictor，不是神經網路：

- CV：Constant Velocity
- CA：Constant Acceleration
- CTRV：Constant Turn Rate and Velocity
- model weight 依 acceleration evidence、turn evidence、history length 手動計算。
- `turn_rate` 由速度與加速度的 cross product 估算。
- 不確定性以 scalar uncertainty tube 表示。

目前的 IMM 會輸出：

- fused future trajectory
- uncertainty radius
- CV／CA／CTRV model weights
- estimated turn rate

## 3. 為什麼現在必須強化 IMM

500k PPO fixed-100 評估結果：

| 指標 | 結果 |
|---|---:|
| PPO success | 30% |
| PPO physical collision | 23% |
| PPO timeout | 47% |
| PPO real deadlock | 0% |

修正版 30-trial route graph 中的 7 個 physical-contact cases：

```text
trial 2, 3, 4, 16, 20, 21, 26
```

trace 分析結果：

- 7/7 接觸障礙物的 `motion_kind` 都是 `turning`。
- 5/7 在碰撞前一個 decision sample 為 `predicted_risk=false`。
- 2/7 已經進入 AVOID，但 MPPI 仍穿過障礙物的轉彎掃掠區域。
- 7/7 的 `waitable=0`，因此不是 WAIT_YIELD 主要造成的問題。

目前的主要假設：

1. IMM 太晚辨識 turning motion，導致 PPO 沒有提前收到風險訊號。
2. 融合均值可能把圓弧軌跡拉直，造成轉彎障礙物預測落在錯誤位置。
3. uncertainty tube 目前主要交給 MPPI，Decision Layer 使用 fused mean 加固定 margin，早期風險可能不足。
4. PPO observation 沒有直接包含 turn rate、CTRV weight、arc-intersection time 等特徵。

## 4. IMM 訓練目標

第一階段不直接訓練端到端神經網路，而是先校正／學習 IMM 的預測參數：

- model mode probability：CV／CA／CTRV
- turn rate estimation
- model transition／weight calibration
- position uncertainty calibration
- velocity uncertainty calibration
- turning trajectory residual correction

若傳統 IMM 校正後仍不足，再加入小型 residual predictor；暫不直接用 LSTM／Transformer 取代整個 tracker。

## 5. 訓練資料定義

資料來源使用現有 MuJoCo ground truth 與 LiDAR simulation：

### Input

每個目標的過去 track history：

- 最近 N 個時間點的位置
- 最近 N 個時間點的速度
- acceleration estimate
- timestamp / dt
- tracker confidence
- LiDAR noise
- dropout pattern

### Label

- 真實 motion family：static、crossing、same_direction、accelerating、turning、irregular
- 未來 0.5、1、2、3 秒的真實位置
- 未來速度與 acceleration
- 真實 turn rate
- 真實與機器人 path 的 intersection time
- 未來最小 clearance

重要限制：ground truth 只可用於訓練與評估，部署時預測器只能使用 LiDAR track history。

## 6. 資料切分

不能用同一個 episode 的相鄰片段同時放進 train 與 test，避免時間洩漏。建議：

- Train：70% seeds
- Validation：15% seeds
- Test：15% 完全未見過的 seeds

每個 split 都要包含 turning、non-turning、people、object、noise／dropout，以及不同 radius、turn rate、phase。turning case 必須額外分層抽樣，避免被 static 或 straight-line samples 淹沒。

## 7. 必須先建立的程式

下一個 IMM 對話預計新增：

```text
tools/generate_d1_edu_imm_dataset.py
tools/train_d1_edu_imm.py
tools/evaluate_d1_edu_imm.py
tools/test_d1_edu_imm_training.py
```

建議輸出：

```text
output/d1_imm_dataset.npz
output/d1_imm_model.json
output/d1_imm_predictions_fixed100.json
output/d1_imm_training_summary.json
```

若使用 neural residual predictor，再另外輸出：

```text
output/d1_imm_residual_predictor.zip
```

## 8. IMM 評估指標

不能只看平均位置誤差，至少要包含：

| 指標 | 意義 |
|---|---|
| ADE | 整段預測平均位移誤差 |
| FDE | 預測終點位置誤差 |
| turn classification recall | 是否辨識 turning |
| turn rate MAE | 角速度估計誤差 |
| uncertainty coverage | 真實軌跡落在 tube 內的比例 |
| collision-risk recall | 危險案例是否提前預測 |
| false alarm rate | 是否過度把安全目標判成危險 |
| warning lead time | 距離實際碰撞前多久發出警告 |
| arc-intersection recall | 是否預測到未來圓弧與 path 相交 |

關鍵 acceptance gate：

1. turning recall 必須高於目前 heuristic baseline。
2. collision-risk recall 必須提升，不能只降低 ADE。
3. uncertainty coverage 不可透過無限制放大 tube 作弊。
4. false alarm 不可讓所有目標都被判成危險。

## 9. 接回 PPO／MPPI 的順序

IMM 訓練完成後做三組 ablation：

```text
A. 舊 IMM + 舊 PPO + 舊 MPPI
B. 新 IMM + 舊 PPO + 舊 MPPI
C. 新 IMM + 新 observation PPO + 舊 MPPI
```

若要讓 MPPI 使用新的 swept-arc prediction，再增加：

```text
D. 新 IMM + 新 PPO + swept-arc MPPI
```

解讀方式：

- A→B：純預測改善效果。
- B→C：PPO 是否能利用新預測特徵。
- C→D：MPPI 是否能把預測轉成實際安全軌跡。

如果新增 observation 欄位，500k PPO 不能直接作為最終模型，必須重新訓練或明確做 fine-tune。

## 10. 新對話開場提示

下一個對話可以直接貼上：

```text
請延續 /Users/tsengpochien/Desktop/cuvslam_project/docs/IMM_TRAINING_HANDOFF_2026-08-07.md。

目前目標是單獨訓練與評估 IMM，不要先改 PPO、MPPI 或 SimulatedD1SDK。
請先檢查現有 d1_edu_tracker_models.py、d1_edu_scenario.py 與 500k turning collision trace，
建立 turning-balanced 的 IMM dataset，新增 dataset generator、training script、evaluation script
與測試。先比較 heuristic IMM，再做可學習的 model weight／turn-rate calibration。
最後用 ADE、FDE、turn recall、collision-risk recall、uncertainty coverage 和 warning lead time 評估。
```

## 11. 目前不要做的事

- 不要先把 PPO 訓練到 1M。
- 不要把 ground-truth future trajectory 當成部署時 observation。
- 不要同時改 IMM、PPO reward、MPPI cost，否則無法做因果歸因。
- 不要只用 ADE 變好就宣稱避障變好。
- 不要直接把 uncertainty tube 無限制放大來降低 collision。

## 12. 目前結論

目前最合理的研究順序是：

```text
先單獨提升 IMM turning prediction
        ↓
確認 collision-risk recall 與 warning lead time 改善
        ↓
把新預測特徵接入 PPO
        ↓
重新訓練 PPO
        ↓
再讓 MPPI 使用 swept-arc uncertainty
```

本日誌可作為下一個 IMM 對話的唯一 handoff 文件。
