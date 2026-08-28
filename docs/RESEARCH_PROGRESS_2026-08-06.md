# D1 Edu 動態避障研究進度

更新時間：2026-08-06 17:18 CST  
研究主題：Nav2-style 全域路徑、PPO 高層決策、MPPI 局部控制與動態障礙物避障

## 1. 今日結論

目前已完成一套可重複訓練與評估的 D1 Edu 動態避障模擬流程。系統能在 10 m 路線中處理兩名行人與一個一般物體，障礙物可採用靜止、穿越、同向、加速、轉彎及不規則運動。

目前最重要的研究結論如下：

- `WAIT_YIELD` 在專門課程情境中已學會「等待動態障礙通過再繼續」，30 次測試成功 26 次，成功率 86.7%。
- 換到完整隨機化場景後，PPO 相較 baseline 將 timeout 從 66% 降到 55%，成功率從 22% 提升到 29%，但 collision 從 12% 增加到 16%。
- PPO 已有改善決策的效果，但尚未充分泛化；碰撞與終點收斂不只由 PPO 決定，MPPI、路徑追蹤和終端控制仍是主要瓶頸。
- 現在不適合繼續頻繁修改架構。下一階段固定 observation、action、MPPI 與 reward，依序訓練 300k、500k、1M，並使用同一組 100 seeds 比較 learning curve。

## 2. 目前系統架構

```text
Nav2-style 全域參考路徑
          ↓
PPO 高層決策
CRUISE / AVOID_LEFT / AVOID_RIGHT / SLOWDOWN / WAIT_YIELD
          ↓
MPPI 連續局部速度最佳化
          ↓
SimulatedD1SDK
速度上限 / 加速度上限 / 延遲 / 一階響應
          ↓
MuJoCo D1 Edu kinematic base update
```

環境感知與預測流程：

```text
2D LiDAR simulation
          ↓
MultiTargetTracker
          ↓
IMM trajectory prediction: CV + CA + CTRV
          ↓
PPO observation 與 MPPI uncertainty tube
```

目前的全域路徑是 Mac 模擬端的 Nav2-style 參考路徑，不是 ROS 2 Nav2 process。這讓 PPO 可以先在 Mac 訓練，之後再將參考路徑來源替換成 Jetson 上的 Nav2，而不改寫 PPO→MPPI 的主要介面。

## 3. 評估定義

### 3.1 成功

Episode 只有同時符合下列條件才算成功：

1. 沒有 MuJoCo robot–obstacle physical contact。
2. 在 duration 內進入 goal tolerance。
3. 沒有 timeout。
4. 滿足實驗設定的最低安全淨空要求。

「停止但沒撞」不算成功；「走得很近」也不能只因 proxy 圓重疊就直接算物理碰撞。

### 3.2 碰撞

- Physical collision：以 MuJoCo contact 為 ground truth。
- Geometric overlap：精確幾何重疊，作為診斷資訊。
- Proxy clearance：外接圓快速近似，只作風險與距離診斷，不取代物理碰撞。
- Planning risk：Decision Layer／MPPI 對未來碰撞的預測，用於控制，不是事後碰撞真值。

目前 footprint 估計：

- planner radius：0.3359 m
- base extent：0.3359 m
- articulated extent：0.4199 m

### 3.3 Timeout 與 deadlock

- Timeout：在 duration 內未抵達 goal；可能仍有進展。
- Near-goal timeout：時間到時已接近 goal，但尚未進入 tolerance。
- Real deadlock：最近一段時間幾乎沒有有效位移或路徑進展。

因此 timeout 是效率指標，不等同於 deadlock；兩者會分開報告。

## 4. 已完成的模型與正式結果

### 4.1 WAIT_YIELD curriculum 模型

模型：`output/d1_wait_yield_curriculum_30k_v3.zip`  
軌跡圖：`output/d1_wait_yield_curriculum_30k_v3_gated_routes.html`  
數據：`output/d1_wait_yield_curriculum_30k_v3_gated_routes.json`

| 指標 | 結果 |
|---|---:|
| Trials | 30 |
| Success | 26 / 30（86.7%） |
| Physical collision | 2 / 30（6.7%） |
| Timeout | 2 / 30（6.7%） |
| Mean min clearance | 0.463 m |
| WAIT argmax in waitable states | 63 / 64（98.4%） |
| Invalid WAIT | 0 |
| Near-goal timeout | 0 |
| Real deadlock | 0 |

這證明 `WAIT_YIELD` 動作在有明確等待語義的 curriculum 中可以被 PPO 學會。

### 4.2 隨機行人／物體 70k fine-tune

模型：`output/d1_wait_yield_people_random_70k_v6.zip`  
摘要：`output/d1_wait_yield_people_random_70k_v6_imm_footprint_split_fixed100_summary.json`

測試條件：100 個固定 seeds、10 m 路線、50 s、三個動態目標、MPPI 32 samples × 30 horizon。

| 指標 | Baseline | PPO |
|---|---:|---:|
| Success | 22% | 29% |
| Physical collision | 12% | 16% |
| Timeout | 66% | 55% |
| Mean min clearance | — | 0.730 m |
| Mean path length | — | 8.212 m |

PPO 動作比例：

| 動作 | 比例 |
|---|---:|
| CRUISE | 63.47% |
| AVOID_LEFT | 9.80% |
| AVOID_RIGHT | 16.84% |
| SLOWDOWN | 0.00% |
| WAIT_YIELD | 9.88% |

WAIT 診斷：

- waitable states：4,730
- valid WAIT：4,615
- invalid WAIT：36（0.77%）
- WAIT 後出現進展：98.63%
- WAIT 後返回 CRUISE：39.68%
- near-goal timeout：9%
- real deadlock：5%

解讀：PPO 已降低 timeout，但 collision 上升，且 `SLOWDOWN` 未被使用。這是目前 reward 與訓練資料分布仍不平衡的直接證據。

### 4.3 Oracle／MPPI 診斷上限

程式：`tools/run_d1_edu_oracle_benchmark.py`  
摘要：`output/d1_oracle_mppi_fixed100_summary.json`

| 指標 | Oracle + MPPI |
|---|---:|
| Success | 34% |
| Physical collision | 24% |
| Timeout | 42% |
| Mean min clearance | 0.493 m |
| Mean path length | 8.727 m |

Oracle 使用模擬器中的真實障礙物未來狀態，只用於診斷，不是可部署策略。相同 100 seeds 中，baseline、既有 PPO 與 oracle 三者成功集合的聯集為 44%；仍有 56% 三者全部失敗。這表示：

- PPO 尚有高層決策改善空間。
- 只增加訓練量不能保證解決全部問題。
- MPPI 可行路徑、速度執行、終點收斂及場景難度仍限制整體成功率。

20-seed 診斷中把 MPPI 從 32×30 提高到 96×45，成功率沒有進一步上升，顯示單純提高 rollout 解析度不是主要解方。

## 5. 今日新增與修正

### 5.1 多模型目標預測

- 新增 `tools/d1_edu_tracker_models.py`。
- IMM 使用 CV、CA、CTRV 三種運動模型。
- 預測 uncertainty tube 提供 MPPI 使用。
- Decision Layer 使用融合後均值與既有安全 margin，避免風險重複膨脹。

### 5.2 機器人 footprint 與碰撞分層

- 新增 `tools/d1_edu_footprint.py`。
- 將 physical contact、geometric overlap、proxy clearance 分開記錄。
- 物理碰撞由 MuJoCo contact 判定，proxy 不再冒充 ground truth。

### 5.3 Oracle 診斷工具

- 新增 `tools/d1_edu_oracle_policy.py`。
- 新增 `tools/run_d1_edu_oracle_benchmark.py`。
- viewer 可接受 `act_with_context` policy，並允許可信 ground-truth oracle 繞過 scan shield／gate，以測量決策上限。

### 5.4 平衡隨機訓練場景

- 新增 `people_balanced_multi_target` 場景模式。
- 以六個 episode 為一個循環，讓三個物理目標都輪流經歷：static、crossing、same_direction、accelerating、turning、irregular。
- 下一個六 episode 將側向配置鏡射，降低 PPO 學成固定向左或向右避障的偏差。

### 5.5 Reward 修正

- Physical collision penalty：`-20`。
- Timeout penalty：`-4`。
- `WAIT_YIELD` 不再因單純離開 WAIT 就取得 clear bonus。
- 只有在離開 WAIT 後 1.5 s 內真的產生 route／goal progress，才補發 WAIT recovery bonus。

### 5.6 Checkpoint 與長訓練

- SB3 訓練器新增 `--checkpoint-freq` 與 `--checkpoint-dir`。
- 已驗證定期 checkpoint 可以正常保存。
- 第一個無 checkpoint 的約 23.8k 診斷訓練因 entropy 過早下降而主動停止，不列入正式比較。

## 6. Balanced-safe 正式訓練

300k 模型：`output/d1_ppo_balanced_safe_300k.zip`

```bash
./.venv/bin/python tools/run_d1_edu_decision_layer_sb3.py \
  --timesteps 300000 \
  --eval-trials 1 \
  --profile full_randomization \
  --scenario-mode people_balanced_multi_target \
  --seed 42 \
  --route-length 10 \
  --train-duration 50 \
  --eval-duration 50 \
  --mppi-samples 32 \
  --mppi-horizon 30 \
  --collision-penalty 20 \
  --timeout-penalty 4 \
  --policy output/d1_ppo_balanced_safe_300k \
  --summary output/d1_ppo_balanced_safe_300k_train_summary.json \
  --tensorboard-log output/sb3_tensorboard_balanced_safe \
  --checkpoint-freq 25000 \
  --checkpoint-dir output/d1_ppo_balanced_safe_checkpoints
```

300k 已於 17:45 完成，共 300,032 steps。完整 checkpoint 每 25k 保存一次，正式 fixed-100 結果如下：

| 指標 | Baseline | PPO 300k |
|---|---:|---:|
| Success | 19% | 23% |
| Physical collision | 23% | 30% |
| Timeout | 58% | 47% |

PPO 降低 timeout 並小幅提高 success，但 physical collision 上升，因此尚不符合部署安全要求。詳細分析見 `docs/PPO_BALANCED_SAFE_300K_SUMMARY.md`。

已依固定架構從 300k 繼續訓練至 500k。相同 fixed-100 評估結果：

| 指標 | Baseline | PPO 300k | PPO 500k |
|---|---:|---:|---:|
| Success | 19% | 23% | 30% |
| Physical collision | 23% | 30% | 23% |
| Timeout | 58% | 47% | 47% |
| Real deadlock | — | 3% | 0% |

500k 是目前最佳候選：collision 已降回 baseline、success 最高、real deadlock 為 0%；但 AVOID_RIGHT 幾乎不使用、SLOWDOWN 仍為 0%，且 near-goal timeout 上升到 15%。因此下一步先做失敗路徑與動作覆蓋診斷，再決定是否訓練 1M。

模型：`output/d1_ppo_balanced_safe_500k.zip`。詳細比較見 `docs/PPO_BALANCED_SAFE_300K_SUMMARY.md`。

另修正了 500k 路徑圖生成器：原先漏傳 balanced scenario 的
`curriculum_index`，造成圖中的障礙物運動配置與正式 benchmark 不一致。
修正後，`output/d1_ppo_balanced_safe_500k_routes.html` 的前 30 trials
與 fixed-100 benchmark 對齊，包含 7 個 physical-contact cases。

對 7 個 contact trace 的分析顯示，接觸障礙物全部是 `motion_kind=turning`。
其中 5 個在接觸前一個 decision sample 仍為 `predicted_risk=false`，另 2 個
雖已進入 AVOID 仍碰撞。因此下一步應分成 turning trajectory 漏預測與 MPPI
局部避障失敗兩條線診斷；WAIT_YIELD 不是這批 collision 的主要因素。

## 7. 已通過的驗證

- Python syntax／`py_compile`：通過。
- `tools/test_d1_edu_scenario.py`：通過，包含 balanced motion cycle 與 mirror 測試。
- `tools/test_d1_edu_decision_benchmark.py`：通過。
- `tools/test_d1_edu_tracker_models.py`：通過。
- `tools/test_d1_edu_footprint.py`：通過。
- `tools/test_d1_edu_oracle_policy.py`：通過。
- Checkpoint smoke test：成功保存 256 與 512 steps checkpoint。

## 8. 下一步實驗計畫

固定目前架構，不再於 learning curve 中途改 observation、action、reward 或 MPPI：

1. 完成 balanced-safe 300k 訓練。
2. 使用固定 100 seeds 評估 baseline 與 300k PPO。
3. 從 300k 模型續訓到 500k，再跑相同 100 seeds。
4. 從 500k 模型續訓到 1M，再跑相同 100 seeds。
5. 畫出 success、physical collision、timeout、real deadlock、mean clearance 與 action distribution learning curve。
6. 若 300k→500k→1M 已飽和，再分別處理：
   - `SLOWDOWN` 動作未被使用。
   - oncoming／crossing 場景的 physical collision。
   - near-goal oscillation 與 terminal convergence。
   - MPPI 可行解與全域路徑回歸能力。

固定評估原則：

- 相同 100 seeds。
- 相同 route length、duration、MPPI samples/horizon。
- 同時報 success、collision、timeout，不只看單一成功率。
- 保留每個 trial 的 seed、route trace 與 contact 資訊。

## 9. 目前模擬限制

- D1 Edu 仍採 kinematic base update，不是 12 關節 torque-level locomotion。
- Mac 端為 Nav2-style 參考路徑，尚未啟動完整 ROS 2 Nav2 stack。
- LiDAR、tracker 與障礙物行為仍是模擬模型，尚未包含真實人群互動、遮擋、反射與硬體時間同步誤差。
- Oracle 知道模擬器真實未來，不能部署到實機，也不能當作 PPO policy。
- Random scenario 的成功上限仍受 MPPI、SDK 動態限制與 goal terminal controller 影響。
- 目前尚未完成 sim-to-real domain randomization 與 Jetson 延遲量測校正。

## 10. Git 收錄建議

目前 `/Users/tsengpochien/Desktop/cuvslam_project` 尚未偵測到頂層 `.git` repository。日後初始化或搬入 Git 時，建議：

應提交：

- `docs/RESEARCH_PROGRESS_2026-08-06.md`
- `tools/` 中的演算法、訓練、benchmark、測試與視覺化程式
- 小型 summary JSON 與固定 seeds 清單
- 可重現 CLI 與 dependency lock／requirements

不應直接提交：

- `.venv/`
- TensorBoard event logs
- 大量 checkpoint `.zip`
- 大型 route HTML／逐步 trace JSON
- 臨時截圖與系統暫存檔

模型建議使用 Git LFS、GitHub Release 或外部 artifact storage，只在 Git 中保存模型名稱、SHA-256、訓練設定與對應 benchmark summary。

建議第一批 Git 歷史拆成三個提交：

1. `feat: add D1 dynamic-obstacle benchmark and simulated SDK`
2. `feat: add PPO wait-yield policy, IMM tracker, and physical contact metrics`
3. `docs: record 2026-08-06 research results and reproducible training plan`

## 11. 明日接續檢查清單

```bash
# 1. 確認 300k 模型與 summary 是否完成
ls -lh output/d1_ppo_balanced_safe_300k.zip \
       output/d1_ppo_balanced_safe_300k_train_summary.json

# 2. 查看 checkpoints
ls -lh output/d1_ppo_balanced_safe_checkpoints/

# 3. 若正式訓練完成，先跑固定 100 seeds 評估，再續訓 500k
# 不要在完成 300k 評估前改 reward 或場景生成器。
```

下一個可正式回答的研究問題是：

> 在固定架構、平衡 motion curriculum 與較高 physical collision penalty 下，PPO 從 300k 增加到 500k、1M，能否持續提高成功率，同時不增加 MuJoCo physical collision？
