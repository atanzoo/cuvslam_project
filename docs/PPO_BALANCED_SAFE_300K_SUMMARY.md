# Balanced-safe PPO 300k 模型總結

評估日期：2026-08-06  
模型：`output/d1_ppo_balanced_safe_300k.zip`  
正式摘要：`output/d1_ppo_balanced_safe_300k_fixed100_summary.json`

## 1. 結論先行

這個 300k PPO 是一個「效率提高、但安全性退步」的模型，尚不適合部署：

- 成功率由 baseline 的 19% 提高到 23%，增加 4 個百分點。
- Timeout 由 58% 降到 47%，減少 11 個百分點。
- MuJoCo physical collision 由 23% 增加到 30%，惡化 7 個百分點。
- PPO 學會在所有 waitable states 選擇 `WAIT_YIELD`，但也在大量不適合等待的狀態選擇 WAIT。
- `SLOWDOWN` 使用率仍為 0%，顯示五動作策略尚未完整學會。

因此，這次訓練證明平衡場景與較低 timeout penalty 會讓策略更願意完成任務，但 `collision_penalty=-20` 尚未轉化成足夠保守的實際控制行為。不能只看 success 或 timeout；安全指標不合格。

## 2. 模型與控制架構

```text
Nav2-style global path/reference
                 ↓
SB3 PPO high-level policy
                 ↓
CRUISE / AVOID_LEFT / AVOID_RIGHT / SLOWDOWN / WAIT_YIELD
                 ↓
MPPI 32 samples × 30 horizon
                 ↓
SimulatedD1SDK
                 ↓
MuJoCo kinematic D1 execution
```

PPO 不直接輸出關節、力矩或連續速度。它選擇高層模式，MPPI 再依據全域路徑、障礙物預測和該模式產生連續局部控制命令。

### Observation

Observation 共 52 維：

- 4 維 robot／goal 狀態。
- 3 個目標 × 10 維追蹤與風險特徵。
- 2 維整體 risk／elapsed 特徵。
- 7 維全域路徑參考：lateral error、heading error、remaining fraction、lookahead lateral、progress、reference speed、terminal-path risk。
- 9 維 planner wait/yield 特徵，包含 waitable 與 stationary collision risk。

### PPO 超參數

| 參數 | 設定 |
|---|---:|
| Algorithm | Stable-Baselines3 PPO |
| Policy | MlpPolicy |
| Timesteps | 300,032 |
| `n_steps` | 256 |
| Batch size | 64 |
| Learning rate | 3e-4 |
| Gamma | 0.99 |
| GAE lambda | 0.95 |
| Entropy coefficient | 0.01 |
| Seed | 42 |

### Reward 主要設定

- Goal terminal reward：`+5`。
- Physical collision terminal penalty：`-20`。
- Timeout penalty：`-4`。
- 提供 goal progress 與 global-path progress 正獎勵。
- 懲罰 lateral error、每步時間成本、模式頻繁切換與不必要避障。
- WAIT clear bonus 延後到離開 WAIT 後 1.5 s 內真的出現路徑／目標進展才發放。

## 3. 訓練資料

場景模式：`people_balanced_multi_target`。

- 路線長度：10 m。
- Episode duration：50 s。
- 每個 episode：兩名行人與一個一般物體。
- 運動族群：static、crossing、same_direction、accelerating、turning、irregular。
- 六個 episode 為一個 motion cycle；後續循環鏡射左右配置。
- Profile：`full_randomization`，包含感測雜訊、dropout、command delay、速度／加速度與尺寸變化。

這個場景比舊的 `people_random_multi_target` 更強調運動類別與左右方向平衡。因此 baseline collision 也由舊測試的 12% 上升到本次的 23%；跨場景數字不可直接當成模型退步，正式比較必須使用本次同一批 seeds 的 baseline。

## 4. 固定 100-seed 正式結果

| 指標 | Baseline | PPO 300k | 差異 |
|---|---:|---:|---:|
| Success | 19% | 23% | +4 pp |
| Physical collision | 23% | 30% | +7 pp（較差） |
| Timeout | 58% | 47% | −11 pp |
| Mean PPO min clearance | — | 0.522 m | — |
| Mean PPO path length | — | 7.918 m | — |
| Near-goal timeout | — | 13% | — |
| Real deadlock | — | 3% | — |

Success、collision、timeout 合計為 100%，所以 PPO 的主要轉換是：把部分 timeout 轉成 success，但也把另一部分 timeout／安全通行轉成 physical collision。

## 5. PPO 實際學到的行為

| 動作 | 次數 | 比例 |
|---|---:|---:|
| CRUISE | 29,914 | 67.33% |
| AVOID_LEFT | 4,049 | 9.11% |
| AVOID_RIGHT | 5,167 | 11.63% |
| SLOWDOWN | 0 | 0.00% |
| WAIT_YIELD | 5,302 | 11.93% |

正面結果：

- 左右避障都會使用，沒有退化成只往固定一側繞行。
- 3,423 個 waitable states 全部選到 WAIT，wait argmax rate 為 100%。
- WAIT sequence 後有進展的比例為 98.78%。
- Real deadlock 只有 3%，顯示模型不是主要靠永久停車避免碰撞。

主要問題：

- 5,302 次 WAIT 中有 1,879 次發生於非 waitable 狀態，invalid WAIT rate 為 35.44%。
- WAIT 後返回 CRUISE 的比例只有 47.36%，WAIT 的恢復語義仍不穩定。
- `SLOWDOWN` 完全沒有被策略採用，該動作目前是 dead action。
- Collision 30% 表示高 collision penalty 並未充分影響最終物理安全；可能受到 delayed credit、MPPI 執行能力、action shield 與 observation 可辨識性共同限制。
- Near-goal timeout 13%，終點附近的回歸與減速仍然不足。

## 6. 訓練曲線解讀

TensorBoard 的 rollout mean reward 並非單調上升：

| Steps | Mean episode reward | Mean episode length | Entropy magnitude |
|---:|---:|---:|---:|
| 25k | 2.22 | 447.80 | 0.40 |
| 75k | 11.20 | 436.94 | 0.28 |
| 150k | 12.83 | 440.74 | 0.43 |
| 225k | 11.15 | 436.46 | 0.46 |
| 250k | 8.57 | 426.01 | 0.31 |
| 300k | 6.81 | 439.33 | 0.35 |

300k 的 reward 低於 150k 與 225k，代表：

- PPO 後段訓練可能出現策略漂移或 curriculum distribution 下的局部退化。
- 最後 checkpoint 不一定是最佳 checkpoint。
- Learning curve 必須用固定外部 benchmark 評估 150k、225k、300k、500k、1M，而不能只看 TensorBoard reward。

## 7. 為何碰撞仍然偏高

目前最可能的原因依優先度排列：

1. `WAIT_YIELD` 過度泛化：非 waitable 狀態仍大量等待，等到危險變成 imminent risk 才恢復，留給 MPPI 的反應時間不足。
2. `SLOWDOWN` 沒有被使用：策略缺少介於 CRUISE、WAIT 與大幅 AVOID 之間的柔性速度調整。
3. 高層 action 每次只改變 MPPI 的行為偏好；即使 PPO 選對類別，MPPI 仍可能無法在 horizon、速度限制與 command delay 下執行安全軌跡。
4. Terminal collision penalty 是延遲回饋。在長 episode 中，它可能不足以區分真正導致碰撞的早期決策。
5. Balanced scenario 增加 crossing、turning 與 irregular 的覆蓋，但 300k 尚不足以讓策略穩定辨識全部組合。

## 8. 目前判定

| 問題 | 判定 |
|---|---|
| PPO 是否比 baseline 更容易完成任務？ | 小幅是，成功率 +4 pp。 |
| PPO 是否降低 timeout？ | 是，−11 pp。 |
| PPO 是否更安全？ | 否，physical collision +7 pp。 |
| WAIT_YIELD 是否學會？ | 部分學會，但過度泛化。 |
| SLOWDOWN 是否學會？ | 否。 |
| 是否可上實機？ | 否，30% physical collision 不可接受。 |
| 是否值得繼續訓練？ | 值得，但必須保留 checkpoints 並以固定 seeds 選模。 |

## 9. 500k 續訓結果

500k 模型：`output/d1_ppo_balanced_safe_500k.zip`  
正式摘要：`output/d1_ppo_balanced_safe_500k_fixed100_summary.json`

使用完全相同的 100 seeds、場景、duration 與 MPPI 設定：

| 指標 | Baseline | PPO 300k | PPO 500k |
|---|---:|---:|---:|
| Success | 19% | 23% | 30% |
| Physical collision | 23% | 30% | 23% |
| Timeout | 58% | 47% | 47% |
| Mean min clearance | — | 0.522 m | 0.516 m |
| Mean path length | — | 7.918 m | 8.445 m |
| Real deadlock | — | 3% | 0% |
| Near-goal timeout | — | 13% | 15% |

500k 是目前較好的 checkpoint：成功率比 300k 再增加 7 個百分點，collision 降回 baseline 水準，real deadlock 降為 0%。但它還不是完整安全解，因為 near-goal timeout 由 13% 上升到 15%。

500k 的動作分布也出現新的警訊：

- CRUISE：82.15%。
- AVOID_LEFT：10.93%。
- AVOID_RIGHT：0.13%。
- SLOWDOWN：0%。
- WAIT_YIELD：6.79%。

相較 300k，`AVOID_RIGHT` 幾乎消失，表示模型可能重新產生左右方向偏差；`SLOWDOWN` 仍然是 dead action。WAIT 則改善明顯：invalid WAIT 為 0%，WAIT 後產生進展比例為 99.64%，返回 CRUISE 比例為 66.53%。

因此 500k 的結論是：

> 繼續訓練確實改善了 collision 與 deadlock，但沒有解決動作覆蓋不平衡與終點附近效率問題。

### 路徑圖一致性修正

初版 500k 路徑圖沒有顯示 collision，原因是 graph generator 沒有把
`people_balanced_multi_target` 的 `curriculum_index` 傳給 environment。雖然
seed 相同，實際障礙物 motion schedule 並不相同。

已修正 `tools/generate_d1_edu_contact_route_graph.py`。重畫後的 30 trials
與 fixed-100 benchmark 的前 30 trials 完全一致：8 success、7 physical
contact、15 timeout。修正版圖檔為
`output/d1_ppo_balanced_safe_500k_routes.html`。

### Collision trace 分析

修正版 trace 顯示 7 個 physical-contact cases 的接觸障礙物全部具有
`motion_kind=turning`，也就是以固定半徑和角速度沿圓弧運動的目標。

- 5 / 7 cases 在接觸前一個 decision sample 的 `predicted_risk=false`，屬於
  turning trajectory 漏預測或預測太晚。
- 2 / 7 cases 已進入 AVOID，仍然與 turning obstacle 接觸，屬於 MPPI／避障
 方向在局部執行階段未能避開掃掠圓弧。
- 7 / 7 cases 的 `waitable=0`，所以 WAIT_YIELD 並不是這批 collision 的主解。

這表示下一個實驗應針對 turning obstacle 的 swept-arc risk、早期預警和
左右避障候選選擇，而不是繼續增加 WAIT reward。

## 10. 接下來的固定實驗

目前不直接訓練到 1M。先將 500k 視為目前候選最佳模型，完成 failure-route 與 action-coverage 診斷：

1. 產生 500k 的 100-trial 路徑圖，優先檢查 physical contact 與 near-goal timeout。
2. 比較 300k／500k 的 AVOID_LEFT、AVOID_RIGHT、WAIT_YIELD 動作覆蓋率。
3. 分析 `AVOID_RIGHT` 幾乎消失的原因，確認是資料左右不平衡、seed 造成的 policy collapse，還是全域路徑符號慣例問題。
4. 先跑 500k 的 20-seed smoke evaluation，再決定是否值得到 1M。
5. 若 1M 仍維持 `SLOWDOWN=0` 或右側避障消失，再建立獨立修正版，不覆寫目前 500k 結果。

模型選擇的硬條件應是：先限制 physical collision 不高於 baseline，再在安全候選中選 success 較高、timeout 較低者。不能用 success rate 單獨選模型。

## 11. 重現指令

```bash
./.venv/bin/python tools/run_d1_edu_decision_layer_sb3.py \
  --timesteps 1 \
  --eval-trials 100 \
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
  --load-model output/d1_ppo_balanced_safe_300k.zip \
  --summary output/d1_ppo_balanced_safe_300k_fixed100_summary.json
```
