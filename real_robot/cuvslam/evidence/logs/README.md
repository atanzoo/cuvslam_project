# Logs

Runtime logs and test evidence are stored here. Each run should identify the
experiment, hardware, software version, and time base.

## 目前實機建圖最重要的證據

- `real_d435i_quality/20260825_r2_correction_log.md` — 最新 R2 分析、修正與
  下一次驗證條件。
- `real_d435i_quality/20260818_162917_r2_motion_quality.json` — GUI 位移與
  影像品質 JSON 證據。
- `real_d435i_r1_20260818_1034/` — R1 camera/IMU、TF、QoS、resource baseline。
- `real_d435i_r1p5_20260818_1100/` — R1.5 calibration、fusion smoke 與軸向證據。
- `real_d435i_r2/` — 初始 R2 rosbag metadata 與 odometry JSON。

Logs are evidence, not source files. Do not delete a failed run to make the
project appear healthy. The current-state interpretation is maintained in
`../../../../docs/CUVSLAM_CURRENT_STATUS_20260825.md`.
