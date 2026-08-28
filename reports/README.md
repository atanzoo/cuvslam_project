# Reports

Store experiment reports and acceptance reports here. A report must state the
environment, configuration, data source, metrics, failures, and conclusion.

## Real D435i reading order

1. `real_d435i_r1_static_contract_20260818.md`
2. `real_d435i_r1p5_imu_qualification_20260818.md`
3. `real_d435i_r2_stereo_imu_ab_20260818.md`
4. `real_d435i_lighting_robustness_20260818.md`

The latest 2026-08-25 R2 analysis and correction record is under
`../logs/real_d435i_quality/20260825_r2_correction_log.md`; it is not replaced
by a report because it is also the GUI/runtime correction log.

Simulation reports remain useful for estimator and observability history, but
must not be presented as real-camera accuracy. Do not replace a failed report
with a successful rerun. Preserve both and link the rerun to the original
experiment.
