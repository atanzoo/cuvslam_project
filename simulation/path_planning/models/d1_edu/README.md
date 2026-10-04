# 智元 D1 Edu 本機模擬模型

此資料夾保存智元官方提供的 D1 Edu URDF 模型，供本機 MuJoCo／Isaac Sim 模擬使用。

## 目前狀態

- 官方 URDF：`edu_description/urdf/edu.urdf`
- 官方網格：`edu_description/meshes/`
- 關節／參數表：`edu_description/urdf/edu.csv`
- MuJoCo 模型：`d1_edu.xml`
- Jetson：目前不需要

## 建議流程

```text
edu.urdf
  ↓
確認關節名稱、軸向與限制
  ↓
轉成 MuJoCo MJCF
  ↓
補上 actuator、接觸與摩擦參數
  ↓
先做站立與關節 PD 測試
  ↓
加入 MPPI
```

本機模擬階段可直接使用模擬器真值，不需要 SLAM、Jetson 或實機定位。只有在要測試實際 SDK 延遲、通訊頻率、CPU 負載與 sim-to-real 時，才需要把控制器移到 Jetson 或接上實機。

官方下載來源：<https://agibot.com.cn/filepage/329.html>

## 本機 smoke test

```bash
./.venv/bin/python simulation/path_planning/tools/test_d1_edu_mujoco.py
```

目前已完成 URDF → MJCF 轉換、網格路徑修正、空白材質修正、地面加入，以及 MuJoCo 載入／步進測試。下一步是校正站立姿態與 PD 參數，之後再接 MPPI。
