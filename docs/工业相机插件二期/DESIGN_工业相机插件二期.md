# DESIGN — 工业相机插件二期

```mermaid
flowchart LR
  UI --> MechEyeCamera
  UI --> BoardDetector
  BoardDetector --> OpenCV
  HandEye --> Ensemble
  HandEye --> MechOfficial
  MechEyeCamera --> MechSDK
```

| 模块 | 路径 |
|------|------|
| MechEyeCamera | `source/mech/` |
| BoardDetector | `inc/BoardDetector.h` + `source/calib/` |
| MechOfficial | `solveMechOfficialHandEye` in calib |
