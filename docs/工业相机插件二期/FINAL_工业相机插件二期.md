# FINAL — 工业相机插件二期（梅卡）

## 总结

在一期 `ICamera` / Ensemble / 单侧栏 Tab 之上，二期落地：

1. **MechEyeCamera**：`CLOUDSIM_HAS_MECH_EYE` 启真 SDK；否则明确错误
2. **BoardDetector**：OpenCV 棋盘格主路径；ArUco 预留；无宏可手填兜底
3. **MechOfficialHandEye**：官方 `HandEyeCalibration` 作为 Ensemble 额外候选，经 `mergeHandEyeCandidate` 残差择优

## 与一期关系

海康宏与真机运维仍为可选；不阻塞梅卡交付。UI 未回退为双侧栏。

## 后续

见 [`../工业相机插件/TODO_工业相机插件.md`](../工业相机插件/TODO_工业相机插件.md) 与 [`TODO_工业相机插件二期.md`](TODO_工业相机插件二期.md)。
