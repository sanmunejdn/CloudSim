# TODO：后端对象显示/隐藏

## 待手工验证

1. 隐藏 1+ 个后端 → 保存 `.json`/`.pcp` → 重开：隐藏态与勾选正确
2. 打开无 `visible` 字段的旧工程：对象全部显示
3. 导入新对象：默认显示；再隐藏并保存可恢复

## 可选后续

- 属性面板暴露 `visible` 行（当前仅树/右键；`BackendVisualSync` 已支持 visible key）
- 单测：`saveToJson`/`loadFromJson` 往返 `visible=false`

## 配置

无额外环境变量或密钥；需按序编译 Data / CloudSimCore / CloudSimHost / Widget。
