# PlcCommUI 开发指南

Qt 5.14 通讯页 DLL，经 **PlcCommSDK** 访问 PLC。**不**链接 `Widget` / `Data` / `CloudSimPluginHost`，**不**包含 `libplctag.h`。

## 入口

```cpp
#include "PlcCommWidget.h"

QWidget* panel = createPlcCommWidget(parent);
panel->setUseChinese(true);   // 默认中文
panel->applyLanguage();
```

## 架构

```text
PlcCommWidget (UI 线程)
    → PlcCommController
        → QThread + PlcCommWorker
            → IPlcCommClient (唯一实例，工作线程)
```

| 类 | 线程 | 职责 |
|----|------|------|
| `PlcCommWidget` | UI | 连接区、标签表、数值编辑、格式切换、轮询、日志 |
| `PlcCommController` | UI | 将操作 `invokeMethod` 到 Worker |
| `PlcCommWorker` | 工作线程 | 独占 `IPlcCommClient`，执行 connect/read/write |

## 界面功能

### 连接区

| 控件 | AB EIP | Modbus TCP |
|------|--------|------------|
| 协议 | AB EIP | Modbus TCP |
| IP | PLC 地址 | 从站地址 |
| 端口 | 禁用 | 502（可改） |
| Path / 单元 ID | 路由 `1,0` | 单元 ID，默认 `1` |
| CPU | 如 `lgx` | 禁用 |
| 超时 | 创建/读写毫秒超时 | 同左 |

切换协议时自动 **断开** 当前连接，避免 AB「参数就绪」与 Modbus 真连接状态混淆。

### 标签区

- 添加/删除标签、表格显示句柄/名称/最近数据
- **读** / **写** / **轮询**（间隔 ms）
- **显示** 下拉：读写与表格展示共用同一格式

### 数值显示格式

读取后缓存原始 `QByteArray`（`rawByHandle_`），切换格式时 **无需重新读 PLC**。

| 格式 | 读显示 | 写入解析 |
|------|--------|----------|
| 十六进制 | `01 FF 2A` | 空格分隔十六进制 |
| 十进制(字节) | 每字节 0–255 | 空格/逗号分隔 |
| 二进制(字节) | 每字节 8 位 | 连续 8 位一组 |
| UInt16 小端 | 每 2 字节一个无符号数 | 如 `1234` |
| Int32 小端 | 每 4 字节一个有符号数 | 如 `-1` |

Modbus 单寄存器（2 字节）建议使用 **UInt16 小端** 查看。

## 构建

| 项 | 值 |
|----|-----|
| 工程 | `PlcCommUI.vcxproj` |
| 依赖 | `PlcCommSDK`（ProjectReference） |
| 输出 | `bin/x64(d)/PlcCommUI.dll` |
| 运行时同目录 | `PlcCommSDK.dll`、`plctag.dll` |

## 嵌入方式

### 方式 A：CloudSim 插件（推荐）

使用 [`PlcCommPlugin`](../PlcCommPlugin/DEVELOPER_GUIDE.md)，自动注册侧栏 **PLC 通讯** 页签。

### 方式 B：任意 Qt 宿主

1. 链接 `PlcCommUI.lib`、`PlcCommSDK.lib`
2. 运行时部署 `PlcCommUI.dll`、`PlcCommSDK.dll`、`plctag.dll`
3. 调用 `createPlcCommWidget()`

## 中英文

| API | 说明 |
|-----|------|
| `setUseChinese(bool)` | 默认 `true` |
| `applyLanguage()` | 刷新控件与格式占位符 |

CloudSim 内由 `PlcCommPlugin` 在 `onLanguageChanged` 中调用，与 **设置 → 语言** 同步。

## 日志含义

| 日志 | 含义 |
|------|------|
| Modbus TCP 已连通 | Modbus `connect` 探测成功 |
| EIP 参数已就绪… | AB 仅参数校验，未建 EIP 会话 |
| 连接状态：已连接/未连接 | `isConnected()` 变化 |

## 相关文档

- SDK：[`../PlcCommSDK/DEVELOPER_GUIDE.md`](../PlcCommSDK/DEVELOPER_GUIDE.md)
- 插件：[`../PlcCommPlugin/DEVELOPER_GUIDE.md`](../PlcCommPlugin/DEVELOPER_GUIDE.md)
