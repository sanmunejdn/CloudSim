# PlcCommSDK 开发指南

基于 [libplctag](https://github.com/libplctag/libplctag) 的 PLC 通讯后端 DLL，支持 **Allen-Bradley EtherNet/IP** 与 **Modbus TCP**。与 CloudSim 主程序、插件宿主 **解耦**（仅头文件 + `PlcCommSDK.lib`）。

## 定位

| 项 | 说明 |
|----|------|
| **传输层** | Modbus TCP / AB EIP 均走 **TCP**（由 libplctag 管理套接字） |
| **非目标** | 不提供原始 Socket API、UDP、Modbus RTU 串口、自定义二进制帧 |
| **消费者** | `PlcCommUI.dll`、未来其它 Qt/非 Qt 宿主 |

```text
宿主 (PlcCommUI / 自定义)
    → IPlcCommClient
        → PlcCommClientImpl
            → PlcTagStringBuilder
            → libplctag (plctag.dll)
                → TCP → PLC / Modbus Slave
```

## 依赖与部署

- 预编译包：`bin/SDK/libplctag-2.6-vc14-64/`（见 [`README_DEPLOY.md`](../../../../bin/SDK/libplctag-2.6-vc14-64/README_DEPLOY.md)）
- 获取脚本：仓库根目录 `scripts/fetch_libplctag.ps1`
- 运行时：`plctag.dll` 与 `PlcCommSDK.dll` 同在 `bin/x64d/` 或 `bin/x64/`

## 工程与构建

| 项 | 值 |
|----|-----|
| 工程 | `PlcCommSDK.vcxproj`（`CloudSim.sln`） |
| 定义 | `PLCCOMM_SDK_LIB`（导出） |
| 工具链 | x64、v142、C++17、`/utf-8` |
| 输出 | `bin/x64(d)/PlcCommSDK.dll` |
| PostBuild | 复制 `plctag.dll` → `$(CloudSimBinDir)` |

## 对外 API

| 头文件 | 说明 |
|--------|------|
| `inc/IPlcCommClient.h` | 客户端接口 + `createPlcCommClient()` |
| `inc/PlcCommTypes.h` | `PlcProtocol`、`PlcConnectionConfig`、`PlcTagSpec`、`PlcTagValue` |
| `inc/PlcTagStringBuilder.h` | 属性串生成（`buildPlcTagAttributeString`） |
| `inc/plc_comm_sdk_global.h` | `PLCCOMM_SDK_EXPORT`、`PLCCOMM_SDK_VERSION` |

### PlcConnectionConfig

| 字段 | AB EIP | Modbus TCP |
|------|--------|------------|
| `gateway` | PLC IP（可带端口） | 从站 IP |
| `port` | 通常忽略（EIP 默认 44818） | 默认 **502** |
| `path` | 路由，如 `1,0` | **单元 ID**（0–255），默认 `1` |
| `cpu` | 如 `lgx` | 不使用 |
| `timeoutMs` | 创建/读写超时（默认 10000） | 同左 |

### IPlcCommClient 方法

| 方法 | 说明 |
|------|------|
| `connect` | Modbus：**TCP 探测**（`hr0` 建 tag + status OK）；AB：仅校验参数，**不**立即建 EIP 会话 |
| `disconnect` | 销毁所有 tag，调用 `plc_tag_shutdown()` |
| `isConnected` | 逻辑连接标志 |
| `addTag` / `removeTag` | 创建/销毁 libplctag 标签，返回句柄 id |
| `readTag` / `writeTag` | 同步读写原始字节 |
| `lastError` | 最近一次错误（含超时中文提示） |

## 连接语义（与 UI 提示一致）

- **Modbus TCP**：`connect` 成功表示已与目标 IP:端口 完成 libplctag 建连探测。
- **AB EIP**：`connect` 仅表示连接参数有效；**首次 `addTag` 时**才建立真实 EIP 会话。UI 日志为「EIP 参数已就绪（添加标签后建立会话）」。

## Modbus 地址映射

UI/调用方 `PlcTagSpec::name` 在 `PlcTagStringBuilder` 中规范化：

| 输入示例 | libplctag `name` | 含义 |
|----------|------------------|------|
| `40001` | `hr0` | 保持寄存器，1-based → 0-based |
| `40002` | `hr1` | 同上 |
| `30001` | `ir0` | 输入寄存器 |
| `10001` | `di0` | 离散输入 |
| `1` / `00001` | `co0` | 线圈 |
| `hr66` | `hr66` | 已是 libplctag 格式则原样使用 |

属性串示例：

```text
protocol=modbus_tcp&gateway=127.0.0.1&port=502&path=1&name=hr0&elem_count=1
```

AB 示例：

```text
protocol=ab_eip&gateway=192.168.0.10&path=1,0&cpu=lgx&name=MyTag&elem_count=1
```

## 线程约定

- `readTag` / `writeTag`：**同步阻塞**，且 **仅允许在单一工作线程**调用（`PlcCommUI` 的 `PlcCommWorker` 线程）。
- `connect` / `addTag` 等内部有 `std::mutex`，仍建议与读写在同一线程串行，避免与 libplctag 回调冲突。
- **禁止**在 UI 线程直接调用 `readTag` / `writeTag`。

## 错误与排查

| 现象 | 常见原因 |
|------|----------|
| `PLCTAG_ERR_CREATE` | Modbus `name` 非法；缺 `path`；IP 不可达 |
| `PLCTAG_ERR_TIMEOUT` | IP/端口错误；Slave 未 Connect；防火墙；本机联调可试 `127.0.0.1` |
| AB 显示已连接但读失败 | 需先 `addTag` 真实标签名；检查 `path`/`cpu` |

失败时 `lastError()` 可能附带完整属性串，便于对照 libplctag 文档。

## 本地 Modbus Slave 联调

1. Slave：**Connection → Modbus TCP/IP**，IP 与 CloudSim **一致**（同机建议 `127.0.0.1`），端口 **502**，**Connect**。
2. 寄存器表：至少定义保持寄存器 **地址 0**（对应 `hr0` / `40001`）。
3. CloudSim：协议 Modbus TCP，单元 ID **1**，超时可适当加大。

## 相关文档

- UI：[`../PlcCommUI/DEVELOPER_GUIDE.md`](../PlcCommUI/DEVELOPER_GUIDE.md)
- CloudSim 插件：[`../PlcCommPlugin/DEVELOPER_GUIDE.md`](../PlcCommPlugin/DEVELOPER_GUIDE.md)
- 架构总览：[`../../../ARCHITECTURE_SUMMARY.md`](../../../ARCHITECTURE_SUMMARY.md) §10.5
