# PlcCommSDK

基于 [libplctag](https://github.com/libplctag/libplctag) 的 PLC 通信 SDK（**AB EIP** + **Modbus TCP**），与 CloudSim 主程序、插件宿主解耦。

## 依赖

- 预编译库：`bin/SDK/libplctag-2.6-vc14-64/`（见 `README_DEPLOY.md`）
- 运行时：`plctag.dll` 与 `PlcCommSDK.dll` 同置于 `bin/x64d/` 或 `bin/x64/`

## 对外 API

| 头文件 | 说明 |
|--------|------|
| `inc/IPlcCommClient.h` | 客户端接口 + `createPlcCommClient()` 工厂 |
| `inc/PlcCommTypes.h` | `PlcProtocol`、`PlcConnectionConfig`、`PlcTagSpec`、`PlcTagValue` |
| `inc/plc_comm_sdk_global.h` | `PLCCOMM_SDK_EXPORT`、`PLCCOMM_SDK_VERSION` |

### 线程约定

- `readTag` / `writeTag`：**同步**，且设计上仅由**单一工作线程**调用（UI 层通过 `PlcCommUI` 的 `QThread` 满足此约束）
- `connect` / `disconnect` / `addTag` / `removeTag`：内部 `std::mutex` 保护，但仍建议与读写在同一线程串行使用

## 标签属性串

由 `PlcTagStringBuilder` 生成，例如：

- AB：`protocol=ab_eip&gateway=192.168.0.10&path=1,0&cpu=lgx&name=MyTag`
- Modbus：`protocol=modbus_tcp&gateway=192.168.0.10&port=502&path=1&name=hr0`
  - `path`：从站单元 ID（0–255），默认 `1`
  - `name`：须为 libplctag 格式（`hr`/`co`/`di`/`ir` + 从 0 起的序号）。UI 输入 **40001** 会自动转为 **hr0**（保持寄存器区 4xxxx，1-based）

## 构建

- 工程：`PlcCommSDK.vcxproj`（`CloudSim.sln`）
- 定义：`PLCCOMM_SDK_LIB`（导出）
- 工具集：v142，C++17，`/utf-8`
