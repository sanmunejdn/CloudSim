# HPLTPStrategy 框架说明文档

## 1. 概述

HPLTPStrategy 是一个基于策略模式的刀具路径处理框架，用于机器人加工轨迹的离散化、变换、预览和指令生成。框架采用策略+管道架构，支持灵活的策略组合和动态配置。

## 2. 架构设计

### 2.1 核心设计模式

- **策略模式 (Strategy Pattern)**: 每个处理步骤封装为独立策略类
- **管道模式 (Pipeline Pattern)**: 策略按顺序组成处理管道
- **工厂模式 (Factory Pattern)**: 通过工厂创建和管理管道
- **注册表模式 (Registry Pattern)**: 策略自动注册到全局注册表
- **观察者模式 (Observer Pattern)**: UI 事件通过观察者系统分发

### 2.2 整体架构图

```
┌─────────────────────────────────────────────────────────────┐
│                      UI 层 (HPLTPStrategyUI)                │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────┐ │
│  │ HPLTPWidget     │  │ PropertyBrowser │  │ CSV Config  │ │
│  │ (策略选择/排序)  │  │ (参数编辑)      │  │ (参数定义)  │ │
│  └────────┬────────┘  └────────┬────────┘  └─────────────┘ │
└───────────┼─────────────────────┼───────────────────────────┘
            │                     │
            ▼                     ▼
┌─────────────────────────────────────────────────────────────┐
│                   Observer 层 (HPLTPStrategyObserver)       │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ HPLTPStrategyObserverUI                             │   │
│  │ - Creat_Pipeline_Auto: 自动创建并执行管道            │   │
│  │ - Creat_Pipeline: 从选中实体创建管道并执行           │   │
│  │ - Execute_Pipeline: 执行已创建的管道                 │   │
│  │ - Pipeline_UpData_Node: 更新节点参数并重跑           │   │
│  │ - Pipeline_MoveUp/Down_Node: 节点排序                │   │
│  │ - Pipeline_Delete_Node: 删除节点                     │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
            │
            ▼
┌─────────────────────────────────────────────────────────────┐
│                    核心层 (HPLTPStrategy)                    │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ HPLTPStrategyPipelineFactory (管道工厂)              │   │
│  │ - Creat(): 从 JSON 配置创建管道                      │   │
│  │ - ReWork(): 返工已有管道                             │   │
│  └─────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ HPLTPStrategyPipeline (管道引擎)                     │   │
│  │ - 双向链表实现的处理管道                             │   │
│  │ - 支持节点插入、删除、移动、从指定节点重跑           │   │
│  └─────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ HPLTPStrategyManager (策略管理器)                    │   │
│  │ - 管理可用策略原型列表                               │   │
│  │ - 通过 Clone() 创建策略实例                          │   │
│  └─────────────────────────────────────────────────────┘   │
│  ┌─────────────────────────────────────────────────────┐   │
│  │ HPLTPStrategyRegistry (策略注册表)                   │   │
│  │ - 单例模式，维护 className -> creator 映射           │   │
│  │ - REGISTER_STRATEGY 宏自动注册                       │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
            │
            ▼
┌─────────────────────────────────────────────────────────────┐
│                    策略层 (具体策略实现)                      │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐  │
│  │ Discrete │  │Processing│  │PreDisplay│  │  Create  │  │
│  │ (离散化) │  │ (变换)   │  │ (预显示) │  │ (创建)   │  │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘  │
└─────────────────────────────────────────────────────────────┘
            │
            ▼
┌─────────────────────────────────────────────────────────────┐
│                    数据层                                    │
│  ┌─────────────────┐  ┌─────────────────┐                  │
│  │ ToolPath        │  │ ToolPathPoint   │                  │
│  │ (路径点集合)     │  │ (单个路径点)     │                  │
│  └─────────────────┘  └─────────────────┘                  │
└─────────────────────────────────────────────────────────────┘
```

## 3. 核心类说明

### 3.1 HPLTPStrategyBase (策略基类)

**文件**: `inc/HPLTPStrategy/HPLTPStrategy/HPLTPStrategyBase.h`

所有策略的抽象基类，定义统一接口：

```cpp
class HPLTPStrategyBase {
public:
    // 构造函数
    HPLTPStrategyBase(void* entity, const nlohmann::json& para_json);
    
    // 策略处理接口（核心）
    virtual bool Process(ToolPath& machiningPath) = 0;
    
    // 策略名称
    virtual std::string getName() const = 0;
    
    // 克隆接口（原型模式）
    virtual HPLTPStrategyBase* Clone() const = 0;
    
    // 返回策略类型
    virtual OperationType ReturnType() const = 0;
    
    // 更新参数
    void UpdataParameters(void* entity, const nlohmann::json& para_json);
    
protected:
    void* m_entity;        // 几何特征指针
    json m_para_data;      // 策略参数 (nlohmann::json)
};
```

### 3.2 OperationType (策略类型枚举)

```cpp
enum class OperationType {
    Discrete,      // 离散化：将几何实体转换为 ToolPath
    Processing,    // 加工变换：对 ToolPath 进行姿态变换
    PreDisplay,    // 预显示：在 Hoops 中渲染路径预览
    Create         // 创建：生成加工指令
};
```

### 3.3 ToolPath / ToolPathPoint (数据结构)

**文件**: `inc/HPLTPStrategy/HPLTPStrategy/HPLTPPathTypes.h`

```cpp
// 路径点类型
enum class PathPointType {
    APPROACH,  // 进刀点
    CUTTING,   // 加工点
    RETRACT    // 退刀点
};

// 单个路径点
struct ToolPathPoint {
    Eigen::Vector3d position;       // 位置坐标 (mm)
    Eigen::Quaterniond orientation; // 姿态四元数
    PathPointType type;             // 路径点类型
    std::string comment;            // 注释字符串
    EditableStatus editable;        // 编辑状态
    nlohmann::json info;            // 扩展信息
};

// 路径点集合
struct ToolPath {
    std::vector<ToolPathPoint> points;
    
    // 容器操作: push_back, insert, clear, resize, reserve
    // 查询接口: size, empty, front, back
    // 访问接口: operator[], at
    // 迭代器: begin, end, cbegin, cend
    // 类型查询: getApproachPoints, getCuttingPoints, getRetractPoints
};
```

### 3.4 HPLTPStrategyRegistry (策略注册表)

**文件**: `inc/HPLTPStrategy/HPLTPStrategy/HPLTPStrategyRegistry.h`

单例模式，管理所有策略的创建函数：

```cpp
class HPLTPStrategyRegistry {
public:
    static HPLTPStrategyRegistry& GetInstance();
    
    void RegisterStrategy(const std::string& className,
        std::function<HPLTPStrategyBase*(void*, const nlohmann::json&)> creator);
    
    std::function<HPLTPStrategyBase*(void*, const nlohmann::json&)>
        GetStrategyCreator(const std::string& className);
};

// 自动注册宏
#define REGISTER_STRATEGY(ClassName) \
    static ClassName##_Registrar ClassName##_registrar;
```

**使用方式**: 在策略头文件末尾添加 `REGISTER_STRATEGY(ClassName);`

### 3.5 HPLTPStrategyPipeline (管道引擎)

**文件**: `inc/HPLTPStrategy/HPLTPStrategy/HPLTPStrategyPipeline.h`

基于双向链表的处理管道：

```cpp
class HPLTPStrategyPipeline {
public:
    // 节点管理
    int addNode(std::shared_ptr<HPLTPStrategyBase> strategy, int index = -1);
    int insertAfter(int targetIndex, std::shared_ptr<HPLTPStrategyBase> strategy);
    int insertBefore(int targetIndex, std::shared_ptr<HPLTPStrategyBase> strategy);
    void removeNode(int index);
    
    // 节点排序
    bool moveNodeBefore(int sourceIndex, int targetIndex);
    bool moveNodeAfter(int sourceIndex, int targetIndex);
    bool swapNodes(int firstIndex, int secondIndex);
    bool moveNodeUp(int index);
    bool moveNodeDown(int index);
    
    // 执行控制
    void execute(bool passcreat = true);
    void runFromNode(int index, bool passcreat = true);
    void runFromLastNode(int index, bool passcreat = true);
    
    // 参数更新
    void upDataNodePara(int index, const nlohmann::json& para_json);
    
    // 节点查询
    HPLTPProcessingNodePtr findNodeByIndex(int index);
    HPLTPProcessingNodePtr findNodeByName(const std::string& name);
    std::vector<int> getAllNodeIndices() const;
    
private:
    HPLTPProcessingNodePtr m_head;
    HPLTPProcessingNodePtr m_tail;
    std::map<int, HPLTPProcessingNodePtr> m_indexToNodeMap;
};
```

### 3.6 HPLTPProcessingNode (处理节点)

**文件**: `inc/HPLTPStrategy/HPLTPStrategy/HPLTPProcessingNode.h`

双向链表节点，包装策略对象：

```cpp
class HPLTPProcessingNode {
public:
    HPLTPProcessingNode(std::shared_ptr<HPLTPStrategyBase> strategy);
    
    // 链表操作
    void setPrev(HPLTPProcessingNodePtr prevNode);
    void setNext(HPLTPProcessingNodePtr nextNode);
    HPLTPProcessingNodePtr getPrev() const;
    HPLTPProcessingNodePtr getNext() const;
    
    // 策略执行
    bool process(ToolPath& machiningPath);
    
    // 信息获取
    std::string getName() const;
    OperationType getType() const;
    ToolPath getResult();
    
    // 参数更新
    void updataJsonPara(const nlohmann::json& para_json);
    
private:
    std::shared_ptr<HPLTPStrategyBase> strategy_;
    HPLTPProcessingNodePtr prev_;
    HPLTPProcessingNodePtr next_;
    ToolPath m_cachedResult;
};
```

### 3.7 HPLTPStrategyPipelineFactory (管道工厂)

**文件**: `inc/HPLTPStrategy/HPLTPStrategy/HPLTPStrategyPipelineFactory.h`

负责创建和返工管道：

```cpp
class HPLTPStrategyPipelineFactory {
public:
    HPLTPStrategyPipelineFactory(void* entity, const nlohmann::json& para_json);
    
    HPLTPStrategyPipeline* Creat();
    void ReWork(HPLTPStrategyPipeline* StrategyPipeline);
    
private:
    nlohmann::json m_para_json;
    void* m_entity;
};
```

### 3.8 HPLTPStrategyManager (策略管理器)

**文件**: `inc/HPLTPStrategy/HPLTPStrategy/HPLTPStrategyManager.h`

管理可用策略原型：

```cpp
class HPLTPStrategyManager {
public:
    void RegisterStrategiesFromConfig(const std::string& configFile);
    
    template<typename StrategyClass>
    void RegisterAvailableStrategies();
    
    std::shared_ptr<HPLTPStrategyBase> CreateStrategy(
        const std::string strategyName, void* entity, const nlohmann::json& para_json);
    
private:
    std::vector<std::shared_ptr<HPLTPStrategyBase>> m_AvailableStrategies;
};
```

## 4. 策略分类

### 4.1 离散化策略 (Discrete)

将几何实体转换为 ToolPath 点集：


| 策略类名                                 | 策略名称     | 功能描述        |
| ------------------------------------ | -------- | ----------- |
| HPLTPDiscretizerEdgeStrategy         | 边线离散策略   | 将边线离散为路径点   |
| HPLTPDiscretizerFaceStrategy         | 面离散策略    | 将曲面离散为路径点   |
| HPLTPDiscretizerSplineStrategy       | 样条曲线离散策略 | 将样条曲线离散为路径点 |
| HPLTPStrategyDiscretizerProjCurve    | 曲线投影离散策略 | 将曲线投影到曲面后离散 |
| HPLTPDiscretizerParamSurfaceStrategy | 参数面离散策略  | 参数化曲面离散     |


### 4.2 加工变换策略 (Processing)

对 ToolPath 进行各种变换：


| 策略类名                                  | 策略名称     | 功能描述         |
| ------------------------------------- | -------- | ------------ |
| HPLTPPostureCopyStrategy              | 复制策略     | 复制路径点        |
| HPLTPFixedPostureStrategy             | 固定加工姿态策略 | 固定加工姿态       |
| HPLTPStartPointAdjustStrategy         | 轨迹起点调整策略 | 调整轨迹起点       |
| HPLTPPostureReversalStrategy          | 反向变换策略   | 反向路径         |
| HPLTPPostureRotationStrategy          | 旋转变换策略   | 旋转路径         |
| HPLTPPositionMovementStrategy         | 移动变换策略   | 移动路径         |
| HPLTPCorrectionStrategy               | 点云纠偏策略   | 基于点云纠偏       |
| HPLTPLinearFeedApproachStrategy       | 直线进刀工艺策略 | 直线进刀         |
| HPLTPLinearRetractionStrategy         | 直线退刀工艺策略 | 直线退刀         |
| HPLTPArcFeedApproachStrategy          | 圆弧进刀工艺策略 | 圆弧进刀         |
| HPLTPArcRetractionStrategy            | 圆弧退刀工艺策略 | 圆弧退刀         |
| HPLTPToWorkpieceInHandStrategy        | 转换工件型策略  | 转换为工件型轨迹     |
| HPLTPNonRigidRegistrationStrategy     | 非刚性配准策略  | 非刚性配准        |
| HPLTPTrajectoryExtensionStrategy      | 轨迹延长策略   | 延长轨迹         |
| HPLTPPointCuttingStrategy             | 轨迹裁剪策略   | 裁剪轨迹         |
| HPLTPTrajectoryIntervalDeleteStrategy | 轨迹区间删除策略 | 删除轨迹区间       |
| HPLTPSucculentHandlingStrategy        | 多肉处理策略   | 多肉处理         |
| HPLTPSetScopeStrategy                 | 设置作用域策略  | 设置作用域        |
| HPLTPInsertCommentStrategy            | 插入指令策略   | 插入注释指令       |
| HPLTPCallPythonStrategy               | 调用脚本策略   | 调用 Python 脚本 |


### 4.3 预显示策略 (PreDisplay)

在 Hoops 中渲染路径预览：


| 策略类名                   | 策略名称    | 功能描述           |
| ---------------------- | ------- | -------------- |
| HPLTPPreviewerStrategy | 通用预显示策略 | 预览路径、轴、箭头、工具模型 |


### 4.4 创建策略 (Create)

生成加工指令：


| 策略类名                          | 策略名称     | 功能描述   |
| ----------------------------- | -------- | ------ |
| HPLTPCreatorApproachStrategy  | 进刀指令创建策略 | 生成进刀指令 |
| HPLTPCreatorMachiningStrategy | 加工指令创建策略 | 生成加工指令 |
| HPLTPCreatorRetractStrategy   | 退刀指令创建策略 | 生成退刀指令 |


## 5. 配置系统

### 5.1 StrategyConfig.csv

**路径**: `HPLTPStrategyWidgetCsv/StrategyConfig.csv`

定义每个策略在各工艺类型下的启用状态：

```csv
类名称,策略名称,线打磨工艺,面打磨工艺,水火弯板工艺,曲线投影工艺,调用脚本工艺,参数面打磨工艺
HPLTPDiscretizerEdgeStrategy,边线离散策略,启用,不启用,不启用,不启用,不启用,不启用
HPLTPPreviewerStrategy,通用预显示策略,启用,启用,启用,启用,启用,启用
```

### 5.2 NodeWidget/*.csv

**路径**: `HPLTPStrategyWidgetCsv/NodeWidget/`

定义每个策略的 UI 参数：

```csv
策略名称,组合,
参数名,类型,默认值,最小值,最大值,依赖项
```

**参数类型**:

- `数值`: 数值参数 (double)
- `布尔`: 布尔参数 (true/false)
- `多选`: 枚举参数
- `文本`: 文本参数
- `文件`: 文件路径参数

**依赖项语法**: `依赖项:控制属性名=期望值`

### 5.3 ProcessFlow/*.csv

**路径**: `HPLTPStrategyWidgetCsv/ProcessFlow/`

定义各工艺的默认管道配置：

```csv
工艺名称,策略1,策略2,策略3,...
面打磨工艺,面离散策略,直线进刀工艺策略,直线退刀工艺策略,通用预显示策略,...
```

## 6. 数据流

### 6.1 典型处理流程

```
几何实体 (面/边/曲线)
        │
        ▼
┌─────────────────┐
│  Discrete 策略  │  将几何离散为路径点
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Processing 策略 │  进刀 -> 变换 -> 纠偏 -> ...
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ PreDisplay 策略 │  Hoops 3D 渲染预览
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│   Create 策略   │  生成加工指令
└─────────────────┘
```

### 6.2 JSON 参数格式

```json
{
    "Linkedlist": [
        {
            "Node": [
                {
                    "id": 1,
                    "name": "策略名称",
                    "参数1": "值1",
                    "参数2": "值2"
                }
            ]
        }
    ]
}
```

## 7. 使用示例

### 7.1 创建新策略

1. **头文件** (`inc/HPLTPStrategy/HPLTPStrategy/HPLTPMyStrategy.h`):

```cpp
#ifndef _HPLTPSTRATEGY_HPLTPMYSTRATEGY_H_
#define _HPLTPSTRATEGY_HPLTPMYSTRATEGY_H_

#include "dcl_TPStrategy.h"
#include "HPLTPStrategyBase.h"
#include "HPLTPStrategyRegistry.h"

class DECL_HPLTPSTRATEGY HPLTPMyStrategy : public HPLTPStrategyBase {
public:
    HPLTPMyStrategy(void* entity, const nlohmann::json& para_json);
    virtual ~HPLTPMyStrategy();
    
    virtual bool Process(ToolPath& path) override;
    virtual std::string getName() const override;
    virtual HPLTPStrategyBase* Clone() const override {
        return new HPLTPMyStrategy(m_entity, m_para_data);
    }
    virtual OperationType ReturnType() const override { 
        return OperationType::Processing; 
    }
};

REGISTER_STRATEGY(HPLTPMyStrategy);
#endif
```

1. **实现文件** (`src/HPLTPStrategy/HPLTPStrategy/src/HPLTPMyStrategy.cpp`):

```cpp
#include "HPLTPMyStrategy.h"

HPLTPMyStrategy::HPLTPMyStrategy(void* entity, const nlohmann::json& para_json)
    : HPLTPStrategyBase(entity, para_json) {}

HPLTPMyStrategy::~HPLTPMyStrategy() {}

bool HPLTPMyStrategy::Process(ToolPath& path) {
    // 读取参数
    double param1 = m_para_data.value("参数1", 0.0);
    
    // 处理路径
    for (auto& point : path.points) {
        // 变换逻辑...
    }
    return true;
}

std::string HPLTPMyStrategy::getName() const {
    return "我的策略";
}
```

1. **CSV 配置** (`HPLTPStrategyWidgetCsv/NodeWidget/我的策略.csv`):

```csv
我的策略,组合,
参数1,数值,0,-9999,9999
参数2,布尔,TRUE
```

1. **注册到 StrategyConfig.csv**:

```csv
HPLTPMyStrategy,我的策略,启用,启用,启用,启用,启用,启用
```

### 7.2 参数读取模式

```cpp
// 数值参数
double value = m_para_data.value("参数名", 默认值);

// 布尔参数
bool flag = m_para_data.value("参数名", false);

// 字符串参数
std::string str = m_para_data.value("参数名", "");

// 枚举参数
std::string option = m_para_data.value("参数名", "选项1");
```

## 8. 目录结构

```
HPLProgram/
├── inc/HPLTPStrategy/
│   ├── HPLTPStrategy/           # 核心策略头文件
│   │   ├── HPLTPStrategyBase.h
│   │   ├── HPLTPPathTypes.h
│   │   ├── HPLTPStrategyRegistry.h
│   │   ├── HPLTPStrategyPipeline.h
│   │   ├── HPLTPProcessingNode.h
│   │   ├── HPLTPStrategyManager.h
│   │   ├── HPLTPStrategyPipelineFactory.h
│   │   └── HPLTP*Strategy.h     # 各策略头文件
│   ├── HPLTPStrategyUI/         # UI 层头文件
│   └── HPLTPStrategyObserver/   # Observer 层头文件
├── src/HPLTPStrategy/
│   ├── HPLTPStrategy/           # 核心策略实现
│   │   └── src/
│   │       ├── HPLTPStrategy*.cpp
│   │       └── HPLTP*Strategy.cpp
│   ├── HPLTPStrategyUI/         # UI 层实现
│   └── HPLTPStrategyObserver/   # Observer 层实现
└── HPLProduct/HPLPolishTool/bin/resource/
    └── HPLTPStrategyWidgetCsv/
        ├── StrategyConfig.csv   # 策略配置
        ├── NodeWidget/          # 策略参数定义
        └── ProcessFlow/         # 工艺流程定义
```

## 9. 技术栈

- **语言**: C++17
- **几何内核**: OpenCASCADE (OCCT)
- **矩阵运算**: Eigen 3.4
- **JSON**: nlohmann/json
- **3D 渲染**: HOOPS 3DF
- **日志**: spdlog
- **UI**: Qt5
- **构建**: Visual Studio 2019 (vcxproj)

## 10. 扩展指南

### 10.1 添加新策略

1. 创建头文件，继承 `HPLTPStrategyBase`
2. 实现 `Process()`, `getName()`, `Clone()`, `ReturnType()`
3. 在头文件末尾添加 `REGISTER_STRATEGY(ClassName);`
4. 创建 CSV 参数配置文件
5. 在 `StrategyConfig.csv` 中注册
6. 在需要的 `ProcessFlow/*.csv` 中添加

### 10.2 修改现有策略

1. 找到策略头文件和实现文件
2. 修改 `Process()` 方法实现
3. 如需新增参数，在 CSV 中添加并修改代码读取
4. 重新编译

### 10.3 调试技巧

- 使用 `spdlog::info()` 输出调试信息
- 检查 `m_para_data` 中的参数值
- 使用 `HC_Show_Segment()` 查看 HOOPS segment 结构
- 查看 `HPLTPStrategyObserverUI.cpp` 了解事件处理流程

