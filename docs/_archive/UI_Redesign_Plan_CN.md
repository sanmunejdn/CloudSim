# CloudSim 界面重新设计方案

## 概述

本文档为 CloudSim 桌面应用程序提供全面的界面/用户体验重新设计方案。重新设计的重点是现代化视觉外观、改善用户体验，并建立专业的设计系统，同时保持现有的 Qt Widgets 框架和所有功能。

**核心原则：**
- 基于现有 Qt Widgets 框架（不进行框架迁移）
- 保留所有功能和逻辑
- 针对性、高影响力的改进
- 保持明暗主题支持

---

## 1. 现状分析

### 1.1 技术栈
- **框架：** Qt 5/6 Widgets（C++）
- **样式：** 使用 Fusion 风格的 QPalette
- **图标：** 自定义图标系统（UiIcons），支持明暗主题变体
- **布局：** QMainWindow 配合 QDockWidget 面板
- **主题系统：** ApplicationStyle 命名空间，使用明暗 QPalette

### 1.2 架构概览
```
MainWindow（主窗口）
├── 菜单栏（文件、视图、设置）
├── 中央部件
│   └── 文档标签页（QTabWidget）
│       └── DocumentPage（3D 视口 + 控件）
├── 左侧停靠栏：属性面板
│   ├── 属性浏览器（QtTreePropertyBrowser）
│   └── 设备页面部件（DevicePageWidget）
├── 右侧停靠栏：工作区 + AI
│   ├── 后端树（QTreeWidget）
│   ├── 机器人仿真停靠部件
│   ├── 场景树（QTreeWidget）
│   └── AI 助手停靠部件
└── 底部停靠栏：运行时输出（RunInfoPage）
```

### 1.3 已识别的设计问题

#### 字体排版
- **问题：** 使用系统默认字体，无自定义排版
- **影响：** 外观普通，缺乏特色
- **优先级：** 高

#### 颜色
- **问题：** 基础 QPalette 颜色（纯灰色、标准蓝色高亮）
- **影响：** 扁平、企业化外观，缺乏层次感
- **优先级：** 高

#### 布局
- **问题：** 传统停靠部件布局，间距最小化
- **影响：** 密集、工具化的感觉
- **优先级：** 中

#### 交互性
- **问题：** 默认 Qt 悬停/按下状态
- **影响：** 界面感觉静态
- **优先级：** 中

#### 组件
- **问题：** 标准 Qt 部件外观
- **影响：** 通用桌面应用程序外观
- **优先级：** 中

---

## 2. 设计审查发现

### 2.1 字体排版问题

| 问题 | 现状 | 建议 |
|------|------|------|
| 默认字体 | 系统默认（Windows 上为 Segoe UI） | 使用 `Segoe UI Variable` 或添加自定义字体 |
| 字重 | 仅 Regular（400）和 Bold（700） | 添加 Medium（500）、SemiBold（600） |
| 字距调整 | 无调整 | 为标题、标签添加字距 |
| 等宽数字 | 未实现 | 为数据显示启用（关节角度、坐标） |

### 2.2 颜色和表面问题

| 问题 | 现状 | 建议 |
|------|------|------|
| 背景色 | `rgb(240, 240, 240)`（亮色）、`rgb(53, 53, 53)`（暗色） | 使用更暖/冷的色调 |
| 强调色 | `rgb(0, 120, 215)`（蓝色） | 稍微降低饱和度，添加次要强调色 |
| 阴影 | 无自定义阴影 | 添加带色调的方向性阴影 |
| 纹理 | 纯平面表面 | 添加微妙的噪点/纹理叠加 |

### 2.3 布局问题

| 问题 | 现状 | 建议 |
|------|------|------|
| 间距 | 8px 边距，8px 间距 | 增加到 12-16px 以增加呼吸空间 |
| 容器宽度 | 无最大宽度约束 | 为宽显示器添加 |
| 留白 | 面板内填充最小 | 增加树形图、列表的填充 |
| 垂直节奏 | 不一致 | 建立一致的间距比例 |

### 2.4 交互性问题

| 问题 | 现状 | 建议 |
|------|------|------|
| 悬停状态 | 默认 Qt | 添加带背景变化的自定义悬停 |
| 过渡效果 | 即时 | 添加 200-300ms 过渡 |
| 焦点环 | 默认 Qt | 增强可见性 |
| 加载状态 | 基础 | 添加骨架屏加载器 |

### 2.5 组件模式问题

| 问题 | 现状 | 建议 |
|------|------|------|
| 卡片/面板 | 默认 QDockWidget | 带微妙边框的自定义样式面板 |
| 按钮 | 默认 QPushButton | 带悬停/按下的自定义按钮样式 |
| 树形图 | 默认 QTreeWidget | 带更好间距的自定义样式 |
| 标签页 | 默认 QTabWidget | 现代标签页设计 |

---

## 3. 重新设计建议

### 3.1 字体排版改进

#### 3.1.1 字体选择
```cpp
// 在 ApplicationStyle.cpp 或新的 TypographyManager 中
namespace Typography {
    // 主字体：Segoe UI Variable（Windows 11）或 Segoe UI
    const QString PrimaryFont = "Segoe UI Variable";
    const QString FallbackFont = "Segoe UI";
    
    // 等宽字体用于数据显示
    const QString MonoFont = "Cascadia Code";
    const QString MonoFallback = "Consolas";
    
    // 字号（像素）
    constexpr int DisplaySize = 28;   // 展示标题
    constexpr int H1Size = 22;        // 一级标题
    constexpr int H2Size = 18;        // 二级标题
    constexpr int H3Size = 15;        // 三级标题
    constexpr int BodySize = 13;      // 正文
    constexpr int SmallSize = 11;     // 小号文本
    constexpr int CaptionSize = 10;   // 说明文字
}
```

#### 3.1.2 字重层次
- **展示/一级标题：** SemiBold（600）
- **二级/三级标题：** Medium（500）
- **正文：** Regular（400）
- **标签/说明：** Medium（500）

#### 3.1.3 等宽数字用于数据显示
```cpp
// 用于坐标显示、关节角度等
QFont monoFont(Typography::MonoFont);
monoFont.setStyleHint(QFont::Monospace);
monoFont.setLetterSpacing(QFont::PercentageSpacing, 100);
```

### 3.2 颜色和表面改进

#### 3.2.1 新颜色方案

**亮色主题：**
```cpp
namespace LightTheme {
    // 背景色
    const QColor BackgroundPrimary = QColor(248, 248, 250);    // 微冷色调
    const QColor BackgroundSecondary = QColor(240, 241, 243);  // 面板
    const QColor BackgroundTertiary = QColor(232, 233, 236);   // 卡片
    
    // 文本颜色
    const QColor TextPrimary = QColor(28, 28, 30);             // 主要文本
    const QColor TextSecondary = QColor(99, 99, 102);          // 次要文本
    const QColor TextTertiary = QColor(142, 142, 147);         // 辅助文本
    
    // 强调色
    const QColor AccentPrimary = QColor(0, 102, 204);          // 降低饱和度的蓝色
    const QColor AccentSecondary = QColor(88, 86, 214);        // 紫色强调
    
    // 边框
    const QColor BorderLight = QColor(218, 218, 220);          // 浅边框
    const QColor BorderMedium = QColor(200, 200, 202);         // 中等边框
    
    // 表面
    const QColor SurfaceElevated = QColor(255, 255, 255);      // 提升表面
    const QColor SurfaceSunken = QColor(238, 239, 242);        // 凹陷表面
}
```

**暗色主题：**
```cpp
namespace DarkTheme {
    // 背景色
    const QColor BackgroundPrimary = QColor(28, 28, 30);
    const QColor BackgroundSecondary = QColor(36, 36, 38);
    const QColor BackgroundTertiary = QColor(44, 44, 46);
    
    // 文本颜色
    const QColor TextPrimary = QColor(242, 242, 247);
    const QColor TextSecondary = QColor(142, 142, 147);
    const QColor TextTertiary = QColor(99, 99, 102);
    
    // 强调色
    const QColor AccentPrimary = QColor(40, 140, 240);
    const QColor AccentSecondary = QColor(120, 100, 230);
    
    // 边框
    const QColor BorderLight = QColor(56, 56, 58);
    const QColor BorderMedium = QColor(68, 68, 70);
    
    // 表面
    const QColor SurfaceElevated = QColor(44, 44, 46);
    const QColor SurfaceSunken = QColor(22, 22, 24);
}
```

#### 3.2.2 阴影系统
```cpp
namespace Shadows {
    // 小阴影用于微妙的提升
    const QString Small = "0 1px 2px rgba(0, 0, 0, 0.05)";
    
    // 中阴影用于卡片
    const QString Medium = "0 4px 8px rgba(0, 0, 0, 0.08)";
    
    // 大阴影用于模态框
    const QString Large = "0 12px 24px rgba(0, 0, 0, 0.12)";
    
    // 带色调的阴影（匹配背景色调）
    const QString TintedLight = "0 4px 8px rgba(0, 20, 60, 0.06)";
    const QString TintedDark = "0 4px 8px rgba(0, 0, 0, 0.3)";
}
```

#### 3.2.3 表面纹理
添加微妙的噪点叠加以增加层次感：
```cpp
// 创建噪点纹理叠加部件
class NoiseOverlay : public QWidget {
    // 渲染微妙的纹理
    // 固定位置，不接收指针事件
};
```

### 3.3 布局改进

#### 3.3.1 间距比例
```cpp
namespace Spacing {
    constexpr int XSmall = 4;    // 超小间距
    constexpr int Small = 8;     // 小间距
    constexpr int Medium = 12;   // 中等间距
    constexpr int Large = 16;    // 大间距
    constexpr int XLarge = 24;   // 超大间距
    constexpr int XXLarge = 32;  // 特大间距
}
```

#### 3.3.2 容器约束
```cpp
// 为内容区域添加最大宽度
constexpr int MaxContentWidth = 1440;  // 最大内容宽度
constexpr int MinWindowWidth = 1024;   // 最小窗口宽度
```

#### 3.3.3 面板改进
- 增加树形部件的填充：8px → 12px
- 停靠面板内统一 16px 填充
- 增加 UI 区域之间的间距

#### 3.3.4 视觉层次
```cpp
// 停靠部件样式
QDockWidget {
    border: 1px solid BorderLight;
    border-radius: 8px;
    background: BackgroundSecondary;
}

QDockWidget::title {
    background: BackgroundTertiary;
    padding: 8px 12px;
    font-weight: 500;
}
```

### 3.4 交互性改进

#### 3.4.1 悬停状态
```cpp
// 按钮悬停
QPushButton:hover {
    background: AccentPrimary.lighter(110%);
    transition: background 200ms ease;
}

QPushButton:pressed {
    background: AccentPrimary.darker(110%);
    transform: translateY(1px);
}

// 树形项目悬停
QTreeWidget::item:hover {
    background: AccentPrimary.lighter(180%);
    border-radius: 4px;
}
```

#### 3.4.2 过渡效果
```cpp
// 添加动画支持
namespace Animation {
    constexpr int DurationFast = 150;     // 快速过渡
    constexpr int DurationNormal = 200;   // 普通过渡
    constexpr int DurationSlow = 300;     // 慢速过渡
    
    const QEasingCurve EasingDefault = QEasingCurve::OutCubic;
}
```

#### 3.4.3 焦点状态
```cpp
// 增强焦点环
*:focus {
    outline: 2px solid AccentPrimary;
    outline-offset: 2px;
}
```

### 3.5 组件改进

#### 3.5.1 现代按钮样式
```cpp
// 主要按钮
.QPushButton--primary {
    background: AccentPrimary;
    color: white;
    border: none;
    border-radius: 6px;
    padding: 8px 16px;
    font-weight: 500;
}

// 次要按钮
.QPushButton--secondary {
    background: transparent;
    color: AccentPrimary;
    border: 1px solid AccentPrimary;
    border-radius: 6px;
    padding: 8px 16px;
}

// 幽灵按钮
.QPushButton--ghost {
    background: transparent;
    color: TextPrimary;
    border: none;
    padding: 8px 12px;
}
```

#### 3.5.2 现代树形部件
```cpp
QTreeWidget {
    background: SurfaceElevated;
    border: 1px solid BorderLight;
    border-radius: 8px;
    padding: 4px;
}

QTreeWidget::item {
    padding: 8px 12px;
    border-radius: 4px;
    margin: 2px 0;
}

QTreeWidget::item:selected {
    background: AccentPrimary;
    color: white;
}

QTreeWidget::item:hover {
    background: BackgroundTertiary;
}
```

#### 3.5.3 现代标签页部件
```cpp
QTabWidget::pane {
    border: 1px solid BorderLight;
    border-radius: 8px;
    background: SurfaceElevated;
}

QTabBar::tab {
    background: transparent;
    padding: 8px 16px;
    margin-right: 4px;
    border-bottom: 2px solid transparent;
}

QTabBar::tab:selected {
    background: SurfaceElevated;
    border-bottom: 2px solid AccentPrimary;
    font-weight: 500;
}

QTabBar::tab:hover:!selected {
    background: BackgroundTertiary;
}
```

#### 3.5.4 属性浏览器样式
```cpp
QtTreePropertyBrowser {
    background: SurfaceElevated;
    alternate-background-color: BackgroundSecondary;
}

QtTreePropertyBrowser::item {
    padding: 6px 8px;
    border-bottom: 1px solid BorderLight;
}
```

---

## 4. 实施计划

### 第一阶段：基础（高影响力，低风险）
1. **更新颜色方案** - 在 `ApplicationStyle.cpp` 中替换调色板
2. **添加字体配置** - 设置正确的字重和字号
3. **增加间距** - 在整个应用程序中增加间距
4. **添加自定义阴影** - 为提升层次添加阴影

### 第二阶段：组件样式（中等影响力）
1. **创建 QSS 样式表** - 统一样式管理
2. **样式化按钮** - 添加悬停/按下状态
3. **样式化树形部件** - 现代外观
4. **样式化标签页部件** - 活动指示器

### 第三阶段：打磨（较低影响力）
1. **添加过渡效果** - 为交互元素添加动画
2. **添加噪点纹理** - 叠加层增加质感
3. **增强焦点状态** - 提高可见性
4. **添加加载状态** - 骨架屏等

### 第四阶段：高级（可选）
1. **自定义标题栏**
2. **浮动面板**
3. **动画系统**
4. **手势支持**

---

## 5. 具体文件修改

### 5.1 `ApplicationStyle.cpp`
- 替换 `makeLightPalette()` 和 `makeDarkPalette()` 为新的颜色系统
- 添加字体配置
- 添加阴影定义

### 5.2 新文件：`CloudSimStyles.qss`
- 集中管理所有部件的样式表
- 应用程序内一致的样式

### 5.3 `MainWindowUiSetup.cpp`
- 更新布局中的间距
- 应用新的部件样式
- 添加微妙的动画

### 5.4 `UiIcons.h/.cpp`
- 审查图标集的一致性
- 考虑替代图标库（Phosphor、Heroicons）

---

## 6. 风险评估

| 变更 | 风险 | 缓解措施 |
|------|------|----------|
| 颜色方案更新 | 低 | 易于回滚，测试两种主题 |
| 字体更改 | 低 | 回退到系统字体 |
| QSS 样式表 | 中 | 充分测试，某些部件可能响应不佳 |
| 布局更改 | 中 | 保留所有功能，测试调整大小行为 |
| 动画 | 低 | 可选，可禁用 |

---

## 7. 测试清单

- [ ] 亮色主题正确显示
- [ ] 暗色主题正确显示
- [ ] 所有部件保持功能性
- [ ] 属性浏览器编辑正常工作
- [ ] 树形选择正常工作
- [ ] 标签页切换正常工作
- [ ] 停靠栏取消停靠/重新停靠正常工作
- [ ] 窗口调整大小保持布局
- [ ] 高 DPI 缩放正常工作
- [ ] 无障碍性（焦点状态可见）

---

## 8. 参考资源

### 设计灵感
- **VS Code：** 清洁、专业的暗色主题
- **Figma：** 带微妙阴影的现代 UI
- **Linear：** 最小化、专注的界面设计

### Qt 样式资源
- Qt 样式表参考文档
- Qt 部件库
- QPalette 文档

---

## 9. 后续步骤

1. **审查此方案** - 与相关方讨论
2. **确定优先级** - 基于影响力排序
3. **创建实施任务** - 为每个阶段创建任务
4. **开始第一阶段** - 基础工作
5. **基于反馈迭代** - 持续改进

---

*文档创建时间：2026-06-18*
*最后更新：2026-06-18*
