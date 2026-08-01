# -*- coding: utf-8 -*-
"""Generate CloudSim user help HTML (zh/en) from feature outline."""
from pathlib import Path

ROOT = Path(r"d:\Project\VSprogram\CGAL5.5.2\CloudSim\help")

NAV_ZH = [
    ("index.html", "首页"),
    ("getting-started.html", "入门与界面"),
    ("projects.html", "工程与文件"),
    ("view-3d.html", "三维视图"),
    ("import.html", "导入资源"),
    ("scene.html", "场景树与属性"),
    ("geometry.html", "几何建模"),
    ("drawing.html", "工程图纸"),
    ("robot.html", "机器人仿真"),
    ("trajectory.html", "轨迹生成与编辑"),
    ("process-flow.html", "工艺流程"),
    ("pointcloud.html", "点云与网格"),
    ("geometry-plugin.html", "几何分析插件"),
    ("labeling.html", "标注与深度学习"),
    ("plc-camera.html", "PLC 与相机"),
    ("ai.html", "AI 助手"),
    ("settings.html", "设置"),
    ("appendix.html", "附录"),
]

NAV_EN = [
    ("index.html", "Home"),
    ("getting-started.html", "Getting Started"),
    ("projects.html", "Projects & Files"),
    ("view-3d.html", "3D View"),
    ("import.html", "Import"),
    ("scene.html", "Scene & Properties"),
    ("geometry.html", "Geometric Modeling"),
    ("drawing.html", "Engineering Drawing"),
    ("robot.html", "Robot Simulation"),
    ("trajectory.html", "Trajectories"),
    ("process-flow.html", "Process Flow"),
    ("pointcloud.html", "Point Cloud & Mesh"),
    ("geometry-plugin.html", "Geometry Plugin"),
    ("labeling.html", "Labeling & DL"),
    ("plc-camera.html", "PLC & Camera"),
    ("ai.html", "AI Assistant"),
    ("settings.html", "Settings"),
    ("appendix.html", "Appendix"),
]


FIGURES = {
    "index.html": ("overview.png", "CloudSim 能力域一览", "CloudSim capability domains"),
    "getting-started.html": ("ui-layout.png", "主窗口布局示意", "Main window layout"),
    "projects.html": ("projects.png", "工程打开与保存", "Project open and save"),
    "view-3d.html": ("view-3d.png", "三维交互与拾取模式", "3D interaction and pick modes"),
    "import.html": ("import.png", "模型 / 点云 / URDF 导入", "Model / cloud / URDF import"),
    "scene.html": ("scene.png", "对象树与属性面板", "Object tree and property panel"),
    "geometry.html": ("geometry.png", "几何建模工作区", "Geometric modeling workspace"),
    "drawing.html": ("drawing.png", "工程图纸多视图", "Engineering drawing views"),
    "robot.html": ("robot.png", "机器人仿真与指令", "Robot simulation and instructions"),
    "trajectory.html": ("trajectory.png", "轨迹生成与编辑流水线", "Trajectory generation pipeline"),
    "process-flow.html": ("process-flow.png", "工艺流程节点示意", "Process flow nodes"),
    "pointcloud.html": ("pointcloud.png", "点云处理流程", "Point cloud workflow"),
    "geometry-plugin.html": ("geometry-plugin.png", "几何分析插件", "Geometry analysis plugin"),
    "labeling.html": ("labeling.png", "标注与 PointNet", "Labeling and PointNet"),
    "plc-camera.html": ("plc-camera.png", "PLC 与工业相机", "PLC and industrial camera"),
    "ai.html": ("ai.png", "AI 助手分域对话", "AI assistant domains"),
    "settings.html": ("settings.png", "主题、语言与模式", "Theme, language and modes"),
}


def figure_html(filename, lang):
    fig = FIGURES.get(filename)
    if not fig:
        return ""
    img, cap_zh, cap_en = fig
    cap = cap_zh if lang == "zh" else cap_en
    return f'''<figure class="help-figure">
  <img src="../images/{img}" alt="{cap}" width="680" />
  <figcaption>{cap}</figcaption>
</figure>
'''


def page(lang, filename, title, body, nav):
    footer = (
        '<p class="footer">CloudSim 用户帮助 · 左侧目录选择章节</p>'
        if lang == "zh"
        else '<p class="footer">CloudSim User Help · Select a chapter from the left TOC</p>'
    )
    css = "../styles.css"
    return f"""<!DOCTYPE html>
<html lang="{'zh-CN' if lang == 'zh' else 'en'}">
<head>
  <meta charset="utf-8" />
  <title>{title}</title>
  <link rel="stylesheet" href="{css}" />
</head>
<body>
{figure_html(filename, lang)}
{body}
{footer}
</body>
</html>
"""


PAGES_ZH = {}

PAGES_ZH["index.html"] = (
    "CloudSim 帮助",
    """
<h1>CloudSim 帮助文档</h1>
<p>CloudSim 是面向<strong>工业机器人仿真</strong>的桌面应用，覆盖三维场景、机器人编程与轨迹、几何建模、工程图纸、工艺流程仿真，以及点云/标注/PLC/工业相机等插件能力，并集成 AI 助手。</p>
<div class="note">通过菜单 <strong>帮助 → 帮助文档</strong> 打开本手册；界面语言切换后将自动选择中文或英文入口页。</div>
<h2>手册目录</h2>
<ol>
  <li><a href="getting-started.html">入门与主界面</a> — 菜单、Dock、工作区模式</li>
  <li><a href="projects.html">工程与文件</a> — 新建/打开/保存、.pcp 与侧车数据</li>
  <li><a href="view-3d.html">三维视图与拾取</a> — 视图模式、点线面拾取、Gizmo</li>
  <li><a href="import.html">导入模型、点云与机器人</a></li>
  <li><a href="scene.html">场景树、属性与坐标系</a></li>
  <li><a href="geometry.html">几何建模工作区</a></li>
  <li><a href="drawing.html">工程图纸工作区</a></li>
  <li><a href="robot.html">机器人编程与仿真</a></li>
  <li><a href="trajectory.html">轨迹生成与编辑</a></li>
  <li><a href="process-flow.html">工艺流程与离散事件仿真</a></li>
  <li><a href="pointcloud.html">点云与网格工具</a></li>
  <li><a href="geometry-plugin.html">几何分析插件</a></li>
  <li><a href="labeling.html">标注与深度学习</a></li>
  <li><a href="plc-camera.html">PLC 通讯与工业相机</a></li>
  <li><a href="ai.html">AI 助手</a></li>
  <li><a href="settings.html">设置（主题、语言、模式）</a></li>
  <li><a href="appendix.html">附录：格式与快捷键</a></li>
</ol>
<h2>能力域一览</h2>
<table>
  <tr><th>能力域</th><th>说明</th></tr>
  <tr><td>三维场景</td><td>OSG 渲染、拾取、Gizmo 变换、标注</td></tr>
  <tr><td>机器人</td><td>URDF 导入、关节运动学、指令编程、轨迹规划与品牌程序导出</td></tr>
  <tr><td>几何引擎</td><td>OCC B-rep、草图特征、点云与网格后处理</td></tr>
  <tr><td>AI 助手</td><td>分域对话：建模、轨迹、工艺、点云等</td></tr>
  <tr><td>插件</td><td>动态加载，扩展点云、标注、PLC、相机、几何分析等</td></tr>
</table>
""",
)

PAGES_ZH["getting-started.html"] = (
    "入门与界面",
    """
<h1>入门与主界面</h1>
<h2>启动后布局</h2>
<ul>
  <li><strong>中央</strong>：文档标签页中的三维视口；下方为运行日志（Run Info）</li>
  <li><strong>左侧</strong>：对象/单元树（Units）、场景结构</li>
  <li><strong>右侧</strong>：工作区标签（属性、设备、仿真相关页、插件页、AI 助手等）</li>
  <li><strong>顶栏</strong>：工作区模式切换（主程序 / 几何建模 / 工艺流程 / 工程图）</li>
</ul>
<div class="tip">工作区模式只切换工具与 Ribbon，<strong>不会清空场景对象</strong>；同一文档内的模型在各模式间共享。</div>
<h2>菜单栏</h2>
<table>
  <tr><th>菜单</th><th>主要功能</th></tr>
  <tr><td>文件</td><td>新建文档、打开/保存工程、打开模型、打开点云、退出</td></tr>
  <tr><td>视图</td><td>重置布局、左右面板显示、交互模式、Gizmo 局部/世界系</td></tr>
  <tr><td>插入</td><td>创建坐标系</td></tr>
  <tr><td>设置</td><td>模式切换、主题、语言</td></tr>
  <tr><td>帮助</td><td>帮助文档、关于 CloudSim</td></tr>
</table>
<p>部分插件还会注册 <strong>工具（Tools）</strong> 等菜单项。</p>
<h2>工作区模式</h2>
<ul>
  <li><strong>主程序</strong>：三维仿真、导入、机器人与通用插件侧栏</li>
  <li><strong>几何建模</strong>：特征树 + Ribbon 草图/实体特征</li>
  <li><strong>工艺流程</strong>：流程画布与 DES 仿真（替换中央三维区）</li>
  <li><strong>工程图</strong>：出图、标注、导出图纸</li>
</ul>
<p>也可通过 <strong>设置 → 模式切换</strong> 或快捷键 <code>Ctrl+1</code>～<code>Ctrl+4</code> 切换（插件已加载时可用）。</p>
<h2>仿真相关 Dock（主程序）</h2>
<p>典型标签包括：指令、关节轴、坐标系（工具/用户）、外部轴、轨迹编辑/生成、机器人通讯、碰撞等（以当前加载插件与布局为准）。</p>
<p>设备页提供 URDF 等机器人导入入口。</p>
""",
)

PAGES_ZH["projects.html"] = (
    "工程与文件",
    """
<h1>工程与文件</h1>
<h2>基本操作</h2>
<ul>
  <li><strong>新建</strong>：文件 → 新建，打开空白文档页</li>
  <li><strong>打开工程</strong>：文件 → 打开工程…</li>
  <li><strong>保存工程</strong>：文件 → 保存工程…</li>
</ul>
<h2>工程格式</h2>
<table>
  <tr><th>格式</th><th>说明</th></tr>
  <tr><td><code>.pcp</code></td><td>打包工程（zip 目录树），推荐分发与备份</td></tr>
  <tr><td><code>.json</code> / 工程 JSON</td><td>工程根描述：对象、层级边、相机、机器人程序等</td></tr>
</table>
<h2>工程内容概要</h2>
<ul>
  <li>场景对象与父子层级（edges）</li>
  <li>可见性、标注、相机位姿</li>
  <li>机器人运动学绑定与程序指令</li>
  <li>可选侧车：<code>processFlow</code>（工艺）、<code>geometricModeling</code>（建模）、<code>engineeringDrawing</code>（工程图）等</li>
</ul>
<div class="note">在工艺流程等工作区编辑后，请记得保存工程，侧车数据会一并写入。</div>
<h2>从对象树导出</h2>
<p>在左侧对象树右键，可将网格导出为 PLY，或将 B-rep 导出为 STEP（视对象类型而定）。</p>
""",
)

PAGES_ZH["view-3d.html"] = (
    "三维视图",
    """
<h1>三维视图与拾取</h1>
<h2>交互模式（视图菜单）</h2>
<table>
  <tr><th>模式</th><th>用途</th></tr>
  <tr><td>视图模式</td><td>旋转/平移/缩放浏览场景</td></tr>
  <tr><td>对象选择</td><td>选中后端对象，显示变换 Gizmo</td></tr>
  <tr><td>点选模式</td><td>在几何上拾取点</td></tr>
  <tr><td>线选择模式</td><td>拾取网格边/线</td></tr>
  <tr><td>面选择模式</td><td>拾取网格面</td></tr>
</table>
<h2>变换 Gizmo</h2>
<ul>
  <li><strong>变换：物体系</strong> — 沿对象局部轴</li>
  <li><strong>变换：世界系</strong> — 沿世界坐标轴</li>
  <li>典型操作：拖动平移；旋转手柄调整姿态（以当前 Gizmo 实现为准）</li>
</ul>
<p>松手提交后，属性面板中的位姿会与场景同步。</p>
<h2>布局</h2>
<ul>
  <li><strong>重置布局</strong>：恢复默认 Dock 排布</li>
  <li><strong>左侧面板 / 右侧面板</strong>：显示或隐藏侧栏</li>
</ul>
""",
)

PAGES_ZH["import.html"] = (
    "导入资源",
    """
<h1>导入模型、点云与机器人</h1>
<h2>打开模型</h2>
<p>文件 → 打开模型… 支持常见网格与 CAD 交换格式，例如：</p>
<ul>
  <li>网格：OBJ、STL、PLY、OFF、DXF、DAE、3DS、FBX 等</li>
  <li>B-rep：STEP、IGES 等</li>
</ul>
<p>导入部分网格格式时，会提示<strong>网格质量</strong>（粗 / 中 / 细），影响离散精度与性能。</p>
<h2>打开点云</h2>
<p>文件 → 打开点云… 支持 PLY、LAS、LAZ、XYZ 等。</p>
<p>也可将文件拖放到主窗口，由当前文档接收导入。</p>
<h2>导入机器人（URDF）</h2>
<ul>
  <li>在<strong>设备</strong>页或相关入口选择 URDF</li>
  <li>系统按连杆注册后端对象，并建立正运动学（FK）绑定</li>
  <li>导入后可在仿真面板中示教、编程与回放</li>
</ul>
<div class="tip">多机器人文档可同时存在多个机器人实例，各自独立维护关节与坐标系。</div>
""",
)

PAGES_ZH["scene.html"] = (
    "场景树与属性",
    """
<h1>场景树、属性与坐标系</h1>
<h2>对象树（Units）</h2>
<ul>
  <li>每个打开的文档对应一个根节点</li>
  <li>勾选框控制对象在场景中的可见性，并随工程保存</li>
  <li>选中对象后，右侧属性面板显示位姿、颜色等可编辑字段</li>
</ul>
<h2>属性面板</h2>
<ul>
  <li>编辑位姿 / 旋转 / 颜色等；数值编辑带防抖，避免拖动时频繁全量重建</li>
  <li>跟随（Follow）等关系可通过属性绑定父对象</li>
</ul>
<h2>插入坐标系</h2>
<p>插入 → 坐标系… 创建命名坐标系对象，用于场景参考或机器人工具/用户坐标系相关操作。</p>
<h2>标注</h2>
<p>场景中可创建/显示/隐藏标注；与拾取反馈配合，便于记录空间点信息。</p>
""",
)

PAGES_ZH["geometry.html"] = (
    "几何建模",
    """
<h1>几何建模工作区</h1>
<p>通过顶栏切换到<strong>几何建模</strong>模式（或设置 → 模式切换）。界面提供 Ribbon、特征树与参数侧栏。</p>
<h2>工作区能力</h2>
<ul>
  <li>特征树：编辑、隐藏、删除、回退、面上草图</li>
  <li>Undo / Redo</li>
  <li>命名参数（圆/线/椭圆及 Pad 深度、拔模角等）</li>
  <li>原点与基准平面（XY / XZ / YZ）可见性；多 Body</li>
  <li>工程侧车持久化建模历史</li>
</ul>
<h2>草图</h2>
<ul>
  <li>新建草图（面或原点平面）、结束草图</li>
  <li>绘制：直线、圆弧、圆、矩形、椭圆、正多边形、槽口、样条、构造线</li>
  <li>编辑：修剪、镜像、删除；求解与自由度（DOF）</li>
  <li>尺寸：长度、距离、半径、角度等</li>
  <li>约束：水平/竖直、重合、平行、垂直、等长、相切、对称、中点、固定、到原点等</li>
  <li>引用：投影边、转换实体、等距 Offset</li>
</ul>
<h2>实体特征</h2>
<table>
  <tr><th>特征</th><th>说明</th></tr>
  <tr><td>拉伸 Pad / 切除 Pocket</td><td>多种终止条件（盲孔、到面、中面、双向、贯通、到顶点、偏置等），可设拔模角</td></tr>
  <tr><td>扫描 / 扫描切除</td><td>沿路径扫掠；可选用模型边作路径</td></tr>
  <tr><td>旋转 / 旋转切除</td><td>绕轴旋转轮廓</td></tr>
  <tr><td>圆角 / 倒角</td><td>边修饰</td></tr>
  <tr><td>线性/圆周阵列、镜像</td><td>特征阵列与对称</td></tr>
  <tr><td>放样 / 抽壳 / 拔模</td><td>多截面与薄壁、独立拔模</td></tr>
  <tr><td>基准面</td><td>等距、三点、成角等</td></tr>
</table>
<h2>脚本建模</h2>
<p>支持特征历史 JSON 导入导出、Compose 文件，以及进程内 <code>cloudsim_geom</code> 脚本接口（详见产品脚本建模文档）。</p>
""",
)

PAGES_ZH["drawing.html"] = (
    "工程图纸",
    """
<h1>工程图纸工作区</h1>
<p>切换到<strong>工程图</strong>模式后，使用 Ribbon（视图 / 绘制 / 标注 / 修改 / 捕捉 / 输出）从三维 B-rep 生成二维图纸。右侧<strong>属性</strong>面板可改图层与 ByLayer / ByBlock 覆盖。</p>

<h2>进入与生成</h2>
<ul>
  <li>基于 HLR 的多视图：正视 / 俯视 / 右视、等轴测、剖视；第一角 / 第三角投影</li>
  <li>剖视：切线握柄可拖；再生时可驱动切面；支持<strong>半剖</strong>（正视一侧表达层裁剪）</li>
  <li>局部放大：父视图细节圆可拖改；局部视图随区域更新</li>
  <li>可选<strong>钉住投影</strong>，避免拖视图时投影约束被冲掉</li>
  <li>图幅 A0–A4 / 自定义、标题栏字段、内容比例；侧车 <code>engineeringDrawing</code> 随项目保存</li>
</ul>

<h2>绘制与修改</h2>
<ul>
  <li>草图：直线、矩形、圆、弧、样条；修剪 / 延伸 / 偏移 / 缩放 / 倒圆 / 倒角 / 打断 / 合并；拉伸；矩形阵列与环形阵列（含草图实体）</li>
  <li>对象捕捉：端点 / 中点 / 交点 / 圆心 / 垂足 / 最近点；正交与极轴</li>
  <li>拾取时高亮点（端点方框、中点三角等）与悬停线/实体，便于区分点选与线选</li>
</ul>

<h2>标注</h2>
<ul>
  <li>线性、角度、半径、直径；<strong>连续尺寸</strong>与<strong>基线尺寸</strong></li>
  <li>尺寸样式、公差覆盖；弱关联边键（移动视图时尽量跟边）</li>
  <li>引线注释、多行文字；<strong>粗糙度</strong>与<strong>形位公差框</strong>（轻量符号，DXF 以文字/几何近似）</li>
</ul>

<h2>图层、图块与互通</h2>
<ul>
  <li>用户图层：可见 / 锁定 / 冻结 / 可打印；实体 ByLayer / ByBlock / 覆盖</li>
  <li>建块、插入、炸开；图框属性上图（ATTDEF）；属性面板改参照</li>
  <li>DXF：圆 / 弧 / 块与属性往返；导出另支持 SVG、PDF；打印预览与 CTB 灰度表</li>
</ul>
""",
)

PAGES_ZH["robot.html"] = (
    "机器人仿真",
    """
<h1>机器人编程与仿真</h1>
<h2>指令类型</h2>
<p>在仿真指令面板中可添加并编辑常见指令，例如：</p>
<ul>
  <li>运动：PTP、直线 LINE、圆弧 ARC（多点示教）</li>
  <li>逻辑与等待：Wait、If / While</li>
  <li>IO：数字量 / 模拟量</li>
  <li>路径规划相关指令（PathPlan 等）</li>
</ul>
<h2>操作流程</h2>
<ul>
  <li>关节点动（Axis）与程序运行 / 停止 / 预览</li>
  <li>工具坐标系与用户坐标系：从 TCP 捕获、复位等</li>
  <li>TCP 拖拽示教：拖动末端，数值 IK 求解关节角</li>
  <li>外部轴（如导轨）：配置、搜索与联动回放</li>
  <li>碰撞检测：开关与规划采样；可写入工程</li>
</ul>
<h2>程序导出与通讯</h2>
<ul>
  <li>品牌程序导出：ABB、FANUC、AIR、汇川、ROKAE、线加热等（以当前导出器为准）</li>
  <li>机器人通讯页：经 Bridge 与实机/仿真控制器镜像（ABB / Fanuc / KUKA 等）</li>
  <li>支持多程序存储；指令编辑可撤销</li>
</ul>
""",
)

PAGES_ZH["trajectory.html"] = (
    "轨迹生成与编辑",
    """
<h1>轨迹生成与编辑</h1>
<h2>CAD / 网格轨迹生成</h2>
<ul>
  <li><strong>特征轨迹</strong>：从 CAD 特征规格离散为原始轨迹（RawTrajectory）</li>
  <li><strong>网格轨迹</strong>：截面、B 样条等会话式生成</li>
</ul>
<h2>轨迹编辑</h2>
<ul>
  <li>丰富的编辑算子（平移、投影到几何等，十余种）</li>
  <li>流水线草稿撤销；命名模板库导入/导出</li>
  <li>应用后可绑定到路径规划 / 机器人程序</li>
  <li>视口可叠加显示原始轨迹与指令轴</li>
</ul>
<div class="tip">AI 助手域 <code>trajectory.feature</code> 可协助识别焊缝/边并确认离散化。</div>
""",
)

PAGES_ZH["process-flow.html"] = (
    "工艺流程",
    """
<h1>工艺流程与离散事件仿真</h1>
<p>切换到<strong>工艺流程</strong>模式后，中央区域变为流程画布。</p>
<h2>建模</h2>
<ul>
  <li>节点类型：开始、工位、缓冲、仓库、传送带、装配、检测、结束，以及 AGV 等（以插件版本为准）</li>
  <li>连线、网格对齐、自动布局；导出流程 JSON</li>
  <li>作业集（JobSet）模板；可由路径生成</li>
</ul>
<h2>仿真（DES）</h2>
<ul>
  <li>调度规则：FIFO、SPT、LPT、EDD、CR 等</li>
  <li>批量、装配、MTBF、废品率；班次日历；到达分布（固定/指数）</li>
  <li>策略对比与甘特图；优化后仿真</li>
  <li>结果：汇总、利用率、甘特、轨迹 Trace、回放、CSV/JSON 导出</li>
</ul>
<p>流程数据保存在工程侧车 <code>processFlow</code> 中。</p>
""",
)

PAGES_ZH["pointcloud.html"] = (
    "点云与网格",
    """
<h1>点云与网格工具</h1>
<p>通过侧栏「点云」页或菜单 <strong>工具 → 点云</strong> 使用（需加载 PointCloud 插件）。</p>
<h2>常用能力</h2>
<ul>
  <li>导入与列表管理</li>
  <li>体素下采样；盒/球/折线裁剪</li>
  <li>配准：ICP、SPARE、SDF/DDF 非刚性等</li>
  <li>重建与导出 PLY</li>
  <li>VCG 网格：简化、Laplacian/Taubin 平滑、修复、重网格</li>
  <li>多阶段曲面重构（含 NURBS 相关管线）</li>
  <li>扫描点云驱动 CAD 模板 B-rep 更新</li>
  <li>特征构建页（管件/铸造等分阶段流程，视插件配置）</li>
</ul>
""",
)

PAGES_ZH["geometry-plugin.html"] = (
    "几何分析插件",
    """
<h1>几何分析插件</h1>
<p>Geometry 插件提供离散与求交等通用几何工具（侧栏入口，插件加载后可见）。</p>
<ul>
  <li>离散化 STEP 文件或场景中的 B-rep/网格</li>
  <li>密度控制：质量档、目标边长、面片数等</li>
  <li>边–面 / 面–面求交，并可在视口中拾取结果</li>
  <li>由交线生成 Tube / Ribbon 网格</li>
</ul>
""",
)

PAGES_ZH["labeling.html"] = (
    "标注与深度学习",
    """
<h1>标注与深度学习</h1>
<h2>点云标注（Labeling）</h2>
<ul>
  <li>定义标注类别</li>
  <li>工具：单击、笔刷、套索、擦除</li>
  <li>撤销/重做；导出 PLY + NPY（及数据集清单）</li>
  <li>可借助 PointNet 分割模型预标注</li>
</ul>
<h2>PointNet</h2>
<ul>
  <li>训练页：运行分类/分割训练流程</li>
  <li>亦可经 AI 助手域 <code>pointnet.classify</code> / <code>pointnet.segment</code> 调用</li>
</ul>
""",
)

PAGES_ZH["plc-camera.html"] = (
    "PLC 与相机",
    """
<h1>PLC 通讯与工业相机</h1>
<h2>PLC</h2>
<ul>
  <li>侧栏连接 AB Ethernet/IP、Modbus TCP 等</li>
  <li>标签表：读/写/轮询；多种显示格式</li>
</ul>
<h2>工业相机</h2>
<ul>
  <li>支持海康、Mech-Eye、模拟相机等后端</li>
  <li>手眼标定：Ensemble 与 Mech 官方候选；标定板检测</li>
  <li>可结合机器人 TCP 位姿 JSON 完成标定流程</li>
</ul>
""",
)

PAGES_ZH["ai.html"] = (
    "AI 助手",
    """
<h1>AI 助手</h1>
<p>右侧 <strong>AI 助手</strong> Dock：选择领域后进行对话。解析链路一般为规则 → 本地模型（如 Ollama）→ 远程 LLM（取决于配置）。</p>
<h2>常用领域</h2>
<table>
  <tr><th>领域</th><th>用途</th></tr>
  <tr><td>mesh.create / mesh.compose</td><td>基础实体创建与组合</td></tr>
  <tr><td>feature.compose</td><td>参数化特征链，同步到几何建模特征树</td></tr>
  <tr><td>trajectory.feature</td><td>轨迹特征识别与离散确认</td></tr>
  <tr><td>process.flow</td><td>自然语言描述生成/驱动工艺仿真摘要</td></tr>
  <tr><td>pointnet.*</td><td>点云分类与分割</td></tr>
  <tr><td>document.import</td><td>与文档导入相关的辅助（若已配置）</td></tr>
</table>
<div class="note">远程 LLM 与端点配置见可执行文件旁的 <code>ai_config.json</code>（可由默认模板生成）。尺寸不明时助手可能先追问再生成。</div>
""",
)

PAGES_ZH["settings.html"] = (
    "设置",
    """
<h1>设置</h1>
<table>
  <tr><th>项</th><th>说明</th></tr>
  <tr><td>模式切换</td><td>与顶栏工作区模式一致：主程序 / 几何建模 / 工艺流程 / 工程图</td></tr>
  <tr><td>主题</td><td>浅色 / 深色，写入本地设置并影响视口背景</td></tr>
  <tr><td>语言</td><td>English / 中文；菜单、Dock 与已适配插件文案同步；帮助文档切换 zh/en</td></tr>
</table>
""",
)

PAGES_ZH["appendix.html"] = (
    "附录",
    """
<h1>附录</h1>
<h2>常用快捷键</h2>
<table>
  <tr><th>快捷键</th><th>功能</th></tr>
  <tr><td>Ctrl+1 … Ctrl+4</td><td>切换工作区模式（主程序 / 几何建模 / 工艺流程 / 工程图）</td></tr>
</table>
<p class="muted">更多快捷键以当前版本菜单与插件为准。</p>
<h2>常见文件扩展名</h2>
<table>
  <tr><th>扩展名</th><th>用途</th></tr>
  <tr><td>.pcp</td><td>CloudSim 打包工程</td></tr>
  <tr><td>.json</td><td>工程描述、流程、配置等</td></tr>
  <tr><td>.urdf</td><td>机器人描述</td></tr>
  <tr><td>.step / .stp / .iges</td><td>CAD B-rep</td></tr>
  <tr><td>.obj / .stl / .ply</td><td>网格</td></tr>
  <tr><td>.las / .laz / .xyz</td><td>点云</td></tr>
  <tr><td>.svg / .dxf / .pdf</td><td>工程图导出</td></tr>
</table>
<h2>获取帮助</h2>
<ul>
  <li>菜单：帮助 → 帮助文档（本手册）</li>
  <li>菜单：帮助 → 关于 CloudSim</li>
</ul>
""",
)

# English pages — parallel structure, concise
PAGES_EN = {}

PAGES_EN["index.html"] = (
    "CloudSim Help",
    """
<h1>CloudSim Help</h1>
<p>CloudSim is a desktop application for <strong>industrial robot simulation</strong>, covering 3D scenes, robot programming and trajectories, geometric modeling, engineering drawings, process-flow DES, plus plugins for point clouds, labeling, PLC, cameras, and an AI assistant.</p>
<div class="note">Open via <strong>Help → Documentation</strong>. The UI language selects the <code>help/zh</code> or <code>help/en</code> entry.</div>
<h2>Contents</h2>
<ol>
  <li><a href="getting-started.html">Getting started &amp; UI</a></li>
  <li><a href="projects.html">Projects &amp; files</a></li>
  <li><a href="view-3d.html">3D view &amp; picking</a></li>
  <li><a href="import.html">Import assets &amp; robots</a></li>
  <li><a href="scene.html">Scene tree &amp; properties</a></li>
  <li><a href="geometry.html">Geometric modeling</a></li>
  <li><a href="drawing.html">Engineering drawings</a></li>
  <li><a href="robot.html">Robot simulation</a></li>
  <li><a href="trajectory.html">Trajectory generation &amp; editing</a></li>
  <li><a href="process-flow.html">Process flow</a></li>
  <li><a href="pointcloud.html">Point cloud &amp; mesh</a></li>
  <li><a href="geometry-plugin.html">Geometry analysis plugin</a></li>
  <li><a href="labeling.html">Labeling &amp; deep learning</a></li>
  <li><a href="plc-camera.html">PLC &amp; cameras</a></li>
  <li><a href="ai.html">AI assistant</a></li>
  <li><a href="settings.html">Settings</a></li>
  <li><a href="appendix.html">Appendix</a></li>
</ol>
<h2>Capability domains</h2>
<table>
  <tr><th>Domain</th><th>Description</th></tr>
  <tr><td>3D scene</td><td>OSG rendering, picking, gizmo, annotations</td></tr>
  <tr><td>Robot</td><td>URDF, FK/IK, instructions, trajectories, brand export</td></tr>
  <tr><td>Geometry</td><td>OCC B-rep, sketches/features, point-cloud &amp; mesh tools</td></tr>
  <tr><td>AI</td><td>Domain-scoped chat for modeling, trajectories, process, etc.</td></tr>
  <tr><td>Plugins</td><td>Dynamic DLLs for cloud, labeling, PLC, cameras, analysis</td></tr>
</table>
""",
)

PAGES_EN["getting-started.html"] = (
    "Getting Started",
    """
<h1>Getting started &amp; main window</h1>
<h2>Layout</h2>
<ul>
  <li><strong>Center</strong>: document tabs with 3D view; run log below</li>
  <li><strong>Left</strong>: object/units tree</li>
  <li><strong>Right</strong>: workspace tabs (properties, devices, simulation, plugins, AI)</li>
  <li><strong>Top bar</strong>: workspace modes (Main / Modeling / Process Flow / Drawing)</li>
</ul>
<div class="tip">Modes switch tools and ribbons; they do <strong>not</strong> clear scene objects.</div>
<h2>Menus</h2>
<table>
  <tr><th>Menu</th><th>Highlights</th></tr>
  <tr><td>File</td><td>New, open/save project, open model/point cloud, exit</td></tr>
  <tr><td>View</td><td>Reset layout, side panels, interaction modes, gizmo frame</td></tr>
  <tr><td>Insert</td><td>Coordinate frame</td></tr>
  <tr><td>Settings</td><td>Mode switch, theme, language</td></tr>
  <tr><td>Help</td><td>Documentation, About</td></tr>
</table>
<p>Plugins may add a <strong>Tools</strong> menu. Use <code>Ctrl+1</code>–<code>Ctrl+4</code> for workspace modes when available.</p>
""",
)

PAGES_EN["projects.html"] = (
    "Projects & Files",
    """
<h1>Projects &amp; files</h1>
<ul>
  <li><strong>New / Open / Save</strong> under File</li>
  <li><code>.pcp</code>: packaged zip project (recommended)</li>
  <li>JSON project root: objects, hierarchy edges, camera, robot programs</li>
  <li>Sidecars: <code>processFlow</code>, <code>geometricModeling</code>, <code>engineeringDrawing</code>, …</li>
</ul>
<p>Tree visibility is persisted. Export PLY/STEP from the object tree context menu when applicable.</p>
""",
)

PAGES_EN["view-3d.html"] = (
    "3D View",
    """
<h1>3D view &amp; picking</h1>
<table>
  <tr><th>Mode</th><th>Use</th></tr>
  <tr><td>View Mode</td><td>Navigate the scene</td></tr>
  <tr><td>Object Select</td><td>Select objects and show the transform gizmo</td></tr>
  <tr><td>Point / Line / Face Pick</td><td>Pick geometry elements</td></tr>
</table>
<p>Gizmo frame: <strong>Local</strong> vs <strong>World</strong>. Reset layout and toggle left/right panels from View.</p>
""",
)

PAGES_EN["import.html"] = (
    "Import",
    """
<h1>Import models, point clouds &amp; robots</h1>
<ul>
  <li><strong>Open Model</strong>: OBJ/STL/PLY/OFF/… and STEP/IGES; mesh quality prompt for some formats</li>
  <li><strong>Open Point Cloud</strong>: PLY/LAS/LAZ/XYZ; drag-and-drop also supported</li>
  <li><strong>URDF</strong>: via Devices page — per-link backends and FK binding</li>
</ul>
""",
)

PAGES_EN["scene.html"] = (
    "Scene & Properties",
    """
<h1>Scene tree, properties &amp; frames</h1>
<ul>
  <li>One units-tree root per document; visibility checkboxes persist</li>
  <li>Property panel: pose, color, follow targets, etc.</li>
  <li>Insert → Coordinate Frame… for reference frames</li>
  <li>Annotations can be shown/hidden with pick feedback</li>
</ul>
""",
)

PAGES_EN["geometry.html"] = (
    "Geometric Modeling",
    """
<h1>Geometric modeling workspace</h1>
<p>Switch to <strong>Geometric Modeling</strong> for ribbon, feature tree, and parameter panels.</p>
<ul>
  <li>Feature tree edit/hide/delete/rollback; Undo/Redo; named parameters</li>
  <li>Sketch tools: line, arc, circle, rect, ellipse, polygon, slot, spline, construction</li>
  <li>Dimensions &amp; constraints (H/V, coincident, parallel, …); project/convert/offset</li>
  <li>Solids: Pad/Pocket, Sweep, Revolve, Fillet/Chamfer, patterns, mirror, loft, shell, draft, datum planes</li>
  <li>Script modeling: history/compose JSON and <code>cloudsim_geom</code></li>
</ul>
""",
)

PAGES_EN["drawing.html"] = (
    "Engineering Drawing",
    """
<h1>Engineering drawing workspace</h1>
<p>Switch to <strong>Drawing</strong> mode and use the Ribbon (Views / Draw / Annotate / Modify / Snap / Output) to generate 2D sheets from B-rep. The right <strong>Properties</strong> panel edits layers and ByLayer / ByBlock overrides.</p>

<h2>Views &amp; generation</h2>
<ul>
  <li>HLR multi-views: front / top / right, isometric, sections; 1st / 3rd angle</li>
  <li>Section cut marks are draggable and can drive the plane on regenerate; <strong>half-section</strong> clips the front view expression</li>
  <li>Detail bubbles are editable; detail views refresh with the region</li>
  <li>Optional <strong>pin projection</strong> to keep projection constraints when dragging views</li>
  <li>Sheet sizes A0–A4 / custom, title block, scale; sidecar <code>engineeringDrawing</code></li>
</ul>

<h2>Draw &amp; modify</h2>
<ul>
  <li>Sketch lines, rect, circle, arc, spline; Trim / Extend / Offset / Scale / Fillet / Chamfer / Break / Join; Stretch; rectangular and polar arrays (including sketch entities)</li>
  <li>Osnap: end / mid / intersection / center / perpendicular / nearest; ortho and polar</li>
  <li>Pick feedback highlights snap points vs hovered edges/entities</li>
</ul>

<h2>Annotation</h2>
<ul>
  <li>Linear, angular, radius, diameter; <strong>continued</strong> and <strong>baseline</strong> dimensions</li>
  <li>Dim styles, tolerance overrides; weak edge keys when views move</li>
  <li>Leaders, MText; <strong>roughness</strong> and lightweight <strong>GD&amp;T</strong> frames (DXF as text/geometry approx.)</li>
</ul>

<h2>Layers, blocks &amp; interchange</h2>
<ul>
  <li>User layers (visible / locked / frozen / plot); entity ByLayer / ByBlock / override</li>
  <li>Block create / insert / explode; title-block attributes on sheet</li>
  <li>DXF round-trip for circles, arcs, blocks/attributes; also SVG / PDF; print preview and CTB table</li>
</ul>
""",
)

PAGES_EN["robot.html"] = (
    "Robot Simulation",
    """
<h1>Robot programming &amp; simulation</h1>
<ul>
  <li>Instructions: PTP, LINE, ARC, Wait, If/While, Dig/Analog IO, PathPlan, …</li>
  <li>Axis jog; run/stop/preview; tool/user frames from TCP</li>
  <li>TCP drag teach with IK; external axes; collision dock</li>
  <li>Brand program export; Robot Comm bridge (ABB/Fanuc/KUKA, …)</li>
</ul>
""",
)

PAGES_EN["trajectory.html"] = (
    "Trajectories",
    """
<h1>Trajectory generation &amp; editing</h1>
<ul>
  <li>CAD feature trajectories and mesh/section/B-spline sessions</li>
  <li>Many edit operators; draft undo; template library</li>
  <li>Apply into path planning / programs; viewport overlays</li>
  <li>AI domain <code>trajectory.feature</code> for assisted edge/weld workflows</li>
</ul>
""",
)

PAGES_EN["process-flow.html"] = (
    "Process Flow",
    """
<h1>Process flow &amp; DES</h1>
<ul>
  <li>Nodes: start, station, buffer, warehouse, conveyor, assembly, inspect, end, AGV, …</li>
  <li>Connect, layout, JobSet templates; export JSON</li>
  <li>DES rules FIFO/SPT/LPT/EDD/CR; shifts; arrivals; Gantt &amp; CSV/JSON results</li>
  <li>Persisted as <code>processFlow</code> sidecar</li>
</ul>
""",
)

PAGES_EN["pointcloud.html"] = (
    "Point Cloud & Mesh",
    """
<h1>Point cloud &amp; mesh tools</h1>
<ul>
  <li>Import, downsample, crop; ICP / SPARE / non-rigid registration</li>
  <li>Reconstruct; export PLY; VCG simplify/smooth/repair/remesh</li>
  <li>Multi-stage surface reconstruction; template B-rep update from scans</li>
</ul>
""",
)

PAGES_EN["geometry-plugin.html"] = (
    "Geometry Plugin",
    """
<h1>Geometry analysis plugin</h1>
<ul>
  <li>Discretize STEP or in-scene B-rep/mesh with density controls</li>
  <li>Edge–face / face–face intersection with viewport picking</li>
  <li>Tube / Ribbon meshes from intersections</li>
</ul>
""",
)

PAGES_EN["labeling.html"] = (
    "Labeling & DL",
    """
<h1>Labeling &amp; deep learning</h1>
<ul>
  <li>Label classes; Click / Brush / Lasso / Erase; undo; export PLY+NPY</li>
  <li>PointNet prelabel; training tab; AI domains classify/segment</li>
</ul>
""",
)

PAGES_EN["plc-camera.html"] = (
    "PLC & Camera",
    """
<h1>PLC &amp; industrial cameras</h1>
<ul>
  <li>PLC: AB EIP / Modbus TCP; tag read/write/poll</li>
  <li>Cameras: Hikvision, Mech-Eye, simulated; hand-eye calibration</li>
</ul>
""",
)

PAGES_EN["ai.html"] = (
    "AI Assistant",
    """
<h1>AI assistant</h1>
<p>Right-side dock with domain dropdown. Parse chain: rules → local (e.g. Ollama) → remote LLM.</p>
<table>
  <tr><th>Domain</th><th>Use</th></tr>
  <tr><td>mesh.* / feature.compose</td><td>Meshes and parametric feature chains</td></tr>
  <tr><td>trajectory.feature</td><td>Trajectory feature assist</td></tr>
  <tr><td>process.flow</td><td>Process-flow NL assist</td></tr>
  <tr><td>pointnet.*</td><td>Classify / segment</td></tr>
</table>
<p>Configure remote endpoints in <code>ai_config.json</code> next to the executable.</p>
""",
)

PAGES_EN["settings.html"] = (
    "Settings",
    """
<h1>Settings</h1>
<ul>
  <li><strong>Mode Switch</strong>: same as top workspace bar</li>
  <li><strong>Theme</strong>: Light / Dark</li>
  <li><strong>Language</strong>: English / 中文 (also selects help locale)</li>
</ul>
""",
)

PAGES_EN["appendix.html"] = (
    "Appendix",
    """
<h1>Appendix</h1>
<h2>Shortcuts</h2>
<table>
  <tr><th>Keys</th><th>Action</th></tr>
  <tr><td>Ctrl+1 … Ctrl+4</td><td>Workspace modes</td></tr>
</table>
<h2>Extensions</h2>
<table>
  <tr><th>Ext</th><th>Use</th></tr>
  <tr><td>.pcp</td><td>Packaged project</td></tr>
  <tr><td>.urdf</td><td>Robot description</td></tr>
  <tr><td>.step / .stl / .ply / .las</td><td>CAD / mesh / cloud</td></tr>
  <tr><td>.svg / .dxf / .pdf</td><td>Drawing export</td></tr>
</table>
""",
)


def write_lang(lang, nav, pages):
    out_dir = ROOT / lang
    out_dir.mkdir(parents=True, exist_ok=True)
    for filename, (title, body) in pages.items():
        html = page(lang, filename, title, body, nav)
        (out_dir / filename).write_text(html, encoding="utf-8")
        print("wrote", out_dir / filename)


write_lang("zh", NAV_ZH, PAGES_ZH)
write_lang("en", NAV_EN, PAGES_EN)
print("done", len(PAGES_ZH), "zh pages,", len(PAGES_EN), "en pages")
