/// @file UrdfRobotLoader.cpp
/// @brief 零位姿（q=0）下的 parent_T_child，仅由该关节的 URDF 决定，不依赖 jointAnglesRad 下标顺序

// UrdfRobotLoader：URDF 解析、FK、层级 OSG 场景；多机键前缀由上层加 backendId::
#include "UrdfRobotLoader.h"

#include "BackendVisualRegistry.h"
#include "MeshBackendData.h"
#include "RunLogger.h"

#include <QByteArray>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QMutex>
#include <QMutexLocker>
#include <QRegularExpression>
#include <QStringList>
#include <QVector>
#include <QXmlStreamReader>
#include <algorithm>
#include <cmath>
#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Adapters.h>
#include <osg/Array>
#include <osg/Depth>
#include <osg/GL>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/Math>
#include <osg/MatrixTransform>
#include <osg/Matrixd>
#include <osg/NodeVisitor>
#include <osg/PolygonOffset>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/ref_ptr>
#include <osgDB/ReadFile>
#include <osgText/Text>
#include <osgUtil/SmoothingVisitor>

namespace
{
std::string qstrToUtf8Std(const QString& s)
{
	const QByteArray utf8 = s.toUtf8();
	return std::string(utf8.constData(), static_cast<size_t>(utf8.size()));
}

// URDF 导入与网格烘焙说明（本匿名命名空间内实现）
//
// 坐标约定：URDF 遵循 ROS 右手系、Z 向上（REP-103）。本模块内部长度统一为毫米（全局 mm）：
// URDF 文件中 origin 的 xyz 为米，读入后换算为 mm；网格顶点默认按毫米文件坐标进入 mm 连杆系。
// 齐次矩阵将「网格文件顶点」变到「用于显示的世界坐标」。每个 link 的 mesh 顶点在该 link 局部系中定义，
// 经 <joint><origin> 链从根连杆累积到世界；<visual><origin> 为「视觉几何系相对连杆系」位姿，
// 即 worldFromMesh = worldFromLink * visualInLink * p_mesh（各量均为 mm）
//
// 若实际 OBJ 等文件把全部几何都放在「整机/基座」同一坐标系中，则与上述假设不符，仅靠 <visual>
// 的小幅 xyz/rpy 往往无法对齐，需在资源侧按连杆重导出或调整 URDF

// 若为 true：在输出到 OSG 前对整条链再乘 Rx(-90°)，使 URDF 的 +Z 对齐常见视窗的 +Y
// 调试时可关：保持 ROS Z 向上，便于与 RViz 对比；若 mesh 已与当前视窗约定一致也可关
//
// 与层级场景 buildHierarchicalRobotScene 的关系：此处若启用，仅在 RobotAssembly 下增加一层
// MatrixTransform（matRosWorldToOsgWorld），关节与 mesh 矩阵仍在 ROS/mm 系中计算，不会与
// urdfWorldFromMeshVertices / kUrdfBake* 叠加——后者用于 computeMeshWorldMatrices 等「矩阵导出」
// 路径；本函数不在层级构建里烘焙顶点，故不会出现「根旋转乘两次」
constexpr bool kApplyRosZUpToOsgYUp = false;

// 为 true（默认）：导入时将「从根到当前连杆的关节链」×「<visual><origin>」烘焙进顶点，
// 使静态显示与 URDF 一致，并与 computeMeshWorldMatrices / 关节滑条所用 FK 一致
// 仅当 mesh 已在单一装配坐标系中预摆好时再设为 false（详见 UrdfRobotLoader.h）
constexpr bool kUrdfBakeJointChainIntoMesh = true;

// 为 true（默认）：把 <visual><origin> 烘焙进顶点；适用于按连杆导出的 CAD mesh、位姿由 URDF 描述
// 当 kUrdfBakeJointChainIntoMesh 为 false 时：true 仍只应用 visual；false 则保持文件顶点不动
constexpr bool kUrdfBakeVisualOriginIntoMesh = true;

// URDF 文件中 \<joint\>\<visual\> 的 \<origin xyz\> 为米（REP-103）→ 内部毫米。
constexpr double kUrdfOriginXyzMetersToInternalMm = 1000.0;
// 网格顶点文件单位 → 内部毫米：STL 多为毫米时用 1.0；若网格文件已是米则用 1000.0（不读 URDF mesh scale 属性）
constexpr double kMeshFileVertexUnitsToInternalMm = 1;

// OSG SmoothingVisitor 会对已三角化的 STL/STEP 导出网格合并顶点并重算法线，工业网格上易出现「斑马纹 / 条纹状」错误着色。
// 插件读入的网格通常已带法线；需要更圆滑外观时可改为 true
constexpr bool kUrdfMeshUseSmoothingVisitor = false;

// 为 true 时 urdfDebugLogRevoluteJointSubtree 打印各关节子树包围球（默认关）
constexpr bool kUrdfDebugJointSubtreeDiagnostics = false;

// 为 true 时在转动关节处绘制旋转轴线（黄线）与关节原点标记（红球）；默认关，与层级运动学无关
constexpr bool kUrdfShowRevoluteJointDebugVisuals = false;

// 为 true 时使用 MeshBackendData 后端对象加载连杆几何（推荐，更快且支持属性编辑）
// 为 false 时回退到旧的 osgDB::readNodeFile 直接读取
constexpr bool kUseMeshBackendForLinks = true;
// 关节结构节点（JointOrigin/JointRotation/LinkShell）在非零 origin + 动态旋转时，
// 个别 OSG 版本会出现父包围球低估，导致整段子树被误剔除。关闭结构层裁剪可避免“子连杆消失”
// 几何节点仍保留裁剪，开销主要是层级遍历，不会禁用整机裁剪
constexpr bool kDisableCullingOnJointStructuralNodes = true;

// 连杆系原点处 RGB 坐标轴长度（mm）：无网格时用默认；有网格时按包围球半径比例缩放
constexpr double kUrdfLinkFrameAxisMmDefault = 100.0;
constexpr double kUrdfLinkFrameAxisMmMin = 25.0;
constexpr double kUrdfLinkFrameAxisMmMax = 500.0;

struct Mat4
{
	double m[16]{}; // 列主序（column-major），与 OpenGL/OSG 一致
};

Mat4 matIdentity()
{
	Mat4 r{};
	r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0;
	return r;
}

Mat4 matTranslate(double x, double y, double z)
{
	Mat4 r = matIdentity();
	r.m[12] = x;
	r.m[13] = y;
	r.m[14] = z;
	return r;
}

Mat4 matScale(double sx, double sy, double sz)
{
	Mat4 r = matIdentity();
	r.m[0] = sx;
	r.m[5] = sy;
	r.m[10] = sz;
	return r;
}

// URDF 中 rpy 的固定轴旋转：R = Rz(yaw) * Ry(pitch) * Rx(roll)，与常见 ROS/URDF 解析一致。
Mat4 matFromRpy(double roll, double pitch, double yaw)
{
	const double cr = std::cos(roll);
	const double sr = std::sin(roll);
	const double cp = std::cos(pitch);
	const double sp = std::sin(pitch);
	const double cy = std::cos(yaw);
	const double sy = std::sin(yaw);

	const double r00 = cy * cp;
	const double r01 = cy * sp * sr - sy * cr;
	const double r02 = cy * sp * cr + sy * sr;
	const double r10 = sy * cp;
	const double r11 = sy * sp * sr + cy * cr;
	const double r12 = sy * sp * cr - cy * sr;
	const double r20 = -sp;
	const double r21 = cp * sr;
	const double r22 = cp * cr;

	Mat4 r = matIdentity();
	r.m[0] = r00;
	r.m[4] = r01;
	r.m[8] = r02;
	r.m[1] = r10;
	r.m[5] = r11;
	r.m[9] = r12;
	r.m[2] = r20;
	r.m[6] = r21;
	r.m[10] = r22;
	return r;
}

Mat4 matMul(const Mat4& a, const Mat4& b)
{
	Mat4 r{};
	for (int col = 0; col < 4; ++col)
	{
		for (int row = 0; row < 4; ++row)
		{
			r.m[col * 4 + row] = a.m[0 * 4 + row] * b.m[col * 4 + 0] + a.m[1 * 4 + row] * b.m[col * 4 + 1] +
								 a.m[2 * 4 + row] * b.m[col * 4 + 2] + a.m[3 * 4 + row] * b.m[col * 4 + 3];
		}
	}
	return r;
}

void transformPoint(const Mat4& t, double& x, double& y, double& z)
{
	const double nx = t.m[0] * x + t.m[4] * y + t.m[8] * z + t.m[12];
	const double ny = t.m[1] * x + t.m[5] * y + t.m[9] * z + t.m[13];
	const double nz = t.m[2] * x + t.m[6] * y + t.m[10] * z + t.m[14];
	x = nx;
	y = ny;
	z = nz;
}

// <origin xyz="..." rpy="..."/> 的齐次变换：先平移再旋转，即 T = Translate(xyz) * R(rpy)，
// 对列向量 p 有 p' = R*p + xyz（与 URDF 常用实现一致）
Mat4 matFromXyzRpy(double x, double y, double z, double roll, double pitch, double yaw)
{
	return matMul(matTranslate(x, y, z), matFromRpy(roll, pitch, yaw));
}

// ROS/URDF 世界为 Z 上；许多 OSG 视窗为 Y 上。在管线末端可选乘 Rx(-90°)，使 URDF +Z 映射到视窗 +Y。
Mat4 matRosWorldToOsgWorld()
{
	static const double kHalfPi = 1.57079632679489661923;
	return matFromRpy(-kHalfPi, 0.0, 0.0);
}

// 根据 kUrdfBake* 开关，决定导入时烘焙到顶点上的「从 mesh 文件系到 URDF 世界」变换
// worldFromLink：根经关节链到当前连杆；visualInLink：<visual><origin>。
Mat4 urdfWorldFromMeshVertices(const Mat4& worldFromLink, const Mat4& visualInLink)
{
	if (kUrdfBakeJointChainIntoMesh)
	{
		return matMul(worldFromLink, visualInLink);
	}
	if (kUrdfBakeVisualOriginIntoMesh)
	{
		return visualInLink;
	}
	return matIdentity();
}

// 在 URDF 世界系结果上可选施加 ROS→OSG 轴向修正（见 kApplyRosZUpToOsgYUp）
Mat4 osgWorldFromUrdfMeshFrame(const Mat4& urdfWorldFromMesh)
{
	return kApplyRosZUpToOsgYUp ? matMul(matRosWorldToOsgWorld(), urdfWorldFromMesh) : urdfWorldFromMesh;
}

// 解析空格分隔的三个浮点数（用于 xyz、rpy）；空串视为 0,0,0
bool parseThreeDoubles(const QString& s, double& a, double& b, double& c)
{
	const QString t = s.trimmed();
	if (t.isEmpty())
	{
		a = b = c = 0.0;
		return true;
	}
	const QStringList parts = t.split(QRegularExpression(QStringLiteral("\\s+")), Qt::SkipEmptyParts);
	if (parts.size() < 3)
	{
		return false;
	}
	bool ok = false;
	a = parts[0].toDouble(&ok);
	if (!ok)
	{
		return false;
	}
	b = parts[1].toDouble(&ok);
	if (!ok)
	{
		return false;
	}
	c = parts[2].toDouble(&ok);
	return ok;
}

// 解析空格分隔的四个浮点数（用于 rgba）；空串视为 0,0,0,1
bool parseFourDoubles(const QString& s, double& a, double& b, double& c, double& d)
{
	const QString t = s.trimmed();
	if (t.isEmpty())
	{
		a = b = c = 0.0;
		d = 1.0;
		return true;
	}
	const QStringList parts = t.split(QRegularExpression(QStringLiteral("\\s+")), Qt::SkipEmptyParts);
	if (parts.size() < 4)
	{
		return false;
	}
	bool ok = false;
	a = parts[0].toDouble(&ok);
	if (!ok)
	{
		return false;
	}
	b = parts[1].toDouble(&ok);
	if (!ok)
	{
		return false;
	}
	c = parts[2].toDouble(&ok);
	if (!ok)
	{
		return false;
	}
	d = parts[3].toDouble(&ok);
	return ok;
}

static bool parseUrdfRgbaAttribute(const QString& rgba, float& r, float& g, float& b, float& a)
{
	double dr = 0.0;
	double dg = 0.0;
	double db = 0.0;
	double da = 1.0;
	if (!parseFourDoubles(rgba, dr, dg, db, da))
	{
		return false;
	}
	r = static_cast<float>(dr);
	g = static_cast<float>(dg);
	b = static_cast<float>(db);
	a = static_cast<float>(da);
	return true;
}

// 每个 link 仅解析第一个 <visual>：mesh 路径与 <visual><origin>（无则视为 0）
struct UrdfLinkVisual
{
	bool hasMesh = false;
	QString meshUri; // package:// 或相对 urdf 目录的路径，后续由 resolveMeshFilename 解析
	double vx = 0.0;
	double vy = 0.0;
	double vz = 0.0;
	double vr = 0.0; // rpy：roll
	double vp = 0.0; // pitch
	double vw = 0.0; // yaw
	// <visual><material><color rgba="r g b a"/>（0..1）；无则 hasMaterialColor=false，显示侧保留默认色
	bool hasMaterialColor = false;
	float matR = 0.0f;
	float matG = 0.0f;
	float matB = 0.0f;
	float matA = 1.0f;
	QString pendingMaterialName; // <material name="..."/> 引用，在 parseUrdfModel 内解析
};

static void applyUrdfVisualMaterialToBackend(MeshBackendData& backend, const UrdfLinkVisual& vis)
{
	if (!vis.hasMaterialColor)
	{
		return;
	}
	BackendColor color;
	color.r = vis.matR;
	color.g = vis.matG;
	color.b = vis.matB;
	color.a = vis.matA;
	backend.setColor(color);
}

static osg::Vec4 urdfVisualMaterialOsgColor(const UrdfLinkVisual& vis)
{
	if (vis.hasMaterialColor)
	{
		return osg::Vec4(vis.matR, vis.matG, vis.matB, vis.matA);
	}
	return osg::Vec4(0.65f, 0.82f, 0.95f, 1.0f);
}

// 读取 \<material\>：内联 \<color rgba\>，或仅 name 引用顶层材质
static void readMaterialColorBlock(QXmlStreamReader& xml, UrdfLinkVisual& out)
{
	const QString matName = xml.attributes().value(QStringLiteral("name")).toString().trimmed();
	bool inlineColor = false;
	while (xml.readNextStartElement())
	{
		if (xml.name() == QLatin1String("color"))
		{
			const QString rgba = xml.attributes().value(QStringLiteral("rgba")).toString();
			if (parseUrdfRgbaAttribute(rgba, out.matR, out.matG, out.matB, out.matA))
			{
				out.hasMaterialColor = true;
				inlineColor = true;
			}
			xml.skipCurrentElement();
		}
		else
		{
			xml.skipCurrentElement();
		}
	}
	if (!inlineColor && !matName.isEmpty())
	{
		out.pendingMaterialName = matName;
	}
}

static void applyMaterialRefIfNeeded(UrdfLinkVisual& vis,
									 const std::unordered_map<QString, UrdfLinkVisual>& namedMaterials)
{
	if (vis.hasMaterialColor || vis.pendingMaterialName.isEmpty())
	{
		return;
	}
	const auto it = namedMaterials.find(vis.pendingMaterialName);
	if (it != namedMaterials.end() && it->second.hasMaterialColor)
	{
		vis.matR = it->second.matR;
		vis.matG = it->second.matG;
		vis.matB = it->second.matB;
		vis.matA = it->second.matA;
		vis.hasMaterialColor = true;
	}
}

// 网格文件顶点 → 连杆系（内部 mm）：p_link = V * S * p_file。S 将文件顶点变到 mm；V 的平移来自 URDF 米（乘 kUrdfOriginXyzMetersToInternalMm）
// 例：xyz="-0.075 0 0"（米）→ 平移 -75 mm
Mat4 meshFileToLinkFrameFromVisual(const UrdfLinkVisual& vis)
{
	const Mat4 scaleM =
		matScale(kMeshFileVertexUnitsToInternalMm, kMeshFileVertexUnitsToInternalMm, kMeshFileVertexUnitsToInternalMm);
	return matMul(matFromXyzRpy(vis.vx * kUrdfOriginXyzMetersToInternalMm, vis.vy * kUrdfOriginXyzMetersToInternalMm,
								vis.vz * kUrdfOriginXyzMetersToInternalMm, vis.vr, vis.vp, vis.vw),
				  scaleM);
}

// <joint>：父连杆名、子连杆名、类型，以及 <joint><origin> 与转动轴、限位。
struct UrdfJoint
{
	QString name;
	QString type;
	QString parent;
	QString child;
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
	// 转动/连续关节的 axis（在关节坐标系中，默认 0,0,1）
	double ax = 0.0;
	double ay = 0.0;
	double az = 1.0;
	// <limit lower upper/>，revolute 单位为弧度
	bool hasLimit = false;
	double limitLower = 0.0;
	double limitUpper = 0.0;
};

// 读取单个 <visual> 内的 <origin xyz rpy> 与 <geometry><mesh filename="..."/>
void readVisualBlock(QXmlStreamReader& xml, UrdfLinkVisual& out)
{
	out = UrdfLinkVisual{};
	while (xml.readNextStartElement())
	{
		if (xml.name() == QLatin1String("origin"))
		{
			const QString xyz = xml.attributes().value(QStringLiteral("xyz")).toString();
			const QString rpy = xml.attributes().value(QStringLiteral("rpy")).toString();
			(void)parseThreeDoubles(xyz, out.vx, out.vy, out.vz);
			(void)parseThreeDoubles(rpy, out.vr, out.vp, out.vw);
			xml.skipCurrentElement();
		}
		else if (xml.name() == QLatin1String("geometry"))
		{
			while (xml.readNextStartElement())
			{
				if (xml.name() == QLatin1String("mesh"))
				{
					const QString fn = xml.attributes().value(QStringLiteral("filename")).toString();
					if (!fn.isEmpty())
					{
						out.meshUri = fn;
						out.hasMesh = true;
					}
					xml.skipCurrentElement();
				}
				else
				{
					xml.skipCurrentElement();
				}
			}
		}
		else if (xml.name() == QLatin1String("material"))
		{
			readMaterialColorBlock(xml, out);
		}
		else
		{
			xml.skipCurrentElement();
		}
	}
}

// SolidWorks / 手改 URDF 常在 name、link 属性上带首尾空白；与 <link name="..."> 键统一 trim，避免
//「parent/child 字符串看似不同却映射到同一 QHash 键」或相反导致查找失败。
QString normalizedUrdfName(const QString& s)
{
	return s.trimmed();
}

// 读取 <joint> 子元素：origin、parent、child、axis、limit
void readJointBlock(QXmlStreamReader& xml, UrdfJoint& j)
{
	while (xml.readNextStartElement())
	{
		if (xml.name() == QLatin1String("origin"))
		{
			const QString xyz = xml.attributes().value(QStringLiteral("xyz")).toString();
			const QString rpy = xml.attributes().value(QStringLiteral("rpy")).toString();
			(void)parseThreeDoubles(xyz, j.x, j.y, j.z);
			(void)parseThreeDoubles(rpy, j.roll, j.pitch, j.yaw);
			xml.skipCurrentElement();
		}
		else if (xml.name() == QLatin1String("parent"))
		{
			j.parent = normalizedUrdfName(xml.attributes().value(QStringLiteral("link")).toString());
			xml.skipCurrentElement();
		}
		else if (xml.name() == QLatin1String("child"))
		{
			j.child = normalizedUrdfName(xml.attributes().value(QStringLiteral("link")).toString());
			xml.skipCurrentElement();
		}
		else if (xml.name() == QLatin1String("axis"))
		{
			const QString xyz = xml.attributes().value(QStringLiteral("xyz")).toString();
			(void)parseThreeDoubles(xyz, j.ax, j.ay, j.az);
			xml.skipCurrentElement();
		}
		else if (xml.name() == QLatin1String("limit"))
		{
			const QString lo = xml.attributes().value(QStringLiteral("lower")).toString();
			const QString hi = xml.attributes().value(QStringLiteral("upper")).toString();
			bool okLo = false;
			bool okHi = false;
			j.limitLower = lo.toDouble(&okLo);
			j.limitUpper = hi.toDouble(&okHi);
			j.hasLimit = okLo && okHi;
			xml.skipCurrentElement();
		}
		else
		{
			xml.skipCurrentElement();
		}
	}
}

// 将 URDF 中的 mesh 引用解析为本地绝对路径：package:// 与 model:// 映射到 packageRoot 下相对路径，
// 其余相对路径相对 urdf 所在目录，绝对路径直接使用
QString resolveMeshFilename(const QString& uri, const QString& packageRoot, const QString& urdfDirPath)
{
	if (uri.startsWith(QStringLiteral("package://")))
	{
		QString rest = uri.mid(QStringLiteral("package://").length());
		const int slash = rest.indexOf(QLatin1Char('/'));
		if (slash >= 0)
		{
			const QString tail = rest.mid(slash + 1);
			return QDir(packageRoot).absoluteFilePath(tail);
		}
		return QString();
	}
	if (uri.startsWith(QStringLiteral("model://")))
	{
		QString rest = uri.mid(QStringLiteral("model://").length());
		const int slash = rest.indexOf(QLatin1Char('/'));
		if (slash >= 0)
		{
			const QString tail = rest.mid(slash + 1);
			return QDir(packageRoot).absoluteFilePath(tail);
		}
		return QString();
	}
	QFileInfo fi(uri);
	if (fi.isAbsolute())
	{
		return fi.absoluteFilePath();
	}
	return QDir(urdfDirPath).absoluteFilePath(uri);
}

// 三角网顶点缓冲：每连续三个 float 为 (x,y,z)，左乘齐次矩阵 t 烘焙到世界（或当前选定的目标系）
void transformTriangleSoup(std::vector<float>& soup, const Mat4& t)
{
	for (std::size_t i = 0; i + 2 < soup.size(); i += 3)
	{
		double x = soup[i];
		double y = soup[i + 1];
		double z = soup[i + 2];
		transformPoint(t, x, y, z);
		soup[i] = static_cast<float>(x);
		soup[i + 1] = static_cast<float>(y);
		soup[i + 2] = static_cast<float>(z);
	}
}

// 内部 Mat4（列主序，列向量左乘 M*v）转为 osg::Matrixd（行向量右乘 v*M）
// 使用一致转置映射：C(M)=M^T，即 o(r,c)=M(c,r)=m[r*4+c]
// 在该映射下组合满足 C(A*B)=C(B)*C(A)（顺序反转）
osg::Matrixd mat4ToOsg(const Mat4& m)
{
	osg::Matrixd o;

	for (int r = 0; r < 4; ++r)
	{
		for (int c = 0; c < 4; ++c)
		{
			o(r, c) = m.m[r * 4 + c];
		}
	}
	return o;
}

// 右手系：单位轴 (ax,ay,az)、转角 angle（弧度），列向量 v' = R*v
// 使用 Rodrigues 显式式，避免经 osg::Matrixd::rotate 再拷贝时与列主序/约定不一致导致深层关节绕错轴
Mat4 matAxisAngleRad(double ax, double ay, double az, double angle)
{
	const double len = std::sqrt(ax * ax + ay * ay + az * az);
	if (len > 1e-9)
	{
		ax /= len;
		ay /= len;
		az /= len;
	}
	else
	{
		ax = 0.0;
		ay = 0.0;
		az = 1.0;
	}
	const double c = std::cos(angle);
	const double s = std::sin(angle);
	const double t = 1.0 - c;
	const double tx = t * ax;
	const double ty = t * ay;
	const double tz = t * az;
	Mat4 out = matIdentity();
	const double r00 = tx * ax + c;
	const double r01 = tx * ay - s * az;
	const double r02 = tx * az + s * ay;
	const double r10 = ty * ax + s * az;
	const double r11 = ty * ay + c;
	const double r12 = ty * az - s * ax;
	const double r20 = tz * ax - s * ay;
	const double r21 = tz * ay + s * ax;
	const double r22 = tz * az + c;
	out.m[0] = r00;
	out.m[4] = r01;
	out.m[8] = r02;
	out.m[1] = r10;
	out.m[5] = r11;
	out.m[9] = r12;
	out.m[2] = r20;
	out.m[6] = r21;
	out.m[10] = r22;
	return out;
}

// 仅绕 URDF \<joint\>\<axis\> 的 R(q)，与场景图中 JointRotationMt 一致；不含 \<origin\>（平移/rpy 由 JointN 承担）
// 与 jointChildTransformForFk 中转动部分、computeJointTransformMatrices 输出共用，避免「先算整条 FK 再逆解剥离」
static Mat4 jointRevoluteRotationOnly(const UrdfJoint& j, double qRad)
{
	double ax = j.ax;
	double ay = j.ay;
	double az = j.az;
	const double len = std::sqrt(ax * ax + ay * ay + az * az);
	if (len > 1e-9)
	{
		ax /= len;
		ay /= len;
		az /= len;
	}
	else
	{
		ax = 0.0;
		ay = 0.0;
		az = 1.0;
	}
	return matAxisAngleRad(ax, ay, az, qRad);
}

static Mat4 jointPrismaticTranslationOnly(const UrdfJoint& j, double qMm)
{
	double ax = j.ax;
	double ay = j.ay;
	double az = j.az;
	const double len = std::sqrt(ax * ax + ay * ay + az * az);
	if (len > 1e-9)
	{
		ax /= len;
		ay /= len;
		az /= len;
	}
	else
	{
		ax = 0.0;
		ay = 0.0;
		az = 1.0;
	}
	Mat4 T = matIdentity();
	T.m[12] = ax * qMm;
	T.m[13] = ay * qMm;
	T.m[14] = az * qMm;
	return T;
}

// 正运动学用：子连杆相对父连杆的变换，即 parent_T_child = <joint><origin> * 绕 axis 的 q 角旋转（若为转动关节）
// \<joint\>\<origin\> xyz 按 REP-103 为米：乘 kUrdfOriginXyzMetersToInternalMm 得到内部 mm，与 visual、网格一致。axis 与 rpy 不缩放。
// jointAnglesRad 与 qIndex 须与树遍历顺序一致（见 computeMeshWorldMatricesFromModel / loadMeshHierarchyParts）。
Mat4 jointChildTransformForFk(const UrdfJoint& j, const QVector<double>& jointAnglesRad, int& qIndex)
{
	const Mat4 T_origin = matFromXyzRpy(j.x * kUrdfOriginXyzMetersToInternalMm, j.y * kUrdfOriginXyzMetersToInternalMm,
										j.z * kUrdfOriginXyzMetersToInternalMm, j.roll, j.pitch, j.yaw);
	const QString jt = j.type.toLower();
	if (jt == QLatin1String("revolute") || jt == QLatin1String("continuous"))
	{
		double q = 0.0;
		if (qIndex < jointAnglesRad.size())
		{
			q = jointAnglesRad[qIndex];
		}
		++qIndex;
		return matMul(T_origin, jointRevoluteRotationOnly(j, q));
	}
	if (jt == QLatin1String("prismatic"))
	{
		double q = 0.0;
		if (qIndex < jointAnglesRad.size())
		{
			// URDF prismatic 单位为米，内部 mm
			q = jointAnglesRad[qIndex] * kUrdfOriginXyzMetersToInternalMm;
		}
		++qIndex;
		return matMul(T_origin, jointPrismaticTranslationOnly(j, q));
	}
	// 只有转动/移动关节才递增 qIndex
	return T_origin;
}

/// 零位姿（q=0）下的 parent_T_child，仅由该关节的 URDF 决定，不依赖 jointAnglesRad 下标顺序
/// 用于 buildHierarchicalRobotScene 静态挂接；动态 FK 仍用 jointChildTransformForFk + 与 BFS 一致的 qIndex
static Mat4 jointChildTransformAtZeroConfiguration(const UrdfJoint& j)
{
	const Mat4 T_origin = matFromXyzRpy(j.x * kUrdfOriginXyzMetersToInternalMm, j.y * kUrdfOriginXyzMetersToInternalMm,
										j.z * kUrdfOriginXyzMetersToInternalMm, j.roll, j.pitch, j.yaw);

	const QString jt = j.type.toLower();
	if (jt == QLatin1String("revolute") || jt == QLatin1String("continuous"))
	{
		return matMul(T_origin, jointRevoluteRotationOnly(j, 0.0));
	}
	if (jt == QLatin1String("prismatic"))
	{
		return matMul(T_origin, jointPrismaticTranslationOnly(j, 0.0));
	}
	return T_origin;
}

/// 仅 \<joint\>\<origin\> xyz+rpy（米→mm），不含绕 axis 的 q；与 FK 中 T_origin 一致，用于关节坐标系可视化
static Mat4 jointOriginFixedTransform(const UrdfJoint& j)
{
	return matFromXyzRpy(j.x * kUrdfOriginXyzMetersToInternalMm, j.y * kUrdfOriginXyzMetersToInternalMm,
						 j.z * kUrdfOriginXyzMetersToInternalMm, j.roll, j.pitch, j.yaw);
}

/// parent link 与 child link 必须为不同名称；相同则父/子对应同一 OSG 容器，无法挂接（自环关节）。
bool jointConnectsDistinctLinks(const UrdfJoint& j)
{
	return j.parent != j.child;
}

// 解析 URDF：收集各 link 的首个 visual、全部 joint；urdfDirOut 为 urdf 文件目录；
// packageRootOut 默认为 urdf 的上一级目录（用于 package:// 解析）。
// 根连杆：优先在「从未作为 child 出现」的 link 中取 base_link / world / odom / base（不区分大小写），否则取任意非 child
bool parseUrdfModel(const QString& urdfFilePath, std::unordered_map<QString, UrdfLinkVisual>& linkVisuals,
					std::vector<UrdfJoint>& joints, QString& rootLink, QString& urdfDirOut, QString& packageRootOut,
					QString* errorMessage)
{
	linkVisuals.clear();
	joints.clear();
	std::unordered_map<QString, UrdfLinkVisual> namedMaterials;
	QFile f(urdfFilePath);
	if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Cannot open URDF: %1").arg(urdfFilePath);
		}
		return false;
	}

	const QFileInfo urdfFi(urdfFilePath);
	urdfDirOut = urdfFi.absolutePath();
	packageRootOut = QDir(urdfDirOut).absoluteFilePath(QStringLiteral(".."));

	QXmlStreamReader xml(&f);
	while (!xml.atEnd())
	{
		xml.readNext();
		if (!xml.isStartElement())
		{
			continue;
		}
		if (xml.name() == QLatin1String("link"))
		{
			const QString linkName = normalizedUrdfName(xml.attributes().value(QStringLiteral("name")).toString());
			UrdfLinkVisual vis;
			bool gotVisual = false; // 仅第一个 <visual> 生效，其余跳过（与多 visual 的 URDF 需注意）
			while (xml.readNextStartElement())
			{
				if (xml.name() == QLatin1String("visual") && !gotVisual)
				{
					readVisualBlock(xml, vis);
					gotVisual = true;
				}
				else
				{
					xml.skipCurrentElement();
				}
			}
			if (!linkName.isEmpty())
			{
				applyMaterialRefIfNeeded(vis, namedMaterials);
				linkVisuals[linkName] = vis;
			}
		}
		else if (xml.name() == QLatin1String("material"))
		{
			UrdfLinkVisual matVis;
			readMaterialColorBlock(xml, matVis);
			const QString matName = xml.attributes().value(QStringLiteral("name")).toString().trimmed();
			if (!matName.isEmpty() && matVis.hasMaterialColor)
			{
				namedMaterials[matName] = matVis;
			}
		}
		else if (xml.name() == QLatin1String("joint"))
		{
			UrdfJoint j;
			j.name = normalizedUrdfName(xml.attributes().value(QStringLiteral("name")).toString());
			j.type = xml.attributes().value(QStringLiteral("type")).toString();
			readJointBlock(xml, j);
			if (!j.parent.isEmpty() && !j.child.isEmpty())
			{
				joints.push_back(std::move(j));
			}
		}
	}
	if (xml.hasError())
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("URDF XML error: %1").arg(xml.errorString());
		}
		return false;
	}

	std::unordered_set<QString> childNames;
	for (const UrdfJoint& j : joints)
	{
		childNames.insert(j.child);
	}

	auto pickRootLink = [&]() -> QString
	{
		const QStringList preferred{
			QStringLiteral("base_link"),
			QStringLiteral("world"),
			QStringLiteral("odom"),
			QStringLiteral("base"),
		};
		for (const QString& want : preferred)
		{
			for (const auto& kv : linkVisuals)
			{
				if (kv.first.compare(want, Qt::CaseInsensitive) == 0 && !childNames.count(kv.first))
				{
					return kv.first;
				}
			}
		}
		for (const auto& kv : linkVisuals)
		{
			if (!childNames.count(kv.first))
			{
				return kv.first;
			}
		}
		return QString();
	};

	rootLink = pickRootLink();
	if (rootLink.isEmpty() && !linkVisuals.empty())
	{
		rootLink = linkVisuals.begin()->first;
	}
	if (rootLink.isEmpty())
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("URDF contains no links.");
		}
		return false;
	}
	return true;
}

// 解析结果缓存：供 FK、关节元数据、网格导入共用；jointsByParent 在首次构建时按父连杆名索引子关节列表
struct UrdfFkModelData
{
	std::unordered_map<QString, UrdfLinkVisual> linkVisuals;
	std::vector<UrdfJoint> joints;
	QString rootLink;
	QString urdfDir;
	QString packageRoot;
	std::unordered_map<QString, std::vector<UrdfJoint>> jointsByParent;
	/// 热路径索图：避免 Jacobian BFS 拷贝 QString
	QStringList linkNames;
	std::unordered_map<QString, int> linkNameToIndex;
	std::vector<std::vector<int>> childJointIndices;
};

static void buildFkIndexGraph(UrdfFkModelData& data)
{
	data.linkNames.clear();
	data.linkNameToIndex.clear();
	data.childJointIndices.clear();

	std::queue<QString> q;
	if (!data.rootLink.isEmpty())
	{
		q.push(data.rootLink);
	}
	while (!q.empty())
	{
		const QString link = q.front();
		q.pop();
		if (data.linkNameToIndex.find(link) != data.linkNameToIndex.end())
		{
			continue;
		}
		const int id = data.linkNames.size();
		data.linkNames.append(link);
		data.linkNameToIndex.emplace(link, id);
		const auto jit = data.jointsByParent.find(link);
		if (jit == data.jointsByParent.end())
		{
			continue;
		}
		for (const UrdfJoint& j : jit->second)
		{
			if (!jointConnectsDistinctLinks(j))
			{
				continue;
			}
			if (data.linkNameToIndex.find(j.child) == data.linkNameToIndex.end())
			{
				q.push(j.child);
			}
		}
	}
	for (const UrdfJoint& j : data.joints)
	{
		if (data.linkNameToIndex.find(j.parent) == data.linkNameToIndex.end())
		{
			const int id = data.linkNames.size();
			data.linkNames.append(j.parent);
			data.linkNameToIndex.emplace(j.parent, id);
		}
		if (data.linkNameToIndex.find(j.child) == data.linkNameToIndex.end())
		{
			const int id = data.linkNames.size();
			data.linkNames.append(j.child);
			data.linkNameToIndex.emplace(j.child, id);
		}
	}

	data.childJointIndices.assign(static_cast<size_t>(data.linkNames.size()), {});
	for (int ji = 0; ji < static_cast<int>(data.joints.size()); ++ji)
	{
		const UrdfJoint& j = data.joints[static_cast<size_t>(ji)];
		const auto pit = data.linkNameToIndex.find(j.parent);
		if (pit == data.linkNameToIndex.end())
		{
			continue;
		}
		data.childJointIndices[static_cast<size_t>(pit->second)].push_back(ji);
	}
}

struct UrdfModelCacheEntry
{
	std::shared_ptr<const UrdfFkModelData> model;
	qint64 lastModifiedMs = -1; // 与磁盘 mtime 一致则命中缓存，避免重复解析
};

static QMutex g_urdfModelCacheMutex;
static QHash<QString, UrdfModelCacheEntry> g_urdfModelCache;

// 缓存键：优先规范路径，减少同文件不同路径写法导致的重复解析
static QString urdfCacheKey(const QString& urdfFilePath)
{
	const QFileInfo fi(urdfFilePath);
	if (!fi.exists())
	{
		return fi.absoluteFilePath();
	}
	const QString c = fi.canonicalFilePath();
	return c.isEmpty() ? fi.absoluteFilePath() : c;
}

// 线程安全：按路径 + 修改时间缓存解析结果；未命中则 parseUrdfModel 并填充 jointsByParent
bool getOrCreateUrdfModel(const QString& urdfFilePath, std::shared_ptr<const UrdfFkModelData>& out,
						  QString* errorMessage)
{
	const QString key = urdfCacheKey(urdfFilePath);
	const QFileInfo fi(urdfFilePath);
	const qint64 mtime = fi.exists() ? fi.lastModified().toMSecsSinceEpoch() : -1;

	QMutexLocker lock(&g_urdfModelCacheMutex);
	const auto it = g_urdfModelCache.find(key);
	if (it != g_urdfModelCache.end() && it->model && it->lastModifiedMs == mtime)
	{
		out = it->model;
		return true;
	}

	std::unordered_map<QString, UrdfLinkVisual> linkVisuals;
	std::vector<UrdfJoint> joints;
	QString rootLink;
	QString urdfDir;
	QString packageRoot;
	if (!parseUrdfModel(urdfFilePath, linkVisuals, joints, rootLink, urdfDir, packageRoot, errorMessage))
	{
		return false;
	}

	auto data = std::make_shared<UrdfFkModelData>();
	data->linkVisuals = std::move(linkVisuals);
	data->joints = std::move(joints);
	data->rootLink = rootLink;
	data->urdfDir = urdfDir;
	data->packageRoot = packageRoot;
	for (const UrdfJoint& j : data->joints)
	{
		data->jointsByParent[j.parent].push_back(j);
	}
	buildFkIndexGraph(*data);

	out = data;
	UrdfModelCacheEntry ent;
	ent.model = data;
	ent.lastModifiedMs = mtime;
	g_urdfModelCache.insert(key, std::move(ent));
	return true;
}

// 按与导入时相同的 BFS 顺序遍历连杆树，用 jointAnglesRad 做正解；对每个带 mesh 的 link 输出
// 「从 mesh 文件系到当前显示世界」的 osg::Matrixd（完整累积位姿）。用于烘焙/相对绑定/旧后端根矩阵等。
// 关节滑条对应的 JointRotationMt 只应写入 R(q)：请用 computeJointTransformMatrices，勿把本函数的连杆世界矩阵当作关节旋转节点矩阵
// 与 loadMeshHierarchyParts 里 kUrdfBake* 烘焙顶点无关
void computeMeshWorldMatricesFromModel(const UrdfFkModelData& model, const QVector<double>& jointAnglesRad,
									   QHash<QString, osg::Matrixd>& outLinkNameToMeshWorld,
									   bool meshVerticesAlreadyInLinkFrame)
{
	outLinkNameToMeshWorld.clear();
	const QVector<double>& angles = jointAnglesRad;

	struct QueueItem
	{
		QString link;
		Mat4 worldFromLink;
	};
	std::queue<QueueItem> q;
	QueueItem start{};
	start.link = model.rootLink;
	start.worldFromLink = matIdentity();
	q.push(start);
	int qIndex = 0;
	while (!q.empty())
	{
		const QueueItem cur = q.front();
		q.pop();

		auto visIt = model.linkVisuals.find(cur.link);
		if (visIt != model.linkVisuals.end() && visIt->second.hasMesh)
		{
			const UrdfLinkVisual& vis = visIt->second;
			const Mat4 visualInLink =
				meshVerticesAlreadyInLinkFrame ? matIdentity() : meshFileToLinkFrameFromVisual(vis);
			const Mat4 urdfWorldFromMesh = matMul(cur.worldFromLink, visualInLink);
			const Mat4 osgWorldFromMesh = osgWorldFromUrdfMeshFrame(urdfWorldFromMesh);
			outLinkNameToMeshWorld.insert(cur.link, mat4ToOsg(osgWorldFromMesh));
		}

		const auto jit = model.jointsByParent.find(cur.link);
		if (jit == model.jointsByParent.end())
		{
			continue;
		}
		for (const UrdfJoint& j : jit->second)
		{
			if (!jointConnectsDistinctLinks(j))
			{
				continue;
			}
			const Mat4 jointFromChild = jointChildTransformForFk(j, angles, qIndex);
			QueueItem nxt{};
			nxt.link = j.child;
			nxt.worldFromLink = matMul(cur.worldFromLink, jointFromChild);
			q.push(nxt);
		}
	}
}

void computeLinkWorldMatricesFromModel(const UrdfFkModelData& model, const QVector<double>& jointAnglesRad,
									   QHash<QString, osg::Matrixd>& outLinkNameToLinkWorld)
{
	outLinkNameToLinkWorld.clear();
	const QVector<double>& angles = jointAnglesRad;

	struct QueueItem
	{
		QString link;
		Mat4 worldFromLink;
	};
	std::queue<QueueItem> q;
	QueueItem start{};
	start.link = model.rootLink;
	start.worldFromLink = matIdentity();
	q.push(start);
	int qIndex = 0;
	while (!q.empty())
	{
		const QueueItem cur = q.front();
		q.pop();
		const Mat4 osgWorldFromLink = osgWorldFromUrdfMeshFrame(cur.worldFromLink);
		outLinkNameToLinkWorld.insert(cur.link, mat4ToOsg(osgWorldFromLink));

		const auto jit = model.jointsByParent.find(cur.link);
		if (jit == model.jointsByParent.end())
		{
			continue;
		}
		for (const UrdfJoint& j : jit->second)
		{
			if (!jointConnectsDistinctLinks(j))
			{
				continue;
			}
			const Mat4 jointFromChild = jointChildTransformForFk(j, angles, qIndex);
			QueueItem nxt{};
			nxt.link = j.child;
			nxt.worldFromLink = matMul(cur.worldFromLink, jointFromChild);
			q.push(nxt);
		}
	}
}

// loadRevoluteJointMeta 与 BFS 遍历共用：追加单个 revolute/continuous 关节的名称与弧度限位。
static void appendRevoluteJointMetaForJoint(const UrdfJoint& j, const QString& jtLower, QStringList& outJointNames,
											QVector<double>& outLowerRad, QVector<double>& outUpperRad)
{
	static constexpr double kPi = 3.14159265358979323846;
	static constexpr double kTwoPi = 6.2831853071795864769;

	const QString label = !j.name.isEmpty() ? j.name : QStringLiteral("joint_%1").arg(j.child);
	outJointNames.append(label);

	double lo = -kPi;
	double hi = kPi;
	if (j.hasLimit)
	{
		lo = j.limitLower;
		hi = j.limitUpper;
	}
	else if (jtLower == QLatin1String("continuous"))
	{
		lo = -kTwoPi;
		hi = kTwoPi;
	}
	if (lo > hi)
	{
		std::swap(lo, hi);
	}
	if (std::abs(hi - lo) < 1e-9)
	{
		hi = lo + 1e-3;
	}
	outLowerRad.append(lo);
	outUpperRad.append(hi);
}

} // namespace

// 按树遍历顺序列出 revolute/continuous 关节名及弧度上下限（无 limit 时 revolute 默认 ±π，continuous ±2π）
bool UrdfRobotLoader::loadRevoluteJointMeta(const QString& urdfFilePath, QStringList& outJointNames,
											QVector<double>& outLowerRad, QVector<double>& outUpperRad,
											QString* errorMessage)
{
	outJointNames.clear();
	outLowerRad.clear();
	outUpperRad.clear();

	std::shared_ptr<const UrdfFkModelData> model;
	if (!getOrCreateUrdfModel(urdfFilePath, model, errorMessage) || !model)
	{
		return false;
	}

	const UrdfFkModelData& m = *model;
	const std::unordered_map<QString, std::vector<UrdfJoint>>& jointsByParent = m.jointsByParent;

	struct QueueItem
	{
		QString link;
	};
	std::queue<QueueItem> qq;
	qq.push(QueueItem{m.rootLink});

	while (!qq.empty())
	{
		const QueueItem cur = qq.front();
		qq.pop();

		const auto jit = jointsByParent.find(cur.link);
		if (jit == jointsByParent.end())
		{
			continue;
		}

		for (const UrdfJoint& j : jit->second)
		{
			if (!jointConnectsDistinctLinks(j))
			{
				continue;
			}

			const QString jt = j.type.toLower();
			if (jt == QLatin1String("revolute") || jt == QLatin1String("continuous"))
			{
				appendRevoluteJointMetaForJoint(j, jt, outJointNames, outLowerRad, outUpperRad);
			}
			qq.push(QueueItem{j.child});
		}
	}
	return true;
}

bool UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(const QString& urdfFilePath, QStringList& outChildLinkNames,
														 QString* errorMessage)
{
	outChildLinkNames.clear();

	std::shared_ptr<const UrdfFkModelData> model;
	if (!getOrCreateUrdfModel(urdfFilePath, model, errorMessage) || !model)
	{
		return false;
	}

	const UrdfFkModelData& m = *model;
	const std::unordered_map<QString, std::vector<UrdfJoint>>& jointsByParent = m.jointsByParent;

	struct QueueItem
	{
		QString link;
	};
	std::queue<QueueItem> qq;
	qq.push(QueueItem{m.rootLink});

	while (!qq.empty())
	{
		const QueueItem cur = qq.front();
		qq.pop();

		const auto jit = jointsByParent.find(cur.link);
		if (jit == jointsByParent.end())
		{
			continue;
		}

		for (const UrdfJoint& j : jit->second)
		{
			if (!jointConnectsDistinctLinks(j))
			{
				continue;
			}
			const QString jt = j.type.toLower();
			if (jt == QLatin1String("revolute") || jt == QLatin1String("continuous"))
			{
				outChildLinkNames.append(j.child);
			}
			qq.push(QueueItem{j.child});
		}
	}
	return true;
}

bool UrdfRobotLoader::loadPrimaryTerminalLinkName(const QString& urdfFilePath, QString& outLinkName,
												  QString* errorMessage)
{
	outLinkName.clear();

	std::shared_ptr<const UrdfFkModelData> model;
	if (!getOrCreateUrdfModel(urdfFilePath, model, errorMessage) || !model)
	{
		return false;
	}

	const UrdfFkModelData& m = *model;
	const std::unordered_map<QString, std::vector<UrdfJoint>>& jointsByParent = m.jointsByParent;

	struct QueueItem
	{
		QString link;
		int depth = 0;
	};

	std::queue<QueueItem> qq;
	qq.push(QueueItem{m.rootLink, 0});
	int bestDepth = -1;
	QString bestLeaf;
	while (!qq.empty())
	{
		const QueueItem cur = qq.front();
		qq.pop();

		const auto jit = jointsByParent.find(cur.link);
		if (jit == jointsByParent.end() || jit->second.empty())
		{
			if (cur.depth > bestDepth)
			{
				bestDepth = cur.depth;
				bestLeaf = cur.link;
			}
			continue;
		}

		bool pushed = false;
		for (const UrdfJoint& j : jit->second)
		{
			if (!jointConnectsDistinctLinks(j))
			{
				continue;
			}
			qq.push(QueueItem{j.child, cur.depth + 1});
			pushed = true;
		}
		if (!pushed && cur.depth > bestDepth)
		{
			bestDepth = cur.depth;
			bestLeaf = cur.link;
		}
	}

	if (bestLeaf.isEmpty())
	{
		bestLeaf = m.rootLink;
	}
	if (bestLeaf.isEmpty())
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("URDF terminal link is empty.");
		}
		return false;
	}
	outLinkName = bestLeaf;
	return true;
}

bool UrdfRobotLoader::loadLinkChildToParentMap(const QString& urdfFilePath,
											   QHash<QString, QString>& outChildLinkToParentLinkName,
											   QString* errorMessage)
{
	outChildLinkToParentLinkName.clear();

	std::shared_ptr<const UrdfFkModelData> model;
	if (!getOrCreateUrdfModel(urdfFilePath, model, errorMessage) || !model)
	{
		return false;
	}

	for (const UrdfJoint& j : model->joints)
	{
		if (!jointConnectsDistinctLinks(j))
		{
			continue;
		}
		if (j.parent.isEmpty() || j.child.isEmpty())
		{
			continue;
		}
		outChildLinkToParentLinkName.insert(j.child, j.parent);
	}
	return true;
}

// 仅关节名列表，顺序与 loadRevoluteJointMeta 一致，供与 jointAnglesRad 对齐
bool UrdfRobotLoader::loadRevoluteJointNamesInOrder(const QString& urdfFilePath, QStringList& outJointNames,
													QString* errorMessage)
{
	QVector<double> lo;
	QVector<double> hi;
	return loadRevoluteJointMeta(urdfFilePath, outJointNames, lo, hi, errorMessage);
}

// 给定与各转动关节顺序一致的 jointAnglesRad（弧度），计算每个带 mesh 的连杆对应的 mesh→世界矩阵
bool UrdfRobotLoader::computeMeshWorldMatrices(const QString& urdfFilePath, const QVector<double>& jointAnglesRad,
											   QHash<QString, osg::Matrixd>& outLinkNameToMeshWorld,
											   QString* errorMessage, bool meshVerticesAlreadyInLinkFrame)
{
	std::shared_ptr<const UrdfFkModelData> model;
	if (!getOrCreateUrdfModel(urdfFilePath, model, errorMessage) || !model)
	{
		outLinkNameToMeshWorld.clear();
		return false;
	}
	computeMeshWorldMatricesFromModel(*model, jointAnglesRad, outLinkNameToMeshWorld, meshVerticesAlreadyInLinkFrame);
	return true;
}

bool UrdfRobotLoader::linkMeshFileToLinkColumnMajor16(const QString& urdfFilePath, const QString& linkName,
													  double outColumnMajor16[16], QString* errorMessage)
{
	std::shared_ptr<const UrdfFkModelData> model;
	if (!getOrCreateUrdfModel(urdfFilePath, model, errorMessage) || !model)
	{
		return false;
	}
	const auto it = model->linkVisuals.find(linkName);
	if (it == model->linkVisuals.end() || !it->second.hasMesh)
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("URDF has no mesh visual for link '%1'.").arg(linkName);
		}
		return false;
	}
	const Mat4 m = meshFileToLinkFrameFromVisual(it->second);
	for (int i = 0; i < 16; ++i)
	{
		outColumnMajor16[i] = m.m[i];
	}
	return true;
}

bool UrdfRobotLoader::linkMeshFileToLinkOsgMatrix(const QString& urdfFilePath, const QString& linkName,
												  osg::Matrixd& out, QString* errorMessage)
{
	std::shared_ptr<const UrdfFkModelData> model;
	if (!getOrCreateUrdfModel(urdfFilePath, model, errorMessage) || !model)
	{
		return false;
	}
	const auto it = model->linkVisuals.find(linkName);
	if (it == model->linkVisuals.end() || !it->second.hasMesh)
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("URDF has no mesh visual for link '%1'.").arg(linkName);
		}
		return false;
	}
	out = mat4ToOsg(meshFileToLinkFrameFromVisual(it->second));
	return true;
}

bool UrdfRobotLoader::computeLinkWorldMatrices(const QString& urdfFilePath, const QVector<double>& jointAnglesRad,
											   QHash<QString, osg::Matrixd>& outLinkNameToLinkWorld,
											   QString* errorMessage)
{
	std::shared_ptr<const UrdfFkModelData> model;
	if (!getOrCreateUrdfModel(urdfFilePath, model, errorMessage) || !model)
	{
		outLinkNameToLinkWorld.clear();
		return false;
	}
	computeLinkWorldMatricesFromModel(*model, jointAnglesRad, outLinkNameToLinkWorld);
	return true;
}

bool UrdfRobotLoader::computeLinkPoseAndGeometricJacobian(const QString& urdfFilePath,
														  const QVector<double>& jointAnglesRad,
														  const QString& linkName, double outPosMm[3],
														  double* outQuatXyzw, std::vector<double>& outJ_rowMajor,
														  const bool includeOrientation,
														  const double orientationWeight, QString* errorMessage)
{
	return computeLinkPoseAndGeometricJacobian(urdfFilePath, jointAnglesRad, linkName, outPosMm, outQuatXyzw,
											   outJ_rowMajor, includeOrientation, orientationWeight, errorMessage,
											   nullptr);
}

bool UrdfRobotLoader::computeLinkPoseAndGeometricJacobian(const QString& urdfFilePath,
														  const QVector<double>& jointAnglesRad,
														  const QString& linkName, double outPosMm[3],
														  double* outQuatXyzw, std::vector<double>& outJ_rowMajor,
														  const bool includeOrientation,
														  const double orientationWeight, QString* errorMessage,
														  UrdfKinematicsWorkspace* wsIn)
{
	outJ_rowMajor.clear();
	if (!outPosMm || linkName.isEmpty())
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Invalid link pose Jacobian arguments.");
		}
		return false;
	}
	std::shared_ptr<const UrdfFkModelData> model;
	if (!getOrCreateUrdfModel(urdfFilePath, model, errorMessage) || !model)
	{
		return false;
	}
	const auto lit = model->linkNameToIndex.find(linkName);
	if (lit == model->linkNameToIndex.end())
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Link '%1' not found in URDF.").arg(linkName);
		}
		return false;
	}
	const int targetLinkId = lit->second;

	UrdfKinematicsWorkspace& ws = wsIn ? *wsIn : threadLocalKinematicsWorkspace();
	const int nLinks = model->linkNames.size();
	ws.ensureCapacity(jointAnglesRad.size(), nLinks, includeOrientation ? 6 : 3);
	ws.jacCols.clear();
	ws.jacCols.reserve(static_cast<size_t>(jointAnglesRad.size()));

	struct QueueItem
	{
		int linkId = -1;
		Mat4 worldFromLink;
	};
	std::vector<QueueItem> queue;
	queue.reserve(static_cast<size_t>(nLinks));
	QueueItem start{};
	const auto rootIt = model->linkNameToIndex.find(model->rootLink);
	if (rootIt == model->linkNameToIndex.end())
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("URDF root link missing from index graph.");
		}
		return false;
	}
	start.linkId = rootIt->second;
	start.worldFromLink = matIdentity();
	queue.push_back(start);
	int qIndex = 0;
	bool foundLink = false;
	Mat4 targetWorldUrdf = matIdentity();
	size_t qi = 0;
	while (qi < queue.size())
	{
		const QueueItem cur = queue[qi++];
		if (cur.linkId == targetLinkId)
		{
			foundLink = true;
			targetWorldUrdf = cur.worldFromLink;
		}
		if (cur.linkId < 0 || cur.linkId >= static_cast<int>(model->childJointIndices.size()))
		{
			continue;
		}
		for (const int jointIdx : model->childJointIndices[static_cast<size_t>(cur.linkId)])
		{
			const UrdfJoint& j = model->joints[static_cast<size_t>(jointIdx)];
			if (!jointConnectsDistinctLinks(j))
			{
				continue;
			}
			const QString jt = j.type.toLower();
			const bool isRev = jt == QLatin1String("revolute") || jt == QLatin1String("continuous");
			const bool isPri = jt == QLatin1String("prismatic");
			if (isRev || isPri)
			{
				const Mat4 T_origin = jointOriginFixedTransform(j);
				const Mat4 worldJointUrdf = matMul(cur.worldFromLink, T_origin);
				const Mat4 worldJointOsg = osgWorldFromUrdfMeshFrame(worldJointUrdf);
				UrdfJacCol col{};
				col.prismatic = isPri;
				col.px = worldJointOsg.m[12];
				col.py = worldJointOsg.m[13];
				col.pz = worldJointOsg.m[14];
				const double ax = j.ax;
				const double ay = j.ay;
				const double az = j.az;
				col.zx = worldJointOsg.m[0] * ax + worldJointOsg.m[4] * ay + worldJointOsg.m[8] * az;
				col.zy = worldJointOsg.m[1] * ax + worldJointOsg.m[5] * ay + worldJointOsg.m[9] * az;
				col.zz = worldJointOsg.m[2] * ax + worldJointOsg.m[6] * ay + worldJointOsg.m[10] * az;
				const double zn = std::sqrt(col.zx * col.zx + col.zy * col.zy + col.zz * col.zz);
				if (zn > 1e-12)
				{
					col.zx /= zn;
					col.zy /= zn;
					col.zz /= zn;
				}
				ws.jacCols.push_back(col);
			}
			const Mat4 jointFromChild = jointChildTransformForFk(j, jointAnglesRad, qIndex);
			const auto cit = model->linkNameToIndex.find(j.child);
			if (cit == model->linkNameToIndex.end())
			{
				continue;
			}
			QueueItem nxt{};
			nxt.linkId = cit->second;
			nxt.worldFromLink = matMul(cur.worldFromLink, jointFromChild);
			queue.push_back(nxt);
		}
	}

	if (!foundLink)
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Link '%1' not found in URDF.").arg(linkName);
		}
		return false;
	}

	const Mat4 targetOsg = osgWorldFromUrdfMeshFrame(targetWorldUrdf);
	const osg::Matrixd targetMat = mat4ToOsg(targetOsg);
	const osg::Vec3d t = targetMat.getTrans();
	outPosMm[0] = t.x();
	outPosMm[1] = t.y();
	outPosMm[2] = t.z();
	if (outQuatXyzw)
	{
		const osg::Quat rq = targetMat.getRotate();
		outQuatXyzw[0] = rq.x();
		outQuatXyzw[1] = rq.y();
		outQuatXyzw[2] = rq.z();
		outQuatXyzw[3] = rq.w();
	}

	const int n = static_cast<int>(ws.jacCols.size());
	const int taskDim = includeOrientation ? 6 : 3;
	outJ_rowMajor.assign(static_cast<size_t>(taskDim * n), 0.0);
	const double pee[3] = {outPosMm[0], outPosMm[1], outPosMm[2]};
	for (int j = 0; j < n; ++j)
	{
		const UrdfJacCol& c = ws.jacCols[static_cast<size_t>(j)];
		if (c.prismatic)
		{
			outJ_rowMajor[static_cast<size_t>(0 * n + j)] = c.zx * kUrdfOriginXyzMetersToInternalMm;
			outJ_rowMajor[static_cast<size_t>(1 * n + j)] = c.zy * kUrdfOriginXyzMetersToInternalMm;
			outJ_rowMajor[static_cast<size_t>(2 * n + j)] = c.zz * kUrdfOriginXyzMetersToInternalMm;
		}
		else
		{
			const double rx = pee[0] - c.px;
			const double ry = pee[1] - c.py;
			const double rz = pee[2] - c.pz;
			outJ_rowMajor[static_cast<size_t>(0 * n + j)] = c.zy * rz - c.zz * ry;
			outJ_rowMajor[static_cast<size_t>(1 * n + j)] = c.zz * rx - c.zx * rz;
			outJ_rowMajor[static_cast<size_t>(2 * n + j)] = c.zx * ry - c.zy * rx;
			if (includeOrientation)
			{
				outJ_rowMajor[static_cast<size_t>(3 * n + j)] = c.zx * orientationWeight;
				outJ_rowMajor[static_cast<size_t>(4 * n + j)] = c.zy * orientationWeight;
				outJ_rowMajor[static_cast<size_t>(5 * n + j)] = c.zz * orientationWeight;
			}
		}
	}
	return true;
}

bool UrdfRobotLoader::computeLinkWorldRigidTransforms(const QString& urdfFilePath,
													  const QVector<double>& jointAnglesRad,
													  QHash<QString, engine::RigidTransform>& outLinkNameToLinkWorld,
													  QString* errorMessage)
{
	QHash<QString, osg::Matrixd> osgMats;
	if (!computeLinkWorldMatrices(urdfFilePath, jointAnglesRad, osgMats, errorMessage))
	{
		outLinkNameToLinkWorld.clear();
		return false;
	}
	outLinkNameToLinkWorld.clear();
	for (auto it = osgMats.constBegin(); it != osgMats.constEnd(); ++it)
	{
		outLinkNameToLinkWorld.insert(it.key(), engine::rigidTransformFromOsg(it.value()));
	}
	return true;
}

// 强制丢弃已缓存的解析结果（例如 URDF 在磁盘上被替换但路径与 mtime 处理异常时）
void UrdfRobotLoader::clearUrdfModelCache()
{
	QMutexLocker lock(&g_urdfModelCacheMutex);
	g_urdfModelCache.clear();
}

bool UrdfRobotLoader::enumerateLinkVisualMeshes(const QString& urdfFilePath, QString& outRootLink,
												QHash<QString, QString>& outLinkNameToAbsoluteMeshPath,
												QString* errorMessage)
{
	outRootLink.clear();
	outLinkNameToAbsoluteMeshPath.clear();

	std::shared_ptr<const UrdfFkModelData> model;
	if (!getOrCreateUrdfModel(urdfFilePath, model, errorMessage) || !model)
	{
		return false;
	}
	const UrdfFkModelData& m = *model;
	outRootLink = m.rootLink;
	for (const auto& kv : m.linkVisuals)
	{
		const QString& linkName = kv.first;
		const UrdfLinkVisual& vis = kv.second;
		if (!vis.hasMesh)
		{
			continue;
		}
		const QString absMesh = resolveMeshFilename(vis.meshUri, m.packageRoot, m.urdfDir);
		if (absMesh.isEmpty() || !QFile::exists(absMesh))
		{
			if (errorMessage)
			{
				*errorMessage = QStringLiteral("Mesh not found for link '%1': %2").arg(linkName, vis.meshUri);
			}
			return false;
		}
		outLinkNameToAbsoluteMeshPath.insert(linkName, absMesh);
	}
	return true;
}

bool UrdfRobotLoader::loadLinkVisualMaterialColors(const QString& urdfFilePath,
												   QHash<QString, BackendColor>& outLinkNameToColor,
												   QString* errorMessage)
{
	outLinkNameToColor.clear();

	std::shared_ptr<const UrdfFkModelData> model;
	if (!getOrCreateUrdfModel(urdfFilePath, model, errorMessage) || !model)
	{
		return false;
	}
	for (const auto& kv : model->linkVisuals)
	{
		const UrdfLinkVisual& vis = kv.second;
		if (!vis.hasMaterialColor)
		{
			continue;
		}
		BackendColor c;
		c.r = vis.matR;
		c.g = vis.matG;
		c.b = vis.matB;
		c.a = vis.matA;
		outLinkNameToColor.insert(kv.first, c);
	}
	return true;
}

// 计算给定关节角度下的 Joint 矩阵（与 buildHierarchicalRobotScene 中可 setMatrix 的节点一致）
// - revolute/continuous：输出仅绕 \<axis\> 的 R(q)（关节 MatrixTransform 无平移，\<origin\> 由 JointN 节点承担）
// - 其他类型：输出完整 parent_T_child
bool UrdfRobotLoader::computeJointTransformMatrices(const QString& urdfFilePath, const QVector<double>& jointAnglesRad,
													QHash<QString, osg::Matrixd>& outJointMatrices,
													QString* errorMessage)
{
	outJointMatrices.clear();
	std::shared_ptr<const UrdfFkModelData> model;
	if (!getOrCreateUrdfModel(urdfFilePath, model, errorMessage) || !model)
	{
		return false;
	}

	const UrdfFkModelData& m = *model;
	const std::unordered_map<QString, std::vector<UrdfJoint>>& jointsByParent = m.jointsByParent;
	const QString& rootLink = m.rootLink;

	struct QueueItem
	{
		QString link;
		Mat4 worldFromLink;
	};
	std::queue<QueueItem> q;
	QueueItem start{};
	start.link = rootLink;
	start.worldFromLink = matIdentity();
	q.push(start);

	int qIndex = 0;

	while (!q.empty())
	{
		const QueueItem cur = q.front();
		q.pop();

		const auto jit = jointsByParent.find(cur.link);
		if (jit == jointsByParent.end())
		{
			continue;
		}

		for (const UrdfJoint& j : jit->second)
		{
			if (!jointConnectsDistinctLinks(j))
			{
				continue;
			}
			const QString jtLower = j.type.toLower();
			Mat4 parent_T_child;
			if (jtLower == QLatin1String("revolute") || jtLower == QLatin1String("continuous"))
			{
				double q = 0.0;
				if (qIndex < jointAnglesRad.size())
				{
					q = jointAnglesRad[qIndex];
				}
				++qIndex;
				const Mat4 Rq = jointRevoluteRotationOnly(j, q);
				outJointMatrices[j.name] = mat4ToOsg(Rq);
				const Mat4 T_origin = jointOriginFixedTransform(j);
				parent_T_child = matMul(T_origin, Rq);
			}
			else
			{
				parent_T_child = jointChildTransformForFk(j, jointAnglesRad, qIndex);
				outJointMatrices[j.name] = mat4ToOsg(parent_T_child);
			}

			// 计算子 Link 的世界矩阵，继续 BFS
			Mat4 childWorldFromLink = matMul(cur.worldFromLink, parent_T_child);

			QueueItem nxt{};
			nxt.link = j.child;
			nxt.worldFromLink = childWorldFromLink;
			q.push(nxt);
		}
	}

	return !outJointMatrices.isEmpty();
}

// ============================================================================
// 三层分离模型场景图构建 - 核心实现
// 实现四阶段流程：资源预加载 -> 容器化场景图 -> 根节点组装 -> 状态重置
// ============================================================================

/// 辅助：将 URDF Mat4 内部格式转为 osg::Matrixd (已在匿名命名空间有 mat4ToOsg，这里复用)

// OSG 读入的 STL/OBJ 常无法线且默认不参与光照，与 BackendVisual 中 lit mesh 对齐：材质 + LIGHT0。
// Geode 与独立 Geometry（如部分 DAE 直接挂 Group）共用同一 StateSet 设置。
static void applyLitPlasticToStateSet(osg::StateSet* ss, const osg::Vec4& baseColor)
{
	if (!ss)
	{
		return;
	}
	osg::ref_ptr<osg::Material> mat = new osg::Material;
	const float amb = 0.22f;
	mat->setAmbient(osg::Material::FRONT_AND_BACK,
					osg::Vec4(baseColor.r() * amb, baseColor.g() * amb, baseColor.b() * amb, baseColor.a()));
	mat->setDiffuse(osg::Material::FRONT_AND_BACK, baseColor);
	mat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0.62f, 0.62f, 0.58f, 1.0f));
	mat->setShininess(osg::Material::FRONT_AND_BACK, 64.0f);
	const float em = 0.014f;
	mat->setEmission(osg::Material::FRONT_AND_BACK,
					 osg::Vec4(baseColor.r() * em, baseColor.g() * em, baseColor.b() * em, baseColor.a()));

	ss->setAttributeAndModes(mat.get(), osg::StateAttribute::ON);
	//ss->setMode(GL_COLOR_MATERIAL, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	ss->setMode(GL_LIGHTING, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	ss->setMode(GL_LIGHT0, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	ss->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
}

class UrdfMeshLightingVisitor : public osg::NodeVisitor
{
public:
	explicit UrdfMeshLightingVisitor(const osg::Vec4& defaultBaseColor)
		: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN), m_defaultBaseColor(defaultBaseColor)
	{
	}

	void apply(osg::Geometry& geometry) override
	{
		osg::Array* va = geometry.getVertexArray();
		if (va && va->getNumElements() >= 3U)
		{
			osg::Vec4 baseColor = m_defaultBaseColor;
			osg::Vec4Array* ca = dynamic_cast<osg::Vec4Array*>(geometry.getColorArray());
			if (ca && !ca->empty() && osg::getBinding(ca) == osg::Array::BIND_OVERALL)
			{
				baseColor = ca->front();
			}
			applyLitPlasticToStateSet(geometry.getOrCreateStateSet(), baseColor);
		}
		traverse(geometry);
	}

private:
	osg::Vec4 m_defaultBaseColor;

	// 仅向下遍历到 Drawable；实际材质在 apply(Geometry) 中设置，避免与 Geode 上整包材质重复叠加
	void apply(osg::Geode& geode) override { traverse(geode); }
};

static void finalizeUrdfImportedMeshRendering(osg::Node* root, const osg::Vec4& baseColor)
{
	if (!root)
	{
		return;
	}
	if (kUrdfMeshUseSmoothingVisitor)
	{
		osgUtil::SmoothingVisitor smoother;
		smoother.setCreaseAngle(osg::DegreesToRadians(60.0));
		root->accept(smoother);
	}
	UrdfMeshLightingVisitor lighter(baseColor);
	root->accept(lighter);
}

/// 辅助：加载 Mesh 文件为 OSG 节点
static osg::ref_ptr<osg::Node> loadMeshNode(const QString& filePath, QString* errorMessage)
{
	const QByteArray nativeEnc = QFile::encodeName(filePath);
	const std::string nativePath(nativeEnc.constData(), static_cast<std::size_t>(nativeEnc.size()));
	osg::ref_ptr<osg::Node> node = osgDB::readNodeFile(nativePath);
	if (!node)
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("OSG failed to load mesh: %1").arg(filePath);
		}
		return nullptr;
	}
	return node;
}

/// mesh 文件系 -> 连杆系（内部 mm）：\<visual\>\<origin\>（米→mm）× 顶点尺度（与 FK / computeMeshWorldMatrices 一致；不读 URDF mesh scale）
static osg::ref_ptr<osg::MatrixTransform> createLinkVisualFromMesh(const QString& linkName, const UrdfLinkVisual& vis,
																   osg::ref_ptr<osg::Node> meshNode)
{
	finalizeUrdfImportedMeshRendering(meshNode.get(), urdfVisualMaterialOsgColor(vis));
	const Mat4 meshToLink = meshFileToLinkFrameFromVisual(vis);
	osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
	mt->setName(linkName.toStdString() + "_Geometry");
	mt->setMatrix(mat4ToOsg(meshToLink));
	mt->addChild(meshNode.get());
	return mt;
}

// 连杆容器 Group；保持裁剪开，关裁剪曾致包围球膨胀+ zFar 裁黑屏
static osg::ref_ptr<osg::Group> createContainerLayer(const QString& linkName)
{
	osg::ref_ptr<osg::Group> container = new osg::Group;
	container->setName(linkName.toStdString() + "_Container");
	//container->setCullingActive(true);
	container->getOrCreateStateSet()->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
	return container;
}

// MeshBackend 路径：visual origin 烘焙到顶点；pose/rotation 须清零防重复
static osg::ref_ptr<osg::Node> createLinkVisualFromBackend(const QString& linkName, const UrdfLinkVisual& vis,
														   const QString& packageRoot, const QString& urdfDir,
														   QString* errorMessage)
{
	if (!vis.hasMesh)
	{
		return nullptr;
	}

	// 判空 mesh 路径
	const QString absMesh = resolveMeshFilename(vis.meshUri, packageRoot, urdfDir);
	if (absMesh.isEmpty() || !QFile::exists(absMesh))
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Mesh not found for link '%1': %2").arg(linkName, vis.meshUri);
		}
		return nullptr;
	}

	auto backend = std::make_shared<MeshBackendData>();
	backend->setName(linkName.toStdString());

	// 加载mesh文件（使用快速后端读取）
	QElapsedTimer timer;
	timer.start();

	std::string nativePath = absMesh.toStdString();
	std::string loadErr;
	bool loaded = backend->loadFromFile(nativePath, &loadErr);

	if (!loaded)
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Failed to load mesh for link '%1': %2")
								.arg(linkName)
								.arg(QString::fromStdString(loadErr));
		}
		RunLogger::warn(
			qstrToUtf8Std(QStringLiteral("[UrdfRobotLoader] Backend load failed for link='%1' mesh='%2' error='%3'")
							  .arg(linkName, absMesh, QString::fromStdString(loadErr))));
		return nullptr;
	}

	RunLogger::info(qstrToUtf8Std(QStringLiteral("[UrdfRobotLoader] Backend loaded link='%1' triangles=%2 timeMs=%3")
									  .arg(linkName)
									  .arg(static_cast<qulonglong>(backend->geometryElementCount()))
									  .arg(timer.elapsed())));

	applyUrdfVisualMaterialToBackend(*backend, vis);

	// 清零 pose/rotation，避免与 mesh→link 矩阵重复
	BackendVec3 zeroPose{0.0f, 0.0f, 0.0f};
	BackendVec3 zeroRot{0.0f, 0.0f, 0.0f};
	backend->setPose(zeroPose);
	backend->setRotation(zeroRot);

	// 计算visual origin变换矩阵（与旧方案完全一致）
	// p_link = meshToLink * p_file
	const Mat4 meshToLink = meshFileToLinkFrameFromVisual(vis);

	// 【调试输出】检查矩阵是否有异常值
	if (kUrdfDebugJointSubtreeDiagnostics)
	{
		qDebug().nospace() << "[UrdfBackendDiag] link " << linkName << " meshToLink[0,0]=" << meshToLink.m[0]
						   << " [1,1]=" << meshToLink.m[5] << " [2,2]=" << meshToLink.m[10] << " translate=("
						   << meshToLink.m[12] << "," << meshToLink.m[13] << "," << meshToLink.m[14] << ")";
	}

	// 使用BackendVisualRegistry创建OSG节点
	MeshVisualOptions options;
	options.showWireOutline = false;
	options.useSceneLighting = true;

	std::string err;
	osg::ref_ptr<osg::Node> visualNode = BackendVisualRegistry::buildMeshDisplayNode(*backend, options, &err);

	if (!visualNode)
	{
		if (errorMessage)
			*errorMessage = QString::fromStdString(err);
		return nullptr;
	}

	visualNode->setName(linkName.toStdString() + "_BackendVisual");

	osg::ref_ptr<osg::MatrixTransform> geometryXf = new osg::MatrixTransform;
	geometryXf->setName(linkName.toStdString() + "_Geometry");
	geometryXf->setMatrix(mat4ToOsg(meshToLink));
	geometryXf->addChild(visualNode.get());

	return geometryXf;
}

/// 连杆坐标系原点：X 红、Y 绿、Z 蓝（与 ROS / RViz 常见约定一致）；长度单位为 mm
/// 渲染状态与视图 compass 一致：关闭深度测试 / 深度始终通过，避免被三角网格遮挡
static osg::ref_ptr<osg::Geode> createLinkFrameAxesGeode(double axisLengthMm, const std::string& nodeName)
{
	const float L = static_cast<float>(std::max(1.0, axisLengthMm));
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	vertices->push_back(osg::Vec3(L, 0.0f, 0.0f));
	vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	vertices->push_back(osg::Vec3(0.0f, L, 0.0f));
	vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	vertices->push_back(osg::Vec3(0.0f, 0.0f, L));

	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.0f, 0.15f, 0.15f, 1.0f));
	colors->push_back(osg::Vec4(0.15f, 1.0f, 0.15f, 1.0f));
	colors->push_back(osg::Vec4(0.15f, 0.35f, 1.0f, 1.0f));

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setVertexArray(vertices.get());
	geom->setColorArray(colors.get(), osg::Array::BIND_PER_PRIMITIVE_SET);
	geom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2));
	geom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 2, 2));
	geom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 4, 2));

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->setName(nodeName.empty() ? "LinkFrameAxes" : nodeName);
	geode->addDrawable(geom.get());
	osg::StateSet* ss = geode->getOrCreateStateSet();
	ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	ss->setAttribute(new osg::LineWidth(2.5f));
	ss->setAttributeAndModes(new osg::PolygonOffset(-1.0f, -1.0f), osg::StateAttribute::ON);
	ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	osg::ref_ptr<osg::Depth> depth = new osg::Depth;
	depth->setFunction(osg::Depth::ALWAYS);
	depth->setWriteMask(false);
	ss->setAttributeAndModes(depth.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	return geode;
}

/// 在 \<joint\>\<origin\> 关节系下仅画 URDF \<axis\> 方向黄线（与 FK 转动轴一致）；可选在轴线端旁加文字标签
static osg::ref_ptr<osg::Geode> createJointRotationAxisLineGeode(double axisLengthMm, double jax, double jay,
																 double jaz, const std::string& nodeName,
																 const QString& axisLabelText = QString())
{
	const float L = static_cast<float>(std::max(1.0, axisLengthMm));
	double nx = jax;
	double ny = jay;
	double nz = jaz;
	const double alen = std::sqrt(nx * nx + ny * ny + nz * nz);
	if (alen > 1e-9)
	{
		nx /= alen;
		ny /= alen;
		nz /= alen;
	}
	else
	{
		nx = 0.0;
		ny = 0.0;
		nz = 1.0;
	}

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
	vertices->push_back(osg::Vec3(static_cast<float>(nx * L), static_cast<float>(ny * L), static_cast<float>(nz * L)));

	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back(osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));

	osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
	geom->setVertexArray(vertices.get());
	geom->setColorArray(colors.get(), osg::Array::BIND_PER_PRIMITIVE_SET);
	geom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2));

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->setName(nodeName.empty() ? "JointRotationAxis" : nodeName);
	geode->addDrawable(geom.get());

	if (!axisLabelText.isEmpty())
	{
		osg::ref_ptr<osgText::Text> label = new osgText::Text;
		// 不显式 setFont(磁盘路径)：无 osgdb_freetype 或路径经 osgDB 时会反复 “file not handled”；用 osgText 默认字体即可
		const float charSize = static_cast<float>(std::max(12.0, axisLengthMm * 0.22));
		label->setCharacterSize(charSize);
		label->setFontResolution(48, 48);
		label->setColor(osg::Vec4(1.0f, 1.0f, 0.35f, 1.0f));
		label->setBackdropType(osgText::Text::NONE);
		label->setAxisAlignment(osgText::TextBase::SCREEN);
		label->setAlignment(osgText::TextBase::LEFT_CENTER);
		const float pad = static_cast<float>(std::max(8.0, axisLengthMm * 0.08));
		label->setPosition(osg::Vec3(static_cast<float>(nx * (static_cast<double>(L) + pad)),
									 static_cast<float>(ny * (static_cast<double>(L) + pad)),
									 static_cast<float>(nz * (static_cast<double>(L) + pad))));
		label->setText(axisLabelText.toStdString());
		geode->addDrawable(label.get());
	}

	osg::StateSet* ss = geode->getOrCreateStateSet();
	ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	ss->setAttribute(new osg::LineWidth(2.5f));
	ss->setAttributeAndModes(new osg::PolygonOffset(-1.0f, -1.0f), osg::StateAttribute::ON);
	ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
	osg::ref_ptr<osg::Depth> depth = new osg::Depth;
	depth->setFunction(osg::Depth::ALWAYS);
	depth->setWriteMask(false);
	ss->setAttributeAndModes(depth.get(), osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

	return geode;
}

static double linkFrameAxisLengthMmFromMesh(osg::Node* meshNode)
{
	if (!meshNode)
	{
		return kUrdfLinkFrameAxisMmDefault;
	}
	const osg::BoundingSphere& bs = meshNode->getBound();
	if (!bs.valid())
	{
		return kUrdfLinkFrameAxisMmDefault;
	}
	const double r = static_cast<double>(bs.radius());
	if (!(r > 1e-9))
	{
		return kUrdfLinkFrameAxisMmDefault;
	}
	double len = r * 0.2;
	if (len < kUrdfLinkFrameAxisMmMin)
	{
		len = kUrdfLinkFrameAxisMmMin;
	}
	if (len > kUrdfLinkFrameAxisMmMax)
	{
		len = kUrdfLinkFrameAxisMmMax;
	}
	return len;
}

/// 辅助：创建 Joint 的运动学层 (MatrixTransform)
/// 非转动关节：矩阵为完整 parent_T_child。转动/连续关节：矩阵仅为 R(q)，与 computeJointTransformMatrices 一致
static osg::ref_ptr<osg::MatrixTransform> createJointTransform(const QString& jointName, const Mat4& parent_T_child)
{
	osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
	mt->setName(jointName.toStdString());
	mt->setMatrix(mat4ToOsg(parent_T_child));
	return mt;
}

/// 从所有父 Group 上摘除节点
/// 注意：removeChild 会对子节点 unref；若调用方仅用裸指针保存且场景是唯一引用，必须在调用本函数前用 osg::ref_ptr 先接住节点，否则会析构子节点并在循环内崩溃
static void detachNodeFromAllParents(osg::Node* node)
{
	if (!node)
	{
		return;
	}
	while (node->getNumParents() > 0)
	{
		osg::Group* p = node->getParent(0);
		if (!p)
		{
			break;
		}
		p->removeChild(node);
	}
}

/// 仅使用 osg::Group::addChild(Node*)；子连杆容器若已有父节点（重复 joint / 重入），先 detach 再挂，避免 getNumParents/addChild 失败
/// 注意：detach 期间若多线程 Viewer 正在遍历场景，可能竞态；本函数仅在导入构建阶段、主线程调用时安全
static bool attachLinkJointLink(osg::Group* parentLinkContainer, osg::MatrixTransform* jointMt,
								osg::Group* childLinkContainer, const QString& jointName, const QString& parentLinkName,
								const QString& childLinkName, QString* errorMessage)
{
	if (!parentLinkContainer || !jointMt || !childLinkContainer)
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Internal: null OSG node while building joint '%1'.").arg(jointName);
		}
		return false;
	}
	if (parentLinkContainer == childLinkContainer)
	{
		if (errorMessage)
		{
			*errorMessage =
				QStringLiteral(
					"Invalid URDF: joint '%1' parent link '%2' and child link '%3' map to the same scene node. "
					"Use two different link names for parent and child (self-loop joints are not supported).")
					.arg(jointName, parentLinkName, childLinkName);
		}
		return false;
	}

	// 子连杆 Group 仅由 QHash 裸指针引用；removeChild 会 unref，引用归零会析构节点。detach 期间必须用 ref_ptr 保持存活
	osg::ref_ptr<osg::Group> keepChildAlive(childLinkContainer);
	// 关节 MatrixTransform 由上层 jointMT ref_ptr 持有，detach 安全
	detachNodeFromAllParents(jointMt);
	detachNodeFromAllParents(childLinkContainer);

	if (!parentLinkContainer->addChild(jointMt))
	{
		if (errorMessage)
		{
			*errorMessage =
				QStringLiteral(
					"OSG addChild failed: parent link container -> joint '%1'. "
					"If OSG was built with ENSURE_CHILD_IS_UNIQUE, the joint node may already be under this parent.")
					.arg(jointName);
		}
		return false;
	}
	if (!jointMt->addChild(childLinkContainer))
	{
		if (errorMessage)
		{
			*errorMessage =
				QStringLiteral("OSG addChild failed: joint '%1' -> child link '%2' (duplicate child under same joint?)")
					.arg(jointName, childLinkName);
		}
		parentLinkContainer->removeChild(jointMt);
		return false;
	}
	if (kDisableCullingOnJointStructuralNodes)
	{
		jointMt->setCullingActive(false);
	}
	return true;
}

static void urdfDebugLogRevoluteJointSubtree(const QString& jointName, const Mat4& T_origin,
											 osg::MatrixTransform* jointOriginMt, osg::MatrixTransform* jointRotationMt,
											 osg::Group* linkShell, osg::Group* childLinkContainer,
											 osg::Node* linkGeometryOptional, osg::Node* axisVisual = nullptr)
{
	if (!kUrdfDebugJointSubtreeDiagnostics)
	{
		return;
	}
	const double tx = T_origin.m[12];
	const double ty = T_origin.m[13];
	const double tz = T_origin.m[14];
	const double tlen = std::sqrt(tx * tx + ty * ty + tz * tz);
	qDebug().nospace() << "[UrdfJointDiag] joint " << jointName << " T_origin translation(mm) xyz=(" << tx << "," << ty
					   << "," << tz << ") len=" << tlen;

	auto logBound = [&](const char* tag, osg::Node* n, bool checkParent = false)
	{
		if (!n)
		{
			qDebug() << "[UrdfJointDiag]" << tag << ": null";
			return;
		}
		n->dirtyBound();
		const osg::BoundingSphere& bs = n->getBound();
		qDebug().nospace() << "[UrdfJointDiag]" << tag << ": valid=" << bs.valid()
						   << " cullingActive=" << n->getCullingActive() << " radius=" << bs.radius();

		if (checkParent && n->getNumParents() > 0)
		{
			qDebug().nospace() << "[UrdfJointDiag]" << tag << " parent=" << n->getParent(0)->getName().c_str();
		}
	};
	logBound("JointN(jointOriginMt)", jointOriginMt, true);
	logBound("jointRotation(URDF name)", jointRotationMt, true);
	logBound("LinkN(linkShell)", linkShell, true);
	logBound("child link_Container", childLinkContainer, true);
	logBound("child link_Geometry", linkGeometryOptional, true);
	logBound("Axis_Visual", axisVisual, true);

	// 【关键调试】检查 jointOriginMt 的矩阵值
	if (jointOriginMt)
	{
		osg::Matrix m = jointOriginMt->getMatrix();
		qDebug().nospace() << "[UrdfJointDiag]JointN(jointOriginMt) matrix:"
						   << " [3,0]=" << m(3, 0) << " [3,1]=" << m(3, 1) << " [3,2]=" << m(3, 2);
	}
}

/// Parent_Link_Container -> JointN(T_origin) -> JointContent(Group) -> { Axis_Visual, jointRotation(R) -> Link壳 -> child }。
/// JointContent 避免 JointN(MatrixTransform) 直接多子节点时部分 OSG 父包围球合并错误；运动学仍为 T_origin*R(q)。
static bool attachRevoluteJointDecomposed(osg::Group* parentLinkContainer, osg::MatrixTransform* jointOriginMt,
										  osg::Node* axisVisualRoot, osg::MatrixTransform* jointRotationMt,
										  osg::Group* linkShell, osg::Group* childLinkContainer,
										  const QString& jointName, const QString& parentLinkName,
										  const QString& childLinkName, QString* errorMessage)
{
	if (!parentLinkContainer || !jointOriginMt || !jointRotationMt || !linkShell || !childLinkContainer)
	{
		if (errorMessage)
		{
			*errorMessage =
				QStringLiteral("Internal: null OSG node while building revolute joint '%1'.").arg(jointName);
		}
		return false;
	}
	if (parentLinkContainer == childLinkContainer)
	{
		if (errorMessage)
		{
			*errorMessage =
				QStringLiteral(
					"Invalid URDF: joint '%1' parent link '%2' and child link '%3' map to the same scene node. "
					"Use two different link names for parent and child (self-loop joints are not supported).")
					.arg(jointName, parentLinkName, childLinkName);
		}
		return false;
	}

	osg::ref_ptr<osg::Group> keepChildAlive(childLinkContainer);
	detachNodeFromAllParents(jointOriginMt);
	detachNodeFromAllParents(jointRotationMt);
	detachNodeFromAllParents(linkShell);
	detachNodeFromAllParents(childLinkContainer);

	if (!parentLinkContainer->addChild(jointOriginMt))
	{
		if (errorMessage)
		{
			*errorMessage =
				QStringLiteral(
					"OSG addChild failed: parent link container -> joint '%1'. "
					"If OSG was built with ENSURE_CHILD_IS_UNIQUE, the joint node may already exist under this parent.")
					.arg(jointName);
		}
		return false;
	}
	if (!linkShell->addChild(childLinkContainer))
	{
		if (errorMessage)
		{
			*errorMessage =
				QStringLiteral("OSG addChild failed: link shell -> child link '%1' (duplicate child under same shell?)")
					.arg(childLinkName);
		}
		parentLinkContainer->removeChild(jointOriginMt);
		return false;
	}
	if (!jointRotationMt->addChild(linkShell))
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("OSG addChild failed: joint rotation '%1' -> link shell (duplicate child "
										   "under same rotation node?)")
								.arg(jointName);
		}
		linkShell->removeChild(childLinkContainer);
		parentLinkContainer->removeChild(jointOriginMt);
		return false;
	}

	osg::ref_ptr<osg::Group> jointContent = new osg::Group;
	jointContent->setName((jointName + QStringLiteral("_JointContent")).toStdString());
	if (axisVisualRoot && !jointContent->addChild(axisVisualRoot))
	{
		if (errorMessage)
		{
			*errorMessage =
				QStringLiteral("OSG addChild failed: joint '%1' -> axis visual under JointContent").arg(jointName);
		}
		parentLinkContainer->removeChild(jointOriginMt);
		return false;
	}
	if (!jointContent->addChild(jointRotationMt))
	{
		if (errorMessage)
		{
			*errorMessage =
				QStringLiteral("OSG addChild failed: joint '%1' -> joint rotation under JointContent").arg(jointName);
		}
		parentLinkContainer->removeChild(jointOriginMt);
		return false;
	}
	if (!jointOriginMt->addChild(jointContent.get()))
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("OSG addChild failed: joint '%1' -> JointContent group").arg(jointName);
		}
		parentLinkContainer->removeChild(jointOriginMt);
		return false;
	}

	if (axisVisualRoot)
	{
		axisVisualRoot->dirtyBound();
	}
	jointOriginMt->dirtyBound();
	jointContent->dirtyBound();
	jointRotationMt->dirtyBound();
	linkShell->dirtyBound();
	childLinkContainer->dirtyBound();
	parentLinkContainer->dirtyBound();

	if (kDisableCullingOnJointStructuralNodes)
	{
		jointOriginMt->setCullingActive(false);
		jointContent->setCullingActive(false);
		jointRotationMt->setCullingActive(false);
		linkShell->setCullingActive(false);
	}
	else
	{
		jointOriginMt->setCullingActive(true);
		jointRotationMt->setCullingActive(true);
	}
	return true;
}

/// 构建层级化 URDF 机器人场景图（三层分离架构）
osg::Group* UrdfRobotLoader::buildHierarchicalRobotScene(const QString& urdfFilePath,
														 QHash<QString, osg::Node*>& outLinkToGeometry,
														 QHash<QString, osg::Group*>& outLinkToContainer,
														 QHash<QString, osg::MatrixTransform*>& outJointTransforms,
														 QString* errorMessage)
{
	// 清空输出参数
	outLinkToGeometry.clear();
	outLinkToContainer.clear();
	outJointTransforms.clear();

	// ========================================================================
	// 【第一阶段】资源预加载与标准化
	// ========================================================================
	std::shared_ptr<const UrdfFkModelData> model;
	if (!getOrCreateUrdfModel(urdfFilePath, model, errorMessage) || !model)
	{
		return nullptr;
	}

	const UrdfFkModelData& m = *model;
	const std::unordered_map<QString, UrdfLinkVisual>& linkVisuals = m.linkVisuals;
	const std::unordered_map<QString, std::vector<UrdfJoint>>& jointsByParent = m.jointsByParent;
	const QString& rootLink = m.rootLink;
	const QString& urdfDir = m.urdfDir;
	const QString& packageRoot = m.packageRoot;

	// 阶段 1 里若仅用局部 ref_ptr<Group>，循环末尾析构后连杆容器 refcount 归零会被 delete；
	// outLinkToContainer 存裸指针会悬空，后续 detach/addChild 在 osg 内崩溃。整段构建期间保持引用。
	std::vector<osg::ref_ptr<osg::Group>> keepLinkContainersAlive;
	keepLinkContainersAlive.reserve(static_cast<size_t>(linkVisuals.size()));

	// 预加载所有 Mesh 并创建几何体层和容器层
	for (const auto& kv : linkVisuals)
	{
		const QString& linkName = kv.first;
		const UrdfLinkVisual& vis = kv.second;

		if (!vis.hasMesh)
		{
			// 创建空容器（无几何体的 Link）
			osg::ref_ptr<osg::Group> container = createContainerLayer(linkName);
			// 连杆系原点坐标轴（暂时关闭，改用关节坐标系可视化）
			// container->addChild(createLinkFrameAxesGeode(
			//	kUrdfLinkFrameAxisMmDefault, linkName.toStdString() + "_LinkFrameAxes").get());
			keepLinkContainersAlive.push_back(container);
			outLinkToContainer[linkName] = container.get();
			continue;
		}

		// 解析 Mesh 文件路径
		const QString absMesh = resolveMeshFilename(vis.meshUri, packageRoot, urdfDir);
		if (absMesh.isEmpty() || !QFile::exists(absMesh))
		{
			if (errorMessage)
			{
				*errorMessage = QStringLiteral("Mesh not found for link '%1': %2").arg(linkName, vis.meshUri);
			}
			return nullptr;
		}

		// 【第二阶段】创建视觉容器层 (Container)
		osg::ref_ptr<osg::Group> container = createContainerLayer(linkName);

		// 几何层：根据开关选择后端加载或OSG直接加载
		osg::ref_ptr<osg::Node> geometryNode;
		if (kUseMeshBackendForLinks)
		{
			// 【优化方案】使用 MeshBackendData 后端对象加载
			// 优势：更快加载、支持属性编辑、可序列化
			QString backendErr;
			geometryNode = createLinkVisualFromBackend(linkName, vis, packageRoot, urdfDir, &backendErr);
			if (!geometryNode)
			{
				RunLogger::warn(
					qstrToUtf8Std(QStringLiteral("[UrdfRobotLoader] Backend loading failed for link='%1' error='%2'")
									  .arg(linkName, backendErr)));
				// 可选：回退到OSG直接读取
				RunLogger::info(qstrToUtf8Std(
					QStringLiteral("[UrdfRobotLoader] Falling back to OSG loading for link='%1'").arg(linkName)));
			}
		}

		if (!geometryNode)
		{
			// 【旧方案】OSG 直接读取（回退方案）
			osg::ref_ptr<osg::Node> meshNode = loadMeshNode(absMesh, errorMessage);
			if (!meshNode)
			{
				return nullptr; // errorMessage 已由 loadMeshNode 填充
			}
			// 几何层：仅 visual origin + 法线/光照
			osg::ref_ptr<osg::MatrixTransform> geometryXf = createLinkVisualFromMesh(linkName, vis, meshNode);
			geometryNode = geometryXf;
		}

		container->addChild(geometryNode.get());

		// 记录几何节点（用于后续更新输出）
		osg::MatrixTransform* geometryXf = dynamic_cast<osg::MatrixTransform*>(geometryNode.get());

		keepLinkContainersAlive.push_back(container);

		// 记录输出（存储原始指针，所有权由 keepLinkContainersAlive 与后续场景图保持）
		outLinkToGeometry[linkName] = geometryXf; // 可能为nullptr（如果是非MatrixTransform）
		outLinkToContainer[linkName] = container.get();
	}

	// ========================================================================
	// 【第二阶段】构建关节链（运动学层）
	//
	// URDF / RViz 约定：关节 j 连接 parent_link → child_link。场景图必须是
	//   parent_link 的 Container → Joint（子树根）→ child_link 的 Container。
	// 因此「link_1 下挂 joint_2（若 joint_2 的 parent 为 link_1）」是正确的，不是挂反
	// 父连杆的 Geometry 在阶段一已作为兄弟挂在同一 Container 下；转动关节只变换子连杆子树，
	// 不会绕错成「转 Joint 2 却转了 Link 1 的 mesh」——除非把父 mesh 误挂在关节节点之下
	// ========================================================================
	struct BfsItem
	{
		QString link;
		osg::Group* container = nullptr;
		Mat4 worldFromLink;
	};
	std::queue<BfsItem> q;

	// 初始化 BFS：根 Link
	auto rootIt = outLinkToContainer.find(rootLink);
	if (rootIt == outLinkToContainer.end())
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Root link container not found: %1").arg(rootLink);
		}
		return nullptr;
	}

	BfsItem start;
	start.link = rootLink;
	start.container = rootIt.value();
	start.worldFromLink = matIdentity();
	q.push(start);

	// BFS 中第几个「转动/连续关节」轴线可视化（与 loadRevoluteJointMeta / 滑块顺序一致，每个旁标注「第 N 个关节的旋转轴」）
	int revoluteJointAxisVisualIndex = 0;

	while (!q.empty())
	{
		const BfsItem cur = q.front();
		q.pop();

		// 查找从当前 Link 出发的 Joints
		const auto jit = jointsByParent.find(cur.link);
		if (jit == jointsByParent.end())
		{
			continue;
		}

		for (const UrdfJoint& j : jit->second)
		{
			if (!jointConnectsDistinctLinks(j))
			{
				continue;
			}
			// 静态零位姿：矩阵仅依赖该关节 URDF，不依赖 jointAnglesRad 下标（与 jointChildTransformForFk(q=0) 等价）
			const Mat4 parent_T_child = jointChildTransformAtZeroConfiguration(j);

			// 查找子级 Container（关节引用的 link 未在 URDF 中出现为 <link> 时无条目，禁止解引用 end()）
			auto childIt = outLinkToContainer.find(j.child);
			if (childIt == outLinkToContainer.end())
			{
				RunLogger::warn(qstrToUtf8Std(
					QStringLiteral("UrdfRobotLoader: skipping joint '%1' - child link container missing for '%2'")
						.arg(j.name, j.child)));
				continue;
			}

			auto parentIt = outLinkToContainer.find(j.parent);
			if (parentIt == outLinkToContainer.end())
			{
				RunLogger::warn(qstrToUtf8Std(
					QStringLiteral("UrdfRobotLoader: skipping joint '%1' - parent link container missing for '%2'")
						.arg(j.name, j.parent)));
				continue;
			}

			// jointsByParent[cur.link] 内关节必有 j.parent == cur.link；父容器以 BFS 当前连杆为准。
			if (j.parent != cur.link)
			{
				RunLogger::warn(qstrToUtf8Std(QStringLiteral("UrdfRobotLoader: joint '%1' parent '%2' != BFS link '%3'")
												  .arg(j.name, j.parent, cur.link)));
				continue;
			}
			osg::Group* parentContainer = cur.container;
			osg::Group* childContainer = childIt.value();

			//// 获取包围盒中心
			//osg::Vec3d parentCenter = parentContainer->getBound().center();
			//osg::Vec3d childCenter = childContainer->getBound().center();

			//// 分别打印 x, y, z
			//qWarning() << "Parent Center:" << parentCenter.x() << parentCenter.y() << parentCenter.z();
			//qWarning() << "Child Center:" << childCenter.x() << childCenter.y() << childCenter.z();

			if (!parentContainer || !childContainer)
			{
				continue;
			}
			if (parentIt.value() != parentContainer)
			{
				RunLogger::warn(qstrToUtf8Std(
					QStringLiteral("UrdfRobotLoader: joint '%1' parent container map vs BFS mismatch.").arg(j.name)));
			}

			const QString jtLower = j.type.toLower();
			const bool isRevolute = (jtLower == QLatin1String("revolute") || jtLower == QLatin1String("continuous"));
			const Mat4 T_origin_anchor = jointOriginFixedTransform(j);

			// 转动/连续：Parent -> JointN(T_origin) -> JointContent -> { Axis_Visual_N, JointRotation(R(q)) } -> LinkN -> Child_Container
			// 与 URDF parent_T_child = T_origin * R(q) 一致；动态更新只写 R(q) 到 URDF 关节名 MatrixTransform（与 computeJointTransformMatrices 一致）
			if (isRevolute)
			{
				const int jointVisIndex = revoluteJointAxisVisualIndex + 1;
				const QString axisLabel = QStringLiteral("第%1个关节的旋转轴").arg(jointVisIndex);
				++revoluteJointAxisVisualIndex;

				const Mat4 T_origin = T_origin_anchor;

				osg::ref_ptr<osg::MatrixTransform> jointOriginMt = new osg::MatrixTransform;
				jointOriginMt->setName(QStringLiteral("Joint%1").arg(jointVisIndex).toStdString());
				jointOriginMt->setMatrix(mat4ToOsg(T_origin));

				osg::ref_ptr<osg::Group> axisShell;
				if (kUrdfShowRevoluteJointDebugVisuals)
				{
					axisShell = new osg::Group;
					axisShell->setName(QStringLiteral("Axis_Visual_%1").arg(jointVisIndex).toStdString());
					axisShell->addChild(createJointRotationAxisLineGeode(
											kUrdfLinkFrameAxisMmDefault, j.ax, j.ay, j.az,
											(j.name + QStringLiteral("_JointAxis")).toStdString(), axisLabel)
											.get());
					osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere(osg::Vec3(0, 0, 0), 5.0f);
					osg::ref_ptr<osg::ShapeDrawable> sd = new osg::ShapeDrawable(sphere.get());
					sd->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
					osg::ref_ptr<osg::Geode> marker = new osg::Geode;
					marker->setName((j.name + QStringLiteral("_OriginMarker")).toStdString());
					marker->addDrawable(sd.get());
					marker->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
					axisShell->addChild(marker.get());
				}

				// 1. 获取或创建独立的状态集
				osg::ref_ptr<osg::StateSet> meshSS = childContainer->getOrCreateStateSet();

				// 2. 【关键】强制开启深度测试（防止拉丝/长条）
				meshSS->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
				meshSS->setRenderingHint(osg::StateSet::OPAQUE_BIN);

				// 3. 【关键】强制开启光照（防止变黑或像线条）
				meshSS->setMode(GL_LIGHTING, osg::StateAttribute::ON);
				meshSS->setMode(GL_LIGHT0, osg::StateAttribute::ON);

				osg::ref_ptr<osg::Group> linkShell = new osg::Group;
				linkShell->setName(QStringLiteral("Link%1").arg(jointVisIndex).toStdString());

				osg::ref_ptr<osg::MatrixTransform> jointMT = createJointTransform(j.name, matIdentity());
				osg::MatrixTransform* jointNode = jointMT.get();

				if (!attachRevoluteJointDecomposed(parentContainer, jointOriginMt.get(), axisShell.get(), jointNode,
												   linkShell.get(), childContainer, j.name, j.parent, j.child,
												   errorMessage))
				{
					return nullptr;
				}

				{
					osg::Node* childGeom = nullptr;
					const auto gIt = outLinkToGeometry.find(j.child);
					if (gIt != outLinkToGeometry.end())
					{
						childGeom = gIt.value();
					}
					urdfDebugLogRevoluteJointSubtree(j.name, T_origin, jointOriginMt.get(), jointNode, linkShell.get(),
													 childContainer, childGeom, axisShell.get());
				}

				outJointTransforms[j.name] = jointNode;
			}
			else
			{
				osg::ref_ptr<osg::MatrixTransform> jointMT = createJointTransform(j.name, parent_T_child);
				osg::MatrixTransform* jointNode = jointMT.get();
				if (!attachLinkJointLink(parentContainer, jointNode, childContainer, j.name, j.parent, j.child,
										 errorMessage))
				{
					return nullptr;
				}
				outJointTransforms[j.name] = jointNode;
			}

			// 计算子 Link 的世界矩阵，继续 BFS
			Mat4 childWorldFromLink = matMul(cur.worldFromLink, parent_T_child);
			BfsItem nxt;
			nxt.link = j.child;
			nxt.container = childIt.value();
			nxt.worldFromLink = childWorldFromLink;
			q.push(nxt);
		}
	}

	// ========================================================================
	// 【第三阶段】根节点组装与场景注入
	// ========================================================================
	osg::ref_ptr<osg::Group> robotAssembly = new osg::Group;
	robotAssembly->setName("RobotAssembly");

	// 坐标系转换：如果启用 ROS Z-up -> OSG Y-up，在根级应用一次转换
	// 这样所有关节矩阵保持在 ROS 坐标系，总根节点统一转换到 OSG 坐标系
	if (kApplyRosZUpToOsgYUp)
	{
		osg::ref_ptr<osg::MatrixTransform> coordTransform = new osg::MatrixTransform;
		coordTransform->setName("RosZUp_to_OsgYUp");
		coordTransform->setMatrix(mat4ToOsg(matRosWorldToOsgWorld()));
		robotAssembly->addChild(coordTransform);

		if (rootIt.value())
		{
			coordTransform->addChild(rootIt.value());
		}
	}
	else
	{
		if (rootIt.value())
		{
			robotAssembly->addChild(rootIt.value());
		}
	}

	// ========================================================================
	// 【第四阶段】状态重置与刷新
	// ========================================================================
	// 同步几何层矩阵（与缓存的 linkVisuals 一致，便于后续若扩展热重载）
	for (auto it = outLinkToGeometry.begin(); it != outLinkToGeometry.end(); ++it)
	{
		osg::MatrixTransform* mt = dynamic_cast<osg::MatrixTransform*>(it.value());
		if (!mt)
		{
			continue;
		}
		const QString linkName = it.key();
		auto vit = linkVisuals.find(linkName);
		if (vit != linkVisuals.end() && vit->second.hasMesh)
		{
			const UrdfLinkVisual& vis = vit->second;
			const Mat4 meshToLink = meshFileToLinkFrameFromVisual(vis);
			const osg::Matrixd expectedLocal = mat4ToOsg(meshToLink);
			mt->setMatrix(expectedLocal);
		}
		else
		{
			mt->setMatrix(osg::Matrixd::identity());
		}
		mt->dirtyBound();
	}

	// 强制刷新所有容器的包围盒
	for (auto it = outLinkToContainer.begin(); it != outLinkToContainer.end(); ++it)
	{
		if (it.value())
		{
			it.value()->dirtyBound();
			it.value()->getBound();
		}
	}

	// RobotAssembly 本身也刷新
	robotAssembly->dirtyBound();

	// 转移所有权：release() 使裸指针引用计数仍为 1，由调用方 addChild 或 osg::ref_ptr 承接；禁止手动 ref()+get()
	return robotAssembly.release();
}
