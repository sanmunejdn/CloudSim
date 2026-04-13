// UrdfRobotLoader.cpp — URDF 解析、模型缓存、正运动学矩阵与 OSG 层级场景构建。
// 多机器人实例由上层用「场景 backendId + "::" + 关节名」区分；本模块不持有实例 ID。
//
// 分段：模型缓存与 XML 解析 → FK 辅助 → 场景构建（buildHierarchicalRobotScene）→ 对外 API 包装。

#include "UrdfRobotLoader.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QMutex>
#include <QMutexLocker>
#include <QStringList>
#include <QRegularExpression>
#include <QVector>
#include <QXmlStreamReader>
#include <QDebug>

#include <osg/Array>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/Material>
#include <osg/Math>
#include <osg/Matrixd>
#include <osg/MatrixTransform>
#include <osg/NodeVisitor>
#include <osg/Quat>
#include <osg/StateSet>
#include <osg/ref_ptr>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osgDB/ReadFile>
#include <osgUtil/SmoothingVisitor>

#include <algorithm>
#include <cmath>
#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{

// ---------------------------------------------------------------------------
// URDF 导入与网格烘焙说明（本匿名命名空间内实现）
//
// 坐标约定：URDF 遵循 ROS 右手系、Z 向上（REP-103）。本模块内部长度统一为毫米（全局 mm）：
// URDF 文件中 origin 的 xyz 为米，读入后换算为 mm；网格顶点默认按毫米文件坐标进入 mm 连杆系。
// 齐次矩阵将「网格文件顶点」变到「用于显示的世界坐标」。每个 link 的 mesh 顶点在该 link 局部系中定义，
// 经 <joint><origin> 链从根连杆累积到世界；<visual><origin> 为「视觉几何系相对连杆系」位姿，
// 即 worldFromMesh = worldFromLink * visualInLink * p_mesh（各量均为 mm）。
//
// 若实际 OBJ 等文件把全部几何都放在「整机/基座」同一坐标系中，则与上述假设不符，仅靠 <visual>
// 的小幅 xyz/rpy 往往无法对齐，需在资源侧按连杆重导出或调整 URDF。
// ---------------------------------------------------------------------------

// 若为 true：在输出到 OSG 前对整条链再乘 Rx(-90°)，使 URDF 的 +Z 对齐常见视窗的 +Y。
// 调试时可关：保持 ROS Z 向上，便于与 RViz 对比；若 mesh 已与当前视窗约定一致也可关。
//
// 与层级场景 buildHierarchicalRobotScene 的关系：此处若启用，仅在 RobotAssembly 下增加一层
// MatrixTransform（matRosWorldToOsgWorld），关节与 mesh 矩阵仍在 ROS/mm 系中计算，不会与
// urdfWorldFromMeshVertices / kUrdfBake* 叠加——后者用于 computeMeshWorldMatrices 等「矩阵导出」
// 路径；本函数不在层级构建里烘焙顶点，故不会出现「根旋转乘两次」。
constexpr bool kApplyRosZUpToOsgYUp = false;

// 为 true（默认）：导入时将「从根到当前连杆的关节链」×「<visual><origin>」烘焙进顶点，
// 使静态显示与 URDF 一致，并与 computeMeshWorldMatrices / 关节滑条所用 FK 一致。
// 仅当 mesh 已在单一装配坐标系中预摆好时再设为 false（详见 UrdfRobotLoader.h）。
constexpr bool kUrdfBakeJointChainIntoMesh = true;

// 为 true（默认）：把 <visual><origin> 烘焙进顶点；适用于按连杆导出的 CAD mesh、位姿由 URDF 描述。
// 当 kUrdfBakeJointChainIntoMesh 为 false 时：true 仍只应用 visual；false 则保持文件顶点不动。
constexpr bool kUrdfBakeVisualOriginIntoMesh = true;

// URDF 文件中 \<joint\>\<visual\> 的 \<origin xyz\> 为米（REP-103）→ 内部毫米。
constexpr double kUrdfOriginXyzMetersToInternalMm = 1000.0;
// 网格顶点文件单位 → 内部毫米：STL 多为毫米时用 1.0；若网格文件已是米则用 1000.0（不读 URDF mesh scale 属性）。
constexpr double kMeshFileVertexUnitsToInternalMm = 1.0;

// OSG SmoothingVisitor 会对已三角化的 STL/STEP 导出网格合并顶点并重算法线，工业网格上易出现「斑马纹 / 条纹状」错误着色。
// 插件读入的网格通常已带法线；需要更圆滑外观时可改为 true。
constexpr bool kUrdfMeshUseSmoothingVisitor = false;

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

	Mat4 r = matIdentity();
	// R = Rz * Ry * Rx
	r.m[0] = cy * cp;
	r.m[1] = cy * sp * sr - sy * cr;
	r.m[2] = cy * sp * cr + sy * sr;
	r.m[4] = sy * cp;
	r.m[5] = sy * sp * sr + cy * cr;
	r.m[6] = sy * sp * cr - cy * sr;
	r.m[8] = -sp;
	r.m[9] = cp * sr;
	r.m[10] = cp * cr;
	return r;
}

Mat4 matMul(const Mat4& a, const Mat4& b)
{
	Mat4 r{};
	for (int col = 0; col < 4; ++col)
	{
		for (int row = 0; row < 4; ++row)
		{
			r.m[col * 4 + row] = a.m[0 * 4 + row] * b.m[col * 4 + 0] + a.m[1 * 4 + row] * b.m[col * 4 + 1]
				+ a.m[2 * 4 + row] * b.m[col * 4 + 2] + a.m[3 * 4 + row] * b.m[col * 4 + 3];
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
// 对列向量 p 有 p' = R*p + xyz（与 URDF 常用实现一致）。
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

// 根据 kUrdfBake* 开关，决定导入时烘焙到顶点上的「从 mesh 文件系到 URDF 世界」变换。
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

// 在 URDF 世界系结果上可选施加 ROS→OSG 轴向修正（见 kApplyRosZUpToOsgYUp）。
Mat4 osgWorldFromUrdfMeshFrame(const Mat4& urdfWorldFromMesh)
{
	return kApplyRosZUpToOsgYUp ? matMul(matRosWorldToOsgWorld(), urdfWorldFromMesh) : urdfWorldFromMesh;
}

// 解析空格分隔的三个浮点数（用于 xyz、rpy）；空串视为 0,0,0。
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

// 每个 link 仅解析第一个 <visual>：mesh 路径与 <visual><origin>（无则视为 0）。
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
};

// 网格文件顶点 → 连杆系（内部 mm）：p_link = V * S * p_file。S 将文件顶点变到 mm；V 的平移来自 URDF 米（乘 kUrdfOriginXyzMetersToInternalMm）。
// 例：xyz="-0.075 0 0"（米）→ 平移 -75 mm。
Mat4 meshFileToLinkFrameFromVisual(const UrdfLinkVisual& vis)
{
	const Mat4 scaleM = matScale(kMeshFileVertexUnitsToInternalMm, kMeshFileVertexUnitsToInternalMm, kMeshFileVertexUnitsToInternalMm);
	return matMul(
		matFromXyzRpy(
			vis.vx * kUrdfOriginXyzMetersToInternalMm,
			vis.vy * kUrdfOriginXyzMetersToInternalMm,
			vis.vz * kUrdfOriginXyzMetersToInternalMm,
			vis.vr,
			vis.vp,
			vis.vw),
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

// 读取单个 <visual> 内的 <origin xyz rpy> 与 <geometry><mesh filename="..."/>。
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

// 读取 <joint> 子元素：origin、parent、child、axis、limit。
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
// 其余相对路径相对 urdf 所在目录，绝对路径直接使用。
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

// 三角网顶点缓冲：每连续三个 float 为 (x,y,z)，左乘齐次矩阵 t 烘焙到世界（或当前选定的目标系）。
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

// 内部 Mat4（列主序）转为 osg::Matrixd，供返回给 OSG 场景使用。
osg::Matrixd mat4ToOsg(const Mat4& m)
{
	osg::Matrixd o;
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			o(r, c) = m.m[c * 4 + r];
		}
	}
	return o;
}

Mat4 matAxisAngleRad(double ax, double ay, double az, double angle)
{
	osg::Quat q;
	q.makeRotate(angle, ax, ay, az);
	const osg::Matrixd R = osg::Matrixd::rotate(q);
	Mat4 out{};
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			out.m[c * 4 + r] = R(r, c);
		}
	}
	return out;
}

// 正运动学用：子连杆相对父连杆的变换，即 parent_T_child = <joint><origin> * 绕 axis 的 q 角旋转（若为转动关节）。
// \<joint\>\<origin\> xyz 按 REP-103 为米：乘 kUrdfOriginXyzMetersToInternalMm 得到内部 mm，与 visual、网格一致。axis 与 rpy 不缩放。
// jointAnglesRad 与 qIndex 须与树遍历顺序一致（见 computeMeshWorldMatricesFromModel / loadMeshHierarchyParts）。
Mat4 jointChildTransformForFk(const UrdfJoint& j, const QVector<double>& jointAnglesRad, int& qIndex)
{
	const Mat4 T_origin = matFromXyzRpy(
		j.x * kUrdfOriginXyzMetersToInternalMm,
		j.y * kUrdfOriginXyzMetersToInternalMm,
		j.z * kUrdfOriginXyzMetersToInternalMm,
		j.roll,
		j.pitch,
		j.yaw);
	const QString jt = j.type.toLower();
	if (jt == QLatin1String("revolute") || jt == QLatin1String("continuous"))
	{
		double q = 0.0;
		if (qIndex < jointAnglesRad.size())
		{
			q = jointAnglesRad[qIndex];
		}
		++qIndex;
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
		return matMul(T_origin, matAxisAngleRad(ax, ay, az, q));
	}
	// 只有转动关节才递增qIndex，否则保持不变
	return T_origin;
}

/// 零位姿（q=0）下的 parent_T_child，仅由该关节的 URDF 决定，不依赖 jointAnglesRad 下标顺序。
/// 用于 buildHierarchicalRobotScene 静态挂接；动态 FK 仍用 jointChildTransformForFk + 与 BFS 一致的 qIndex。
static Mat4 jointChildTransformAtZeroConfiguration(const UrdfJoint& j)
{
	const Mat4 T_origin = matFromXyzRpy(
		j.x * kUrdfOriginXyzMetersToInternalMm,
		j.y * kUrdfOriginXyzMetersToInternalMm,
		j.z * kUrdfOriginXyzMetersToInternalMm,
		j.roll,
		j.pitch,
		j.yaw);
	const QString jt = j.type.toLower();
	if (jt == QLatin1String("revolute") || jt == QLatin1String("continuous"))
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
		return matMul(T_origin, matAxisAngleRad(ax, ay, az, 0.0));
	}
	return T_origin;
}

/// parent link 与 child link 必须为不同名称；相同则父/子对应同一 OSG 容器，无法挂接（自环关节）。
bool jointConnectsDistinctLinks(const UrdfJoint& j)
{
	return j.parent != j.child;
}

// 解析 URDF：收集各 link 的首个 visual、全部 joint；urdfDirOut 为 urdf 文件目录；
// packageRootOut 默认为 urdf 的上一级目录（用于 package:// 解析）。
// 根连杆：优先在「从未作为 child 出现」的 link 中取 base_link / world / odom / base（不区分大小写），否则取任意非 child。
bool parseUrdfModel(
	const QString& urdfFilePath,
	std::unordered_map<QString, UrdfLinkVisual>& linkVisuals,
	std::vector<UrdfJoint>& joints,
	QString& rootLink,
	QString& urdfDirOut,
	QString& packageRootOut,
	QString* errorMessage)
{
	linkVisuals.clear();
	joints.clear();
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
				linkVisuals[linkName] = vis;
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

	auto pickRootLink = [&]() -> QString {
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

// 解析结果缓存：供 FK、关节元数据、网格导入共用；jointsByParent 在首次构建时按父连杆名索引子关节列表。
struct UrdfFkModelData
{
	std::unordered_map<QString, UrdfLinkVisual> linkVisuals;
	std::vector<UrdfJoint> joints;
	QString rootLink;
	QString urdfDir;
	QString packageRoot;
	std::unordered_map<QString, std::vector<UrdfJoint>> jointsByParent;
};

struct UrdfModelCacheEntry
{
	std::shared_ptr<const UrdfFkModelData> model;
	qint64 lastModifiedMs = -1; // 与磁盘 mtime 一致则命中缓存，避免重复解析
};

static QMutex g_urdfModelCacheMutex;
static QHash<QString, UrdfModelCacheEntry> g_urdfModelCache;

// 缓存键：优先规范路径，减少同文件不同路径写法导致的重复解析。
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

// 线程安全：按路径 + 修改时间缓存解析结果；未命中则 parseUrdfModel 并填充 jointsByParent。
bool getOrCreateUrdfModel(const QString& urdfFilePath, std::shared_ptr<const UrdfFkModelData>& out, QString* errorMessage)
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

	out = data;
	UrdfModelCacheEntry ent;
	ent.model = data;
	ent.lastModifiedMs = mtime;
	g_urdfModelCache.insert(key, std::move(ent));
	return true;
}

// 按与导入时相同的 BFS 顺序遍历连杆树，用 jointAnglesRad 做正解；对每个带 mesh 的 link 输出
// 「从 mesh 文件系到当前显示世界」的 osg::Matrixd。此处始终使用完整 FK（关节链 × visual），
// 与 loadMeshHierarchyParts 里 kUrdfBake* 烘焙顶点无关；用于关节角变化时更新位姿（如 RobotSceneKinematics）。
void computeMeshWorldMatricesFromModel(
	const UrdfFkModelData& model,
	const QVector<double>& jointAnglesRad,
	QHash<QString, osg::Matrixd>& outLinkNameToMeshWorld)
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
			const Mat4 visualInLink = meshFileToLinkFrameFromVisual(vis);
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

} // namespace

// 按树遍历顺序列出 revolute/continuous 关节名及弧度上下限（无 limit 时 revolute 默认 ±π，continuous ±2π）。
bool UrdfRobotLoader::loadRevoluteJointMeta(
	const QString& urdfFilePath,
	QStringList& outJointNames,
	QVector<double>& outLowerRad,
	QVector<double>& outUpperRad,
	QString* errorMessage)
{
	outJointNames.clear();
	outLowerRad.clear();
	outUpperRad.clear();
	static constexpr double kPi = 3.14159265358979323846;
	static constexpr double kTwoPi = 6.2831853071795864769;

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
				const QString label = !j.name.isEmpty() ? j.name : QStringLiteral("joint_%1").arg(j.child);
				outJointNames.append(label);
				double lo = -kPi;
				double hi = kPi;
				if (j.hasLimit)
				{
					lo = j.limitLower;
					hi = j.limitUpper;
				}
				else if (jt == QLatin1String("continuous"))
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
			qq.push(QueueItem{j.child});
		}
	}
	return true;
}

// 仅关节名列表，顺序与 loadRevoluteJointMeta 一致，供与 jointAnglesRad 对齐。
bool UrdfRobotLoader::loadRevoluteJointNamesInOrder(
	const QString& urdfFilePath,
	QStringList& outJointNames,
	QString* errorMessage)
{
	QVector<double> lo;
	QVector<double> hi;
	return loadRevoluteJointMeta(urdfFilePath, outJointNames, lo, hi, errorMessage);
}

// 给定与各转动关节顺序一致的 jointAnglesRad（弧度），计算每个带 mesh 的连杆对应的 mesh→世界矩阵。
bool UrdfRobotLoader::computeMeshWorldMatrices(
	const QString& urdfFilePath,
	const QVector<double>& jointAnglesRad,
	QHash<QString, osg::Matrixd>& outLinkNameToMeshWorld,
	QString* errorMessage)
{
	std::shared_ptr<const UrdfFkModelData> model;
	if (!getOrCreateUrdfModel(urdfFilePath, model, errorMessage) || !model)
	{
		outLinkNameToMeshWorld.clear();
		return false;
	}
	computeMeshWorldMatricesFromModel(*model, jointAnglesRad, outLinkNameToMeshWorld);
	return true;
}

// 强制丢弃已缓存的解析结果（例如 URDF 在磁盘上被替换但路径与 mtime 处理异常时）。
void UrdfRobotLoader::clearUrdfModelCache()
{
	QMutexLocker lock(&g_urdfModelCacheMutex);
	g_urdfModelCache.clear();
}

// 【中文】计算给定关节角度下的所有 Joint 变换矩阵（parent_T_child）。
// 用于动态层级法：关节角度更新时，直接获取新的矩阵并设置到对应的 MatrixTransform。
bool UrdfRobotLoader::computeJointTransformMatrices(
	const QString& urdfFilePath,
	const QVector<double>& jointAnglesRad,
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
			// 计算 Joint 的 parent_T_child 矩阵（包含关节角度）
			const Mat4 parent_T_child = jointChildTransformForFk(j, jointAnglesRad, qIndex);

			// 转换为 osg::Matrixd 并保存
			osg::Matrixd osgMatrix = mat4ToOsg(parent_T_child);
			outJointMatrices[j.name] = osgMatrix;

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
// 【中文】三层分离模型场景图构建 - 核心实现
// 实现四阶段流程：资源预加载 -> 容器化场景图 -> 根节点组装 -> 状态重置
// ============================================================================

/// 【中文】辅助：将 URDF Mat4 内部格式转为 osg::Matrixd (已在匿名命名空间有 mat4ToOsg，这里复用)

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
	ss->setMode(GL_COLOR_MATERIAL, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
	ss->setMode(GL_LIGHTING, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	ss->setMode(GL_LIGHT0, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
	ss->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
}

class UrdfMeshLightingVisitor : public osg::NodeVisitor
{
public:
	UrdfMeshLightingVisitor()
		: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
	{
	}

	void apply(osg::Geometry& geometry) override
	{
		osg::Array* va = geometry.getVertexArray();
		if (va && va->getNumElements() >= 3U)
		{
			osg::Vec4 baseColor(0.65f, 0.82f, 0.95f, 1.0f);
			osg::Vec4Array* ca = dynamic_cast<osg::Vec4Array*>(geometry.getColorArray());
			if (ca && !ca->empty() && osg::getBinding(ca) == osg::Array::BIND_OVERALL)
			{
				baseColor = ca->front();
			}
			applyLitPlasticToStateSet(geometry.getOrCreateStateSet(), baseColor);
		}
		traverse(geometry);
	}

	// 仅向下遍历到 Drawable；实际材质在 apply(Geometry) 中设置，避免与 Geode 上整包材质重复叠加。
	void apply(osg::Geode& geode) override
	{
		traverse(geode);
	}
};

static void finalizeUrdfImportedMeshRendering(osg::Node* root)
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
	UrdfMeshLightingVisitor lighter;
	root->accept(lighter);
}

/// 【中文】辅助：加载 Mesh 文件为 OSG 节点
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
static osg::ref_ptr<osg::MatrixTransform> createLinkVisualFromMesh(
	const QString& linkName,
	const UrdfLinkVisual& vis,
	osg::ref_ptr<osg::Node> meshNode)
{
	finalizeUrdfImportedMeshRendering(meshNode.get());
	const Mat4 meshToLink = meshFileToLinkFrameFromVisual(vis);
	osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
	mt->setName(linkName.toStdString() + "_Geometry");
	mt->setMatrix(mat4ToOsg(meshToLink));
	mt->addChild(meshNode.get());
	return mt;
}

/// 【中文】辅助：创建 Link 的视觉容器层 (Group)
/// 保持裁剪开启，使包围盒与真实几何一致；关闭裁剪曾导致父级包围球膨胀、相机极远 + 默认 zFar 裁掉整场景（表现为全黑）。
static osg::ref_ptr<osg::Group> createContainerLayer(const QString& linkName)
{
	osg::ref_ptr<osg::Group> container = new osg::Group;
	container->setName(linkName.toStdString() + "_Container");
	container->setCullingActive(true);
	container->getOrCreateStateSet()->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);
	return container;
}

/// 【中文】辅助：创建 Joint 的运动学层 (MatrixTransform)
/// 运动学层：存储 parent_T_child 矩阵
static osg::ref_ptr<osg::MatrixTransform> createJointTransform(
	const QString& jointName,
	const Mat4& parent_T_child)
{
	osg::ref_ptr<osg::MatrixTransform> mt = new osg::MatrixTransform;
	mt->setName(jointName.toStdString());
	mt->setMatrix(mat4ToOsg(parent_T_child));
	return mt;
}

/// 从所有父 Group 上摘除节点。
/// 注意：removeChild 会对子节点 unref；若调用方仅用裸指针保存且场景是唯一引用，必须在调用本函数前用 osg::ref_ptr 先接住节点，否则会析构子节点并在循环内崩溃。
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

/// Parent_Link_Container -> Joint(MatrixTransform) -> Child_Link_Container。
/// 仅使用 osg::Group::addChild(Node*)；子连杆容器若已有父节点（重复 joint / 重入），先 detach 再挂，避免 getNumParents/addChild 失败。
/// 注意：detach 期间若多线程 Viewer 正在遍历场景，可能竞态；本函数仅在导入构建阶段、主线程调用时安全。
static bool attachLinkJointLink(
	osg::Group* parentLinkContainer,
	osg::MatrixTransform* jointMt,
	osg::Group* childLinkContainer,
	const QString& jointName,
	const QString& parentLinkName,
	const QString& childLinkName,
	QString* errorMessage)
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
			*errorMessage = QStringLiteral(
				"Invalid URDF: joint '%1' parent link '%2' and child link '%3' map to the same scene node. "
				"Use two different link names for parent and child (self-loop joints are not supported).")
				.arg(jointName, parentLinkName, childLinkName);
		}
		return false;
	}

	// 子连杆 Group 仅由 QHash 裸指针引用；removeChild 会 unref，引用归零会析构节点。detach 期间必须用 ref_ptr 保持存活。
	osg::ref_ptr<osg::Group> keepChildAlive(childLinkContainer);
	// 关节 MatrixTransform 由上层 jointMT ref_ptr 持有，detach 安全
	detachNodeFromAllParents(jointMt);
	detachNodeFromAllParents(childLinkContainer);

	if (!parentLinkContainer->addChild(jointMt))
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral(
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
			*errorMessage = QStringLiteral(
				"OSG addChild failed: joint '%1' -> child link '%2' (duplicate child under same joint?)")
				.arg(jointName, childLinkName);
		}
		parentLinkContainer->removeChild(jointMt);
		return false;
	}
	return true;
}

/// 【中文】构建层级化 URDF 机器人场景图（三层分离架构）
/// 【English】Build hierarchical URDF robot scene graph with three-layer separation
osg::Group* UrdfRobotLoader::buildHierarchicalRobotScene(
	const QString& urdfFilePath,
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
				*errorMessage = QStringLiteral("Mesh not found for link '%1': %2")
					.arg(linkName, vis.meshUri);
			}
			return nullptr;
		}

		// 加载 Mesh 文件为 OSG 节点
		osg::ref_ptr<osg::Node> meshNode = loadMeshNode(absMesh, errorMessage);
		if (!meshNode)
		{
			return nullptr; // errorMessage 已由 loadMeshNode 填充
		}

		// 【第二阶段】创建视觉容器层 (Container)
		osg::ref_ptr<osg::Group> container = createContainerLayer(linkName);

		// 几何层：仅 visual origin + 法线/光照（与关节 MatrixTransform 分工；不读 mesh scale）
		osg::ref_ptr<osg::MatrixTransform> geometryXf = createLinkVisualFromMesh(linkName, vis, meshNode);
		container->addChild(geometryXf.get());

		keepLinkContainersAlive.push_back(container);

		// 记录输出（存储原始指针，所有权由 keepLinkContainersAlive 与后续场景图保持）
		outLinkToGeometry[linkName] = geometryXf.get();
		outLinkToContainer[linkName] = container.get();
	}

	// ========================================================================
	// 【第二阶段】构建关节链（运动学层）
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
				qWarning() << "UrdfRobotLoader: skipping joint" << j.name << "- child link container not found for"
						   << j.child << "(missing <link> or not in model?)";
				continue;
			}

			auto parentIt = outLinkToContainer.find(j.parent);
			if (parentIt == outLinkToContainer.end())
			{
				qWarning() << "UrdfRobotLoader: skipping joint" << j.name << "- parent link container not found for"
						   << j.parent;
				continue;
			}

			// 【关键】创建运动学层：Joint MatrixTransform
			// 矩阵直接存储 parent_T_child（在父 Link 系中，子 Link 的位姿）
			osg::ref_ptr<osg::MatrixTransform> jointMT = createJointTransform(j.name, parent_T_child);
			osg::Group* parentContainer = parentIt.value();
			osg::MatrixTransform* jointNode = jointMT.get();
			osg::Group* childContainer = childIt.value();
			if (!parentContainer || !jointNode || !childContainer)
			{
				continue;
			}

			if (!attachLinkJointLink(parentContainer, jointNode, childContainer, j.name, j.parent, j.child, errorMessage))
			{
				return nullptr;
			}

			// 记录 Joint 变换节点（存储原始指针，所有权由场景图保持）
			outJointTransforms[j.name] = jointNode;

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

		// 挂载根 Container 到坐标系转换节点下
		if (rootIt.value())
		{
			coordTransform->addChild(rootIt.value());
		}
	}
	else
	{
		// 无坐标系转换：直接挂载根 Container
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
			mt->setMatrix(mat4ToOsg(meshToLink));
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
		}
	}

	// RobotAssembly 本身也刷新
	robotAssembly->dirtyBound();

	// 转移所有权：release() 使裸指针引用计数仍为 1，由调用方 addChild 或 osg::ref_ptr 承接；禁止手动 ref()+get()。
	return robotAssembly.release();
}
