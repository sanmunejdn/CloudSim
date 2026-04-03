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

#include <osg/Matrixd>
#include <osg/Quat>

#include <algorithm>
#include <cmath>
#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace
{

// ---------------------------------------------------------------------------
// URDF 导入与网格烘焙说明（本匿名命名空间内实现）
//
// 坐标约定：URDF 遵循 ROS 右手系、Z 向上（REP-103）。本模块用齐次矩阵将「网格文件顶点」
// 变到「用于显示的世界坐标」。标准假设是：每个 link 的 mesh 顶点在该 link 的局部系中定义，
// 再通过 <joint><origin> 链从根连杆累积到世界；<visual><origin> 表示「视觉几何系相对连杆系」
// 的位姿，即 worldFromMesh = worldFromLink * visualInLink * p_mesh。
//
// 若实际 OBJ 等文件把全部几何都放在「整机/基座」同一坐标系中，则与上述假设不符，仅靠 <visual>
// 的小幅 xyz/rpy 往往无法对齐，需在资源侧按连杆重导出或调整 URDF。
// ---------------------------------------------------------------------------

// 若为 true：在输出到 OSG 前对整条链再乘 Rx(-90°)，使 URDF 的 +Z 对齐常见视窗的 +Y。
// 调试时可关：保持 ROS Z 向上，便于与 RViz 对比；若 mesh 已与当前视窗约定一致也可关。
constexpr bool kApplyRosZUpToOsgYUp = true;

// 为 true（默认）：导入时将「从根到当前连杆的关节链」×「<visual><origin>」烘焙进顶点，
// 使静态显示与 URDF 一致，并与 computeMeshWorldMatrices / 关节滑条所用 FK 一致。
// 仅当 mesh 已在单一装配坐标系中预摆好时再设为 false（详见 UrdfRobotLoader.h）。
constexpr bool kUrdfBakeJointChainIntoMesh = true;

// 为 true（默认）：把 <visual><origin> 烘焙进顶点；适用于按连杆导出的 CAD mesh、位姿由 URDF 描述。
// 当 kUrdfBakeJointChainIntoMesh 为 false 时：true 仍只应用 visual；false 则保持文件顶点不动。
constexpr bool kUrdfBakeVisualOriginIntoMesh = true;

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
			j.parent = xml.attributes().value(QStringLiteral("link")).toString();
			xml.skipCurrentElement();
		}
		else if (xml.name() == QLatin1String("child"))
		{
			j.child = xml.attributes().value(QStringLiteral("link")).toString();
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
// jointAnglesRad 与 qIndex 须与树遍历顺序一致（见 computeMeshWorldMatricesFromModel / loadMeshHierarchyParts）。
Mat4 jointChildTransformForFk(const UrdfJoint& j, const QVector<double>& jointAnglesRad, int& qIndex)
{
	const Mat4 T_origin = matFromXyzRpy(j.x, j.y, j.z, j.roll, j.pitch, j.yaw);
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
			const QString linkName = xml.attributes().value(QStringLiteral("name")).toString();
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
			j.name = xml.attributes().value(QStringLiteral("name")).toString();
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
			const Mat4 visualInLink = matFromXyzRpy(vis.vx, vis.vy, vis.vz, vis.vr, vis.vp, vis.vw);
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
			const Mat4 jointFromChild = jointChildTransformForFk(j, angles, qIndex);
			QueueItem nxt{};
			nxt.link = j.child;
			nxt.worldFromLink = matMul(cur.worldFromLink, jointFromChild);
			q.push(nxt);
		}
	}
}

} // namespace

// 从 URDF 加载层次化三角网：关节角固定为 0；将 worldFromLink×visual 烘焙进顶点（受 kUrdfBake* 控制），
// 并记录父子 part 路径供场景图使用。遍历顺序与 jointChildTransformForFk 的 qIndex 递增顺序一致。
bool UrdfRobotLoader::loadMeshHierarchyParts(
	const QString& urdfFilePath,
	std::vector<MeshHierarchyPart>& outParts,
	QString* errorMessage)
{
	outParts.clear();
	std::shared_ptr<const UrdfFkModelData> model;
	if (!getOrCreateUrdfModel(urdfFilePath, model, errorMessage) || !model)
	{
		return false;
	}
	const UrdfFkModelData& m = *model;
	const std::unordered_map<QString, UrdfLinkVisual>& linkVisuals = m.linkVisuals;
	const std::unordered_map<QString, std::vector<UrdfJoint>>& jointsByParent = m.jointsByParent;
	const QString& rootLink = m.rootLink;
	const QString& urdfDir = m.urdfDir;
	const QString& packageRoot = m.packageRoot;

	struct QueueItem
	{
		QString link;
		Mat4 worldFromLink;
		QString parentLinkName;
	};
	std::queue<QueueItem> q;
	QueueItem start{};
	start.link = rootLink;
	start.worldFromLink = matIdentity();
	start.parentLinkName = QString();
	q.push(start);

	QVector<double> zeroAngles(2048, 0.0); // 全零关节角：静态零位姿
	int qIndex = 0;

	while (!q.empty())
	{
		const QueueItem cur = q.front();
		q.pop();

		auto visIt = linkVisuals.find(cur.link);
		if (visIt != linkVisuals.end() && visIt->second.hasMesh)
		{
			// 加载网格 → 三角 soup → 乘 urdfWorldFromMeshVertices × osgWorldFromUrdfMeshFrame 写入顶点
			const UrdfLinkVisual& vis = visIt->second;
			const QString absMesh = resolveMeshFilename(vis.meshUri, packageRoot, urdfDir);
			if (absMesh.isEmpty() || !QFile::exists(absMesh))
			{
				if (errorMessage)
				{
					*errorMessage = QStringLiteral("Mesh not found for link '%1': %2")
						.arg(cur.link, vis.meshUri);
				}
				return false;
			}
			MeshBackendData mesh;
			const QByteArray nativeEnc = QFile::encodeName(absMesh);
			const std::string nativePath(nativeEnc.constData(), static_cast<std::size_t>(nativeEnc.size()));
			std::string loadErr;
			if (!mesh.loadFromFile(nativePath, &loadErr))
			{
				if (errorMessage)
				{
					*errorMessage = QStringLiteral("Failed to load mesh '%1': %2")
						.arg(absMesh, QString::fromStdString(loadErr));
				}
				return false;
			}
			std::vector<float> soup = mesh.triangleSoup();
			const Mat4 visualInLink = matFromXyzRpy(vis.vx, vis.vy, vis.vz, vis.vr, vis.vp, vis.vw);
			const Mat4 urdfWorldFromMesh = urdfWorldFromMeshVertices(cur.worldFromLink, visualInLink);
			const Mat4 osgWorldFromMesh = osgWorldFromUrdfMeshFrame(urdfWorldFromMesh);
			transformTriangleSoup(soup, osgWorldFromMesh);

			MeshHierarchyPart part;
			part.displayName = cur.link.toStdString();
			part.partPath = cur.link.toStdString();
			part.parentPartPath = cur.parentLinkName.isEmpty() ? std::string() : cur.parentLinkName.toStdString();
			part.triangleSoup = std::move(soup);
			outParts.push_back(std::move(part));
		}

		const auto jit = jointsByParent.find(cur.link);
		if (jit == jointsByParent.end())
		{
			continue;
		}
		for (const UrdfJoint& j : jit->second)
		{
			const Mat4 jointFromChild = jointChildTransformForFk(j, zeroAngles, qIndex);
			QueueItem nxt{};
			nxt.link = j.child;
			nxt.worldFromLink = matMul(cur.worldFromLink, jointFromChild);
			nxt.parentLinkName = cur.link;
			q.push(nxt);
		}
	}

	if (outParts.empty())
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("URDF produced no mesh geometry (missing visual meshes?).");
		}
		return false;
	}
	return true;
}

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
