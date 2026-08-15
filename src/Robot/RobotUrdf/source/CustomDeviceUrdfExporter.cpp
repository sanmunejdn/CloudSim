/// @file CustomDeviceUrdfExporter.cpp
/// @brief 自定义设备导出 ROS URDF 包（校验 → 几何落盘 → package/urdf）

#include "CustomDeviceUrdfExporter.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendFollowMath.h"
#include "BrepBackendData.h"
#include "CustomDeviceBackendData.h"
#include "MeshBackendData.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QtGlobal>

#include <cmath>
#include <cstring>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace
{
constexpr double kMmToMeters = 0.001;
constexpr double kDegToRad = 0.017453292519943295769;

QString sanitizeUrdfToken(const QString& raw, const QString& fallback)
{
	QString s;
	s.reserve(raw.size());
	for (const QChar c : raw.trimmed())
	{
		if (c.isLetterOrNumber() || c == QLatin1Char('_'))
		{
			s.append(c.toLower());
		}
		else if (c == QLatin1Char('-') || c.isSpace())
		{
			s.append(QLatin1Char('_'));
		}
	}
	while (s.startsWith(QLatin1Char('_')))
	{
		s.remove(0, 1);
	}
	if (s.isEmpty() || !s[0].isLetter())
	{
		s = fallback + (s.isEmpty() ? QString() : (QLatin1Char('_') + s));
	}
	return s;
}

QString uniqueName(const QString& base, std::unordered_set<std::string>& used)
{
	QString name = base;
	int n = 2;
	while (used.count(name.toStdString()) > 0)
	{
		name = base + QLatin1Char('_') + QString::number(n++);
	}
	used.insert(name.toStdString());
	return name;
}

bool isCadExt(const QString& ext)
{
	const QString e = ext.toLower();
	return e == QLatin1String("step") || e == QLatin1String("stp") || e == QLatin1String("igs") ||
		   e == QLatin1String("iges");
}

bool isMeshExt(const QString& ext)
{
	const QString e = ext.toLower();
	return e == QLatin1String("obj") || e == QLatin1String("stl") || e == QLatin1String("ply") ||
		   e == QLatin1String("off") || e == QLatin1String("dae") || e == QLatin1String("3ds") ||
		   e == QLatin1String("fbx") || e == QLatin1String("dxf");
}

bool copyFileUtf8(const QString& src, const QString& dst, QString* err)
{
	if (!QFileInfo::exists(src))
	{
		if (err)
		{
			*err = QStringLiteral("源文件不存在：%1").arg(src);
		}
		return false;
	}
	if (QFile::exists(dst) && !QFile::remove(dst))
	{
		if (err)
		{
			*err = QStringLiteral("无法覆盖：%1").arg(dst);
		}
		return false;
	}
	if (!QFile::copy(src, dst))
	{
		if (err)
		{
			*err = QStringLiteral("拷贝失败：%1 → %2").arg(src, dst);
		}
		return false;
	}
	return true;
}

bool mat4ToUrdfOriginMeters(const double restMm[16], double& xM, double& yM, double& zM, double& roll, double& pitch,
							double& yaw)
{
	BackendMat4 m{};
	std::memcpy(m.v, restMm, sizeof(double) * 16);
	BackendVec3 t{};
	BackendVec3 eDeg{};
	backend_trans_euler_from_rigid_mat(m, t, eDeg);
	xM = t.x * kMmToMeters;
	yM = t.y * kMmToMeters;
	zM = t.z * kMmToMeters;
	roll = eDeg.x * kDegToRad;
	pitch = eDeg.y * kDegToRad;
	yaw = eDeg.z * kDegToRad;
	return true;
}

void normalizeAxis(double& ax, double& ay, double& az)
{
	const double len = std::sqrt(ax * ax + ay * ay + az * az);
	if (len < 1e-12)
	{
		ax = 0.0;
		ay = 0.0;
		az = 1.0;
		return;
	}
	ax /= len;
	ay /= len;
	az /= len;
}

QString xmlEscapeAttr(const QString& s)
{
	QString o = s;
	o.replace(QLatin1Char('&'), QLatin1String("&amp;"));
	o.replace(QLatin1Char('"'), QLatin1String("&quot;"));
	o.replace(QLatin1Char('<'), QLatin1String("&lt;"));
	o.replace(QLatin1Char('>'), QLatin1String("&gt;"));
	return o;
}

bool writeTextFile(const QString& path, const QString& content, QString* err)
{
	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
	{
		if (err)
		{
			*err = QStringLiteral("无法写入：%1").arg(path);
		}
		return false;
	}
	QTextStream ts(&f);
	ts.setCodec("UTF-8");
	ts << content;
	return true;
}

struct GeomAsset
{
	QString packageRelative;
};

bool resolveLinkGeometry(const CustomDeviceLink& link, const BackendDataManager& backend,
						 const QMap<QString, QString>& sourcePathByBackendId, const QString& packageRoot,
						 const QString& linkUrdfName, GeomAsset& out, QString* err)
{
	if (link.geometryBackendId.empty())
	{
		if (err)
		{
			*err = QStringLiteral("连杆缺少几何：%1").arg(QString::fromStdString(link.id));
		}
		return false;
	}
	const QString geomId = QString::fromStdString(link.geometryBackendId);
	const QString src = sourcePathByBackendId.value(geomId);
	const QFileInfo srcInfo(src);
	if (!src.isEmpty() && srcInfo.exists() && srcInfo.isFile())
	{
		const QString ext = srcInfo.suffix().toLower();
		const QString fileName = linkUrdfName + QLatin1Char('.') + ext;
		QString rel;
		QString abs;
		if (isCadExt(ext))
		{
			rel = QStringLiteral("cad/%1").arg(fileName);
			abs = QDir(packageRoot).filePath(rel);
			if (!QDir(packageRoot).mkpath(QStringLiteral("cad")))
			{
				if (err)
				{
					*err = QStringLiteral("无法创建 cad/");
				}
				return false;
			}
		}
		else if (isMeshExt(ext))
		{
			rel = QStringLiteral("meshes/%1").arg(fileName);
			abs = QDir(packageRoot).filePath(rel);
			if (!QDir(packageRoot).mkpath(QStringLiteral("meshes")))
			{
				if (err)
				{
					*err = QStringLiteral("无法创建 meshes/");
				}
				return false;
			}
		}
		else
		{
			if (err)
			{
				*err = QStringLiteral("不支持的源格式：%1").arg(ext);
			}
			return false;
		}
		if (!copyFileUtf8(src, abs, err))
		{
			return false;
		}
		out.packageRelative = rel;
		return true;
	}

	const auto data = backend.getData(link.geometryBackendId);
	if (!data)
	{
		if (err)
		{
			*err = QStringLiteral("找不到几何对象：%1").arg(geomId);
		}
		return false;
	}

	if (auto* mesh = dynamic_cast<MeshBackendData*>(data.get()))
	{
		if (!QDir(packageRoot).mkpath(QStringLiteral("meshes")))
		{
			if (err)
			{
				*err = QStringLiteral("无法创建 meshes/");
			}
			return false;
		}
		const QString rel = QStringLiteral("meshes/%1.ply").arg(linkUrdfName);
		const QString abs = QDir(packageRoot).filePath(rel);
		std::string ioErr;
		if (!mesh->writeTriangleMeshPly(abs.toStdString(), &ioErr))
		{
			if (err)
			{
				*err = QStringLiteral("写出 PLY 失败（%1）：%2")
						   .arg(geomId, QString::fromStdString(ioErr));
			}
			return false;
		}
		out.packageRelative = rel;
		return true;
	}

	if (auto* brep = dynamic_cast<BrepBackendData*>(data.get()))
	{
		if (!QDir(packageRoot).mkpath(QStringLiteral("cad")))
		{
			if (err)
			{
				*err = QStringLiteral("无法创建 cad/");
			}
			return false;
		}
		const QString rel = QStringLiteral("cad/%1.step").arg(linkUrdfName);
		const QString abs = QDir(packageRoot).filePath(rel);
		std::string ioErr;
		if (!brep->writeStepFile(abs.toStdString(), &ioErr))
		{
			if (err)
			{
				*err = QStringLiteral("写出 STEP 失败（%1）：%2")
						   .arg(geomId, QString::fromStdString(ioErr));
			}
			return false;
		}
		out.packageRelative = rel;
		return true;
	}

	if (err)
	{
		*err = QStringLiteral("几何无法导出：%1（无源路径且非 Mesh/Brep）").arg(geomId);
	}
	return false;
}

/// 单固定根、树状引用合法；无图直接失败
bool validateAssemblyGraph(const CustomDeviceBackendData& device, QString* err)
{
	if (!device.usesLinkJointGraph())
	{
		if (err)
		{
			*err = QStringLiteral("设备缺少 Link/Joint 图，请先在组装画布中定义连杆与运动副。");
		}
		return false;
	}
	std::unordered_set<std::string> linkIds;
	int fixedCount = 0;
	for (const CustomDeviceLink& L : device.links())
	{
		if (L.id.empty() || !linkIds.insert(L.id).second)
		{
			if (err)
			{
				*err = QStringLiteral("连杆 id 无效或重复。");
			}
			return false;
		}
		if (L.fixed)
		{
			++fixedCount;
		}
	}
	if (fixedCount != 1)
	{
		if (err)
		{
			*err = QStringLiteral("需要且仅需要一个固定根连杆（当前 %1 个）。").arg(fixedCount);
		}
		return false;
	}

	std::unordered_set<std::string> childOnce;
	for (const CustomDeviceJoint& J : device.joints())
	{
		if (!linkIds.count(J.parentLinkId) || !linkIds.count(J.childLinkId))
		{
			if (err)
			{
				*err = QStringLiteral("运动副引用了不存在的连杆。");
			}
			return false;
		}
		if (J.parentLinkId == J.childLinkId)
		{
			if (err)
			{
				*err = QStringLiteral("运动副不能连接同一连杆。");
			}
			return false;
		}
		if (!childOnce.insert(J.childLinkId).second)
		{
			if (err)
			{
				*err = QStringLiteral("不支持多父连杆（非树）。");
			}
			return false;
		}
	}
	return true;
}

void buildLinkUrdfNames(const CustomDeviceBackendData& device, std::unordered_map<std::string, QString>& linkUrdfName)
{
	std::unordered_set<std::string> usedLinkNames;
	for (const CustomDeviceLink& L : device.links())
	{
		const QString base = sanitizeUrdfToken(
			QString::fromStdString(L.displayName.empty() ? L.id : L.displayName), QStringLiteral("link"));
		linkUrdfName[L.id] = uniqueName(base, usedLinkNames);
	}
}

bool resolveAndStageGeometry(const CustomDeviceBackendData& device, const BackendDataManager& backend,
							 const QMap<QString, QString>& sourcePathByBackendId, const QString& packageRoot,
							 const std::unordered_map<std::string, QString>& linkUrdfName,
							 std::unordered_map<std::string, GeomAsset>& assets, QString* err)
{
	for (const CustomDeviceLink& L : device.links())
	{
		GeomAsset asset;
		const auto it = linkUrdfName.find(L.id);
		if (it == linkUrdfName.end())
		{
			if (err)
			{
				*err = QStringLiteral("连杆命名缺失：%1").arg(QString::fromStdString(L.id));
			}
			return false;
		}
		if (!resolveLinkGeometry(L, backend, sourcePathByBackendId, packageRoot, it->second, asset, err))
		{
			return false;
		}
		assets[L.id] = asset;
	}
	return true;
}

bool writePackageXml(const QString& packageRoot, const QString& pkgName, QString* err)
{
	const QString packageXml = QStringLiteral(
		"<?xml version=\"1.0\"?>\n"
		"<package format=\"2\">\n"
		"  <name>%1</name>\n"
		"  <version>0.0.1</version>\n"
		"  <description>CloudSim custom device URDF export</description>\n"
		"  <maintainer email=\"noreply@cloudsim.local\">CloudSim</maintainer>\n"
		"  <license>BSD</license>\n"
		"</package>\n");
	return writeTextFile(QDir(packageRoot).filePath(QStringLiteral("package.xml")), packageXml.arg(pkgName), err);
}

bool writeUrdfXml(const CustomDeviceBackendData& device, const QString& packageRoot, const QString& pkgName,
				  const std::unordered_map<std::string, QString>& linkUrdfName,
				  const std::unordered_map<std::string, GeomAsset>& assets, QString* urdfPathOut, QString* err)
{
	QString urdf;
	urdf += QStringLiteral("<?xml version=\"1.0\"?>\n");
	urdf += QStringLiteral("<robot name=\"%1\">\n").arg(xmlEscapeAttr(pkgName));

	for (const CustomDeviceLink& L : device.links())
	{
		const QString& lname = linkUrdfName.at(L.id);
		const QString meshUri =
			QStringLiteral("package://%1/%2").arg(pkgName, assets.at(L.id).packageRelative);
		urdf += QStringLiteral("  <link name=\"%1\">\n").arg(xmlEscapeAttr(lname));
		urdf += QStringLiteral("    <visual>\n");
		urdf += QStringLiteral("      <origin xyz=\"0 0 0\" rpy=\"0 0 0\"/>\n");
		urdf += QStringLiteral("      <geometry>\n");
		urdf += QStringLiteral("        <mesh filename=\"%1\"/>\n").arg(xmlEscapeAttr(meshUri));
		urdf += QStringLiteral("      </geometry>\n");
		urdf += QStringLiteral("    </visual>\n");
		urdf += QStringLiteral("  </link>\n");
	}

	std::unordered_set<std::string> usedJointNames;
	int ji = 0;
	for (const CustomDeviceJoint& J : device.joints())
	{
		++ji;
		const QString jBase = sanitizeUrdfToken(QString::fromStdString(J.id.empty() ? J.motion.jointName : J.id),
												QStringLiteral("joint"));
		const QString jname = uniqueName(jBase.isEmpty() ? QStringLiteral("joint_%1").arg(ji) : jBase, usedJointNames);
		const QString parent = linkUrdfName.at(J.parentLinkId);
		const QString child = linkUrdfName.at(J.childLinkId);

		QString type = QStringLiteral("fixed");
		if (J.motion.motionType == CustomDeviceMotionType::Rotate)
		{
			type = QStringLiteral("revolute");
		}
		else if (J.motion.motionType == CustomDeviceMotionType::Translate)
		{
			type = QStringLiteral("prismatic");
		}

		double xM = 0;
		double yM = 0;
		double zM = 0;
		double roll = 0;
		double pitch = 0;
		double yaw = 0;
		mat4ToUrdfOriginMeters(J.parentToChildRest, xM, yM, zM, roll, pitch, yaw);

		double ax = J.motion.axis[0];
		double ay = J.motion.axis[1];
		double az = J.motion.axis[2];
		normalizeAxis(ax, ay, az);

		double lower = J.motion.lower;
		double upper = J.motion.upper;
		if (type == QLatin1String("prismatic"))
		{
			lower *= kMmToMeters;
			upper *= kMmToMeters;
		}

		urdf += QStringLiteral("  <joint name=\"%1\" type=\"%2\">\n").arg(xmlEscapeAttr(jname), type);
		urdf += QStringLiteral("    <parent link=\"%1\"/>\n").arg(xmlEscapeAttr(parent));
		urdf += QStringLiteral("    <child link=\"%1\"/>\n").arg(xmlEscapeAttr(child));
		urdf += QStringLiteral("    <origin xyz=\"%1 %2 %3\" rpy=\"%4 %5 %6\"/>\n")
					.arg(xM, 0, 'g', 12)
					.arg(yM, 0, 'g', 12)
					.arg(zM, 0, 'g', 12)
					.arg(roll, 0, 'g', 12)
					.arg(pitch, 0, 'g', 12)
					.arg(yaw, 0, 'g', 12);
		if (type != QLatin1String("fixed"))
		{
			urdf += QStringLiteral("    <axis xyz=\"%1 %2 %3\"/>\n")
						.arg(ax, 0, 'g', 12)
						.arg(ay, 0, 'g', 12)
						.arg(az, 0, 'g', 12);
			urdf += QStringLiteral("    <limit lower=\"%1\" upper=\"%2\" effort=\"100\" velocity=\"1\"/>\n")
						.arg(lower, 0, 'g', 12)
						.arg(upper, 0, 'g', 12);
		}
		urdf += QStringLiteral("  </joint>\n");
	}

	urdf += QStringLiteral("</robot>\n");

	const QString urdfPath = QDir(packageRoot).filePath(QStringLiteral("urdf/%1.urdf").arg(pkgName));
	if (!writeTextFile(urdfPath, urdf, err))
	{
		return false;
	}
	if (urdfPathOut)
	{
		*urdfPathOut = urdfPath;
	}
	return true;
}

void assembleResult(CustomDeviceUrdfExportResult& result, const QString& packageRoot, const QString& urdfPath)
{
	result.ok = true;
	result.packageRoot = packageRoot;
	result.urdfPath = urdfPath;
	result.error.clear();
}

} // namespace

CustomDeviceUrdfExportResult exportCustomDeviceUrdfPackage(const CustomDeviceBackendData& device,
														  const BackendDataManager& backend,
														  const CustomDeviceUrdfExportOptions& options)
{
	CustomDeviceUrdfExportResult result;
	QString err;
	if (options.packageParentDir.trimmed().isEmpty())
	{
		result.error = QStringLiteral("未指定导出目录。");
		return result;
	}
	if (!validateAssemblyGraph(device, &err))
	{
		result.error = err;
		return result;
	}

	const QString pkgName = sanitizeUrdfToken(
		options.packageName.isEmpty() ? QString::fromStdString(device.name().empty() ? device.id() : device.name())
									  : options.packageName,
		QStringLiteral("custom_device"));
	const QString packageRoot = QDir(options.packageParentDir).filePath(pkgName);
	if (!QDir().mkpath(packageRoot))
	{
		result.error = QStringLiteral("无法创建包目录：%1").arg(packageRoot);
		return result;
	}
	if (!QDir(packageRoot).mkpath(QStringLiteral("urdf")))
	{
		result.error = QStringLiteral("无法创建 urdf/");
		return result;
	}

	std::unordered_map<std::string, QString> linkUrdfName;
	buildLinkUrdfNames(device, linkUrdfName);

	std::unordered_map<std::string, GeomAsset> assets;
	if (!resolveAndStageGeometry(device, backend, options.sourcePathByBackendId, packageRoot, linkUrdfName, assets,
								 &err))
	{
		result.error = err;
		return result;
	}

	if (!writePackageXml(packageRoot, pkgName, &err))
	{
		result.error = err;
		return result;
	}

	QString urdfPath;
	if (!writeUrdfXml(device, packageRoot, pkgName, linkUrdfName, assets, &urdfPath, &err))
	{
		result.error = err;
		return result;
	}

	assembleResult(result, packageRoot, urdfPath);
	return result;
}
