/// @file UrdfIkMachineProfile.cpp
/// @brief 机型 IK 能力表：路径包名 + 旁路 JSON

#include "UrdfIkMachineProfile.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QJsonDocument>
#include <QJsonObject>

namespace UrdfRobotLoader
{
namespace
{
QHash<QString, UrdfIkMachineProfile> g_profileCache;

void mergeJsonObject(const QJsonObject& o, UrdfIkMachineProfile& p)
{
	if (o.contains(QStringLiteral("elbowJointIndex")))
	{
		const int v = o.value(QStringLiteral("elbowJointIndex")).toInt(-1);
		if (v >= 0)
		{
			p.elbowJointIndex = v;
		}
	}
	if (o.contains(QStringLiteral("wristJointIndex")))
	{
		const int v = o.value(QStringLiteral("wristJointIndex")).toInt(-1);
		if (v >= 0)
		{
			p.wristJointIndex = v;
		}
	}
	if (o.contains(QStringLiteral("forceSphericalWrist")))
	{
		const QString f = o.value(QStringLiteral("forceSphericalWrist")).toString().trimmed().toLower();
		if (f == QStringLiteral("yes") || f == QStringLiteral("true") || f == QStringLiteral("1"))
		{
			p.forceSphericalWrist = SphericalWristForce::Yes;
		}
		else if (f == QStringLiteral("no") || f == QStringLiteral("false") || f == QStringLiteral("0"))
		{
			p.forceSphericalWrist = SphericalWristForce::No;
		}
		else
		{
			p.forceSphericalWrist = SphericalWristForce::Auto;
		}
	}
	if (o.contains(QStringLiteral("preferredIkProfile")))
	{
		const QString pref = o.value(QStringLiteral("preferredIkProfile")).toString().trimmed().toLower();
		if (pref == QStringLiteral("drag") || pref == QStringLiteral("preview") || pref == QStringLiteral("run"))
		{
			p.preferredIkProfile = pref.toStdString();
		}
	}
	if (o.contains(QStringLiteral("analyticalFamily")))
	{
		const QString f = o.value(QStringLiteral("analyticalFamily")).toString().trimmed().toLower();
		if (f == QStringLiteral("sphericalwrist6r") || f == QStringLiteral("spherical_wrist_6r"))
		{
			p.analyticalFamily = AnalyticalIkFamily::SphericalWrist6R;
		}
		else if (f == QStringLiteral("none") || f == QStringLiteral("dls") || f == QStringLiteral("numerical"))
		{
			p.analyticalFamily = AnalyticalIkFamily::None;
		}
		else
		{
			p.analyticalFamily = AnalyticalIkFamily::Auto;
		}
	}
}

bool loadSidecarJson(const QString& urdfPath, UrdfIkMachineProfile& p)
{
	const QFileInfo fi(urdfPath);
	if (!fi.exists())
	{
		return false;
	}
	// …/pkg/urdf/x.urdf → …/pkg/ik_profile.json
	const QString sidecar = fi.dir().absoluteFilePath(QStringLiteral("../ik_profile.json"));
	QFile f(QFileInfo(sidecar).absoluteFilePath());
	if (!f.open(QIODevice::ReadOnly))
	{
		return false;
	}
	const QJsonDocument doc = QJsonDocument::fromJson(f.readAll());
	if (!doc.isObject())
	{
		return false;
	}
	mergeJsonObject(doc.object(), p);
	return true;
}

UrdfIkMachineProfile builtinForPackage(const QString& packageKey)
{
	UrdfIkMachineProfile p;
	p.packageKey = packageKey;
	(void)packageKey;
	// 内置表先空：靠 sidecar / 扩展键；后续可按包名填肘腕
	return p;
}
} // namespace

QString ikMachinePackageKeyFromUrdfPath(const QString& urdfAbsolutePath)
{
	if (urdfAbsolutePath.isEmpty())
	{
		return {};
	}
	const QFileInfo fi(urdfAbsolutePath);
	const QDir urdfDir = fi.dir();
	if (urdfDir.dirName().compare(QStringLiteral("urdf"), Qt::CaseInsensitive) == 0)
	{
		QDir pkg = urdfDir;
		if (pkg.cdUp())
		{
			const QString pkgName = pkg.dirName();
			if (!pkgName.isEmpty() && pkgName != QStringLiteral(".") && pkgName != QStringLiteral(".."))
			{
				return pkgName;
			}
		}
	}
	return fi.completeBaseName();
}

UrdfIkMachineProfile resolveIkMachineProfile(const QString& urdfPath)
{
	UrdfIkMachineProfile out;
	if (urdfPath.isEmpty())
	{
		return out;
	}
	const auto it = g_profileCache.constFind(urdfPath);
	if (it != g_profileCache.constEnd())
	{
		return it.value();
	}
	out.packageKey = ikMachinePackageKeyFromUrdfPath(urdfPath);
	out = builtinForPackage(out.packageKey);
	out.packageKey = ikMachinePackageKeyFromUrdfPath(urdfPath);
	(void)loadSidecarJson(urdfPath, out);
	g_profileCache.insert(urdfPath, out);
	return out;
}

void clearIkMachineProfileCache()
{
	g_profileCache.clear();
}

} // namespace UrdfRobotLoader
