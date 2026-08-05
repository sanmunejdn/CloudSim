/// @file DeviceCatalogScan.cpp
/// @brief URDF 设备库扫描

#include "DeviceCatalogScan.h"

#include <QDir>
#include <QDirIterator>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonObject>
#include <QMap>
#include <QRegularExpression>
#include <QSet>
#include <QStringList>
#include <QUrl>

#include <algorithm>

namespace cloudsim::web
{
namespace
{
const QStringList kImageSuffixes{QStringLiteral("png"), QStringLiteral("jpg"), QStringLiteral("jpeg"),
								 QStringLiteral("bmp"), QStringLiteral("webp")};

bool isNonDevicePackageFolderName(const QString& folderName)
{
	const QString n = folderName.toLower();
	static const QStringList kSkip{
		QStringLiteral("urdf"),		 QStringLiteral("meshes"),	  QStringLiteral("mesh"),	  QStringLiteral("config"),
		QStringLiteral("launch"),	 QStringLiteral("materials"), QStringLiteral("textures"), QStringLiteral("cad"),
		QStringLiteral("collision"), QStringLiteral("visual"),	  QStringLiteral("include"),  QStringLiteral("worlds"),
		QStringLiteral("maps"),		 QStringLiteral("rviz"),	  QStringLiteral("test"),	  QStringLiteral("tests"),
	};
	return kSkip.contains(n);
}

QStringList listUrdfFilesInPackage(const QString& packageRoot)
{
	QStringList out;
	QDirIterator it(packageRoot, QStringList{QStringLiteral("*.urdf"), QStringLiteral("*.URDF")},
					QDir::Files, QDirIterator::Subdirectories);
	while (it.hasNext())
	{
		out.append(QFileInfo(it.next()).absoluteFilePath());
	}
	std::sort(out.begin(), out.end());
	return out;
}

QStringList findDevicePackageRoots(const QString& modelsRoot)
{
	QStringList roots;
	if (!QDir(modelsRoot).exists())
	{
		return roots;
	}
	QSet<QString> seenCanon;
	QDirIterator it(modelsRoot, QDir::Dirs | QDir::NoDotAndDotDot, QDirIterator::Subdirectories);
	while (it.hasNext())
	{
		const QString dirPath = QDir::cleanPath(it.next());
		if (dirPath.compare(QDir::cleanPath(modelsRoot), Qt::CaseInsensitive) == 0)
		{
			continue;
		}
		if (isNonDevicePackageFolderName(QFileInfo(dirPath).fileName()))
		{
			continue;
		}
		if (listUrdfFilesInPackage(dirPath).isEmpty())
		{
			continue;
		}
		const QString canon = QFileInfo(dirPath).canonicalFilePath();
		const QString key = !canon.isEmpty() ? canon : dirPath;
		if (seenCanon.contains(key))
		{
			continue;
		}
		seenCanon.insert(key);
		roots.append(key);
	}
	std::sort(roots.begin(), roots.end());
	return roots;
}

void classifyPackagePath(const QString& packageRoot, const QString& modelsRoot, QString& outType, QString& outBrand)
{
	const QDir base(modelsRoot);
	const QString rel = QDir::cleanPath(base.relativeFilePath(packageRoot));
	static const QString kNoBrand = QStringLiteral("未分类");
	if (rel == QLatin1String(".") || rel.isEmpty())
	{
		outType = QStringLiteral("?");
		outBrand = kNoBrand;
		return;
	}
	const QStringList parts = rel.split(QRegularExpression(QStringLiteral("[/\\\\]")), Qt::SkipEmptyParts);
	if (parts.size() >= 3)
	{
		outType = parts[0];
		outBrand = parts[1];
	}
	else if (parts.size() == 2)
	{
		outType = parts[0];
		outBrand = kNoBrand;
	}
	else if (parts.size() == 1)
	{
		outType = parts[0];
		outBrand = kNoBrand;
	}
	else
	{
		outType = QStringLiteral("?");
		outBrand = kNoBrand;
	}
}

QStringList orderedCategoryKeys(const QStringList& categories)
{
	static const QStringList kPreferred{QStringLiteral("Robot"), QStringLiteral("AGV"), QStringLiteral("Worker")};
	QStringList out;
	QStringList rest = categories;
	for (const QString& p : kPreferred)
	{
		if (rest.removeAll(p) > 0)
		{
			out.append(p);
		}
	}
	std::sort(rest.begin(), rest.end());
	out += rest;
	return out;
}

QString pickUrdfForImageBase(const QString& imageBase, const QStringList& urdfAbsPaths)
{
	for (const QString& p : urdfAbsPaths)
	{
		if (QFileInfo(p).completeBaseName().compare(imageBase, Qt::CaseInsensitive) == 0)
		{
			return p;
		}
	}
	return urdfAbsPaths.isEmpty() ? QString() : urdfAbsPaths.front();
}

QString firstImageInPackage(const QString& packageRoot)
{
	const QDir rootDir(packageRoot);
	if (!rootDir.exists())
	{
		return {};
	}
	for (const QFileInfo& fi : rootDir.entryInfoList(QDir::Files))
	{
		if (kImageSuffixes.contains(fi.suffix().toLower()))
		{
			return fi.absoluteFilePath();
		}
	}
	return {};
}
} // namespace

QJsonObject scanDeviceCatalog(const QString& modelsRoot)
{
	QJsonObject root;
	root.insert(QStringLiteral("ok"), true);
	root.insert(QStringLiteral("modelsRoot"), modelsRoot);

	QMap<QString, QMap<QString, QStringList>> packagesByTypeBrand;
	for (const QString& pkg : findDevicePackageRoots(modelsRoot))
	{
		QString t;
		QString b;
		classifyPackagePath(pkg, modelsRoot, t, b);
		packagesByTypeBrand[t][b].append(pkg);
	}

	QStringList typeKeys = packagesByTypeBrand.keys();
	typeKeys = orderedCategoryKeys(typeKeys);
	QJsonArray types;
	QJsonObject brandsByType;
	QJsonArray packages;

	for (const QString& type : typeKeys)
	{
		types.append(type);
		QStringList brands = packagesByTypeBrand[type].keys();
		std::sort(brands.begin(), brands.end());
		QJsonArray brandArr;
		for (const QString& brand : brands)
		{
			brandArr.append(brand);
			for (const QString& packageRoot : packagesByTypeBrand[type][brand])
			{
				const QStringList urdfs = listUrdfFilesInPackage(packageRoot);
				if (urdfs.isEmpty())
				{
					continue;
				}
				const QString thumb = firstImageInPackage(packageRoot);
				QString name = QFileInfo(packageRoot).fileName();
				QString urdfPath = urdfs.front();
				if (!thumb.isEmpty())
				{
					name = QFileInfo(thumb).completeBaseName();
					const QString matched = pickUrdfForImageBase(name, urdfs);
					if (!matched.isEmpty())
					{
						urdfPath = matched;
					}
				}
				QJsonObject pkg;
				pkg.insert(QStringLiteral("type"), type);
				pkg.insert(QStringLiteral("brand"), brand);
				pkg.insert(QStringLiteral("name"), name);
				pkg.insert(QStringLiteral("urdfPath"), urdfPath);
				pkg.insert(QStringLiteral("packageRoot"), packageRoot);
				if (!thumb.isEmpty())
				{
					pkg.insert(QStringLiteral("thumbnailPath"), thumb);
					pkg.insert(QStringLiteral("thumbnailUrl"),
							   QStringLiteral("/api/devices/thumbnail?path=%1")
								   .arg(QString::fromUtf8(QUrl::toPercentEncoding(thumb))));
				}
				packages.append(pkg);
			}
		}
		brandsByType.insert(type, brandArr);
	}

	root.insert(QStringLiteral("types"), types);
	root.insert(QStringLiteral("brandsByType"), brandsByType);
	root.insert(QStringLiteral("packages"), packages);
	return root;
}

} // namespace cloudsim::web
