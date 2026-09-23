/// @file GeometryImportUiFilters.cpp
/// @brief 打开模型 / 混合导入 Qt 过滤器

#include "GeometryImportUiFilters.h"

#include "GeometryFileImporterRegistry.h"

#include <QStringList>

#include <algorithm>
#include <string>

namespace cloudsim::host
{
namespace
{
bool isOsgCaptureExt(const std::string& ext)
{
	return ext == "dae" || ext == "3ds" || ext == "fbx";
}

QStringList modelGlobPatterns(bool includeOsgCapture)
{
	std::vector<std::string> exts = GeometryFileImporterRegistry::instance().allExtensions();
	std::sort(exts.begin(), exts.end());
	QStringList patterns;
	patterns.reserve(static_cast<int>(exts.size()));
	for (const std::string& e : exts)
	{
		if (!includeOsgCapture && isOsgCaptureExt(e))
		{
			continue;
		}
		patterns.append(QStringLiteral("*.%1").arg(QString::fromStdString(e)));
	}
	return patterns;
}
} // namespace

QString geometryOpenModelFileFilter(bool includeOsgCapture)
{
	const QStringList patterns = modelGlobPatterns(includeOsgCapture);
	return QStringLiteral("模型文件 (%1);;所有文件 (*.*)").arg(patterns.join(QLatin1Char(' ')));
}

QString geometryMixedImportFileFilter(bool includeOsgCapture)
{
	const QStringList modelPatterns = modelGlobPatterns(includeOsgCapture);
	return QStringLiteral("模型 (%1);;点云 (*.pcd *.ply *.las *.laz *.xyz);;所有文件 (*.*)")
		.arg(modelPatterns.join(QLatin1Char(' ')));
}

} // namespace cloudsim::host
