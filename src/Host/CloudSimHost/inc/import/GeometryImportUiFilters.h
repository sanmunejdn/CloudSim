#ifndef CLOUDSIMHOST_GEOMETRYIMPORTUIFILTERS_H
#define CLOUDSIMHOST_GEOMETRYIMPORTUIFILTERS_H

/// @file GeometryImportUiFilters.h
/// @brief 从 GeometryFileImporterRegistry 生成 Qt 文件对话框过滤器

#include "cloudsim_host_global.h"

#include <QString>

namespace cloudsim::host
{

/// 打开模型过滤器；网页无 OsgWidget 时令 includeOsgCapture=false
CLOUDSIM_HOST_EXPORT QString geometryOpenModelFileFilter(bool includeOsgCapture = true);

/// 「导入…」混合过滤器：模型（可选 OSG）+ 点云
CLOUDSIM_HOST_EXPORT QString geometryMixedImportFileFilter(bool includeOsgCapture = true);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_GEOMETRYIMPORTUIFILTERS_H
