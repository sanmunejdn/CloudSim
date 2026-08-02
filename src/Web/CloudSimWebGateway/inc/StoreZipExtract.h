#ifndef CLOUDSIMWEBGATEWAY_STOREZIPEXTRACT_H
#define CLOUDSIMWEBGATEWAY_STOREZIPEXTRACT_H

/// @file StoreZipExtract.h
/// @brief STORE-only zip 解压（与桌面 .pcp 约定一致，不链 Widget）

#include "cloudsim_web_gateway_global.h"

#include <QString>

namespace cloudsim::web
{
CLOUDSIM_WEB_GATEWAY_API bool isStoreZipArchive(const QString& filePath);
CLOUDSIM_WEB_GATEWAY_API bool extractStoreZipArchive(const QString& zipFilePath, const QString& destDir,
													 QString* errorMessage = nullptr);
CLOUDSIM_WEB_GATEWAY_API bool packStoreZipArchive(const QString& zipFilePath, const QString& rootDir,
												  QString* errorMessage = nullptr);
} // namespace cloudsim::web

#endif // CLOUDSIMWEBGATEWAY_STOREZIPEXTRACT_H
