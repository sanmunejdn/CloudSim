#pragma once

#include "widget_global.h"

#include <QString>

/// 项目包 ZIP：仅 STORE 无压缩方式的打包与解压，用于工程目录的导入导出（.pcp 等）。
namespace project_package_zip {

WIDGET_EXPORT bool isZipArchiveFile(const QString& filePath);

/** Zip every file under rootDir (recursively), paths in archive relative to rootDir. */
WIDGET_EXPORT bool zipDirectoryTree(const QString& zipFilePath, const QString& rootDir, QString* errorMessage = nullptr);

/** Extract a STORE-only zip to destDir (must exist or be creatable). */
WIDGET_EXPORT bool extractZipArchive(const QString& zipFilePath, const QString& destDir, QString* errorMessage = nullptr);

} // namespace project_package_zip
