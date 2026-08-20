#ifndef WIDGET_PROJECTPACKAGEZIP_H
#define WIDGET_PROJECTPACKAGEZIP_H

/// @file ProjectPackageZip.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 项目包 ZIP：仅 STORE 无压缩方式的打包与解压，工程目录 .pcp 等导入导出

#include "widget_global.h"

#include <QString>

/// 项目包 ZIP：仅 STORE 无压缩方式的打包与解压，工程目录 .pcp 等导入导出
namespace project_package_zip
{
WIDGET_EXPORT bool isZipArchiveFile(const QString& filePath);

/** Zip every file under rootDir (recursively), paths in archive relative to rootDir. */
WIDGET_EXPORT bool zipDirectoryTree(const QString& zipFilePath, const QString& rootDir,
									QString* errorMessage = nullptr);

/** Extract a STORE-only zip to destDir (must exist or be creatable). */
WIDGET_EXPORT bool extractZipArchive(const QString& zipFilePath, const QString& destDir,
									 QString* errorMessage = nullptr);

} // namespace project_package_zip

#endif // WIDGET_PROJECTPACKAGEZIP_H
