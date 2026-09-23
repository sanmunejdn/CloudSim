#ifndef CLOUDSIMHOST_APPLYGEOMETRYIMPORTPARSE_H
#define CLOUDSIMHOST_APPLYGEOMETRYIMPORTPARSE_H

/// @file ApplyGeometryImportParse.h
/// @brief ImportParseResult → DocumentHost 注册

#include "cloudsim_host_global.h"

#include "HierarchyMeshImport.h"
#include "IGeometryFileImporter.h"

#include <QString>

namespace cloudsim::host
{
class DocumentHost;

/// 将 parse 结果写入场景；OsgCapture 依赖 DocumentHost 上的 OsgWidget
CLOUDSIM_HOST_EXPORT bool applyGeometryImportParse(DocumentHost& host, const QString& sourceFilePath,
												   const QString& catalogTypeName, ImportParseResult& parsed,
												   const HierarchyFollowBindingFn& onParentFollow,
												   HierarchyMeshImportResult& out, QString* outError = nullptr);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_APPLYGEOMETRYIMPORTPARSE_H
