#ifndef CLOUDSIMHOST_BACKENDVISUALSYNC_H
#define CLOUDSIMHOST_BACKENDVISUALSYNC_H

/// @file BackendVisualSync.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 属性需同步 OSG

#include "cloudsim_host_global.h"

#include <QString>

class BackendDataBase;

namespace cloudsim::host
{
class DocumentHost;

/// 属性需同步 OSG
CLOUDSIM_HOST_EXPORT bool propertyKeyNeedsVisualSync(const QString& key);

/// 属性需发 PoseCommitted
CLOUDSIM_HOST_EXPORT bool propertyKeyCommitsPose(const QString& key);

/// 属性变更后视觉同步
CLOUDSIM_HOST_EXPORT void syncVisualAfterPropertyChange(DocumentHost& host, const BackendDataBase& data,
														bool applyColor = false);
CLOUDSIM_HOST_EXPORT void syncVisualAfterPropertyChangeById(DocumentHost& host, const QString& objectId,
															bool applyColor = false);

/// IDataService 后处理
CLOUDSIM_HOST_EXPORT void afterDataServicePropertyChange(DocumentHost& host, const BackendDataBase& data,
														 const QString& key);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_BACKENDVISUALSYNC_H
