#pragma once

#include "cloudsim_host_global.h"

#include <QString>

class BackendDataBase;

namespace cloudsim::host {

class DocumentHost;

/// 属性 key 是否需刷新 OSG（pose/颜色/跟随等）
CLOUDSIM_HOST_EXPORT bool propertyKeyNeedsVisualSync(const QString& key);

/// 是否应向 EventHub 发布 PoseCommitted
CLOUDSIM_HOST_EXPORT bool propertyKeyCommitsPose(const QString& key);

/// IDataService 或属性面板提交后：mesh/点云世界矩阵与选中色同步
CLOUDSIM_HOST_EXPORT void syncVisualAfterPropertyChange(DocumentHost& host, const BackendDataBase& data,
	bool applyColor = false);

/// applyPropertyChange 成功后的统一后处理（视觉 + 事件）
CLOUDSIM_HOST_EXPORT void afterDataServicePropertyChange(DocumentHost& host, const BackendDataBase& data,
	const QString& key);

} // namespace cloudsim::host
