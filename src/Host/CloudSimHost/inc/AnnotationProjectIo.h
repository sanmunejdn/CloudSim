#pragma once

#include "cloudsim_host_global.h"

#include <QJsonArray>
#include <QJsonObject>

class OsgWidget;

namespace cloudsim::host {

/// 标注 JSON 读写
CLOUDSIM_HOST_EXPORT QJsonArray buildAnnotationsJsonFromOsg(OsgWidget& osg, QJsonObject& inOutRootExtras);
CLOUDSIM_HOST_EXPORT void applyAnnotationsFromProjectJson(OsgWidget& osg, const QJsonObject& root);

} // namespace cloudsim::host
