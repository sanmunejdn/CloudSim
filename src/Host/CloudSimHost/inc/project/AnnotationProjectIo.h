#ifndef CLOUDSIMHOST_ANNOTATIONPROJECTIO_H
#define CLOUDSIMHOST_ANNOTATIONPROJECTIO_H

/// @file AnnotationProjectIo.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 标注 JSON 读写

#include "cloudsim_host_global.h"

#include <QJsonArray>
#include <QJsonObject>

class IViewportOverlay;

namespace cloudsim::host
{
/// 标注 JSON 读写
CLOUDSIM_HOST_EXPORT QJsonArray buildAnnotationsJsonFromOsg(IViewportOverlay& overlay, QJsonObject& inOutRootExtras);
CLOUDSIM_HOST_EXPORT void applyAnnotationsFromProjectJson(IViewportOverlay& overlay, const QJsonObject& root);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_ANNOTATIONPROJECTIO_H
