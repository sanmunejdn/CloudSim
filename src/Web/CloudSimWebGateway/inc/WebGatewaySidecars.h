#ifndef CLOUDSIMWEBGATEWAY_WEBGATEWAYSIDECARS_H
#define CLOUDSIMWEBGATEWAY_WEBGATEWAYSIDECARS_H

/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
#include <QJsonObject>

namespace cloudsim::web
{
void webGatewayLoadSidecarsFromProject(const QJsonObject& root);
void webGatewayMergeSidecarsIntoProject(QJsonObject& root);
} // namespace cloudsim::web

#endif
