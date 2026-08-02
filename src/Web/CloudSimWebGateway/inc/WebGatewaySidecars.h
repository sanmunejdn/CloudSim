#ifndef CLOUDSIMWEBGATEWAY_WEBGATEWAYSIDECARS_H
#define CLOUDSIMWEBGATEWAY_WEBGATEWAYSIDECARS_H

#include <QJsonObject>

namespace cloudsim::web
{
void webGatewayLoadSidecarsFromProject(const QJsonObject& root);
void webGatewayMergeSidecarsIntoProject(QJsonObject& root);
} // namespace cloudsim::web

#endif
