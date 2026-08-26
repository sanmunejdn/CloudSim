#ifndef CLOUDSIMHOST_HEADLESSGEOMETRYBRIDGE_H
#define CLOUDSIMHOST_HEADLESSGEOMETRYBRIDGE_H

/// @file HeadlessGeometryBridge.h
/// @brief Web/Headless：STEP 离散/求交/布尔

#include "cloudsim_host_global.h"

#include <QJsonObject>

namespace cloudsim::host
{
class DocumentHost;

class CLOUDSIM_HOST_EXPORT HeadlessGeometryBridge
{
public:
	explicit HeadlessGeometryBridge(DocumentHost& host);

	HeadlessGeometryBridge(const HeadlessGeometryBridge&) = delete;
	HeadlessGeometryBridge& operator=(const HeadlessGeometryBridge&) = delete;

	QJsonObject discretize(const QJsonObject& body);
	QJsonObject intersect(const QJsonObject& body);
	QJsonObject op(const QJsonObject& body);

private:
	DocumentHost& m_host;
};

} // namespace cloudsim::host

#endif
