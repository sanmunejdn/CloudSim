#ifndef CLOUDSIMHOST_HEADLESSGEOMODELBRIDGE_H
#define CLOUDSIMHOST_HEADLESSGEOMODELBRIDGE_H

/// @file HeadlessGeomodelBridge.h
/// @brief Web/Headless：参数化 B-rep Body 摘要

#include "cloudsim_host_global.h"

#include <QJsonObject>

namespace cloudsim::host
{
class DocumentHost;

class CLOUDSIM_HOST_EXPORT HeadlessGeomodelBridge
{
public:
	explicit HeadlessGeomodelBridge(DocumentHost& host);

	HeadlessGeomodelBridge(const HeadlessGeomodelBridge&) = delete;
	HeadlessGeomodelBridge& operator=(const HeadlessGeomodelBridge&) = delete;

	QJsonObject summaryJson() const;

private:
	DocumentHost& m_host;
};

} // namespace cloudsim::host

#endif
