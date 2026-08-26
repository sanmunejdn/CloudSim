#ifndef CLOUDSIMHOST_HEADLESSAIBRIDGE_H
#define CLOUDSIMHOST_HEADLESSAIBRIDGE_H

/// @file HeadlessAiBridge.h
/// @brief Web/Headless：LLM 对话

#include "cloudsim_host_global.h"

#include <QJsonObject>

namespace cloudsim::host
{
class DocumentHost;

class CLOUDSIM_HOST_EXPORT HeadlessAiBridge
{
public:
	explicit HeadlessAiBridge(DocumentHost& host);

	HeadlessAiBridge(const HeadlessAiBridge&) = delete;
	HeadlessAiBridge& operator=(const HeadlessAiBridge&) = delete;

	QJsonObject chat(const QJsonObject& body);

private:
	DocumentHost& m_host;
};

} // namespace cloudsim::host

#endif
