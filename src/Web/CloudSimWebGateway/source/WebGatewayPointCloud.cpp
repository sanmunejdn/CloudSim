/// @file WebGatewayPointCloud.cpp
/// @brief 点云 REST（GUI 线程 → HeadlessPointCloudBridge）

#include "WebGateway.h"

#include "CloudSimHost.h"
#include "DocumentHost.h"
#include "HeadlessPointCloudBridge.h"

#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>

#include <cstdlib>
#include <vector>

namespace cloudsim::web
{
namespace
{
cloudsim::host::HeadlessPointCloudBridge* pcBridge(cloudsim::host::DocumentHost* host, QString* err)
{
	if (!host || !host->headlessPointCloudBridge())
	{
		if (err)
			*err = QStringLiteral("no headless point cloud bridge");
		return nullptr;
	}
	return host->headlessPointCloudBridge();
}
} // namespace

QByteArray WebGateway::pointCloudInfoJsonOnGuiThread(const QString& id)
{
	QString err;
	auto* b = pcBridge(m_document ? cloudsim::host::documentHostFromScope(m_document.get()) : nullptr, &err);
	if (!b)
		return QJsonDocument(QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), err}})
			.toJson(QJsonDocument::Compact);
	return QJsonDocument(b->infoJson(id)).toJson(QJsonDocument::Compact);
}

QByteArray WebGateway::pointCloudMeasureJsonOnGuiThread(const QString& id)
{
	QString err;
	auto* b = pcBridge(m_document ? cloudsim::host::documentHostFromScope(m_document.get()) : nullptr, &err);
	if (!b)
		return QJsonDocument(QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), err}})
			.toJson(QJsonDocument::Compact);
	return QJsonDocument(b->measureJson(id)).toJson(QJsonDocument::Compact);
}

bool WebGateway::pointCloudPreviewSoupOnGuiThread(const QString& id, std::size_t maxPoints, std::vector<float>& out,
												  QString* err)
{
	auto* host = m_document ? cloudsim::host::documentHostFromScope(m_document.get()) : nullptr;
	auto* b = pcBridge(host, err);
	if (!b)
		return false;
	return b->previewSoup(id, maxPoints, out, err);
}

bool WebGateway::pointCloudChunkSoupOnGuiThread(const QString& id, int lod, int index, std::size_t maxPoints,
												std::vector<float>& out, QJsonObject* meta, QString* err)
{
	auto* host = m_document ? cloudsim::host::documentHostFromScope(m_document.get()) : nullptr;
	auto* b = pcBridge(host, err);
	if (!b)
		return false;
	return b->chunkSoup(id, lod, index, maxPoints, out, meta, err);
}

QByteArray WebGateway::pointCloudPostJsonOnGuiThread(const QByteArray& body,
													  QJsonObject (cloudsim::host::HeadlessPointCloudBridge::*method)(
														  const QJsonObject&))
{
	QString err;
	auto* b = pcBridge(m_document ? cloudsim::host::documentHostFromScope(m_document.get()) : nullptr, &err);
	if (!b)
		return QJsonDocument(QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), err}})
			.toJson(QJsonDocument::Compact);
	const QJsonObject req = QJsonDocument::fromJson(body).object();
	return QJsonDocument((b->*method)(req)).toJson(QJsonDocument::Compact);
}

} // namespace cloudsim::web
