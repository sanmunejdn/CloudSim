#ifndef CLOUDSIMHOST_HEADLESSPOINTCLOUDBRIDGE_H
#define CLOUDSIMHOST_HEADLESSPOINTCLOUDBRIDGE_H

/// @file HeadlessPointCloudBridge.h
/// @brief Web/Headless 点云：同步 Host 作业与预览 soup（无 PluginHostContext 作业队列）

#include "cloudsim_host_global.h"

#include <QByteArray>
#include <QJsonObject>
#include <QString>
#include <memory>
#include <vector>

namespace cloudsim::host
{
class DocumentHost;

/// Web 点云 REST 真源；突变后 commitPointCloudVisual（Headless 无 OSG 时为 no-op）
class CLOUDSIM_HOST_EXPORT HeadlessPointCloudBridge
{
public:
	static constexpr std::size_t kMixedRenderThreshold = 500000U;
	static constexpr std::size_t kDefaultPreviewMaxPoints = 500000U;
	static constexpr std::size_t kChunkPointCount = 250000U;

	explicit HeadlessPointCloudBridge(DocumentHost& host);
	~HeadlessPointCloudBridge();

	HeadlessPointCloudBridge(const HeadlessPointCloudBridge&) = delete;
	HeadlessPointCloudBridge& operator=(const HeadlessPointCloudBridge&) = delete;

	QJsonObject infoJson(const QString& backendId) const;
	QJsonObject measureJson(const QString& backendId) const;
	bool previewSoup(const QString& backendId, std::size_t maxPoints, std::vector<float>& outXyz, QString* err) const;
	bool chunkSoup(const QString& backendId, int lod, int index, std::size_t maxPoints, std::vector<float>& outXyz,
				   QJsonObject* outMeta, QString* err) const;

	QJsonObject downsample(const QJsonObject& body);
	QJsonObject crop(const QJsonObject& body);
	QJsonObject preprocess(const QJsonObject& body);
	QJsonObject registerCloud(const QJsonObject& body);
	QJsonObject reconstruct(const QJsonObject& body);
	QJsonObject meshPost(const QJsonObject& body);
	QJsonObject meshExportPly(const QJsonObject& body);
	QJsonObject surfaceRun(const QJsonObject& body);
	QJsonObject surfaceReset(const QJsonObject& body);

	/// 兼容旧 /api/pointcloud/op 分发
	QJsonObject deprecatedOp(const QJsonObject& body);

private:
	struct SurfaceSession;

	DocumentHost& m_host;
	std::unique_ptr<SurfaceSession> m_surfaceSession;
};

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_HEADLESSPOINTCLOUDBRIDGE_H
