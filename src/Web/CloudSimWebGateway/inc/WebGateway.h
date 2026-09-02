#ifndef CLOUDSIMWEBGATEWAY_WEBGATEWAY_H
#define CLOUDSIMWEBGATEWAY_WEBGATEWAY_H

/// @file WebGateway.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 独立进程 HTTP/SSE 网关（P1–P5 能力面）

#include "cloudsim_web_gateway_global.h"

#include <QByteArray>
#include <QJsonObject>
#include <QObject>
#include <QString>
#include <memory>
#include <vector>

namespace httplib
{
class Server;
}

namespace cloudsim::core
{
class ICloudSimContext;
class IDocumentScope;
} // namespace cloudsim::core

namespace cloudsim::host
{
class DocumentHost;
class HeadlessPointCloudBridge;
}

namespace cloudsim::web
{
struct WebGatewayConfig
{
	QString bindHost = QStringLiteral("127.0.0.1");
	int port = 8787;
	QString staticRoot;
};

/// IO 在后台线程；契约调用经 BlockingQueued 投递到 Qt 主线程
class CLOUDSIM_WEB_GATEWAY_API WebGateway : public QObject
{
	Q_OBJECT
public:
	explicit WebGateway(cloudsim::core::ICloudSimContext& context, WebGatewayConfig config,
						QObject* parent = nullptr);
	~WebGateway() override;

	bool start(QString* outError = nullptr);
	void stop();

	int port() const { return m_config.port; }
	cloudsim::core::IDocumentScope* document() const { return m_document.get(); }

private:
	void pushEvent(const QString& jsonLine);
	void registerApiRoutes(cloudsim::host::DocumentHost* host);
	void releaseProjectFileLock();
	void reacquireProjectFileLock(const QString& path);

	bool openProjectOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body, QString* err,
								int* objectCount);
	bool newProjectOnGuiThread(cloudsim::host::DocumentHost* host, QString* err);
	bool saveProjectOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body, QString* err);
	QByteArray objectsJsonOnGuiThread();
	QByteArray objectDetailJsonOnGuiThread(const QString& id);
	bool patchObjectOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id, const QByteArray& body,
								QString* err);
	bool importObjectOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body, QString* err,
								 QString* outId);
	/// 插入场景坐标系（FrameBackendData），对齐桌面「插入 → 坐标系」
	bool createCoordinateFrameOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body, QString* err,
										  QString* outId);
	QByteArray coordinateFramesJsonOnGuiThread();
	bool deleteObjectOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id, QString* err);
	bool attachChildOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body, QString* err);
	bool meshSoupOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id, std::vector<float>& out,
							 QString* err);
	bool selectionOnGuiThread(const QByteArray& body, QString* err);
	/// 原生对话框选路径（浏览器拿不到本机绝对路径）
	QByteArray nativeDialogOnGuiThread(const QByteArray& body);

	// 机器人 API
	QByteArray robotProgramsJsonOnGuiThread();
	bool setRobotProgramsOnGuiThread(const QByteArray& body, QString* err);
	bool applyJointsOnGuiThread(const QByteArray& body, QString* err);
	QByteArray robotInstancesJsonOnGuiThread();
	QByteArray robotJointsMetaJsonOnGuiThread(const QString& sceneRootBackendId);
	QByteArray robotResolveJsonOnGuiThread(const QString& backendId);
	bool placeRobotOnGuiThread(const QByteArray& body, QString* err);
	bool tcpIkRobotOnGuiThread(const QByteArray& body, QString* err, QJsonObject* out);
	QByteArray robotTcpPoseJsonOnGuiThread(const QString& sceneRootBackendId);
	QByteArray robotFramesJsonOnGuiThread(const QString& sceneRootBackendId);
	bool putRobotFramesOnGuiThread(const QByteArray& body, QString* err);
	QByteArray robotExternalAxesJsonOnGuiThread(const QString& sceneRootBackendId);
	bool putRobotExternalAxesOnGuiThread(const QByteArray& body, QString* err);
	bool mutateRobotFramesOnGuiThread(const QByteArray& body, QString* err, QJsonObject* out);
	bool captureRobotToolFrameOnGuiThread(const QByteArray& body, QString* err);
	bool captureRobotUserFrameOnGuiThread(const QByteArray& body, QString* err);
	bool resetRobotToolFrameOnGuiThread(const QByteArray& body, QString* err);
	QByteArray robotFrameOverlaysJsonOnGuiThread(const QString& sceneRootBackendId);
	QByteArray instructionPropertiesJsonOnGuiThread(const QString& instructionId);
	bool patchInstructionPropertyOnGuiThread(const QString& instructionId, const QByteArray& body, QString* err);
	bool registerUrdfOnGuiThread(const QByteArray& body, QString* err, QJsonObject* out);
	bool planInstructionOnGuiThread(const QByteArray& body, QString* err, QJsonObject* out);

	// 轨迹 / 特征拾取（无头）
	QByteArray trajectorySessionJsonOnGuiThread();
	QByteArray trajectoryPathPlansJsonOnGuiThread(const QString& sceneRootBackendId);
	bool createPathPlanOnGuiThread(const QByteArray& body, QString* err, QJsonObject* out);
	bool bindPathPlanOnGuiThread(const QByteArray& body, QString* err);
	bool beginTrajectoryEditOnGuiThread(QString* err);
	bool cancelTrajectoryEditOnGuiThread();
	bool pickMeshElementOnGuiThread(const QByteArray& body, QString* err, QJsonObject* out);
	bool pickHoverOnGuiThread(const QByteArray& body, QString* err, QJsonObject* out);
	QByteArray featureCatalogOnGuiThread(const QString& workpieceBackendId, QString* err);
	bool featureSchemaOnGuiThread(const QString& strategyId, QString* err, QJsonObject* out);
	bool discretizeFeaturesOnGuiThread(const QByteArray& body, QString* err);
	bool discretizeMeshSpecOnGuiThread(const QByteArray& body, QString* err);
	bool setTrajectoryPipelineOnGuiThread(const QByteArray& body, QString* err);
	QByteArray trajectoryPipelineJsonOnGuiThread();
	bool trajectoryOpSchemaOnGuiThread(const QString& kind, int opIndex, QString* err, QJsonObject* out);
	bool fillTrajectoryRecipeOnGuiThread(const QByteArray& body, QString* err);
	bool previewTrajectoryOnGuiThread(QString* err, QJsonObject* out);
	bool previewTrajectoryRawOnGuiThread(QString* err, QJsonObject* out);
	bool applyTrajectoryOnGuiThread(QString* err);
	bool emitTrajectoryRawOnGuiThread(QString* err);
	bool trajectoryOpPaletteOnGuiThread(QString* err, QJsonObject* out);
	bool resetTrajectoryPipelineOnGuiThread(QString* err);
	bool undoTrajectoryDraftOnGuiThread(QString* err);
	bool redoTrajectoryDraftOnGuiThread(QString* err);
	QByteArray listTrajectoryTemplatesOnGuiThread(const QString& kind);
	bool saveTrajectoryTemplateOnGuiThread(const QString& kind, const QByteArray& body, QString* err);
	bool loadTrajectoryTemplateOnGuiThread(const QString& kind, const QString& name, QByteArray* out, QString* err);
	bool deleteTrajectoryTemplateOnGuiThread(const QString& kind, const QString& name, QString* err);

	// 点云（无头）
	QByteArray pointCloudInfoJsonOnGuiThread(const QString& id);
	QByteArray pointCloudMeasureJsonOnGuiThread(const QString& id);
	bool pointCloudPreviewSoupOnGuiThread(const QString& id, std::size_t maxPoints, std::vector<float>& out,
										  QString* err);
	bool pointCloudChunkSoupOnGuiThread(const QString& id, int lod, int index, std::size_t maxPoints,
										std::vector<float>& out, QJsonObject* meta, QString* err);
	QByteArray pointCloudPostJsonOnGuiThread(const QByteArray& body,
											 QJsonObject (cloudsim::host::HeadlessPointCloudBridge::*method)(
												 const QJsonObject&));
	void registerPointCloudRoutes(cloudsim::host::DocumentHost* host);
	void registerParityRoutes(cloudsim::host::DocumentHost* host);
	httplib::Server& httpServer();

	// 几何 / 模式 / AI 薄面
	QByteArray sidecarGetOnGuiThread(const QString& key);
	bool sidecarPutOnGuiThread(const QString& key, const QByteArray& body, QString* err);
	QByteArray modesCatalogJson() const;
	QByteArray helpIndexJson() const;
	QByteArray aiStatusJson() const;

	QByteArray ioSignalsJsonOnGuiThread(cloudsim::host::DocumentHost* host);
	bool ioSignalsPutOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body, QString* err);
	QByteArray ioSignalNamesJsonOnGuiThread(cloudsim::host::DocumentHost* host, const QString& kindFilter);
	bool ioSignalRuntimePatchOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body, QString* err);
	bool ioSignalRuntimeResetOnGuiThread(cloudsim::host::DocumentHost* host, QString* err);

	QByteArray ioNetworkJsonOnGuiThread(cloudsim::host::DocumentHost* host);
	bool ioNetworkOwnerSignalsPutOnGuiThread(cloudsim::host::DocumentHost* host, const QString& ownerId,
											 const QByteArray& body, QString* err);
	bool ioNetworkWirePostOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body, QString* err);
	bool ioNetworkWireDeleteOnGuiThread(cloudsim::host::DocumentHost* host, const QString& wireId, QString* err);
	bool ioNetworkOwnerLayoutPatchOnGuiThread(cloudsim::host::DocumentHost* host, const QString& ownerId,
											  const QByteArray& body, QString* err);
	bool ioNetworkRuntimePatchOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body, QString* err);
	bool ioNetworkRuntimeResetOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body, QString* err);

	QByteArray customDevicesListJsonOnGuiThread(cloudsim::host::DocumentHost* host);
	QByteArray customDeviceDetailJsonOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id);
	bool customDevicePutOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id, const QByteArray& body,
									QString* err);
	bool customDeviceApplyQOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id, const QByteArray& body,
									   QString* err);
	bool customDeviceGotoPoseOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id, const QByteArray& body,
										 QString* err);
	bool customDeviceAssemblyPostOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body, QString* err,
											 QString* outId);
	QByteArray customDeviceAssemblyCandidatesJsonOnGuiThread(cloudsim::host::DocumentHost* host);
	bool customDeviceEnsureOnGuiThread(cloudsim::host::DocumentHost* host, const QByteArray& body, QString* err,
									   QString* outId);
	bool customDeviceAttachOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id, const QByteArray& body,
									   QString* err);
	bool customDeviceExportUrdfOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id, const QByteArray& body,
										   QString* err, QString* outDir);
	QByteArray robotsForMountJsonOnGuiThread(cloudsim::host::DocumentHost* host);
	bool customDeviceMountOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id, const QByteArray& body,
									  QString* err);
	bool customDeviceUnmountOnGuiThread(cloudsim::host::DocumentHost* host, const QString& id, QString* err);

	cloudsim::core::ICloudSimContext& m_context;
	WebGatewayConfig m_config;
	std::unique_ptr<cloudsim::core::IDocumentScope> m_document;
	struct Impl;
	std::unique_ptr<Impl> m_impl;
};

} // namespace cloudsim::web

#endif // CLOUDSIMWEBGATEWAY_WEBGATEWAY_H
