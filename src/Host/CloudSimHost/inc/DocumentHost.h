#pragma once

#include "cloudsim_host_global.h"

#include "IDocumentScope.h"

#include <QHash>
#include <QMap>
#include <QStringList>
#include <QVector>
#include <QWidget>

#include <memory>
#include <unordered_set>

namespace cloudsim::core {
class EventHub;
}

class OsgWidget;
class BackendDataManager;
class BackendHierarchyModel;
class MeshBackendData;
class RobotProgramStore;

#include "BackendFollowReverseIndex.h"
#include "OsgWidgetSceneBridge.h"

namespace cloudsim::host {

/// 单文档宿主：Data + OSG 视口 + 机器人元数据（Widget 通过此类型访问后端，不包含 OSG/Data 头文件）。
class CLOUDSIM_HOST_EXPORT DocumentHost : public QWidget, public cloudsim::core::IDocumentScope
{
	Q_OBJECT

public:
	DocumentHost(QWidget* parent, cloudsim::core::EventHub& events, const QString& documentId);
	~DocumentHost() override;

	QString documentId() const override;
	cloudsim::core::IDataService& data() override;
	cloudsim::core::IRobotService& robot() override;
	cloudsim::core::IRenderView& render() override;

	OsgWidget* osgWidget() const;
	BackendDataManager& backend();
	const BackendDataManager& backend() const;
	RobotProgramStore& robotProgramStore();
	BackendHierarchyModel& hierarchyModel();
	BackendFollowReverseIndex& followReverseIndex();
	OsgWidgetSceneBridge& sceneBridge();

	bool loadMeshFromBackendIntoScene(const MeshBackendData& data, QString* errorMessage = nullptr,
		bool resetViewToHome = true, bool showWireOutline = true, bool useSceneLighting = true);

	QMap<QString, QString>& backendSourcePath();
	const QMap<QString, QString>& backendSourcePath() const;
	QMap<QString, QString>& backendSourceType();
	const QMap<QString, QString>& backendSourceType() const;
	QMap<QString, QString>& backendParentId();
	const QMap<QString, QString>& backendParentId() const;
	QStringList removeBackendSubtree(const QString& rootBackendId);

	void setProjectFilePath(const QString& path);
	const QString& projectFilePath() const;

	std::unordered_set<std::string>& followDirtyBackendIds();
	void clearFollowDirtyBackendIds();
	void requestFollowSolveForced();
	bool takeFollowSolveForced();
	bool followSolveForcedPending() const;
	void setSuppressRobotFollowDirtyNotify(bool suppress);
	bool suppressRobotFollowDirtyNotify() const;

	/// IRobotSimulationDocument 等机器人接口仍由 Widget 侧 DocumentPage 包装转发（Phase 后续迁入 Core DTO）。

private:
	QString m_documentId;
	cloudsim::core::EventHub& m_events;
	std::unique_ptr<cloudsim::core::IDataService> m_dataService;
	std::unique_ptr<cloudsim::core::IRobotService> m_robotService;
	std::unique_ptr<cloudsim::core::IRenderView> m_renderView;
	std::unique_ptr<BackendDataManager> m_backend;
	std::unique_ptr<RobotProgramStore> m_robotProgramStore;
	std::unique_ptr<BackendHierarchyModel> m_hierarchyModel;
	BackendFollowReverseIndex m_followReverseIndex;
	OsgWidgetSceneBridge m_sceneBridge;
	OsgWidget* m_osgWidget = nullptr;
	QMap<QString, QString> m_backendSourcePath;
	QMap<QString, QString> m_backendSourceType;
	QMap<QString, QString> m_backendParentId;
	QString m_projectFilePath;
	std::unordered_set<std::string> m_followDirtyBackendIds;
	bool m_followSolveForced = false;
	bool m_suppressRobotFollowDirtyNotify = false;
};

} // namespace cloudsim::host
