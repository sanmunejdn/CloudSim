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
class BackendDataBase;
class MeshBackendData;
class PointCloudBackendData;
class RobotProgramStore;

#include "BackendFollowReverseIndex.h"
#include "OsgWidgetSceneBridge.h"

namespace cloudsim::host {

class IRobotUrdfImportContext;

/// 单文档组合根：聚合 BackendDataManager、OsgWidget 与三个 Core 适配器，实现 IDocumentScope。
/// DocumentPage 继承此类即可，无需在 Widget 内再拼装 Data/OSG/Core。
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
	cloudsim::core::EventHub& events();

	/// 存量直达；新代码走 data()
	BackendDataManager& backend();
	const BackendDataManager& backend() const;
	RobotProgramStore& robotProgramStore();
	BackendHierarchyModel& hierarchyModel();
	const BackendHierarchyModel& hierarchyModel() const;
	/// 跟随反向索引，供帧回调增量求解
	BackendFollowReverseIndex& followReverseIndex();
	/// 后端 id 与场景节点的桥接门面
	OsgWidgetSceneBridge& sceneBridge();

	/// Data 网格节点 → OSG 分支
	bool loadMeshFromBackendIntoScene(const MeshBackendData& data, QString* errorMessage = nullptr,
		bool resetViewToHome = true, bool showWireOutline = true, bool useSceneLighting = true);

	/// 工程 I/O 旁路：导入源路径 / 类型 / 父 id
	QMap<QString, QString>& backendSourcePath();
	const QMap<QString, QString>& backendSourcePath() const;
	QMap<QString, QString>& backendSourceType();
	const QMap<QString, QString>& backendSourceType() const;
	QMap<QString, QString>& backendParentId();
	const QMap<QString, QString>& backendParentId() const;
	/// 逻辑删子树并移除场景视觉
	QStringList removeBackendSubtree(const QString& rootBackendId);

	void setProjectFilePath(const QString& path);
	const QString& projectFilePath() const;

	/// 跟随脏集，与 MainWindow 帧回调协作
	std::unordered_set<std::string>& followDirtyBackendIds();
	void markFollowAttachmentDirtyFromBackendMove(const std::string& seedBackendId);
	void invalidateFollowReverseIndex();
	void clearFollowDirtyBackendIds();
	void requestFollowSolveForced();
	bool takeFollowSolveForced();
	bool followSolveForcedPending() const;
	/// 机器人 tick 批量写 FK 时抑制脏通知，避免每关节触发 Follow
	void setSuppressRobotFollowDirtyNotify(bool suppress);
	bool suppressRobotFollowDirtyNotify() const;

	/// 机器人仿真细粒度接口仍由 DocumentPage 转发，后续迁入 Core DTO

	/// DocumentPage 构造后注册，供 RobotServiceAdapter::registerUrdfRobot 使用
	void setRobotUrdfImportContext(IRobotUrdfImportContext* context);
	IRobotUrdfImportContext* robotUrdfImportContext() const;

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
	IRobotUrdfImportContext* m_robotUrdfImportContext = nullptr;
};

} // namespace cloudsim::host
