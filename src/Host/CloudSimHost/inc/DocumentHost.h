#ifndef CLOUDSIMHOST_DOCUMENTHOST_H
#define CLOUDSIMHOST_DOCUMENTHOST_H

/// @file DocumentHost.h
/// @brief 单文档组合根

#include "cloudsim_host_global.h"

#include "IDocumentScope.h"
#include "IPerLinkKinematicsHost.h"
#include "IPerLinkRobotStateAccessor.h"

#include <QHash>
#include <QMap>
#include <QStringList>
#include <QVector>
#include <QWidget>
#include <memory>
#include <unordered_set>

namespace cloudsim::core
{
class EventHub;
}

class OsgWidget;
class QVBoxLayout;
class BackendDataManager;
class BackendHierarchyModel;
class BackendDataBase;
class MeshBackendData;
class PointCloudBackendData;
class RobotProgramStore;
class BackendSceneDocumentFacade;

#include "BackendFollowReverseIndex.h"
#include "OsgWidgetSceneBridge.h"

namespace cloudsim::host
{
class IRobotUrdfImportContext;
class IRobotInstructionPropertyDelegate;

/// 单文档组合根
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

	/// 文档内 OsgWidget（构造期可用，勿经 render().widget()）
	OsgWidget* osgWidget() { return m_osgWidget; }
	const OsgWidget* osgWidget() const { return m_osgWidget; }

	/// 中央 alternate（流程画布等）；不销毁 OsgWidget
	void setCentralAlternateWidget(QWidget* widget);
	void showCentralScene3D();
	void showCentralAlternate();
	bool isShowingCentralAlternate() const;
	QWidget* centralAlternateWidget() const;

	/// 1.21.0+：把 OsgWidget reparent 到外部槽（建模页中区）
	bool embedRenderWidget(QWidget* slot, QString* outError = nullptr);
	void restoreRenderWidget();
	bool isRenderWidgetEmbedded() const { return m_osgEmbedded; }

	/// 存量 backend 入口（新代码优先 `data()`；勿在 UI 层继续扩散 BackendDataManager 头）
	BackendDataManager& backend();
	const BackendDataManager& backend() const;
	RobotProgramStore& robotProgramStore();
	BackendHierarchyModel& hierarchyModel();
	const BackendHierarchyModel& hierarchyModel() const;
	/// Follow 反向索引（经 IDataService.followTargetId 重建）
	BackendFollowReverseIndex& followReverseIndex();
	/// 场景桥接
	OsgWidgetSceneBridge& sceneBridge();
	/// 场景/后端数据门面（PluginHost 与导入后可见性同步）
	BackendSceneDocumentFacade sceneFacade();

	/// 网格加载到场景
	bool loadMeshFromBackendIntoScene(const MeshBackendData& data, QString* errorMessage = nullptr,
									  bool resetViewToHome = true, bool showWireOutline = true,
									  bool useSceneLighting = true);
	/// URDF 连杆：顶点已在 link 系，跳过内层去心
	bool loadUrdfLinkMeshIntoScene(const MeshBackendData& data, QString* errorMessage = nullptr);
	void clearStagingGeometry();
	void syncSceneBackendParent(const std::string& childBackendId, const std::string& parentBackendId);
	void focusSceneCameraOnBackend(const std::string& backendId);

	/// 工程旁路表
	QMap<QString, QString>& backendSourcePath();
	const QMap<QString, QString>& backendSourcePath() const;
	QMap<QString, QString>& backendSourceType();
	const QMap<QString, QString>& backendSourceType() const;
	QMap<QString, QString>& backendParentId();
	const QMap<QString, QString>& backendParentId() const;
	/// 删子树
	QStringList removeBackendSubtree(const QString& rootBackendId);

	void setProjectFilePath(const QString& path);
	const QString& projectFilePath() const;

	/// Follow 脏集
	std::unordered_set<std::string>& followDirtyBackendIds();
	void markFollowAttachmentDirtyFromBackendMove(const std::string& seedBackendId);
	void invalidateFollowReverseIndex();
	void clearFollowDirtyBackendIds();
	void requestFollowSolveForced();
	bool takeFollowSolveForced();
	bool followSolveForcedPending() const;
	/// URDF 根/连杆位姿由 FK 独占，禁止作 Follow follower
	bool isKinematicsOwnedBackend(const std::string& backendId) const;
	/// 卸掉运动学对象上误装的 FollowAttachment（工程 edges / 旧工程迁移）
	void stripKinematicsOwnedFollowAttachments();
	/// FK 批量抑制脏通知
	void setSuppressRobotFollowDirtyNotify(bool suppress);
	bool suppressRobotFollowDirtyNotify() const;
	/// 属性面板连续数值编辑：跳过逐步全量 OSG 同步，失焦后一次性提交
	void setDeferPropertyPanelVisualFullSync(bool defer);
	bool deferPropertyPanelVisualFullSync() const;

	void ensureSelectionVisualForBackend(const std::string& backendId, bool urdfLinkMesh = false);
	bool syncOuterPatFromBackendId(const std::string& backendId);

	/// URDF 导入上下文
	void setRobotUrdfImportContext(IRobotUrdfImportContext* context);
	IRobotUrdfImportContext* robotUrdfImportContext() const;

	/// 仿真指令属性（Widget 注入，供 IRobotService 转发）
	void setInstructionPropertyDelegate(IRobotInstructionPropertyDelegate* delegate);
	IRobotInstructionPropertyDelegate* instructionPropertyDelegate() const;

	/// per-link 机器人运动学宿主注入（由 DocumentPage 实现或 Host 内部实现）
	void setPerLinkKinematicsHost(IPerLinkKinematicsHost* host);
	IPerLinkKinematicsHost* perLinkKinematicsHost() const;

	/// per-link 机器人状态访问器注入（由 DocumentPage 实现，供 Host 实现类访问状态）
	void setPerLinkRobotStateAccessor(IPerLinkRobotStateAccessor* accessor);
	IPerLinkRobotStateAccessor* perLinkRobotStateAccessor() const;

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
	QVBoxLayout* m_centralLayout = nullptr;
	OsgWidget* m_osgWidget = nullptr;
	QWidget* m_centralAlternate = nullptr;
	bool m_osgEmbedded = false;
	QWidget* m_osgEmbedSlot = nullptr;
	QMap<QString, QString> m_backendSourcePath;
	QMap<QString, QString> m_backendSourceType;
	QMap<QString, QString> m_backendParentId;
	QString m_projectFilePath;
	std::unordered_set<std::string> m_followDirtyBackendIds;
	bool m_followSolveForced = false;
	bool m_suppressRobotFollowDirtyNotify = false;
	bool m_deferPropertyPanelVisualFullSync = false;
	IRobotUrdfImportContext* m_robotUrdfImportContext = nullptr;
	IRobotInstructionPropertyDelegate* m_instructionPropertyDelegate = nullptr;
	IPerLinkKinematicsHost* m_perLinkKinematicsHost = nullptr;
	IPerLinkRobotStateAccessor* m_perLinkRobotStateAccessor = nullptr;
};

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_DOCUMENTHOST_H
