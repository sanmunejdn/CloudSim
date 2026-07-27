#ifndef CLOUDSIMPLUGINSDK_IPLUGINHOSTCONTEXT_H
#define CLOUDSIMPLUGINSDK_IPLUGINHOSTCONTEXT_H

/// @file IPluginHostContext.h
/// @brief IPluginHostContext 接口

#include "cloudsim_plugin_sdk_global.h"

#include "PluginBackendMeta.h"
#include "PluginPrimitiveTypes.h"

#include <QByteArray>
#include <QJsonObject>
#include <QString>
#include <QStringList>
#include <QWidget>
#include <functional>
#include <memory>
#include <vector>

class QAction;
class ICloudSimPlugin;
class IAiAssistantHost;
class IPluginDocument;
class IPluginGeometryHost;
class IPluginLabelingHost;
class IPluginPointCloudHost;
class IProcessFlowAiBridge;
class QDockWidget;
class QMenu;

using PluginJobProgressFn = std::function<void(double fraction, const QString& message)>;

class IPluginHostContext
{
public:
	virtual ~IPluginHostContext() = default;

	virtual unsigned int hostVersion() const = 0;
	virtual QString applicationDirPath() const = 0;

	virtual void logInfo(const QString& message) const = 0;
	virtual void logWarn(const QString& message) const = 0;
	virtual void logError(const QString& message) const = 0;

	virtual int documentCount() const = 0;
	virtual IPluginDocument* activeDocument() = 0;
	virtual const IPluginDocument* activeDocument() const = 0;
	virtual IPluginDocument* documentAt(int index) = 0;
	virtual const IPluginDocument* documentAt(int index) const = 0;

	/// 活动文档切换时 UI 线程回调
	virtual void onActiveDocumentChanged(std::function<void(IPluginDocument*)> callback) = 0;

	/// 投递 fn 到 UI 线程（已在则直跑）
	virtual void invokeOnUiThread(std::function<void()> fn) = 0;

	/// JobSystem 后台任务；onFinished 回 UI 线程
	virtual void enqueueJob(const QString& title, std::function<void(const PluginJobProgressFn&)> work,
							std::function<void(bool threw, const QString& throwMessage)> onFinished) = 0;

	/// 浮动 dock（左/下）；右侧用 registerSidePanelTab
	virtual QDockWidget* registerDockWidget(const QString& title, QWidget* widget,
											Qt::DockWidgetArea area = Qt::LeftDockWidgetArea) = 0;

	/// 右侧面板 Tab 父 widget；UI 未就绪可为 null
	virtual QWidget* sidePanelTabParent() const = 0;

	/// 右侧 Workspace/AI 旁加 Tab，避开仿真 UI
	virtual int registerSidePanelTab(const char* titleUtf8, QWidget* widget) = 0;
	virtual void unregisterSidePanelTab(QWidget* widget) = 0;

	/// 创建菜单路径并返回叶菜单
	virtual QMenu* registerMenuPath(const QStringList& path) = 0;
	virtual QAction* registerAction(QMenu* menu, const QString& text, std::function<void()> handler) = 0;

	/// Phase1：宿主建 MeshBackendData+OSG（同 AI create-mesh）；outBackendId 可选返回注册 id
	virtual bool createPrimitiveMesh(const PluginPrimitiveMeshParams& params, const PluginPrimitiveMeshQuality& quality,
									 const PluginMeshCreateOptions& options, QString* outError,
									 QString* outBackendId = nullptr) = 0;

	/// 两网格布尔（世界坐标 soup）；outResultBackendId 为结果 mesh id
	virtual bool booleanMesh(PluginMeshBooleanOp op, const std::string& targetBackendId,
							 const std::string& toolBackendId, const PluginBooleanMeshOptions& options,
							 std::string* outResultBackendId, QString* outError) = 0;

	/// Phase2：向 BackendRegistry 注册插件后端类型
	virtual bool registerBackendType(const PluginBackendMeta& meta, QString* outError) = 0;

	/// Phase2：三角 soup 注册 mesh（每三角 9 float，mm）
	virtual bool registerTriangleMesh(const std::vector<float>& triangleSoup, const PluginMeshCreateOptions& options,
									  QString* outError) = 0;

	/// 导入到活动文档（mesh/point cloud 各走宿主路径）
	virtual std::string importFileIntoActiveDocument(const std::string& pathUtf8, bool isPointCloud,
													 std::string* outError = nullptr) = 0;

	/// 1.2.0+：点云算法宿主；宿主版本不足时可为 null
	virtual IPluginPointCloudHost* pointCloudHost() = 0;
	virtual const IPluginPointCloudHost* pointCloudHost() const = 0;

	/// 1.3.0+：AI 助手宿主（CloudSimAiSDK）；未链 AiSDK 时可为 null
	virtual IAiAssistantHost* aiAssistantHost() = 0;
	virtual const IAiAssistantHost* aiAssistantHost() const = 0;

	/// 与主窗口 Settings → Language 一致（默认中文）
	virtual bool useChinese() const = 0;

	/// 语言切换时 UI 线程回调（Settings → Language）
	virtual void onLanguageChanged(std::function<void(bool useChinese)> callback) = 0;

	/// 更新已注册侧栏 Tab 标题（UTF-8）
	virtual void setSidePanelTabTitle(QWidget* widget, const char* titleUtf8) = 0;

	/// 1.4.0+（追加在 vtable 末尾，勿插入中间以免破坏旧插件 ABI）
	virtual bool buildPrimitiveMeshSoup(const PluginPrimitiveMeshParams& params,
										const PluginPrimitiveMeshQuality& quality,
										const PluginMeshCreateOptions& placement, std::vector<float>& outWorldSoup,
										QString* outError) = 0;

	virtual bool booleanMeshSoups(PluginMeshBooleanOp op, const std::vector<float>& targetWorldSoup,
								  const std::vector<float>& toolWorldSoup, const PluginBooleanMeshOptions& options,
								  std::string* outResultBackendId, QString* outError) = 0;

	virtual bool booleanPrimitiveMeshes(PluginMeshBooleanOp op, const PluginPrimitiveMeshParams& targetParams,
										const PluginPrimitiveMeshQuality& targetQuality,
										const PluginMeshCreateOptions& targetPlacement,
										const PluginPrimitiveMeshParams& toolParams,
										const PluginPrimitiveMeshQuality& toolQuality,
										const PluginMeshCreateOptions& toolPlacement,
										const PluginBooleanMeshOptions& options, std::string* outResultBackendId,
										QString* outError) = 0;

	/// 1.5.0+：OCC/CGAL 几何算法宿主；宿主版本不足时可为 null
	virtual IPluginGeometryHost* geometryHost() = 0;
	virtual const IPluginGeometryHost* geometryHost() const = 0;

	/// 1.6.0+：活动文档 3D 视口 PNG 截图（供 geometry.recognize 等多模态域）
	virtual bool captureActiveViewportPng(QByteArray& outPng, QString* outError = nullptr) = 0;

	/// 1.7.0+：轨迹生成页 combo 当前 STEP 工件
	virtual bool resolveTrajectoryWorkpiece(QString& outBackendId, QString& outStepPath,
											QString* outError = nullptr) = 0;

	/// 1.7.0+：枚举 catalog 并按用户文本推断 featureAxis 切片
	virtual bool buildTrajectoryFeatureCatalogSlice(const QString& backendId, const QString& stepPathUtf8,
													const QString& userText, QByteArray& outFullCatalogUtf8,
													QByteArray& outSliceUtf8, QString* outError = nullptr) = 0;

	/// 1.7.0+：3D 编号高亮预览（catalog 切片 JSON）
	virtual bool showAiFeatureCandidatePreview(const QByteArray& catalogSliceUtf8, QString* outError = nullptr) = 0;
	virtual void clearAiFeatureCandidatePreview() = 0;

	/// 1.7.0+：离散选中特征并注入轨迹编辑 session + 默认工艺流水线
	virtual bool commitAiTrajectoryFeatures(const QByteArray& featurePlanJsonUtf8, QString* outSummary,
											QString* outError = nullptr) = 0;

	/// 1.16.0+：交互式分割标注宿主；宿主版本不足时可为 null
	virtual IPluginLabelingHost* labelingHost() = 0;
	virtual const IPluginLabelingHost* labelingHost() const = 0;

	/// 1.17.0+：活动文档中央 alternate（流程画布等）；vtable 仅末尾追加
	virtual void setCentralAlternateWidget(QWidget* widget) = 0;
	virtual void showCentralScene3D() = 0;
	virtual void showCentralAlternate() = 0;
	virtual bool isShowingCentralAlternate() const = 0;

	/// 1.17.0+：工艺流程侧栏模式（藏原属性/工作区 Dock）
	/// 1.19.0+：左=节点库+属性，右=JobSet+仿真报表（同槽改签名，须与宿主同编）
	virtual void enterProcessFlowSideUi(QWidget* leftPanel, QWidget* rightPanel) = 0;
	virtual void exitProcessFlowSideUi() = 0;

	/// 1.18.0+：工程保存/加载扩展钩子（插件写入 processFlow 等）
	virtual void onProjectAboutToSave(std::function<void(const QString& documentId, QJsonObject& root)> callback) = 0;
	virtual void onProjectLoaded(std::function<void(const QString& documentId, const QJsonObject& root)> callback) = 0;

	/// 1.20.0+：工艺流程 AI 桥接（插件注册；未加载时为 null）
	virtual void setProcessFlowAiBridge(IProcessFlowAiBridge* bridge) = 0;
	virtual IProcessFlowAiBridge* processFlowAiBridge() = 0;
	virtual const IProcessFlowAiBridge* processFlowAiBridge() const = 0;

	/// 1.21.0+：把活动文档 3D 视口嵌入建模页中区槽；退出时 restore
	virtual bool embedActiveRenderWidget(QWidget* slot, QString* outError = nullptr) = 0;
	virtual void restoreActiveRenderWidget() = 0;

	/// 1.22.0+：菜单栏下方独占模式工具条（nullptr 清除）；几何建模等插件挂 Ribbon
	virtual void setModeToolBar(QWidget* toolBar) = 0;

	/// 1.25.0+：互斥工作区模式。进入工艺流程/几何建模时 claim；其它插件在回调里只清本地状态
	virtual void claimWorkspaceMode(const QString& modeId) = 0;
	virtual void onWorkspaceModeClaimed(std::function<void(const QString& modeId)> callback) = 0;
	virtual QString currentWorkspaceMode() const = 0;
};

#endif // CLOUDSIMPLUGINSDK_IPLUGINHOSTCONTEXT_H
