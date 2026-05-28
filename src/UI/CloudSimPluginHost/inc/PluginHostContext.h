#pragma once

#include "IPluginHostContext.h"

#include <QObject>

#include <functional>
#include <memory>
#include <vector>

class AiAssistantHostImpl;
class PluginPointCloudHostImpl;
class BackendDataBase;
class DocumentPage;
class MainWindow;
class PluginDocumentAdapter;
class PluginSceneBridgeAdapter;

/// IPluginHostContext 宿主实现（仅 Widget，不导出插件）
class PluginHostContext : public QObject, public IPluginHostContext
{
	Q_OBJECT

public:
	explicit PluginHostContext(MainWindow* mainWindow, QObject* parent = nullptr);
	~PluginHostContext() override;

	void attachDocumentTabSignals();
	void refreshDocumentAdapters();

	unsigned int hostVersion() const override;
	QString applicationDirPath() const override;

	void logInfo(const QString& message) const override;
	void logWarn(const QString& message) const override;
	void logError(const QString& message) const override;

	int documentCount() const override;
	IPluginDocument* activeDocument() override;
	const IPluginDocument* activeDocument() const override;
	IPluginDocument* documentAt(int index) override;
	const IPluginDocument* documentAt(int index) const override;

	void onActiveDocumentChanged(std::function<void(IPluginDocument*)> callback) override;
	void invokeOnUiThread(std::function<void()> fn) override;

	void enqueueJob(const QString& title, std::function<void(const PluginJobProgressFn&)> work,
		std::function<void(bool threw, const QString& throwMessage)> onFinished) override;

	QDockWidget* registerDockWidget(const QString& title, QWidget* widget, Qt::DockWidgetArea area) override;
	QWidget* sidePanelTabParent() const override;
	int registerSidePanelTab(const char* titleUtf8, QWidget* widget) override;
	void unregisterSidePanelTab(QWidget* widget) override;
	QMenu* registerMenuPath(const QStringList& path) override;
	QAction* registerAction(QMenu* menu, const QString& text, std::function<void()> handler) override;

	bool createPrimitiveMesh(const PluginPrimitiveMeshParams& params, const PluginPrimitiveMeshQuality& quality,
		const PluginMeshCreateOptions& options, QString* outError, QString* outBackendId = nullptr) override;

	bool booleanMesh(PluginMeshBooleanOp op, const std::string& targetBackendId, const std::string& toolBackendId,
		const PluginBooleanMeshOptions& options, std::string* outResultBackendId, QString* outError) override;

	bool registerBackendType(const PluginBackendMeta& meta, QString* outError) override;
	bool registerTriangleMesh(const std::vector<float>& triangleSoup, const PluginMeshCreateOptions& options,
		QString* outError) override;

	std::string importFileIntoActiveDocument(const std::string& pathUtf8, bool isPointCloud,
		std::string* outError) override;

	IPluginPointCloudHost* pointCloudHost() override;
	const IPluginPointCloudHost* pointCloudHost() const override;

	bool useChinese() const override;
	void onLanguageChanged(std::function<void(bool useChinese)> callback) override;
	void setSidePanelTabTitle(QWidget* widget, const char* titleUtf8) override;

	bool buildPrimitiveMeshSoup(const PluginPrimitiveMeshParams& params, const PluginPrimitiveMeshQuality& quality,
		const PluginMeshCreateOptions& placement, std::vector<float>& outWorldSoup, QString* outError) override;

	bool booleanMeshSoups(PluginMeshBooleanOp op, const std::vector<float>& targetWorldSoup,
		const std::vector<float>& toolWorldSoup, const PluginBooleanMeshOptions& options,
		std::string* outResultBackendId, QString* outError) override;

	bool booleanPrimitiveMeshes(PluginMeshBooleanOp op, const PluginPrimitiveMeshParams& targetParams,
		const PluginPrimitiveMeshQuality& targetQuality, const PluginMeshCreateOptions& targetPlacement,
		const PluginPrimitiveMeshParams& toolParams, const PluginPrimitiveMeshQuality& toolQuality,
		const PluginMeshCreateOptions& toolPlacement, const PluginBooleanMeshOptions& options,
		std::string* outResultBackendId, QString* outError) override;

	IAiAssistantHost* aiAssistantHost() override;
	const IAiAssistantHost* aiAssistantHost() const override;

	void notifyLanguageChanged();

	MainWindow* mainWindow() const { return m_mainWindow; }

private:
	bool booleanSoupsAndRegister(const std::vector<float>& targetWorldSoup, const std::vector<float>& toolWorldSoup,
		PluginMeshBooleanOp op, const PluginBooleanMeshOptions& options, std::string* outResultBackendId,
		QString* outError);

	bool registerMeshFromSoup(std::vector<float> soup, const PluginMeshCreateOptions& options, QString* outError,
		QString* outBackendId = nullptr);

	MainWindow* m_mainWindow = nullptr;
	std::unique_ptr<PluginPointCloudHostImpl> m_pointCloudHost;
	std::unique_ptr<AiAssistantHostImpl> m_aiHost;
	std::vector<std::function<void(bool useChinese)>> m_languageCallbacks;
	std::vector<std::unique_ptr<PluginDocumentAdapter>> m_documents;
	std::vector<std::function<void(IPluginDocument*)>> m_docChangeCallbacks;
	std::vector<QDockWidget*> m_ownedDocks;
};
