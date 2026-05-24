#pragma once

#include "IPluginHostContext.h"

#include <QObject>
#include <vector>

class BackendDataBase;
class DocumentPage;
class MainWindow;
class PluginDocumentAdapter;
class PluginSceneBridgeAdapter;

/// Host implementation of \ref IPluginHostContext (Widget-only; not exported to plugins).
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
		const PluginMeshCreateOptions& options, QString* outError) override;

	bool registerBackendType(const PluginBackendMeta& meta, QString* outError) override;
	bool registerTriangleMesh(const std::vector<float>& triangleSoup, const PluginMeshCreateOptions& options,
		QString* outError) override;

	std::string importFileIntoActiveDocument(const std::string& pathUtf8, bool isPointCloud,
		std::string* outError) override;

	MainWindow* mainWindow() const { return m_mainWindow; }

private:
	bool registerMeshFromSoup(std::vector<float> soup, const PluginMeshCreateOptions& options, QString* outError);

	MainWindow* m_mainWindow = nullptr;
	std::vector<std::unique_ptr<PluginDocumentAdapter>> m_documents;
	std::vector<std::function<void(IPluginDocument*)>> m_docChangeCallbacks;
	std::vector<QDockWidget*> m_ownedDocks;
};
