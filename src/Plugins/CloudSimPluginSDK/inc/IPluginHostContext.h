#pragma once

#include "cloudsim_plugin_sdk_global.h"

#include "PluginBackendMeta.h"
#include "PluginPrimitiveTypes.h"

#include <functional>
#include <memory>
#include <vector>

#include <QDockWidget>
#include <QString>
#include <QStringList>
#include <QWidget>

class QAction;
class ICloudSimPlugin;
class IPluginDocument;
class QMenu;

/// Progress callback for background jobs (fraction in [0,1]).
using PluginJobProgressFn = std::function<void(double fraction, const QString& message)>;

/// Host services available to plugins during \c initialize() and runtime (UI thread unless noted).
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

	/// \a callback invoked on the UI thread when the active document tab changes.
	virtual void onActiveDocumentChanged(std::function<void(IPluginDocument*)> callback) = 0;

	/// Marshals \a fn to the UI thread (no-op if already on UI thread).
	virtual void invokeOnUiThread(std::function<void()> fn) = 0;

	/// Background work via host JobSystem; \a onFinished runs on the UI thread.
	virtual void enqueueJob(const QString& title, std::function<void(const PluginJobProgressFn&)> work,
		std::function<void(bool threw, const QString& throwMessage)> onFinished) = 0;

	/// Adds a floating dock (left/bottom). Avoid \c Qt::RightDockWidgetArea — use \c registerSidePanelTab instead.
	virtual QDockWidget* registerDockWidget(const QString& title, QWidget* widget,
		Qt::DockWidgetArea area = Qt::LeftDockWidgetArea) = 0;

	/// Parent for plugin panel widgets (right \c QTabWidget next to Workspace / AI). May be null before UI ready.
	virtual QWidget* sidePanelTabParent() const = 0;

	/// Adds a tab next to Workspace / AI on the right panel (no overlap with simulation UI).
	virtual int registerSidePanelTab(const char* titleUtf8, QWidget* widget) = 0;
	virtual void unregisterSidePanelTab(QWidget* widget) = 0;

	/// Creates menu path (e.g. {"Tools","Hello"}) and returns the leaf menu for adding actions.
	virtual QMenu* registerMenuPath(const QStringList& path) = 0;
	virtual QAction* registerAction(QMenu* menu, const QString& text,
		std::function<void()> handler) = 0;

	/// Phase 1: host builds \c MeshBackendData + OSG branch (same path as AI create-mesh).
	virtual bool createPrimitiveMesh(const PluginPrimitiveMeshParams& params,
		const PluginPrimitiveMeshQuality& quality, const PluginMeshCreateOptions& options,
		QString* outError) = 0;

	/// Phase 2: register a plugin backend type with the host \c BackendRegistry.
	virtual bool registerBackendType(const PluginBackendMeta& meta, QString* outError) = 0;

	/// Phase 2: register mesh from triangle soup (9 floats per triangle, mm).
	virtual bool registerTriangleMesh(const std::vector<float>& triangleSoup, const PluginMeshCreateOptions& options,
		QString* outError) = 0;

	/// Import file into the active document (mesh: Host \c importFromFile; point cloud: full import path).
	virtual std::string importFileIntoActiveDocument(const std::string& pathUtf8, bool isPointCloud,
		std::string* outError = nullptr) = 0;
};
