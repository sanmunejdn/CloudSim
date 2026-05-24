#include "PluginHostContext.h"

#include "DocumentImportFacade.h"
#include "CoreTypes.h"
#include "IDataService.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendPrimitiveGeometry.h"
#include "BackendRegistry.h"
#include "CloudSimPluginVersion.h"
#include "DocumentPage.h"
#include "JobSystem.h"
#include "MainWindow.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "PluginDelegatedBackend.h"
#include "PluginDocumentAdapter.h"
#include "RunLogger.h"

#include <QAction>
#include <QCoreApplication>
#include <QDockWidget>
#include <QFileInfo>
#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QMetaObject>
#include <QTabWidget>
#include <QThread>

namespace
{
BackendPrimitiveGeometry::PrimitiveKind toDataKind(PluginPrimitiveKind kind)
{
	switch (kind)
	{
	case PluginPrimitiveKind::Cylinder:
		return BackendPrimitiveGeometry::PrimitiveKind::Cylinder;
	case PluginPrimitiveKind::Cone:
		return BackendPrimitiveGeometry::PrimitiveKind::Cone;
	case PluginPrimitiveKind::Sphere:
		return BackendPrimitiveGeometry::PrimitiveKind::Sphere;
	case PluginPrimitiveKind::Box:
	default:
		return BackendPrimitiveGeometry::PrimitiveKind::Box;
	}
}
} // namespace

PluginHostContext::PluginHostContext(MainWindow* mainWindow, QObject* parent)
	: QObject(parent)
	, m_mainWindow(mainWindow)
{
}

PluginHostContext::~PluginHostContext() = default;

void PluginHostContext::attachDocumentTabSignals()
{
	if (!m_mainWindow || !m_mainWindow->documentTabs())
	{
		return;
	}
	QTabWidget* tabs = m_mainWindow->documentTabs();
	connect(tabs, &QTabWidget::currentChanged, this, [this](int) {
		refreshDocumentAdapters();
		IPluginDocument* doc = activeDocument();
		for (const auto& cb : m_docChangeCallbacks)
		{
			if (cb)
			{
				cb(doc);
			}
		}
	});
	refreshDocumentAdapters();
}

void PluginHostContext::refreshDocumentAdapters()
{
	m_documents.clear();
	if (!m_mainWindow || !m_mainWindow->documentTabs())
	{
		return;
	}
	QTabWidget* tabs = m_mainWindow->documentTabs();
	for (int i = 0; i < tabs->count(); ++i)
	{
		if (auto* page = qobject_cast<DocumentPage*>(tabs->widget(i)))
		{
			m_documents.push_back(std::make_unique<PluginDocumentAdapter>(page));
		}
	}
}

unsigned int PluginHostContext::hostVersion() const
{
	return cloudsimPluginHostVersion();
}

QString PluginHostContext::applicationDirPath() const
{
	return QCoreApplication::applicationDirPath();
}

void PluginHostContext::logInfo(const QString& message) const
{
	RunLogger::info(message.toStdString());
}

void PluginHostContext::logWarn(const QString& message) const
{
	RunLogger::warn(message.toStdString());
}

void PluginHostContext::logError(const QString& message) const
{
	RunLogger::error(message.toStdString());
}

int PluginHostContext::documentCount() const
{
	return m_mainWindow ? m_mainWindow->documentTabCount() : 0;
}

IPluginDocument* PluginHostContext::activeDocument()
{
	if (!m_mainWindow)
	{
		return nullptr;
	}
	DocumentPage* page = m_mainWindow->currentPage();
	if (!page)
	{
		return nullptr;
	}
	for (const auto& ad : m_documents)
	{
		if (ad && ad->documentPage() == page)
		{
			return ad.get();
		}
	}
	return nullptr;
}

const IPluginDocument* PluginHostContext::activeDocument() const
{
	return const_cast<PluginHostContext*>(this)->activeDocument();
}

IPluginDocument* PluginHostContext::documentAt(int index)
{
	if (index < 0 || index >= static_cast<int>(m_documents.size()))
	{
		return nullptr;
	}
	return m_documents[static_cast<std::size_t>(index)].get();
}

const IPluginDocument* PluginHostContext::documentAt(int index) const
{
	return const_cast<PluginHostContext*>(this)->documentAt(index);
}

void PluginHostContext::onActiveDocumentChanged(std::function<void(IPluginDocument*)> callback)
{
	if (callback)
	{
		m_docChangeCallbacks.push_back(std::move(callback));
	}
}

void PluginHostContext::invokeOnUiThread(std::function<void()> fn)
{
	if (!fn)
	{
		return;
	}
	if (QThread::currentThread() == thread())
	{
		fn();
		return;
	}
	QMetaObject::invokeMethod(this, [fn]() { fn(); }, Qt::QueuedConnection);
}

void PluginHostContext::enqueueJob(const QString& title, std::function<void(const PluginJobProgressFn&)> work,
	std::function<void(bool threw, const QString& throwMessage)> onFinished)
{
	if (!m_mainWindow || !m_mainWindow->jobSystem())
	{
		if (onFinished)
		{
			onFinished(true, QStringLiteral("JobSystem not available"));
		}
		return;
	}
	m_mainWindow->jobSystem()->enqueue(
		title,
		[work](const JobProgressSink& sink) {
			if (work)
			{
				PluginJobProgressFn pluginSink = [&sink](double fraction, const QString& message) {
					if (sink)
					{
						sink(fraction, message);
					}
				};
				work(pluginSink);
			}
		},
		[onFinished](bool threw, const QString& throwMessage) {
			if (onFinished)
			{
				onFinished(threw, throwMessage);
			}
		});
}

QDockWidget* PluginHostContext::registerDockWidget(const QString& title, QWidget* widget, Qt::DockWidgetArea area)
{
	if (!m_mainWindow || !widget)
	{
		return nullptr;
	}
	if (area == Qt::RightDockWidgetArea)
	{
		logWarn(QStringLiteral("registerDockWidget(Right) is deprecated; rebuild the plugin and call registerSidePanelTab."));
		QTabWidget* tabs = m_mainWindow ? m_mainWindow->rightPanelTabs() : nullptr;
		if (tabs && tabs->indexOf(widget) >= 0)
		{
			return nullptr;
		}
		const QByteArray titleUtf8 = title.toUtf8();
		(void)registerSidePanelTab(titleUtf8.constData(), widget);
		return nullptr;
	}
	auto* dock = new QDockWidget(title, m_mainWindow);
	dock->setObjectName(QStringLiteral("PluginDock_") + title);
	dock->setWidget(widget);
	m_mainWindow->addDockWidget(area, dock);
	m_ownedDocks.push_back(dock);
	return dock;
}

QWidget* PluginHostContext::sidePanelTabParent() const
{
	return m_mainWindow ? m_mainWindow->rightPanelTabs() : nullptr;
}

int PluginHostContext::registerSidePanelTab(const char* titleUtf8, QWidget* widget)
{
	if (!m_mainWindow || !widget || !titleUtf8)
	{
		return -1;
	}
	QTabWidget* tabs = m_mainWindow->rightPanelTabs();
	if (tabs && reinterpret_cast<const void*>(titleUtf8) == static_cast<const void*>(tabs))
	{
		logError(QStringLiteral("registerSidePanelTab: invalid title pointer (plugin/host ABI mismatch — rebuild the plugin)."));
		return -1;
	}
	return m_mainWindow->addPluginSidePanelTab(QString::fromUtf8(titleUtf8), widget);
}

void PluginHostContext::unregisterSidePanelTab(QWidget* widget)
{
	if (!m_mainWindow || !widget)
	{
		return;
	}
	m_mainWindow->removePluginSidePanelTab(widget);
}

QMenu* PluginHostContext::registerMenuPath(const QStringList& path)
{
	if (!m_mainWindow || path.isEmpty())
	{
		return nullptr;
	}
	QMenuBar* bar = m_mainWindow->menuBar();
	if (!bar)
	{
		return nullptr;
	}
	QMenu* current = nullptr;
	for (const QString& segment : path)
	{
		QMenu* found = nullptr;
		if (!current)
		{
			for (QAction* action : bar->actions())
			{
				if (action && action->menu() && action->menu()->title() == segment)
				{
					found = action->menu();
					break;
				}
			}
			if (!found)
			{
				found = bar->addMenu(segment);
			}
		}
		else
		{
			for (QAction* action : current->actions())
			{
				if (action && action->menu() && action->menu()->title() == segment)
				{
					found = action->menu();
					break;
				}
			}
			if (!found)
			{
				found = current->addMenu(segment);
			}
		}
		current = found;
	}
	return current;
}

QAction* PluginHostContext::registerAction(QMenu* menu, const QString& text, std::function<void()> handler)
{
	if (!menu || !handler)
	{
		return nullptr;
	}
	QAction* action = menu->addAction(text);
	QObject::connect(action, &QAction::triggered, m_mainWindow, [handler]() { handler(); });
	return action;
}

bool PluginHostContext::createPrimitiveMesh(const PluginPrimitiveMeshParams& params,
	const PluginPrimitiveMeshQuality& quality, const PluginMeshCreateOptions& options, QString* outError)
{
	BackendPrimitiveGeometry::PrimitiveMeshParams dataParams;
	dataParams.kind = toDataKind(params.kind);
	dataParams.lengthMm = params.lengthMm;
	dataParams.widthMm = params.widthMm;
	dataParams.heightMm = params.heightMm;
	dataParams.radiusMm = params.radiusMm;
	dataParams.radiusTopMm = params.radiusTopMm;

	BackendPrimitiveGeometry::PrimitiveMeshQuality dataQuality;
	dataQuality.segments = quality.segments;
	dataQuality.rings = quality.rings;

	auto soup = BackendPrimitiveGeometry::makePrimitiveTriangleSoup(dataParams, dataQuality);
	return registerMeshFromSoup(std::move(soup), options, outError);
}

bool PluginHostContext::registerTriangleMesh(const std::vector<float>& triangleSoup,
	const PluginMeshCreateOptions& options, QString* outError)
{
	if (triangleSoup.empty() || (triangleSoup.size() % 9U) != 0U)
	{
		if (outError)
		{
			*outError = QStringLiteral("Invalid triangle soup (need 9 floats per triangle).");
		}
		return false;
	}
	std::vector<float> copy = triangleSoup;
	return registerMeshFromSoup(std::move(copy), options, outError);
}

bool PluginHostContext::registerMeshFromSoup(std::vector<float> soup, const PluginMeshCreateOptions& options,
	QString* outError)
{
	if (!m_mainWindow)
	{
		if (outError)
		{
			*outError = QStringLiteral("MainWindow not available.");
		}
		return false;
	}
	DocumentPage* doc = m_mainWindow->currentPage();
	if (!doc)
	{
		if (outError)
		{
			*outError = QStringLiteral("No active document.");
		}
		return false;
	}

	const QString displayName =
		options.displayName.isEmpty() ? QStringLiteral("PluginMesh") : options.displayName;
	const QString sourcePath =
		options.sourcePath.isEmpty() ? QStringLiteral("plugin://mesh") : options.sourcePath;

	auto mesh = std::make_shared<MeshBackendData>();
	mesh->setName(displayName.toStdString());
	mesh->setTriangleSoup(std::move(soup));

	BackendVec3 pos;
	pos.x = options.poseMm.x;
	pos.y = options.poseMm.y;
	pos.z = options.poseMm.z;
	mesh->setPose(pos);

	BackendVec3 rot;
	rot.x = options.rotationDeg.x;
	rot.y = options.rotationDeg.y;
	rot.z = options.rotationDeg.z;
	mesh->setRotation(rot);

	cloudsim::host::AdoptMeshOptions adoptOpt;
	adoptOpt.sourcePath = sourcePath;
	adoptOpt.catalogTypeName = QStringLiteral("Model");
	adoptOpt.resetViewToHome = options.resetViewToHome;
	QString regErr;
	const cloudsim::host::AdoptRegistrationResult adopted =
		cloudsim::host::registerAdoptedMesh(*doc, mesh, adoptOpt, &regErr);
	if (!adopted.ok)
	{
		if (outError)
		{
			*outError = regErr.isEmpty() ? QStringLiteral("Failed to register mesh in backend.") : regErr;
		}
		return false;
	}
	if (options.selectInTree)
	{
		m_mainWindow->focusBackendInTree(mesh);
	}
	return true;
}

std::string PluginHostContext::importFileIntoActiveDocument(const std::string& pathUtf8, const bool isPointCloud,
	std::string* outError)
{
	if (!m_mainWindow)
	{
		if (outError)
		{
			*outError = "MainWindow not available";
		}
		return std::string();
	}
	DocumentPage* doc = m_mainWindow->currentPage();
	if (!doc)
	{
		if (outError)
		{
			*outError = "No active document";
		}
		return std::string();
	}
	const QString path = QString::fromStdString(pathUtf8);
	if (path.isEmpty())
	{
		if (outError)
		{
			*outError = "Empty path";
		}
		return std::string();
	}
	cloudsim::core::ImportOptionsDto opt;
	opt.quietUi = true;
	opt.resetViewToHome = false;
	opt.catalogTypeName = isPointCloud ? QStringLiteral("PointCloud") : QStringLiteral("Model");
	opt.isPointCloud = isPointCloud;
	QString importErr;
	const cloudsim::host::ImportFileKind kind =
		isPointCloud ? cloudsim::host::ImportFileKind::PointCloud : cloudsim::host::ImportFileKind::Mesh;
	const cloudsim::host::ImportFileResult imported =
		cloudsim::host::importFileIntoDocument(*doc, path, kind, opt, &importErr);
	if (!imported.ok)
	{
		if (outError)
		{
			*outError = importErr.toStdString();
		}
		return std::string();
	}
	return imported.rootBackendId.toStdString();
}

bool PluginHostContext::registerBackendType(const PluginBackendMeta& meta, QString* outError)
{
	if (meta.className.empty() || !meta.factory)
	{
		if (outError)
		{
			*outError = QStringLiteral("Invalid PluginBackendMeta (className/factory required).");
		}
		return false;
	}

	BackendMeta reg;
	reg.className = meta.className;
	reg.displayName = meta.displayName.empty() ? meta.className : meta.displayName;
	reg.supportsTransform = meta.supportsTransform;
	reg.supportsVisibility = meta.supportsVisibility;
	reg.factory = [factory = meta.factory]() -> std::shared_ptr<BackendDataBase> {
		const std::shared_ptr<IPluginBackendObject> delegate = factory();
		if (!delegate)
		{
			return nullptr;
		}
		return std::make_shared<PluginDelegatedBackend>(delegate);
	};
	if (meta.propertyRowsProvider)
	{
		reg.propertyEditorFactory = [provider = meta.propertyRowsProvider](BackendDataBase* base) -> void* {
			(void)base;
			(void)provider;
			return nullptr;
		};
	}

	BackendRegistry::instance().registerType(reg);
	return true;
}
