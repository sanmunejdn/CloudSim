#include "HelloPlugin.h"

#include "HelloDockWidget.h"
#include "IPluginHostContext.h"
#include "PluginPrimitiveTypes.h"

#include <QMenu>

QString HelloPlugin::pluginId() const
{
	return QStringLiteral("com.cloudsim.hello");
}

QString HelloPlugin::displayName() const
{
	return QStringLiteral("Hello");
}

bool HelloPlugin::initialize(IPluginHostContext* host)
{
	if (!host)
	{
		return false;
	}
	m_host = host;

	if (!host->sidePanelTabParent())
	{
		return false;
	}
	// Let QTabWidget::addTab own reparenting; do not pre-parent to m_rightPanelTabs.
	m_dockWidget = new HelloDockWidget(host, nullptr);
	if (host->registerSidePanelTab("Hello", m_dockWidget) < 0)
	{
		return false;
	}

	host->onActiveDocumentChanged([this](IPluginDocument*) {
		if (m_dockWidget)
		{
			m_dockWidget->refreshBackendCount();
		}
	});

	QMenu* helloMenu = host->registerMenuPath({ QStringLiteral("Tools"), QStringLiteral("Hello") });
	if (helloMenu)
	{
		host->registerAction(helloMenu, QStringLiteral("Insert Test Box"), [this]() { insertTestBox(); });
	}

	host->logInfo(QStringLiteral("Hello plugin initialized."));
	return true;
}

void HelloPlugin::shutdown()
{
	if (m_host && m_dockWidget)
	{
		m_host->unregisterSidePanelTab(m_dockWidget);
		m_host->logInfo(QStringLiteral("Hello plugin shutdown."));
	}
	m_dockWidget = nullptr;
	m_host = nullptr;
}

void HelloPlugin::insertTestBox()
{
	if (!m_host)
	{
		return;
	}
	PluginPrimitiveMeshParams params;
	params.kind = PluginPrimitiveKind::Box;
	params.lengthMm = 80.0;
	params.widthMm = 60.0;
	params.heightMm = 40.0;

	PluginMeshCreateOptions options;
	options.displayName = QStringLiteral("HelloBox");
	// 宿主侧走 DocumentImportFacade::registerAdoptedMesh，与菜单导入共用注册链
	options.sourcePath = QStringLiteral("plugin://hello/box");

	QString err;
	if (!m_host->createPrimitiveMesh(params, PluginPrimitiveMeshQuality{}, options, &err))
	{
		m_host->logError(QStringLiteral("Insert Test Box failed: %1").arg(err));
		return;
	}
	m_host->logInfo(QStringLiteral("Insert Test Box succeeded."));
	if (m_dockWidget)
	{
		m_dockWidget->refreshBackendCount();
	}
}