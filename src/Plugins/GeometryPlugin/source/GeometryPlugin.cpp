/// @file GeometryPlugin.cpp
/// @brief GeometryPlugin 实现

#include "GeometryPlugin.h"

#include "GeometryDockWidget.h"
#include "IPluginGeometryHost.h"
#include "IPluginHostContext.h"

QString GeometryPlugin::pluginId() const
{
	return QStringLiteral("com.cloudsim.geometry");
}

QString GeometryPlugin::displayName() const
{
	return QStringLiteral("Geometry");
}

bool GeometryPlugin::initialize(IPluginHostContext* host)
{
	if (!host)
	{
		return false;
	}
	if (host->hostVersion() < 0x00010700)
	{
		host->logError(QStringLiteral("GeometryPlugin requires host 1.7.0+"));
		return false;
	}
	if (!host->geometryHost())
	{
		host->logError(QStringLiteral("Geometry host API unavailable"));
		return false;
	}
	m_host = host;
	if (!host->sidePanelTabParent())
	{
		return false;
	}
	m_dockWidget = new GeometryDockWidget(host, nullptr);
	const char* tabTitle = host->useChinese() ? "几何" : "Geometry";
	if (host->registerSidePanelTab(tabTitle, m_dockWidget) < 0)
	{
		return false;
	}
	host->onActiveDocumentChanged(
		[this](IPluginDocument*)
		{
			if (m_dockWidget)
			{
				m_dockWidget->refreshDocumentLabel();
			}
		});
	host->onLanguageChanged([this](const bool) { applyLanguage(); });
	applyLanguage();
	host->logInfo(host->useChinese() ? QStringLiteral("几何插件已加载。")
									 : QStringLiteral("Geometry plugin initialized."));
	return true;
}

void GeometryPlugin::shutdown()
{
	if (m_host && m_dockWidget)
	{
		m_host->unregisterSidePanelTab(m_dockWidget);
	}
	m_dockWidget = nullptr;
	m_host = nullptr;
	m_geometryMenu = nullptr;
}

void GeometryPlugin::applyLanguage()
{
	if (!m_host || !m_dockWidget)
	{
		return;
	}
	m_dockWidget->applyLanguage();
	m_host->setSidePanelTabTitle(m_dockWidget, m_host->useChinese() ? "几何" : "Geometry");
}
