#include "MainWindow.h"
#include "PluginManager.h"
#include <QTabWidget>

int MainWindow::addPluginSidePanelTab(const QString& title, QWidget* widget)
{
	if (!m_rightPanelTabs || !widget)
	{
		return -1;
	}

	const int existingIdx = m_rightPanelTabs->indexOf(widget);

	if (existingIdx >= 0)
	{
		return existingIdx;
	}

	if (widget->parentWidget() == static_cast<QWidget*>(m_rightPanelTabs))
	{
		widget->setParent(nullptr);
	}

	return m_rightPanelTabs->addTab(widget, title);
}


void MainWindow::removePluginSidePanelTab(QWidget* widget)
{
	if (!widget)
	{
		return;
	}
	if (m_rightPanelTabs)
	{
		const int idx = m_rightPanelTabs->indexOf(widget);
		if (idx >= 0)
		{
			m_rightPanelTabs->removeTab(idx);
		}
	}
	delete widget;
}

int MainWindow::documentTabCount() const

{
	return m_documentTabs ? m_documentTabs->count() : 0;
}

void MainWindow::loadPlugins()
{
	if (m_pluginsLoadStarted)
	{
		return;
	}

	m_pluginsLoadStarted = true;

	if (!m_pluginManager)
	{
		m_pluginManager = new PluginManager(this, this);
	}

	m_pluginManager->loadAllFromPluginsDirectory();
}

