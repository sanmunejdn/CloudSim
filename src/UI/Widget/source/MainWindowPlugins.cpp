/// @file MainWindowPlugins.cpp
/// @brief MainWindowPlugins 实现

#include "MainWindow.h"
#include "PluginManager.h"

#include <QAction>
#include <QSignalBlocker>
#include <QTabWidget>

namespace
{
int workspaceTabIndex(QTabWidget* tabs)
{
	return tabs && tabs->count() > 0 ? 0 : -1;
}

} // namespace

void MainWindow::registerSidePanelTabToggle(QWidget* widget, const QString& title, const bool visible)
{
	if (!widget || !m_viewMenu)
	{
		return;
	}

	const auto existing = m_sidePanelTabToggles.constFind(widget);
	if (existing != m_sidePanelTabToggles.cend())
	{
		setPluginSidePanelTabTitle(widget, title);
		if (existing.value().viewAction)
		{
			const QSignalBlocker blocker(existing.value().viewAction);
			existing.value().viewAction->setChecked(visible);
		}
		applySidePanelTabToggleVisibility(widget, visible);
		return;
	}

	SidePanelTabToggleEntry entry;
	entry.title = title;
	entry.viewAction = new QAction(title, this);
	entry.viewAction->setCheckable(true);
	entry.viewAction->setChecked(visible);
	if (m_viewPanelToggleInsertBefore)
	{
		m_viewMenu->insertAction(m_viewPanelToggleInsertBefore, entry.viewAction);
	}
	else
	{
		m_viewMenu->addAction(entry.viewAction);
	}

	connect(entry.viewAction, &QAction::toggled, this,
			[this, widget](const bool checked) { applySidePanelTabToggleVisibility(widget, checked); });

	m_sidePanelTabToggles.insert(widget, entry);
	applySidePanelTabToggleVisibility(widget, visible);
}

void MainWindow::unregisterSidePanelTabToggle(QWidget* widget)
{
	if (!widget)
	{
		return;
	}

	const auto it = m_sidePanelTabToggles.find(widget);
	if (it == m_sidePanelTabToggles.end())
	{
		return;
	}

	if (m_rightPanelTabs)
	{
		const int tabIdx = m_rightPanelTabs->indexOf(widget);
		if (tabIdx >= 0)
		{
			if (m_rightPanelTabs->currentWidget() == widget)
			{
				const int workspaceIdx = workspaceTabIndex(m_rightPanelTabs);
				if (workspaceIdx >= 0)
				{
					m_rightPanelTabs->setCurrentIndex(workspaceIdx);
				}
			}
			m_rightPanelTabs->removeTab(tabIdx);
		}
	}

	if (it.value().viewAction)
	{
		m_viewMenu->removeAction(it.value().viewAction);
		delete it.value().viewAction;
	}
	m_sidePanelTabToggles.erase(it);
}

void MainWindow::applySidePanelTabToggleVisibility(QWidget* widget, const bool visible)
{
	if (!m_rightPanelTabs || !widget)
	{
		return;
	}

	const auto it = m_sidePanelTabToggles.constFind(widget);
	const QString title = it != m_sidePanelTabToggles.cend() ? it.value().title : QString();

	int tabIdx = m_rightPanelTabs->indexOf(widget);
	if (visible)
	{
		if (tabIdx < 0)
		{
			tabIdx = m_rightPanelTabs->addTab(widget, title);
		}
		else if (!title.isEmpty())
		{
			m_rightPanelTabs->setTabText(tabIdx, title);
		}
		m_rightPanelTabs->setCurrentIndex(tabIdx);
		return;
	}

	if (tabIdx < 0)
	{
		return;
	}
	if (m_rightPanelTabs->currentWidget() == widget)
	{
		const int workspaceIdx = workspaceTabIndex(m_rightPanelTabs);
		if (workspaceIdx >= 0)
		{
			m_rightPanelTabs->setCurrentIndex(workspaceIdx);
		}
	}
	m_rightPanelTabs->removeTab(tabIdx);
}

void MainWindow::setPluginSidePanelTabTitle(QWidget* widget, const QString& title)
{
	if (!widget || title.isEmpty())
	{
		return;
	}

	const auto it = m_sidePanelTabToggles.find(widget);
	if (it != m_sidePanelTabToggles.end())
	{
		it.value().title = title;
		if (it.value().viewAction)
		{
			it.value().viewAction->setText(title);
		}
	}

	if (m_rightPanelTabs)
	{
		const int tabIdx = m_rightPanelTabs->indexOf(widget);
		if (tabIdx >= 0)
		{
			m_rightPanelTabs->setTabText(tabIdx, title);
		}
	}
}

int MainWindow::addPluginSidePanelTab(const QString& title, QWidget* widget)
{
	if (!m_rightPanelTabs || !widget)
	{
		return -1;
	}

	if (widget->parentWidget() == static_cast<QWidget*>(m_rightPanelTabs))
	{
		widget->setParent(nullptr);
	}

	registerSidePanelTabToggle(widget, title, true);
	return m_rightPanelTabs->indexOf(widget);
}

void MainWindow::removePluginSidePanelTab(QWidget* widget)
{
	if (!widget)
	{
		return;
	}
	for (int i = m_processFlowDetachedRightTabs.size() - 1; i >= 0; --i)
	{
		if (m_processFlowDetachedRightTabs.at(i).widget.data() == widget)
		{
			m_processFlowDetachedRightTabs.removeAt(i);
		}
	}
	unregisterSidePanelTabToggle(widget);
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
	refreshAiAssistantHost();
}

void MainWindow::notifyPluginsLanguageChanged()
{
	if (m_pluginManager)
	{
		m_pluginManager->notifyLanguageChanged();
	}
}
