/// @file MainWindowPlugins.cpp
/// @brief 插件扫描加载

#include "MainWindow.h"
#include "ApplicationSettings.h"
#include "PluginHostContext.h"
#include "PluginManager.h"

#include <QAction>
#include <QPointer>
#include <QSignalBlocker>
#include <QTabWidget>
#include <QTimer>

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
	entry.guard = widget;
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

	const QPointer<QWidget> guard = widget;
	connect(entry.viewAction, &QAction::toggled, this,
			[this, guard](const bool checked)
			{
				if (!guard)
				{
					return;
				}
				applySidePanelTabToggleVisibility(guard.data(), checked);
			});

	m_sidePanelTabToggles.insert(widget, entry);
	applySidePanelTabToggleVisibility(widget, visible);
}

void MainWindow::applySidePanelTabToggleVisibility(QWidget* widget, const bool visible)
{
	if (!m_rightPanelTabs || !widget)
	{
		return;
	}

	const auto it = m_sidePanelTabToggles.constFind(widget);
	if (it == m_sidePanelTabToggles.cend() || !it.value().guard)
	{
		return;
	}
	widget = it.value().guard.data();
	if (!widget)
	{
		return;
	}

	// 交替侧栏已剥离工作区/插件页签；再 addTab 会与 detach 打架
	if (m_processFlowSideUiActive &&
		static_cast<const void*>(widget) != static_cast<const void*>(m_aiAssistantPage))
	{
		return;
	}

	const QString title = it.value().title;
	int tabIdx = m_rightPanelTabs->indexOf(widget);
	if (visible)
	{
		if (tabIdx < 0)
		{
			if (widget->parentWidget() != nullptr && m_rightPanelTabs->isAncestorOf(widget))
			{
				widget->setParent(nullptr);
			}
			tabIdx = m_rightPanelTabs->addTab(widget, title);
		}
		else if (!title.isEmpty())
		{
			m_rightPanelTabs->setTabText(tabIdx, title);
		}
		if (tabIdx >= 0)
		{
			m_rightPanelTabs->setCurrentIndex(tabIdx);
		}
		if (!m_restoringUiPreferences)
		{
			persistUiPreferencesToStorage();
		}
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
	if (!m_restoringUiPreferences)
	{
		persistUiPreferencesToStorage();
	}
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
		QWidget* live = it.value().guard.data();
		if (live)
		{
			const int tabIdx = m_rightPanelTabs->indexOf(live);
			if (tabIdx >= 0)
			{
				if (m_rightPanelTabs->currentWidget() == live)
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
	}

	if (it.value().viewAction)
	{
		m_viewMenu->removeAction(it.value().viewAction);
		delete it.value().viewAction;
	}
	m_sidePanelTabToggles.erase(it);
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

	if (!m_rightPanelTabs)
	{
		return;
	}
	const int idx = m_rightPanelTabs->indexOf(widget);
	if (idx >= 0)
	{
		m_rightPanelTabs->setTabText(idx, title);
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

	const QString tabKey = ApplicationSettings::sidePanelTabKey(widget);
	const bool visible =
		m_uiPreferences.sidePanelTabs.contains(tabKey) ? m_uiPreferences.sidePanelTabs.value(tabKey) : true;
	registerSidePanelTabToggle(widget, title, visible);
	if (!m_sidePanelTabToggles.contains(widget) || !m_sidePanelTabToggles.value(widget).guard)
	{
		return -1;
	}
	// 按偏好隐藏时 indexOf==-1，仍算注册成功（否则插件会 delete，留下悬空登记）
	const int idx = m_rightPanelTabs->indexOf(widget);
	return idx >= 0 ? idx : 0;
}

void MainWindow::restoreUiPreferencesAfterPlugins()
{
	m_restoringUiPreferences = true;
	// 先恢复侧栏可见性，再进工作区模式（模式会 detach 页签；若先模式后 layout 会把页签加回并崩）
	QTimer::singleShot(0, this, [this]() {
		applySavedViewLayout();
		if (!m_uiPreferences.workspaceModeId.isEmpty() && m_pluginManager && m_pluginManager->hostContext())
		{
			m_pluginManager->hostContext()->enterWorkspaceMode(m_uiPreferences.workspaceModeId);
		}
		QTimer::singleShot(100, this, [this]() {
			if (!m_processFlowSideUiActive)
			{
				applySavedViewLayout();
			}
			m_restoringUiPreferences = false;
		});
	});
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
	m_restoringUiPreferences = true;

	if (!m_pluginManager)
	{
		m_pluginManager = new PluginManager(this, this);
	}

	m_pluginManager->loadAllFromPluginsDirectory();
	refreshAiAssistantHost();
	if (PluginHostContext* ctx = m_pluginManager->hostContext())
	{
		ctx->onWorkspaceModeClaimed(
			[this](const QString& modeId)
			{
				if (!m_workspaceModeMenu)
					return;
				for (QAction* a : m_workspaceModeMenu->actions())
					a->setChecked(a->data().toString() == modeId);
			});
	}
	rebuildWorkspaceModeSwitcher();
	restoreUiPreferencesAfterPlugins();
}

void MainWindow::onWorkspaceModeRequested(const QString& modeId)
{
	if (!m_pluginManager || !m_pluginManager->hostContext())
		return;
	m_pluginManager->hostContext()->enterWorkspaceMode(modeId);
	if (!m_restoringUiPreferences)
	{
		persistUiPreferencesToStorage();
	}
}

void MainWindow::notifyPluginsLanguageChanged()
{
	if (m_pluginManager)
	{
		m_pluginManager->notifyLanguageChanged();
	}
}
