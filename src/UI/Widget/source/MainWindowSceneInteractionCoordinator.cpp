/// @file MainWindowSceneInteractionCoordinator.cpp
/// @brief 文档页场景交互接线（OSG Qt 信号在 MainWindowRobotHost 内完成）

#include "DocumentPage.h"
#include "IRenderView.h"
#include "MainWindow.h"
#include "MainWindowRobotHost.h"
#include "ViewportToolBar.h"

/// 文档页场景交互接线（OSG Qt 信号在 MainWindowRobotHost 内完成）
void MainWindow::wireDocumentPageSignals(DocumentPage* page)
{
	if (page && m_robotHost)
	{
		page->setInstructionPropertyDelegate(m_robotHost.get());
		m_robotHost->wireDocumentPageSceneSignals(page);
	}
	installBackendFollowFrameHook(page);

	if (!page)
	{
		return;
	}
	// 经 IRenderView::widget()，避免本文件直连 OsgWidget
	if (QWidget* view = page->render().widget())
	{
		if (auto* toolbar = view->findChild<ViewportToolBar*>())
		{
			connect(toolbar, &ViewportToolBar::leftPanelVisibilityToggled, this, &MainWindow::setLeftSidePanelVisible,
					Qt::UniqueConnection);
			connect(toolbar, &ViewportToolBar::rightPanelVisibilityToggled, this, &MainWindow::setRightSidePanelVisible,
					Qt::UniqueConnection);
			connect(
				toolbar, &ViewportToolBar::objectSelectionToggled, this,
				[this](const bool on)
				{
					if (on)
					{
						onObjectModeTriggered();
					}
					else
					{
						onViewModeTriggered();
					}
				},
				Qt::UniqueConnection);
			connect(
				toolbar, &ViewportToolBar::multiViewLayoutRequested, this,
				[this](const int layout)
				{
					if (auto* host = currentDocumentHost())
					{
						host->setMultiViewLayout(static_cast<cloudsim::host::MultiViewLayout>(layout));
					}
					if (m_multiViewSingleAction && layout == 0)
						m_multiViewSingleAction->setChecked(true);
					else if (m_multiViewSplitHAction && layout == 1)
						m_multiViewSplitHAction->setChecked(true);
					else if (m_multiViewSplitVAction && layout == 2)
						m_multiViewSplitVAction->setChecked(true);
					else if (m_multiViewQuadAction && layout == 3)
						m_multiViewQuadAction->setChecked(true);
				},
				Qt::UniqueConnection);
			connect(
				toolbar, &ViewportToolBar::openDetachedViewRequested, this,
				[this]()
				{
					if (auto* host = currentDocumentHost())
					{
						host->openDetachedViewWindow();
					}
				},
				Qt::UniqueConnection);
			toolbar->setUseChinese(m_useChinese);
			// Dock 可能尚未创建（首文档早于 setupDockWidgets）；勿用空指针写成「已隐藏」
			if (m_propertyDock || m_unitDock)
			{
				toolbar->setSidePanelToggleState(m_propertyDock && !m_propertyDock->isHidden(),
												 m_unitDock && !m_unitDock->isHidden());
			}
			if (m_objectModeAction)
			{
				toolbar->setObjectSelectionChecked(m_objectModeAction->isChecked());
			}
		}
	}
}
