#include "MainWindow.h"
#include "MainWindowRobotHost.h"

#include "DocumentPage.h"
#include "OsgWidget.h"
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
	if (OsgWidget* ow = page->osgWidget())
	{
		if (auto* toolbar = ow->findChild<ViewportToolBar*>())
		{
			connect(toolbar, &ViewportToolBar::leftPanelVisibilityToggled, this, &MainWindow::setLeftSidePanelVisible);
			connect(toolbar, &ViewportToolBar::rightPanelVisibilityToggled, this, &MainWindow::setRightSidePanelVisible);
			toolbar->setUseChinese(m_useChinese);
			toolbar->setSidePanelToggleState(
				m_propertyDock && m_propertyDock->isVisible(),
				m_unitDock && m_unitDock->isVisible());
		}
	}
}
