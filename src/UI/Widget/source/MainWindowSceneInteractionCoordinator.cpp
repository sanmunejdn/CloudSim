#include "MainWindow.h"
#include "MainWindowRobotHost.h"

#include "DocumentPage.h"

/// 文档页场景交互接线（OSG Qt 信号在 MainWindowRobotHost 内完成）
void MainWindow::wireDocumentPageSignals(DocumentPage* page)
{
	if (page && m_robotHost)
	{
		page->setInstructionPropertyDelegate(m_robotHost.get());
		m_robotHost->wireDocumentPageSceneSignals(page);
	}
	installBackendFollowFrameHook(page);
}
