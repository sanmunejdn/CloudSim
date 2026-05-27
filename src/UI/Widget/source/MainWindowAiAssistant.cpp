#include "MainWindow.h"

#include "AiAssistantCoordinator.h"
#include "AiAssistantDockWidget.h"
#include "IAiAssistantHost.h"
#include "PluginHostContext.h"
#include "PluginManager.h"
#include "RunInfoPage.h"

void MainWindow::finishAiAssistantReply(const QString& reply, bool isError, const QString& parserVia)
{
	QString out = reply;
	if (!parserVia.isEmpty())
		out = QStringLiteral("[%1] %2").arg(parserVia, reply);
	if (m_runInfoPage)
	{
		if (isError)
			m_runInfoPage->appendError(out);
		else
			m_runInfoPage->appendInfo(out);
	}
}

void MainWindow::onAiParseFailed(const QString& message, const QString& parserVia)
{
	if (m_runInfoPage)
	{
		const QString out = parserVia.isEmpty() ? message : QStringLiteral("[%1] %2").arg(parserVia, message);
		m_runInfoPage->appendError(out);
	}
}

void MainWindow::refreshAiAssistantHost()
{
	IAiAssistantHost* aiHost = nullptr;
	if (m_pluginManager && m_pluginManager->hostContext())
		aiHost = m_pluginManager->hostContext()->aiAssistantHost();

	if (m_aiCoordinator)
		m_aiCoordinator->setAiHost(aiHost);
	if (m_aiAssistantPage)
		m_aiAssistantPage->setAiHost(aiHost);
}

void MainWindow::setupAiAssistantCoordinator()
{
	if (!m_aiAssistantPage || m_aiCoordinator)
		return;

	m_aiCoordinator = new AiAssistantCoordinator(m_aiAssistantPage, this);

	connect(m_aiCoordinator, &AiAssistantCoordinator::assistantFinished, this,
		[this](const QString& reply, bool isError, const QString& parserVia) {
			finishAiAssistantReply(reply, isError, parserVia);
		});
	connect(m_aiCoordinator, &AiAssistantCoordinator::parseFailed, this, &MainWindow::onAiParseFailed);
	connect(m_aiAssistantPage, &AiAssistantDockWidget::messageSubmitted, m_aiCoordinator,
		&AiAssistantCoordinator::onUserMessageSubmitted);

	refreshAiAssistantHost();
}
