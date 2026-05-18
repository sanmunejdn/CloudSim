#include "MainWindow.h"

#include "AiCommandExecutor.h"
#include "AiAssistantCoordinator.h"
#include "AiAssistantDockWidget.h"
#include "JobSystem.h"
#include "RunInfoPage.h"

void MainWindow::finishAiAssistantReply(const QString& reply, bool isError, const QString& parserVia)
{
	QString out = reply;
	if (!parserVia.isEmpty())
		out = QStringLiteral("[%1] %2").arg(parserVia, reply);
	if (m_aiAssistantPage)
	{
		m_aiAssistantPage->setBusy(false);
		m_aiAssistantPage->appendAssistantMessage(out);
	}
	if (m_runInfoPage)
	{
		if (isError)
			m_runInfoPage->appendError(out);
		else
			m_runInfoPage->appendInfo(out);
	}
}

void MainWindow::executeAiCreateMeshCommand(const nlohmann::json& cmd, const QString& parserVia)
{
	QString reply;
	QString err;
	if (AiCreateMeshRunner::executeFromJson(*this, cmd, reply, err))
		finishAiAssistantReply(reply, false, parserVia);
	else
		finishAiAssistantReply(err, true, parserVia);
}

void MainWindow::onAiCreateMeshCommandReady(const QByteArray& commandJsonUtf8, const QString& parserVia)
{
	nlohmann::json cmd;
	try
	{
		cmd = nlohmann::json::parse(commandJsonUtf8.constData(), nullptr, true);
	}
	catch (...)
	{
		finishAiAssistantReply(QStringLiteral("Invalid command JSON from AI parser."), true, parserVia);
		return;
	}
	executeAiCreateMeshCommand(cmd, parserVia);
}

void MainWindow::onAiParseFailed(const QString& message, const QString& parserVia)
{
	if (m_runInfoPage)
	{
		const QString out = parserVia.isEmpty() ? message : QStringLiteral("[%1] %2").arg(parserVia, message);
		m_runInfoPage->appendError(out);
	}
}

void MainWindow::setupAiAssistantCoordinator()
{
	if (!m_aiAssistantPage || m_aiCoordinator)
		return;

	m_aiCoordinator = new AiAssistantCoordinator(m_aiAssistantPage, this);
	m_aiCoordinator->setBackgroundEnqueue(
		[this](const QString& title,
			std::function<void(const std::function<void(double, const QString&)>&)> work,
			std::function<void(bool threw, const QString& throwMessage)> onFinished) {
			if (!jobSystem())
			{
				onFinished(true, QStringLiteral("JobSystem not available."));
				return;
			}
			jobSystem()->enqueue(
				title,
				[work](const JobProgressSink& sink) {
					if (work)
					{
						work([&sink](double fraction, const QString& message) {
							if (sink)
								sink(fraction, message);
						});
					}
				},
				std::move(onFinished));
		});

	connect(m_aiCoordinator, &AiAssistantCoordinator::createMeshCommandReady, this,
		&MainWindow::onAiCreateMeshCommandReady);
	connect(m_aiCoordinator, &AiAssistantCoordinator::parseFailed, this, &MainWindow::onAiParseFailed);
	connect(m_aiAssistantPage, &AiAssistantDockWidget::messageSubmitted, m_aiCoordinator,
		&AiAssistantCoordinator::onUserMessageSubmitted);
}
