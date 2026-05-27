#include "AiAssistantCoordinator.h"

#include "AiAssistantDockWidget.h"
#include "AiConfigDefaults.h"
#include "AiConfigDto.h"
#include "AiDomainTypes.h"
#include "AiInferenceTypes.h"
#include "IAiAssistantHost.h"

namespace
{
QString prefixWithParser(const QString& parserVia, const QString& text)
{
	if (parserVia.isEmpty())
		return text;
	return QStringLiteral("[%1] %2").arg(parserVia, text);
}
}

AiAssistantCoordinator::AiAssistantCoordinator(AiAssistantDockWidget* dock, QObject* parent)
	: QObject(parent)
	, m_dock(dock)
{
}

void AiAssistantCoordinator::setAiHost(IAiAssistantHost* host)
{
	m_aiHost = host;
}

void AiAssistantCoordinator::onUserMessageSubmitted(const QString& text)
{
	if (!m_dock)
		return;
	if (!m_aiHost)
	{
		const QString msg = QStringLiteral(
			"AI 宿主未就绪（插件宿主尚未初始化）。请重新编译并启动 CloudSim，或稍后重试。");
		m_dock->appendAssistantMessage(msg);
		emit parseFailed(msg, QString());
		return;
	}

	m_dock->setBusy(true);

	const std::optional<AiConfigDto> cfgOpt = m_aiHost->loadConfig();
	const AiConfigDto cfg = cfgOpt ? *cfgOpt : defaultAiConfigDto();

	AiInferenceRequest req;
	req.domainId = m_dock->selectedDomainId();
	req.userText = text;

	m_aiHost->parseUserTextAsync(
		req,
		cfg,
		[this](double fraction, const QString& message) {
			if (m_dock && !message.isEmpty())
				m_dock->appendSystemMessage(QStringLiteral("%1% — %2").arg(static_cast<int>(fraction * 100)).arg(message));
		},
		[this](AiParseResult result) {
			if (!m_dock)
				return;
			m_dock->setBusy(false);
			if (!result.ok)
			{
				QString msg = result.errorMessage;
				if (!result.hintMessage.isEmpty())
					msg += QStringLiteral("\n") + result.hintMessage;
				m_dock->appendAssistantMessage(prefixWithParser(result.parserVia, msg));
				emit parseFailed(msg, result.parserVia);
				return;
			}

			QString summary;
			QString err;
			bool executed = false;
			if (result.outputKind == AiDomainOutputKind::ActionPlan)
				executed = m_aiHost->executeActionPlan(result.outputJsonUtf8, &summary, &err);
			else
				executed = m_aiHost->executeDomainOutput(result.domainId, result.outputJsonUtf8, &summary, &err);

			if (!executed)
			{
				const QString msg = err.isEmpty() ? QStringLiteral("Failed to execute AI plan.") : err;
				m_dock->appendAssistantMessage(prefixWithParser(result.parserVia, msg));
				emit parseFailed(msg, result.parserVia);
				return;
			}

			const QString reply = summary.isEmpty() ? QStringLiteral("Done.") : summary;
			m_dock->appendAssistantMessage(prefixWithParser(result.parserVia, reply));
			emit assistantFinished(reply, false, result.parserVia);
		});
}
