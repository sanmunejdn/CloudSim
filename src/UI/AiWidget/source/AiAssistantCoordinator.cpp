#include "AiAssistantCoordinator.h"

#include "AiAssistantDockWidget.h"
#include "AiIntentParser.h"
#include "AiLlmClient.h"
#include "AiLlmConfig.h"
#include "AiProgressSink.h"

#include <optional>

#include <QPointer>

#include <memory>

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

void AiAssistantCoordinator::setBackgroundEnqueue(EnqueueBackgroundWork enqueue)
{
	m_enqueue = std::move(enqueue);
}

void AiAssistantCoordinator::onUserMessageSubmitted(const QString& text)
{
	if (!m_dock)
		return;

	m_dock->setBusy(true);

	const AiIntentParser::ParseResult ruleParsed = AiIntentParser::tryParseUserText(text);
	const std::optional<AiLlmConfig> llmCfg = loadAiLlmConfig();

	if (ruleParsed.ok && (!llmCfg || llmCfg->ruleParserFirst))
	{
		m_dock->appendSystemMessage(QStringLiteral("Parser: offline rules"));
		emit createMeshCommandReady(
			QByteArray::fromStdString(ruleParsed.command.dump()), QStringLiteral("Rules"));
		return;
	}

	if (!llmCfg || !llmCfg->enabled)
	{
		QString msg = ruleParsed.errorMessage;
		if (!ruleParsed.hintMessage.isEmpty())
			msg += QStringLiteral("\n") + ruleParsed.hintMessage;
		if (msg.isEmpty())
			msg = QStringLiteral("Rule parser failed. Enable LLM in Settings.");
		else
			msg += QStringLiteral("\n\nTip: open Settings, enable LLM and set API key.");
		m_dock->setBusy(false);
		m_dock->appendAssistantMessage(prefixWithParser(QStringLiteral("Rules"), msg));
		emit parseFailed(msg, QStringLiteral("Rules"));
		return;
	}

	if (!llmCfg->hasApiKey())
	{
		const QString msg = QStringLiteral("Set API key in Settings or %1 environment variable.")
			.arg(llmCfg->apiKeyEnv);
		m_dock->setBusy(false);
		m_dock->appendAssistantMessage(prefixWithParser(QStringLiteral("LLM"), msg));
		emit parseFailed(msg, QStringLiteral("LLM"));
		return;
	}

	if (!m_enqueue)
	{
		const QString msg = QStringLiteral("Background job queue not configured.");
		m_dock->setBusy(false);
		m_dock->appendAssistantMessage(prefixWithParser(QStringLiteral("LLM"), msg));
		emit parseFailed(msg, QStringLiteral("LLM"));
		return;
	}

	const QString llmVia = QStringLiteral("LLM %1").arg(llmCfg->model);
	m_dock->appendSystemMessage(QStringLiteral("Parser: %1 (requesting...)").arg(llmVia));

	struct LlmJobPayload
	{
		AiLlmClient::LlmParseResult result;
	};
	const auto payload = std::make_shared<LlmJobPayload>();
	const QPointer<AiAssistantCoordinator> self(this);
	const QPointer<AiAssistantDockWidget> dockPtr(m_dock);
	const QString userText = text;
	const AiLlmConfig config = *llmCfg;
	const QString parserLabel = llmVia;

	m_enqueue(
		QStringLiteral("AI: parse command"),
		[payload, userText, config](const std::function<void(double, const QString&)>& sink) {
			const AiProgressSink progress = [&sink](double fraction, const QString& message) {
				if (sink)
					sink(fraction, message);
			};
			payload->result = AiLlmClient::parseUserTextWithLlm(userText, config, progress);
		},
		[payload, self, dockPtr, parserLabel](bool threw, const QString& throwMsg) {
			if (!self || !dockPtr)
				return;
			dockPtr->setBusy(false);
			if (threw)
			{
				const QString msg = throwMsg.isEmpty() ? QStringLiteral("LLM job failed.") : throwMsg;
				dockPtr->appendAssistantMessage(prefixWithParser(parserLabel, msg));
				emit self->parseFailed(msg, parserLabel);
				return;
			}
			if (!payload->result.ok)
			{
				dockPtr->appendAssistantMessage(prefixWithParser(parserLabel, payload->result.errorMessage));
				emit self->parseFailed(payload->result.errorMessage, parserLabel);
				return;
			}
			emit self->createMeshCommandReady(
				QByteArray::fromStdString(payload->result.command.dump()), parserLabel);
		});
}
