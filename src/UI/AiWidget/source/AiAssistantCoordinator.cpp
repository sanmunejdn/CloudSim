#include "AiAssistantCoordinator.h"

#include "AiAssistantDockWidget.h"
#include "AiConfigDefaults.h"
#include "AiConfigDto.h"
#include "AiDomainTypes.h"
#include "AiInferenceTypes.h"
#include "IAiAssistantHost.h"
#include "IPluginHostContext.h"

namespace
{
QString prefixWithParser(const QString& parserVia, const QString& text)
{
	if (parserVia.isEmpty())
		return text;
	return QStringLiteral("[%1] %2").arg(parserVia, text);
}

const AiDomainModelConfig* findDomainConfig(const AiConfigDto& cfg, const QString& domainId)
{
	for (const AiDomainModelConfig& d : cfg.domains)
	{
		if (d.id == domainId)
			return &d;
	}
	return nullptr;
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

void AiAssistantCoordinator::setPluginHost(IPluginHostContext* host)
{
	m_pluginHost = host;
}

bool AiAssistantCoordinator::needsViewportCapture(const QString& domainId, const QString& userText,
	const AiConfigDto& cfg) const
{
	const QString resolved =
		m_aiHost ? m_aiHost->resolveDomainId(domainId, userText) : domainId.trimmed();
	if (resolved == AiDomainIds::geometryRecognize())
		return true;
	if (const AiDomainModelConfig* dm = findDomainConfig(cfg, resolved))
		return dm->multimodal;
	return false;
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
	m_dock->hideCreateFromRecognitionButton();
	m_pendingRecognitionJson.clear();
	m_pendingRecognitionParserVia.clear();

	const std::optional<AiConfigDto> cfgOpt = m_aiHost->loadConfig();
	const AiConfigDto cfg = cfgOpt ? *cfgOpt : defaultAiConfigDto();

	AiInferenceRequest req;
	req.domainId = m_dock->selectedDomainId();
	req.userText = text;

	if (needsViewportCapture(req.domainId, text, cfg))
	{
		if (!m_pluginHost)
		{
			m_dock->setBusy(false);
			const QString msg = QStringLiteral("宿主上下文未就绪，无法截取视口。");
			m_dock->appendAssistantMessage(msg);
			emit parseFailed(msg, QString());
			return;
		}
		QString capErr;
		if (!m_pluginHost->captureActiveViewportPng(req.imagePng, &capErr))
		{
			m_dock->setBusy(false);
			const QString msg = capErr.isEmpty()
				? QStringLiteral("无法截取当前 3D 视口，请先打开含视口的文档。")
				: capErr;
			m_dock->appendAssistantMessage(msg);
			emit parseFailed(msg, QString());
			return;
		}
	}

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

			if (result.domainId == AiDomainIds::geometryRecognize())
			{
				m_pendingRecognitionJson = result.outputJsonUtf8;
				m_pendingRecognitionParserVia = result.parserVia;
				m_dock->showRecognitionResult(result.outputJsonUtf8, result.parserVia);
				emit assistantFinished(QStringLiteral("Recognition complete."), false, result.parserVia);
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

			QString reply = summary.isEmpty() ? QStringLiteral("Done.") : summary;
			if (!result.hintMessage.isEmpty())
				reply += QStringLiteral("\n") + result.hintMessage;
			m_dock->appendAssistantMessage(prefixWithParser(result.parserVia, reply));
			emit assistantFinished(reply, false, result.parserVia);
		});
}

void AiAssistantCoordinator::onCreateRecognitionConfirmed()
{
	if (!m_dock || !m_aiHost || m_pendingRecognitionJson.isEmpty())
		return;

	m_dock->setBusy(true);
	QString summary;
	QString err;
	const bool executed =
		m_aiHost->executeDomainOutput(AiDomainIds::geometryRecognize(), m_pendingRecognitionJson, &summary, &err);
	m_dock->setBusy(false);

	if (!executed)
	{
		const QString msg = err.isEmpty() ? QStringLiteral("创建基本体失败。") : err;
		m_dock->appendAssistantMessage(prefixWithParser(m_pendingRecognitionParserVia, msg));
		emit parseFailed(msg, m_pendingRecognitionParserVia);
		return;
	}

	m_dock->hideCreateFromRecognitionButton();
	m_pendingRecognitionJson.clear();
	const QString reply = summary.isEmpty() ? QStringLiteral("基本体已创建。") : summary;
	m_dock->appendAssistantMessage(prefixWithParser(m_pendingRecognitionParserVia, reply));
	emit assistantFinished(reply, false, m_pendingRecognitionParserVia);
}
