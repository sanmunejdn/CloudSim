/// @file HeadlessAiBridge.cpp

#include "headless/HeadlessAiBridge.h"

#include "Ai/AiConfigLoader.h"
#include "Ai/AiLlmClient.h"
#include "Ai/AiLlmConfig.h"
#include "AiConfigDto.h"
#include "DocumentHost.h"

namespace cloudsim::host
{
namespace
{
QJsonObject fail(const QString& err)
{
	return QJsonObject{{QStringLiteral("ok"), false}, {QStringLiteral("error"), err}};
}
} // namespace

HeadlessAiBridge::HeadlessAiBridge(DocumentHost& host) : m_host(host) {}

QJsonObject HeadlessAiBridge::chat(const QJsonObject& body)
{
	(void)m_host;

	const QString userText = body.value(QStringLiteral("message")).toString();
	if (userText.isEmpty())
		return fail(QStringLiteral("message required."));

	std::optional<AiLlmConfig> cfgOpt = loadAiLlmConfig();
	if (!cfgOpt)
	{
		const std::optional<AiConfigDto> dtoOpt = loadAiConfigDto();
		if (!dtoOpt || !dtoOpt->remoteLlm.enabled)
			return fail(QStringLiteral("AI not configured (missing ai_config.json)."));
		AiLlmConfig mapped;
		mapped.enabled = dtoOpt->remoteLlm.enabled;
		mapped.baseUrl = dtoOpt->remoteLlm.baseUrl;
		mapped.apiKey = dtoOpt->remoteLlm.apiKey;
		mapped.apiKeyEnv = dtoOpt->remoteLlm.apiKeyEnv;
		mapped.model = dtoOpt->remoteLlm.model;
		mapped.timeoutMs = dtoOpt->remoteLlm.timeoutMs;
		mapped.temperature = dtoOpt->remoteLlm.temperature;
		cfgOpt = mapped;
	}

	const AiLlmConfig& cfg = *cfgOpt;
	if (!cfg.enabled)
		return fail(QStringLiteral("AI disabled in config."));
	if (!cfg.hasApiKey())
		return fail(QStringLiteral("AI API key not configured."));

	const QString systemPrompt =
		body.value(QStringLiteral("systemPrompt"))
			.toString(QStringLiteral("You are CloudSim assistant. Reply concisely in the user's language."));
	const AiLlmClient::TextChatResult result =
		AiLlmClient::chatText(systemPrompt, userText, cfg);

	if (!result.ok)
		return fail(result.errorMessage);

	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("assistantText"), result.assistantText);
	return o;
}

} // namespace cloudsim::host
