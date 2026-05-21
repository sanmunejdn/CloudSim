#include "AiLlmClient.h"

#include "AiCommandSchema.h"
#include "AiHttpsPost.h"

#include <json.hpp>

#include <QUrl>

namespace AiLlmClient
{
namespace
{
QString resolveApiKey(const AiLlmConfig& config)
{
	if (!config.apiKey.trimmed().isEmpty())
		return config.apiKey.trimmed();
	if (!config.apiKeyEnv.trimmed().isEmpty())
		return QString::fromUtf8(qgetenv(config.apiKeyEnv.toUtf8().constData())).trimmed();
	return {};
}

QString chatCompletionsUrl(const QString& baseUrl)
{
	QString u = baseUrl.trimmed();
	while (u.endsWith(QLatin1Char('/')))
		u.chop(1);
	if (u.endsWith(QStringLiteral("/chat/completions"), Qt::CaseInsensitive))
		return u;
	return u + QStringLiteral("/chat/completions");
}

QString systemPrompt()
{
	return QStringLiteral(
		"You convert user requests into a single JSON object for a CAD mesh command. "
		"Reply with ONLY valid JSON (no markdown), schema:\n"
		"{\"version\":1,\"action\":\"create_mesh\",\"primitive\":\"box|cylinder|cone|sphere\","
		"\"dimensions_mm\":{...},\"name\":\"optional string\","
		"\"pose_mm\":{\"x\":0,\"y\":0,\"z\":0},\"rotation_deg\":{\"x\":0,\"y\":0,\"z\":0},"
		"\"mesh_quality\":{\"segments\":32,\"rings\":16}}\n"
		"Units are millimeters. box: length,width,height. cylinder: radius,height. "
		"cone: radius,height; optional radius_top. sphere: radius or diameter. "
		"Center at origin, Z is height axis for box/cylinder/cone.");
}
}

LlmParseResult parseUserTextWithLlm(const QString& userText, const AiLlmConfig& config, const AiProgressSink& progress)
{
	LlmParseResult out;
	const QString apiKey = resolveApiKey(config);
	if (apiKey.isEmpty())
	{
		out.errorMessage = QStringLiteral("LLM api_key missing in ai_config.json and environment.");
		return out;
	}

	progress(0.1, QStringLiteral("LLM request..."));

	nlohmann::json body;
	body["model"] = config.model.toStdString();
	body["temperature"] = config.temperature;
	body["messages"] = nlohmann::json::array({
		{ { "role", "system" }, { "content", systemPrompt().toStdString() } },
		{ { "role", "user" }, { "content", userText.trimmed().toStdString() } },
	});

	const QUrl url(chatCompletionsUrl(config.baseUrl));
	const QByteArray payload = QByteArray::fromStdString(body.dump());
	QList<QPair<QByteArray, QByteArray>> headers;
	headers.append(qMakePair(QByteArray("Authorization"), QByteArray("Bearer ") + apiKey.toUtf8()));

	QByteArray raw;
	QString httpErr;
	if (!AiHttpsPost::post(url, payload, headers, raw, httpErr, config.timeoutMs))
	{
		out.errorMessage = httpErr;
		return out;
	}

	progress(0.85, QStringLiteral("Parsing response..."));

	nlohmann::json resp;
	try
	{
		resp = nlohmann::json::parse(raw.constData(), nullptr, true);
	}
	catch (...)
	{
		out.errorMessage = QStringLiteral("Invalid JSON from LLM API.");
		return out;
	}

	if (resp.contains("error") && resp["error"].is_object())
	{
		const auto& err = resp["error"];
		out.errorMessage = QString::fromStdString(err.value("message", std::string("LLM API error")));
		return out;
	}

	std::string content;
	if (resp.contains("choices") && resp["choices"].is_array() && !resp["choices"].empty())
	{
		const auto& ch0 = resp["choices"][0];
		if (ch0.contains("message") && ch0["message"].is_object())
			content = ch0["message"].value("content", std::string());
	}
	if (content.empty())
	{
		out.errorMessage = QStringLiteral("LLM response has no message content.");
		return out;
	}

	std::string parseErr;
	if (!AiCommandSchema::tryParseCreateMeshCommandJson(content, out.command, parseErr))
	{
		out.errorMessage = QString::fromStdString(parseErr);
		return out;
	}

	out.ok = true;
	progress(1.0, QString());
	return out;
}

} // namespace AiLlmClient
