#include "AiLlmClient.h"

#include "Ai/AiMeshDefaults.h"
#include "AiCommandSchema.h"
#include "AiHttpsPost.h"

#include <json.hpp>

#include <QByteArray>
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

QString meshSystemPrompt()
{
	const auto d = AiMeshDefaults::activeDefaults();
	return QStringLiteral(
		"You convert user requests into a single JSON object for a CAD mesh command. "
		"Reply with ONLY valid JSON (no markdown), schema:\n"
		"{\"version\":1,\"action\":\"create_mesh\",\"primitive\":\"box|cylinder|cone|sphere\","
		"\"dimensions_mm\":{...},\"name\":\"optional string\","
		"\"pose_mm\":{\"x\":0,\"y\":0,\"z\":0},\"rotation_deg\":{\"x\":0,\"y\":0,\"z\":0},"
		"\"mesh_quality\":{\"segments\":32,\"rings\":16}}\n"
		"Units are millimeters. Always include dimensions_mm with all required fields.\n"
		"If the user omits sizes, use defaults: box length=%1 width=%2 height=%3; "
		"cylinder radius=%4 height=%5; cone radius=%6 height=%7; sphere radius=%8.\n"
		"If the user gives partial sizes, fill missing fields with those defaults.\n"
		"If the user says larger/smaller/thicker/flatter (大一点/小一点/扁/厚), scale defaults (~1.5x or ~0.5x) and still output full dimensions_mm.\n"
		"box: length,width,height. cylinder: radius,height. cone: radius,height. sphere: radius or diameter. "
		"Center at origin, Z is height axis for box/cylinder/cone.")
		.arg(d.boxLengthMm)
		.arg(d.boxWidthMm)
		.arg(d.boxHeightMm)
		.arg(d.cylinderRadiusMm)
		.arg(d.cylinderHeightMm)
		.arg(d.coneRadiusMm)
		.arg(d.coneHeightMm)
		.arg(d.sphereRadiusMm);
}

QString recognitionSystemPrompt()
{
	return QStringLiteral(
		"You analyze a CAD viewport screenshot and optional user text. "
		"Reply with ONLY valid JSON (no markdown):\n"
		"{\"primitive\":\"box|cylinder|cone|sphere|unknown\",\"label\":\"short description\","
		"\"dimensions_mm\":{},\"confidence\":0.0}");
}
}

LlmParseResult parseUserTextWithLlm(const QString& userText, const AiLlmConfig& config, const AiProgressSink& progress,
	const QByteArray& imagePng, bool recognitionSchema)
{
	LlmParseResult out;
	const QString apiKey = resolveApiKey(config);
	const bool localOllama = config.baseUrl.contains(QStringLiteral("127.0.0.1"), Qt::CaseInsensitive)
		|| config.baseUrl.contains(QStringLiteral("localhost"), Qt::CaseInsensitive);
	const QString authKey = apiKey.isEmpty() && localOllama ? QStringLiteral("ollama") : apiKey;
	if (authKey.isEmpty())
	{
		out.errorMessage = QStringLiteral("LLM api_key missing in ai_config.json and environment.");
		return out;
	}

	if (progress)
		progress(0.1, QStringLiteral("LLM request..."));

	const QString sys = recognitionSchema ? recognitionSystemPrompt() : meshSystemPrompt();
	nlohmann::json userMessage;
	if (!imagePng.isEmpty())
	{
		const QString b64 = QString::fromLatin1(imagePng.toBase64());
		const std::string dataUrl = "data:image/png;base64," + b64.toStdString();
		userMessage["role"] = "user";
		userMessage["content"] = nlohmann::json::array({
			{ { "type", "text" }, { "text", userText.trimmed().toStdString() } },
			{ { "type", "image_url" }, { "image_url", { { "url", dataUrl } } } },
		});
	}
	else
	{
		userMessage = { { "role", "user" }, { "content", userText.trimmed().toStdString() } };
	}

	nlohmann::json body;
	body["model"] = config.model.toStdString();
	body["temperature"] = config.temperature;
	body["messages"] = nlohmann::json::array({
		{ { "role", "system" }, { "content", sys.toStdString() } },
		userMessage,
	});

	const QUrl url(chatCompletionsUrl(config.baseUrl));
	const QByteArray payload = QByteArray::fromStdString(body.dump());
	QList<QPair<QByteArray, QByteArray>> headers;
	headers.append(qMakePair(QByteArray("Authorization"), QByteArray("Bearer ") + authKey.toUtf8()));

	QByteArray raw;
	QString httpErr;
	if (!AiHttpsPost::post(url, payload, headers, raw, httpErr, config.timeoutMs))
	{
		out.errorMessage = httpErr;
		return out;
	}

	if (progress)
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

	if (recognitionSchema)
	{
		try
		{
			out.command = nlohmann::json::parse(content, nullptr, true);
		}
		catch (...)
		{
			out.errorMessage = QStringLiteral("Recognition JSON parse failed.");
			return out;
		}
		if (!out.command.is_object())
		{
			out.errorMessage = QStringLiteral("Recognition response must be a JSON object.");
			return out;
		}
	}
	else
	{
		std::string parseErr;
		if (!AiCommandSchema::tryParseCreateMeshCommandJson(content, out.command, parseErr))
		{
			out.errorMessage = QString::fromStdString(parseErr);
			return out;
		}
		AiMeshDefaults::applyMissingDimensions(out.command);
	}

	out.ok = true;
	if (progress)
		progress(1.0, QString());
	return out;
}

} // namespace AiLlmClient
