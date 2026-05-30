#include "AiLlmClient.h"

#include "Ai/AiMeshDefaults.h"
#include "Ai/MeshComposeDomainHandler.h"
#include "AiCommandSchema.h"
#include "AiDomainTypes.h"
#include "AiHttpsPost.h"

#include <json.hpp>

#include <QByteArray>
#include <QBuffer>
#include <QImage>
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

QString ollamaNativeChatUrl(const QString& baseUrl)
{
	QString u = baseUrl.trimmed();
	while (u.endsWith(QLatin1Char('/')))
		u.chop(1);
	if (u.endsWith(QStringLiteral("/v1"), Qt::CaseInsensitive))
		u.chop(2);
	return u + QStringLiteral("/api/chat");
}

bool isLocalOllamaBaseUrl(const QString& baseUrl)
{
	return baseUrl.contains(QStringLiteral("127.0.0.1"), Qt::CaseInsensitive)
		|| baseUrl.contains(QStringLiteral("localhost"), Qt::CaseInsensitive);
}

QByteArray visionImageBase64Raw(const QByteArray& imagePng)
{
	QImage img = QImage::fromData(imagePng);
	if (img.isNull())
		return imagePng.toBase64();
	QImage rgb = img.convertToFormat(QImage::Format_RGB888);
	QByteArray jpeg;
	QBuffer buf(&jpeg);
	if (!buf.open(QIODevice::WriteOnly) || !rgb.save(&buf, "JPEG", 90))
		return imagePng.toBase64();
	return jpeg.toBase64();
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
		"你是 CAD 视口几何识别助手。用户会提供一张 3D 视口截图（通常白底、单个主要基本体）。"
		"识别主要物体的类型并估计尺寸（毫米）。"
		"仅回复一个 JSON 对象，不要 markdown、不要解释：\n"
		"{\"primitive\":\"box|cylinder|cone|sphere|unknown\",\"label\":\"中文短名\","
		"\"dimensions_mm\":{...},\"confidence\":0.0}\n"
		"box 需 length,width,height；cylinder/cone 需 radius,height；sphere 需 radius。"
		"视口里明显是长方体/立方体时 primitive 必须是 box，不要返回 unknown。"
		"仅当完全无法判断时才用 unknown，confidence 取 0~1。");
}

QString recognitionUserPrompt(const QString& userText)
{
	const QString t = userText.trimmed();
	if (t.isEmpty())
		return QStringLiteral("识别截图中的主要基本体类型，并估计 dimensions_mm（毫米）。");
	return t + QStringLiteral("\n请根据截图识别主要基本体，输出 JSON。");
}

QString composeSystemPrompt()
{
	const auto d = AiMeshDefaults::activeDefaults();
	return QStringLiteral(
		"You convert CAD requests into ActionPlan JSON version 2 ONLY (no markdown). "
		"Schema: {\"version\":2,\"domain\":\"mesh.compose\",\"steps\":[{\"id\":\"body\",\"api\":\"createPrimitiveMesh\",\"args\":{...}},...]}. "
		"Use fields id and api (NOT name/type). Args hold primitive, dimensions_mm, etc. "
		"booleanMesh target/tool MUST be $stepId refs. All steps in steps[] only (no result field). "
		"Host runs intermediate primitives in memory; only the final boolean result appears in the scene.\n"
		"APIs:\n"
		"1) createPrimitiveMesh args: primitive box|cylinder|cone|sphere, dimensions_mm (mm, all required), "
		"optional name, pose_mm, rotation_deg. Z is height axis, center at origin.\n"
		"2) booleanMesh args: op difference|union|intersection, target $stepId, tool $stepId, "
		"optional result_name, hide_operands (default true).\n"
		"Op mapping: difference=挖/孔/通孔/盲孔/减去/subtract/drill/hole; "
		"union=并集/合并/拼合/合在一起/combine/merge/附加凸台; "
		"intersection=交集/求交/重叠部分/intersect/common volume.\n"
		"Rules difference: Through-hole = box + cylinder (radius=diameter/2, height > box height) + difference. "
		"Use ids body, hole_tool, result. NEVER use difference when user says union/merge.\n"
		"Rules union: two solids merged; use box+box or box+cylinder boss; add pose_mm when offset needed. "
		"Example: box 80^3 + box 60^3 pose_mm x=40 + union($a,$b).\n"
		"Rules intersection: overlapping volume only; offset second body with pose_mm for partial overlap. "
		"Example: box 100^3 + box 80^3 pose_mm x=50 + intersection($a,$b).\n"
		"Defaults if sizes omitted: box %1x%2x%3, cylinder R%4 H%5.\n"
		"Example difference: 100 cube D50 hole: body box, hole_tool cyl R25 H120, result difference $body $hole_tool.")
		.arg(d.boxLengthMm)
		.arg(d.boxWidthMm)
		.arg(d.boxHeightMm)
		.arg(d.cylinderRadiusMm)
		.arg(d.cylinderHeightMm);
}
}

LlmParseResult parseUserTextWithLlm(const QString& userText, const AiLlmConfig& config, const AiProgressSink& progress,
	const QByteArray& imagePng, const QString& domainId)
{
	LlmParseResult out;
	const QString apiKey = resolveApiKey(config);
	const bool localOllama = isLocalOllamaBaseUrl(config.baseUrl);
	const QString authKey = apiKey.isEmpty() && localOllama ? QStringLiteral("ollama") : apiKey;
	if (authKey.isEmpty())
	{
		out.errorMessage = QStringLiteral("LLM api_key missing in ai_config.json and environment.");
		return out;
	}

	if (progress)
		progress(0.1, QStringLiteral("LLM request..."));

	const bool recognitionSchema = domainId == AiDomainIds::geometryRecognize();
	const bool composeSchema = domainId == AiDomainIds::meshCompose();
	const QString sys = recognitionSchema ? recognitionSystemPrompt()
		: (composeSchema ? composeSystemPrompt() : meshSystemPrompt());
	const QString userPrompt = recognitionSchema ? recognitionUserPrompt(userText) : userText.trimmed();

	nlohmann::json body;
	body["model"] = config.model.toStdString();
	body["temperature"] = config.temperature;

	QUrl url;
	if (localOllama && !imagePng.isEmpty())
	{
		// Ollama 原生 /api/chat + images[] 比 /v1 image_url 更可靠
		url = QUrl(ollamaNativeChatUrl(config.baseUrl));
		body["stream"] = false;
		const std::string imgB64 = visionImageBase64Raw(imagePng).toStdString();
		nlohmann::json userMsg;
		userMsg["role"] = "user";
		userMsg["content"] = userPrompt.toStdString();
		userMsg["images"] = nlohmann::json::array({ imgB64 });
		body["messages"] = nlohmann::json::array({
			{ { "role", "system" }, { "content", sys.toStdString() } },
			userMsg,
		});
	}
	else
	{
		url = QUrl(chatCompletionsUrl(config.baseUrl));
		nlohmann::json userMessage;
		if (!imagePng.isEmpty())
		{
			const std::string dataUrl = "data:image/jpeg;base64," + visionImageBase64Raw(imagePng).toStdString();
			userMessage["role"] = "user";
			userMessage["content"] = nlohmann::json::array({
				{ { "type", "image_url" }, { "image_url", { { "url", dataUrl } } } },
				{ { "type", "text" }, { "text", userPrompt.toStdString() } },
			});
		}
		else
		{
			userMessage = { { "role", "user" }, { "content", userPrompt.toStdString() } };
		}
		body["messages"] = nlohmann::json::array({
			{ { "role", "system" }, { "content", sys.toStdString() } },
			userMessage,
		});
	}

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
	if (localOllama && !imagePng.isEmpty() && resp.contains("message") && resp["message"].is_object())
	{
		content = resp["message"].value("content", std::string());
	}
	else if (resp.contains("choices") && resp["choices"].is_array() && !resp["choices"].empty())
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
		const std::string extracted = AiCommandSchema::extractJsonObjectText(content);
		try
		{
			out.command = nlohmann::json::parse(extracted, nullptr, true);
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
	else if (composeSchema)
	{
		const std::string extracted = AiCommandSchema::extractJsonObjectText(content);
		const std::string repaired = AiCommandSchema::repairComposePlanJsonText(extracted);
		try
		{
			out.command = nlohmann::json::parse(repaired, nullptr, true);
			AiCommandSchema::normalizeComposePlanJson(out.command);
		}
		catch (...)
		{
			out.errorMessage = QStringLiteral("Compose plan JSON parse failed.");
			return out;
		}
		QString planErr;
		if (!MeshComposeDomainHandler::validatePlanJson(out.command, &planErr))
		{
			out.errorMessage = planErr;
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
