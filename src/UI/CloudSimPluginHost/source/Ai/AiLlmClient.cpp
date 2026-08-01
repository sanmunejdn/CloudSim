/// @file AiLlmClient.cpp
/// @brief AiLlmClient 实现

#include "AiLlmClient.h"

#include "Ai/AiArgsSchema.h"
#include "Ai/AiMeshDefaults.h"
#include "Ai/FeatureComposeDomainHandler.h"
#include "Ai/MeshComposeDomainHandler.h"
#include "AiCommandSchema.h"
#include "AiDomainTypes.h"
#include "AiHttpsPost.h"

#include <QBuffer>
#include <QByteArray>
#include <QImage>
#include <QUrl>

#include <json.hpp>

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
	return baseUrl.contains(QStringLiteral("127.0.0.1"), Qt::CaseInsensitive) ||
		   baseUrl.contains(QStringLiteral("localhost"), Qt::CaseInsensitive);
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
			   "If the user says larger/smaller/thicker/flatter (大一点/小一点/扁/厚), scale defaults (~1.5x or ~0.5x) "
			   "and still output full dimensions_mm.\n"
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
	return QStringLiteral("你是 CAD 视口几何识别助手。用户会提供一张 3D 视口截图（通常白底、单个主要基本体）。"
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

QString trajectoryFeatureSystemPrompt()
{
	return QStringLiteral(
		"你是 STEP 工件轨迹特征助手。用户会提供 feature catalog 切片（含 displayIndex、candidateId、summary）和意图。"
		"仅回复一个 JSON 对象，不要 markdown：\n"
		"{\"version\":1,\"featureAxis\":\"line|surface|ambiguous\",\"clarifyMessage\":\"可选\","
		"\"selectedCandidateIds\":[\"edge_0\"],"
		"\"features\":[{\"schemaVersion\":1,\"featureId\":\"...\",\"kind\":\"EdgeChain|FaceBoundary|FaceUVGrid\","
		"\"workpiece\":{\"backendIdUtf8\":\"...\",\"stepPathUtf8\":\"...\"},"
		"\"refs\":{...},\"discretize\":{\"stepMm\":5.0,\"linearDeflectionMm\":0.01}}],"
		"\"suggestedPipelineTemplate\":\"weld_default|glue_default|grind_default\"}\n"
		"若无法判断线/面特征，featureAxis=ambiguous 并填写 clarifyMessage。"
		"selectedCandidateIds 必须从 catalog 的 candidateId 选取。");
}

QString trajectoryFeatureUserPrompt(const QString& userText, const QByteArray& catalogSliceUtf8)
{
	QString prompt = userText.trimmed();
	prompt += QStringLiteral("\n\nFeature catalog slice JSON:\n");
	prompt += QString::fromUtf8(catalogSliceUtf8);
	return prompt;
}

QString composeSystemPrompt()
{
	const auto d = AiMeshDefaults::activeDefaults();
	return QStringLiteral(
			   "You are a text-to-CAD planner for CloudSim. Output ActionPlan JSON version 2 ONLY (no markdown).\n"
			   "Schema: {\"version\":2,\"domain\":\"mesh.compose\",\"steps\":[{\"id\":\"...\",\"api\":\"...\",\"args\":{...}}]}.\n"
			   "Use mesh boolean CSG for through-holes / boolean ops (not parametric history).\n"
			   "Clarify-before-draw: if specs incomplete, ONE askClarify step and STOP.\n"
			   "APIs: askClarify; createPrimitiveMesh; booleanMesh.\n"
			   "Defaults if sizes omitted: box %1x%2x%3, cylinder R%4 H%5.")
		.arg(d.boxLengthMm)
		.arg(d.boxWidthMm)
		.arg(d.boxHeightMm)
		.arg(d.cylinderRadiusMm)
		.arg(d.cylinderHeightMm);
}

QString featureComposeSystemPrompt()
{
	return QStringLiteral(
		"You are a parametric text-to-CAD planner for CloudSim. Output ActionPlan JSON version 2 ONLY (no markdown).\n"
		"Schema: {\"version\":2,\"domain\":\"feature.compose\",\"steps\":[{\"id\":\"...\",\"api\":\"...\",\"args\":{...}}]}.\n"
		"Philosophy (Text2CAD): SEQUENCE of real features — Pad → Pocket → Fillet/Chamfer → Revolve → Pattern → Sweep/Loft → Shell/Draft.\n"
		"Through-holes / cutouts on a plate MUST be Pocket on \"$priorBody\" (end_condition through_all). NEVER use booleanMesh.\n"
		"All dimensions in mm. Prefer clarify over inventing sizes.\n"
		"Clarify-before-draw (Pro-CAD): if critical sizes missing, ONE step askClarify with questions[] and STOP.\n"
		"APIs:\n"
		"1) askClarify args: questions (string array).\n"
		"2) extrudeSketchProfileToBrep args:\n"
		"   mode: pad|pocket; profile: rectangle|polygon|circle; length_mm+width_mm (rect) or sides+radius_mm (polygon)\n"
		"   or diameter_mm/radius_mm + optional center_u_mm/center_v_mm (circle);\n"
		"   extrude_mm (depth); end_condition: blind|through_all; optional name; pocket requires target \"$priorStepId\".\n"
		"   Optional profile_xyz_mm closed polyline (xyz interleaved) instead of profile helpers.\n"
		"3) filletEdgesToBrep args: target \"$stepId\", radius_mm, edge_indices int[] OR edges:\"longest\"|\"top_boundary\"|\"all\".\n"
		"   Default edges=longest (top-K, edge_count default 4). Prefer longest/top_boundary over all.\n"
		"4) chamferEdgesToBrep args: target \"$stepId\", distance_mm, same edges options as fillet.\n"
		"5) revolveSketchProfileToBrep args: mode boss|cut; same profile helpers as extrude; angle_deg (default 360);\n"
		"   default axis origin +Y (axis_dy=1); cut requires target \"$stepId\".\n"
		"6) linearPatternBodyToBrep args: target \"$stepId\", count>=2, dx_mm, dy_mm, dz_mm; optional source_feature_id.\n"
		"7) sweepSketchProfileToBrep args: mode boss|cut; same profile helpers as extrude; path path_xyz_mm float[] OR "
		"path line_z (path_length_mm along +Z) OR path line with path_dx/dy/dz_mm; optional twist_deg; cut requires target \"$stepId\".\n"
		"8) loftSketchProfilesToBrep args: mode boss|cut; profile_a/profile_b helpers (profile_a rectangle + length_a_mm etc.) "
		"or profile_a_xyz_mm/profile_b_xyz_mm; profile_b_z_mm (default 10) separates sections; cut requires target \"$stepId\".\n"
		"9) shellFacesToBrep args: target \"$stepId\", thickness_mm, face_indices int[] (required; Host has no faces=all).\n"
		"10) draftFacesToBrep args: target \"$stepId\", angle_deg, face_indices int[]; optional neutral_ox/oy/oz, neutral_nx/ny/nz (default XY).\n"
		"11) circularPatternBodyToBrep args: target \"$stepId\", count>=2, angle_deg (default 360), axis_ox/oy/oz, axis_dx/dy/dz (default +Z); optional source_feature_id.\n"
		"Example plate 100x80x40 with center through-hole d10:\n"
		"{\"version\":2,\"domain\":\"feature.compose\",\"steps\":["
		"{\"id\":\"body\",\"api\":\"extrudeSketchProfileToBrep\",\"args\":{\"mode\":\"pad\",\"profile\":\"rectangle\","
		"\"length_mm\":100,\"width_mm\":80,\"extrude_mm\":40,\"name\":\"Body\"}},"
		"{\"id\":\"hole\",\"api\":\"extrudeSketchProfileToBrep\",\"args\":{\"mode\":\"pocket\",\"profile\":\"circle\","
		"\"diameter_mm\":10,\"center_u_mm\":50,\"center_v_mm\":40,\"end_condition\":\"through_all\","
		"\"extrude_mm\":40,\"target\":\"$body\"}}]}");
}
} // namespace

LlmParseResult parseUserTextWithLlm(const QString& userText, const AiLlmConfig& config, const AiProgressSink& progress,
									const QByteArray& imagePng, const QString& domainId,
									const QByteArray& catalogSliceUtf8)
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
	const bool featureComposeSchema = domainId == AiDomainIds::featureCompose();
	const bool trajectorySchema = domainId == AiDomainIds::trajectoryFeature();
	const QString sys =
		recognitionSchema
			? recognitionSystemPrompt()
			: (featureComposeSchema
				   ? featureComposeSystemPrompt()
				   : (composeSchema ? composeSystemPrompt()
									: (trajectorySchema ? trajectoryFeatureSystemPrompt() : meshSystemPrompt())));
	const QString userPrompt =
		recognitionSchema
			? recognitionUserPrompt(userText)
			: (trajectorySchema ? trajectoryFeatureUserPrompt(userText, catalogSliceUtf8) : userText.trimmed());

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
		userMsg["images"] = nlohmann::json::array({imgB64});
		body["messages"] = nlohmann::json::array({
			{{"role", "system"}, {"content", sys.toStdString()}},
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
				{{"type", "image_url"}, {"image_url", {{"url", dataUrl}}}},
				{{"type", "text"}, {"text", userPrompt.toStdString()}},
			});
		}
		else
		{
			userMessage = {{"role", "user"}, {"content", userPrompt.toStdString()}};
		}
		body["messages"] = nlohmann::json::array({
			{{"role", "system"}, {"content", sys.toStdString()}},
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

	if (recognitionSchema || trajectorySchema)
	{
		const std::string extracted = AiCommandSchema::extractJsonObjectText(content);
		try
		{
			out.command = nlohmann::json::parse(extracted, nullptr, true);
		}
		catch (...)
		{
			out.errorMessage = trajectorySchema ? QStringLiteral("Trajectory feature JSON parse failed.")
												: QStringLiteral("Recognition JSON parse failed.");
			return out;
		}
		if (!out.command.is_object())
		{
			out.errorMessage = QStringLiteral("Response must be a JSON object.");
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
	else if (featureComposeSchema)
	{
		const std::string extracted = AiCommandSchema::extractJsonObjectText(content);
		try
		{
			out.command = nlohmann::json::parse(extracted, nullptr, true);
		}
		catch (...)
		{
			out.errorMessage = QStringLiteral("feature.compose 计划 JSON 解析失败。");
			return out;
		}
		QString planErr;
		if (!FeatureComposeDomainHandler::validatePlanJson(out.command, &planErr))
		{
			out.errorMessage = planErr.isEmpty() ? QStringLiteral("feature.compose 计划校验失败。") : planErr;
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

QByteArray buildOpenAiToolsFromCatalog(const QByteArray& catalogJsonUtf8, const QString& domainId,
									   const QStringList& excludeApiIds)
{
	return AiArgsSchema::buildOpenAiToolsFromCatalog(catalogJsonUtf8, domainId, excludeApiIds);
}

void appendToolObservation(nlohmann::json& messages, const QString& toolCallId, const QString& toolName,
						   const QByteArray& observationUtf8)
{
	nlohmann::json msg;
	msg["role"] = "tool";
	msg["tool_call_id"] = toolCallId.isEmpty() ? toolName.toStdString() : toolCallId.toStdString();
	msg["name"] = toolName.toStdString();
	msg["content"] = observationUtf8.isEmpty() ? std::string("ok") : std::string(observationUtf8.constData());
	messages.push_back(msg);
}

ToolProposeResult chatWithTools(const QString& userText, const AiLlmConfig& config, const AiProgressSink& progress,
								const QByteArray& toolsJsonUtf8, const QByteArray& sceneSnapshotUtf8,
								const QByteArray& sessionSummaryUtf8, nlohmann::json* inoutMessages)
{
	ToolProposeResult out;
	const QString apiKey = resolveApiKey(config);
	const bool localOllama = isLocalOllamaBaseUrl(config.baseUrl);
	const QString authKey = apiKey.isEmpty() && localOllama ? QStringLiteral("ollama") : apiKey;
	if (authKey.isEmpty())
	{
		out.errorMessage = QStringLiteral("LLM api_key missing in ai_config.json and environment.");
		return out;
	}
	if (progress)
		progress(0.1, QStringLiteral("Agent tool propose..."));

	nlohmann::json tools = nlohmann::json::array();
	try
	{
		if (!toolsJsonUtf8.isEmpty())
			tools = nlohmann::json::parse(toolsJsonUtf8.constData(), nullptr, true);
	}
	catch (...)
	{
		out.errorMessage = QStringLiteral("Invalid tools JSON.");
		return out;
	}

	const QString sys = QStringLiteral(
		"You are CloudSim CAD agent. Call exactly one tool when the user asks to change the scene. "
		"After tool results, call the next tool if needed, or reply with short plain text to finish.");

	nlohmann::json messages;
	if (inoutMessages && inoutMessages->is_array() && !inoutMessages->empty())
	{
		messages = *inoutMessages;
	}
	else
	{
		const QString userPrompt =
			QStringLiteral("User: %1\nScene snapshot:\n%2\nSession:\n%3")
				.arg(userText, QString::fromUtf8(sceneSnapshotUtf8), QString::fromUtf8(sessionSummaryUtf8));
		messages = nlohmann::json::array({
			{{"role", "system"}, {"content", sys.toStdString()}},
			{{"role", "user"}, {"content", userPrompt.toStdString()}},
		});
	}

	nlohmann::json body;
	body["model"] = config.model.toStdString();
	body["temperature"] = config.temperature;
	body["messages"] = messages;
	if (tools.is_array() && !tools.empty())
		body["tools"] = tools;

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
		progress(0.85, QStringLiteral("Parsing tool_calls..."));

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
		out.errorMessage = QString::fromStdString(resp["error"].value("message", std::string("LLM API error")));
		return out;
	}

	nlohmann::json assistantMsg = nlohmann::json::object();
	std::string content;
	nlohmann::json toolCalls = nlohmann::json::array();
	if (resp.contains("choices") && resp["choices"].is_array() && !resp["choices"].empty())
	{
		assistantMsg = resp["choices"][0].value("message", nlohmann::json::object());
		if (assistantMsg.contains("content") && assistantMsg["content"].is_string())
			content = assistantMsg["content"].get<std::string>();
		if (assistantMsg.contains("tool_calls") && assistantMsg["tool_calls"].is_array())
			toolCalls = assistantMsg["tool_calls"];
	}
	out.assistantMessage = assistantMsg;

	auto commitMessages = [&]()
	{
		if (inoutMessages)
		{
			if (!inoutMessages->is_array() || inoutMessages->empty())
				*inoutMessages = messages;
			inoutMessages->push_back(assistantMsg.is_object() ? assistantMsg
															  : nlohmann::json{{"role", "assistant"},
																			   {"content", content}});
		}
	};

	if (!toolCalls.empty() && toolCalls[0].is_object())
	{
		const auto& tc = toolCalls[0];
		const auto& fn = tc.value("function", nlohmann::json::object());
		if (fn.contains("name") && fn["name"].is_string())
		{
			out.hasToolCall = true;
			out.toolName = QString::fromStdString(fn["name"].get<std::string>());
			if (tc.contains("id") && tc["id"].is_string())
				out.toolCallId = QString::fromStdString(tc["id"].get<std::string>());
			nlohmann::json args = nlohmann::json::object();
			if (fn.contains("arguments"))
			{
				if (fn["arguments"].is_string())
				{
					try
					{
						args = nlohmann::json::parse(fn["arguments"].get<std::string>(), nullptr, true);
					}
					catch (...)
					{
						args = nlohmann::json::object();
					}
				}
				else if (fn["arguments"].is_object())
					args = fn["arguments"];
			}
			out.args = args.is_object() ? args : nlohmann::json::object();
			out.ok = true;
			commitMessages();
			if (progress)
				progress(1.0, QString());
			return out;
		}
	}

	if (!content.empty())
	{
		const std::string extracted = AiCommandSchema::extractJsonObjectText(content);
		try
		{
			nlohmann::json j = nlohmann::json::parse(extracted, nullptr, true);
			if (j.is_object() && j.contains("api") && j["api"].is_string())
			{
				out.hasToolCall = true;
				out.toolName = QString::fromStdString(j["api"].get<std::string>());
				out.args = j.value("args", nlohmann::json::object());
				out.toolCallId = out.toolName;
				out.ok = true;
				commitMessages();
				if (progress)
					progress(1.0, QString());
				return out;
			}
		}
		catch (...)
		{
		}
		out.assistantText = QString::fromStdString(content);
		out.ok = true;
		out.hasToolCall = false;
		commitMessages();
		if (progress)
			progress(1.0, QString());
		return out;
	}

	out.errorMessage = QStringLiteral("LLM response has no tool_calls or content.");
	return out;
}

PlanJsonResult chatPlanJson(const QString& userText, const AiLlmConfig& config, const AiProgressSink& progress,
							const QByteArray& catalogJsonUtf8, const QString& domainId,
							const QByteArray& sceneSnapshotUtf8, const QByteArray& sessionSummaryUtf8)
{
	PlanJsonResult out;
	const QString apiKey = resolveApiKey(config);
	const bool localOllama = isLocalOllamaBaseUrl(config.baseUrl);
	const QString authKey = apiKey.isEmpty() && localOllama ? QStringLiteral("ollama") : apiKey;
	if (authKey.isEmpty())
	{
		out.errorMessage = QStringLiteral("LLM api_key missing in ai_config.json and environment.");
		return out;
	}
	if (progress)
		progress(0.1, QStringLiteral("Agent plan..."));

	nlohmann::json toolList = nlohmann::json::array();
	try
	{
		const auto root = nlohmann::json::parse(catalogJsonUtf8.constData(), nullptr, true);
		if (root.contains("apis") && root["apis"].is_array())
		{
			const std::string want = domainId.toStdString();
			for (const auto& api : root["apis"])
			{
				if (!api.is_object() || !api.contains("id") || !api["id"].is_string())
					continue;
				bool ok = domainId.isEmpty() || domainId == AiDomainIds::autoDomain();
				if (!ok && api.contains("domains") && api["domains"].is_array())
				{
					for (const auto& d : api["domains"])
					{
						if (d.is_string() && d.get<std::string>() == want)
						{
							ok = true;
							break;
						}
					}
				}
				if (!ok)
					continue;
				nlohmann::json row;
				row["id"] = api["id"];
				if (api.contains("summary"))
					row["summary"] = api["summary"];
				toolList.push_back(row);
			}
		}
	}
	catch (...)
	{
		out.errorMessage = QStringLiteral("Invalid catalog JSON.");
		return out;
	}

	const QString sys = QStringLiteral(
		"You are CloudSim CAD planner. Decompose the user request into an ordered list of catalog API steps. "
		"Reply with ONLY one JSON object: "
		"{\"summary\":\"...\",\"steps\":[{\"api_id\":\"...\",\"args\":{},\"rationale\":\"...\"}]}. "
		"Use only api_id values from the provided catalog. If nothing applies, return {\"summary\":\"\",\"steps\":[]}.");

	const QString userPrompt =
		QStringLiteral("User: %1\nDomain: %2\nCatalog tools:\n%3\nScene:\n%4\nSession:\n%5")
			.arg(userText, domainId, QString::fromUtf8(QByteArray::fromStdString(toolList.dump(2))),
				 QString::fromUtf8(sceneSnapshotUtf8), QString::fromUtf8(sessionSummaryUtf8));

	nlohmann::json body;
	body["model"] = config.model.toStdString();
	body["temperature"] = config.temperature;
	body["messages"] = nlohmann::json::array({
		{{"role", "system"}, {"content", sys.toStdString()}},
		{{"role", "user"}, {"content", userPrompt.toStdString()}},
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
		progress(0.85, QStringLiteral("Parsing plan JSON..."));

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
		out.errorMessage = QString::fromStdString(resp["error"].value("message", std::string("LLM API error")));
		return out;
	}

	std::string content;
	if (resp.contains("choices") && resp["choices"].is_array() && !resp["choices"].empty())
	{
		const auto msg = resp["choices"][0].value("message", nlohmann::json::object());
		if (msg.contains("content") && msg["content"].is_string())
			content = msg["content"].get<std::string>();
	}
	out.rawText = QString::fromStdString(content);
	if (content.empty())
	{
		out.errorMessage = QStringLiteral("Empty plan response.");
		return out;
	}

	const std::string extracted = AiCommandSchema::extractJsonObjectText(content);
	nlohmann::json j;
	try
	{
		j = nlohmann::json::parse(extracted, nullptr, true);
	}
	catch (...)
	{
		out.errorMessage = QStringLiteral("Plan JSON parse failed.");
		return out;
	}
	if (!j.is_object())
	{
		out.errorMessage = QStringLiteral("Plan root must be object.");
		return out;
	}
	if (j.contains("summary") && j["summary"].is_string())
		out.plan.summary = QString::fromStdString(j["summary"].get<std::string>());
	if (j.contains("steps") && j["steps"].is_array())
	{
		for (const auto& st : j["steps"])
		{
			if (!st.is_object())
				continue;
			QString apiId;
			if (st.contains("api_id") && st["api_id"].is_string())
				apiId = QString::fromStdString(st["api_id"].get<std::string>());
			else if (st.contains("api") && st["api"].is_string())
				apiId = QString::fromStdString(st["api"].get<std::string>());
			if (apiId.isEmpty())
				continue;
			AiAgentPlanStep step;
			step.apiId = apiId;
			const auto args = st.value("args", nlohmann::json::object());
			step.argsJson = QByteArray::fromStdString(args.dump());
			if (st.contains("rationale") && st["rationale"].is_string())
				step.rationale = QString::fromStdString(st["rationale"].get<std::string>());
			out.plan.steps.append(step);
		}
	}
	out.ok = true;
	if (progress)
		progress(1.0, QString());
	return out;
}

} // namespace AiLlmClient
