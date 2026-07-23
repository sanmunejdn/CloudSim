/// @file AiAgentPlanBuilder.cpp
/// @brief scene 规则 → 多 keyword 串联 → LLM JSON 规划

#include "Ai/AiAgentPlanBuilder.h"

#include "Ai/AiCatalogKeywordMatcher.h"
#include "Ai/AiSceneOpsRules.h"
#include "AiDomainTypes.h"
#include "AiLlmClient.h"

#include <QSet>
#include <algorithm>

#include <json.hpp>

namespace AiAgentPlanBuilder
{
namespace
{
QSet<QString> catalogApiIds(const QByteArray& catalogJsonUtf8, const QString& domainId)
{
	QSet<QString> ids;
	try
	{
		const auto root = nlohmann::json::parse(catalogJsonUtf8.constData(), nullptr, true);
		if (!root.contains("apis") || !root["apis"].is_array())
			return ids;
		const std::string want = domainId.toStdString();
		for (const auto& api : root["apis"])
		{
			if (!api.is_object() || !api.contains("id") || !api["id"].is_string())
				continue;
			bool okDomain = domainId.isEmpty() || domainId == AiDomainIds::autoDomain();
			if (!okDomain && api.contains("domains") && api["domains"].is_array())
			{
				for (const auto& d : api["domains"])
				{
					if (d.is_string() && d.get<std::string>() == want)
					{
						okDomain = true;
						break;
					}
				}
			}
			if (okDomain)
				ids.insert(QString::fromStdString(api["id"].get<std::string>()));
		}
	}
	catch (...)
	{
	}
	return ids;
}

AiAgentPlan validateAndTrim(AiAgentPlan plan, const QSet<QString>& allowed, int maxSteps)
{
	AiAgentPlan out;
	out.summary = plan.summary;
	for (const auto& s : plan.steps)
	{
		if (s.apiId.isEmpty() || (!allowed.isEmpty() && !allowed.contains(s.apiId)))
			continue;
		out.steps.append(s);
		if (out.steps.size() >= maxSteps)
			break;
	}
	if (out.summary.isEmpty() && !out.steps.isEmpty())
	{
		QStringList bits;
		for (const auto& s : out.steps)
			bits << (s.rationale.isEmpty() ? s.apiId : s.rationale);
		out.summary = bits.join(QStringLiteral(" → "));
	}
	return out;
}

bool hasMultiCue(const QString& t)
{
	return t.contains(QStringLiteral("然后")) || t.contains(QStringLiteral("再")) ||
		   t.contains(QStringLiteral("并且")) || t.contains(QStringLiteral("接着")) ||
		   t.contains(QStringLiteral("先"));
}

/// 按「然后/再」切段，每段各自最长 keyword 命中（允许同 api）
AiAgentPlan tryMultiKeywordPlan(const QByteArray& catalog, const QString& userText, const QString& domainId,
								int maxSteps)
{
	AiAgentPlan plan;
	if (!hasMultiCue(userText))
		return plan;

	QString text = userText;
	text.replace(QStringLiteral("并且"), QStringLiteral("|"));
	text.replace(QStringLiteral("然后"), QStringLiteral("|"));
	text.replace(QStringLiteral("接着"), QStringLiteral("|"));
	text.replace(QStringLiteral("再"), QStringLiteral("|"));
	const QStringList parts = text.split(QLatin1Char('|'), Qt::SkipEmptyParts);
	if (parts.size() < 2)
		return plan;

	for (const QString& part : parts)
	{
		const auto m = AiCatalogKeywordMatcher::tryMatch(catalog, part.trimmed(), domainId, {});
		if (!m.ok)
			continue;
		AiAgentPlanStep s;
		s.apiId = m.apiId;
		nlohmann::json args = nlohmann::json::object();
		try
		{
			const auto pj = nlohmann::json::parse(m.planJsonUtf8.constData(), nullptr, true);
			if (pj.contains("steps") && pj["steps"].is_array() && !pj["steps"].empty())
				args = pj["steps"][0].value("args", nlohmann::json::object());
		}
		catch (...)
		{
		}
		s.argsJson = QByteArray::fromStdString(args.dump());
		s.rationale = m.hintMessage.isEmpty() ? m.apiId : m.hintMessage;
		plan.steps.append(s);
		if (plan.steps.size() >= maxSteps)
			break;
	}
	if (plan.steps.size() >= 2)
	{
		QStringList bits;
		for (const auto& s : plan.steps)
			bits << s.apiId;
		plan.summary = bits.join(QStringLiteral(" → "));
	}
	else
		plan.steps.clear();
	return plan;
}
} // namespace

AiAgentPlan buildPlan(const BuildInput& in, const QString& failureObservation)
{
	const int maxSteps = std::max(1, in.maxSteps);
	const QSet<QString> allowed = catalogApiIds(in.catalogJsonUtf8, in.domainId);

	if (failureObservation.isEmpty())
	{
		if (in.domainId == AiDomainIds::sceneOps() || in.domainId == AiDomainIds::autoDomain())
		{
			AiAgentPlan scene = AiSceneOpsRules::tryBuildPlan(in.userText, in.sceneSnapshotUtf8);
			scene = validateAndTrim(scene, allowed.isEmpty() ? catalogApiIds(in.catalogJsonUtf8, AiDomainIds::sceneOps())
															 : allowed,
									maxSteps);
			if (!scene.steps.isEmpty())
				return scene;
		}

		AiAgentPlan multi = tryMultiKeywordPlan(in.catalogJsonUtf8, in.userText, in.domainId, maxSteps);
		multi = validateAndTrim(multi, allowed, maxSteps);
		if (!multi.steps.isEmpty())
			return multi;

		// 无多步线索时不调 LLM 规划，留给单步 keyword/tool_calls
		if (!hasMultiCue(in.userText))
			return {};
	}

	if (!in.enableLlmPlan || !in.llm.enabled)
		return {};

	const QString prompt = failureObservation.isEmpty()
							   ? in.userText
							   : QStringLiteral("原需求：%1\n上一步失败：%2\n请只规划尚未完成的剩余步骤。")
									 .arg(in.userText, failureObservation);

	const auto lr = AiLlmClient::chatPlanJson(prompt, in.llm, in.progress, in.catalogJsonUtf8, in.domainId,
											  in.sceneSnapshotUtf8, in.sessionSummaryUtf8);
	if (!lr.ok || lr.plan.steps.isEmpty())
		return {};
	return validateAndTrim(lr.plan, allowed, maxSteps);
}
} // namespace AiAgentPlanBuilder
