/// @file AiCatalogKeywordMatcher.cpp
/// @brief Catalog 按钮名最长匹配

#include "Ai/AiCatalogKeywordMatcher.h"

#include "AiDomainTypes.h"

#include <json.hpp>

#include <algorithm>
#include <vector>

namespace AiCatalogKeywordMatcher
{
namespace
{
struct Candidate
{
	QString keyword;
	QString apiId;
	QString domainId;
};

bool domainAllowed(const nlohmann::json& domains, const QString& want)
{
	if (want.isEmpty() || want == AiDomainIds::autoDomain())
		return true;
	if (!domains.is_array())
		return false;
	const std::string w = want.toStdString();
	for (const auto& d : domains)
	{
		if (d.is_string() && d.get<std::string>() == w)
			return true;
	}
	return false;
}

QString pickDomain(const nlohmann::json& domains, const QString& want)
{
	if (!want.isEmpty() && want != AiDomainIds::autoDomain())
		return want;
	if (domains.is_array() && !domains.empty() && domains[0].is_string())
		return QString::fromStdString(domains[0].get<std::string>());
	return QString();
}
} // namespace

MatchResult tryMatch(const QByteArray& catalogJsonUtf8, const QString& userText, const QString& domainId,
					 const QStringList& excludeApiIds)
{
	MatchResult out;
	const QString text = userText.trimmed();
	if (text.isEmpty())
	{
		out.errorMessage = QStringLiteral("空输入。");
		return out;
	}

	nlohmann::json root;
	try
	{
		root = nlohmann::json::parse(catalogJsonUtf8.constData(), nullptr, true);
	}
	catch (...)
	{
		out.errorMessage = QStringLiteral("ApiCatalog JSON 无效。");
		return out;
	}

	std::vector<Candidate> cands;
	const auto apis = root.contains("apis") && root["apis"].is_array() ? root["apis"] : nlohmann::json::array();
	for (const auto& api : apis)
	{
		if (!api.is_object() || !api.contains("id") || !api["id"].is_string())
			continue;
		if (!domainAllowed(api.value("domains", nlohmann::json::array()), domainId))
			continue;
		const QString apiId = QString::fromStdString(api["id"].get<std::string>());
		const QString dom = pickDomain(api.value("domains", nlohmann::json::array()), domainId);
		if (!api.contains("keywords") || !api["keywords"].is_array())
			continue;
		for (const auto& kw : api["keywords"])
		{
			if (!kw.is_string())
				continue;
			Candidate c;
			c.keyword = QString::fromStdString(kw.get<std::string>());
			c.apiId = apiId;
			c.domainId = dom;
			if (!c.keyword.isEmpty())
				cands.push_back(std::move(c));
		}
	}

	std::sort(cands.begin(), cands.end(),
			  [](const Candidate& a, const Candidate& b) { return a.keyword.size() > b.keyword.size(); });

	const Candidate* best = nullptr;
	for (const auto& c : cands)
	{
		if (excludeApiIds.contains(c.apiId))
			continue;
		if (text.contains(c.keyword, Qt::CaseInsensitive))
		{
			best = &c;
			break;
		}
	}
	if (!best)
	{
		out.errorMessage = QStringLiteral("未匹配到 Host 按钮关键词。");
		return out;
	}

	nlohmann::json plan;
	plan["version"] = 2;
	plan["steps"] = nlohmann::json::array();
	nlohmann::json step;
	step["id"] = "s1";
	step["api"] = best->apiId.toStdString();
	step["args"] = nlohmann::json::object();
	plan["steps"].push_back(step);

	out.ok = true;
	out.apiId = best->apiId;
	out.domainId = best->domainId;
	out.planJsonUtf8 = QByteArray::fromStdString(plan.dump());
	out.hintMessage = QStringLiteral("rules: %1 → %2").arg(best->keyword, best->apiId);
	return out;
}
} // namespace AiCatalogKeywordMatcher
