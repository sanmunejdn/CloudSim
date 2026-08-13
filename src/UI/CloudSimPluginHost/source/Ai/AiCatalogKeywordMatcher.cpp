/// @file AiCatalogKeywordMatcher.cpp
/// @brief Catalog 按钮名最长匹配（最短长度 + 词边界，降低误命中）

#include "Ai/AiCatalogKeywordMatcher.h"

#include "AiDomainTypes.h"

#include <json.hpp>

#include <algorithm>
#include <vector>

namespace AiCatalogKeywordMatcher
{
namespace
{
constexpr int kMinKeywordChars = 2;
constexpr int kMinAsciiKeywordChars = 3;

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

bool isAsciiWordChar(const QChar c)
{
	return c.isLetterOrNumber() || c == QLatin1Char('_');
}

bool isMostlyAscii(const QString& s)
{
	int ascii = 0;
	for (const QChar c : s)
	{
		if (c.unicode() < 128)
			++ascii;
	}
	return !s.isEmpty() && ascii * 2 >= s.size();
}

/// 拉丁词要求边界；中文短语至少 2 字且整段包含即可
bool keywordHits(const QString& text, const QString& kw)
{
	if (kw.size() < kMinKeywordChars)
		return false;
	const bool asciiKw = isMostlyAscii(kw);
	if (asciiKw && kw.size() < kMinAsciiKeywordChars)
		return false;

	const int idx = text.indexOf(kw, 0, Qt::CaseInsensitive);
	if (idx < 0)
		return false;
	if (!asciiKw)
		return true;

	const bool leftOk = idx == 0 || !isAsciiWordChar(text.at(idx - 1));
	const int end = idx + kw.size();
	const bool rightOk = end >= text.size() || !isAsciiWordChar(text.at(end));
	return leftOk && rightOk;
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
			c.keyword = QString::fromStdString(kw.get<std::string>()).trimmed();
			c.apiId = apiId;
			c.domainId = dom;
			if (c.keyword.size() >= kMinKeywordChars)
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
		if (keywordHits(text, c.keyword))
		{
			best = &c;
			break;
		}
	}
	if (!best)
	{
		out.errorMessage = QStringLiteral("未匹配到可靠的 Host 按钮关键词。");
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
