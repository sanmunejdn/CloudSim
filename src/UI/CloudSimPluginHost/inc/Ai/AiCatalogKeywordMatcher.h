#ifndef CLOUDSIMPLUGINHOST_AICATALOGKEYWORDMATCHER_H
#define CLOUDSIMPLUGINHOST_AICATALOGKEYWORDMATCHER_H

/// @file AiCatalogKeywordMatcher.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 按 Catalog keywords（Dock 按钮名）最长匹配生成 ActionPlan

#include <QByteArray>
#include <QString>
#include <QStringList>

namespace AiCatalogKeywordMatcher
{
struct MatchResult
{
	bool ok = false;
	QString apiId;
	QString domainId;
	QByteArray planJsonUtf8;
	QString hintMessage;
	QString errorMessage;
};

/// domainId 为空或 auto 时在全 Catalog 匹配；否则仅匹配该域
MatchResult tryMatch(const QByteArray& catalogJsonUtf8, const QString& userText, const QString& domainId,
					 const QStringList& excludeApiIds = {});
} // namespace AiCatalogKeywordMatcher

#endif
