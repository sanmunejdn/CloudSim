#ifndef CLOUDSIMAISDK_AIPARSETYPES_H
#define CLOUDSIMAISDK_AIPARSETYPES_H

/// @file AiParseTypes.h
/// @brief AiParseTypes 接口

#include "cloudsim_ai_sdk_global.h"

#include "AiDomainTypes.h"

#include <QByteArray>
#include <QString>

struct AiParseResult
{
	bool ok = false;
	QString domainId;
	AiDomainOutputKind outputKind = AiDomainOutputKind::ActionPlan;
	/// ActionPlan 或领域 JSON（UTF-8）
	QByteArray outputJsonUtf8;
	QString errorMessage;
	QString hintMessage;
	QString parserVia;
};

#endif // CLOUDSIMAISDK_AIPARSETYPES_H
