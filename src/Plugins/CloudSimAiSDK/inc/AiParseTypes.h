#pragma once

#include "AiDomainTypes.h"
#include "cloudsim_ai_sdk_global.h"

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
