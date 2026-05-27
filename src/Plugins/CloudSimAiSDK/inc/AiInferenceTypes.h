#pragma once

#include "cloudsim_ai_sdk_global.h"

#include <QByteArray>
#include <QString>
#include <functional>

enum class CLOUDSIM_AI_SDK_EXPORT AiInferenceKind
{
	RulesOnly,
	LocalText,
	LocalMultimodal,
	RemoteCloud
};

struct AiInferenceRequest
{
	QString domainId;
	QString userText;
	QByteArray imagePng;
	QByteArray catalogSliceUtf8;
};

using AiInferenceProgressFn = std::function<void(double fraction, const QString& message)>;
