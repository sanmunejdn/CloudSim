#pragma once

#include "AiInferenceTypes.h"
#include "AiParseTypes.h"
#include "cloudsim_ai_sdk_global.h"

#include <functional>
#include <memory>

class IAiInferenceProvider
{
public:
	virtual ~IAiInferenceProvider() = default;

	virtual QString domainId() const = 0;
	virtual AiInferenceKind kind() const = 0;

	virtual void inferAsync(const AiInferenceRequest& request, const AiInferenceProgressFn& progress,
		std::function<void(AiParseResult)> onFinished) = 0;
};
