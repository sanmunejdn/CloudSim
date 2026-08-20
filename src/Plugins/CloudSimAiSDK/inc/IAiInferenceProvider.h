#ifndef CLOUDSIMAISDK_IAIINFERENCEPROVIDER_H
#define CLOUDSIMAISDK_IAIINFERENCEPROVIDER_H

/// @file IAiInferenceProvider.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief IAiInferenceProvider 接口

#include "cloudsim_ai_sdk_global.h"

#include "AiInferenceTypes.h"
#include "AiParseTypes.h"

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

#endif // CLOUDSIMAISDK_IAIINFERENCEPROVIDER_H
