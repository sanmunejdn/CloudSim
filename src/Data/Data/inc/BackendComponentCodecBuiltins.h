#ifndef DATA_BACKENDCOMPONENTCODECBUILTINS_H
#define DATA_BACKENDCOMPONENTCODECBUILTINS_H

/// @file BackendComponentCodecBuiltins.h
/// @brief 注册 Follow 等内置组件编解码

#include "BackendComponentCodecRegistry.h"
#include "FollowAttachmentComponent.h"
#include "RunLogger.h"

/// 注册 Follow 等内置组件编解码
inline void ensureBackendComponentCodecBuiltinsRegistered()
{
	static bool once = false;
	if (once)
	{
		return;
	}
	once = true;
	BackendComponentCodecRegistry::instance().setWarningHook([](const std::string& message)
															 { RunLogger::warn(message); });

	BackendComponentCodecRegistry::instance().registerCodec(
		FollowAttachmentComponent::typeKeyStatic(),
		[](const BackendComponentPtr& component, nlohmann::json& outData)
		{
			const auto follow = std::dynamic_pointer_cast<FollowAttachmentComponent>(component);
			if (!follow)
			{
				return false;
			}
			follow->writeJson(outData);
			return true;
		},
		[](const nlohmann::json& inData)
		{
			auto follow = std::make_shared<FollowAttachmentComponent>();
			follow->readJson(inData);
			return std::static_pointer_cast<IBackendComponent>(follow);
		});
}

#endif // DATA_BACKENDCOMPONENTCODECBUILTINS_H
