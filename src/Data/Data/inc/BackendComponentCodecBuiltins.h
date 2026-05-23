#pragma once

#include "BackendComponentCodecRegistry.h"
#include "FollowAttachmentComponent.h"
#include "RunLogger.h"

inline void ensureBackendComponentCodecBuiltinsRegistered()
{
	static bool once = false;
	if (once)
	{
		return;
	}
	once = true;
	BackendComponentCodecRegistry::instance().setWarningHook(
		[](const std::string& message) { RunLogger::warn(message); });

	BackendComponentCodecRegistry::instance().registerCodec(
		FollowAttachmentComponent::typeKeyStatic(),
		[](const BackendComponentPtr& component, nlohmann::json& outData) {
			const auto follow = std::dynamic_pointer_cast<FollowAttachmentComponent>(component);
			if (!follow)
			{
				return false;
			}
			follow->writeJson(outData);
			return true;
		},
		[](const nlohmann::json& inData) {
			auto follow = std::make_shared<FollowAttachmentComponent>();
			follow->readJson(inData);
			return std::static_pointer_cast<IBackendComponent>(follow);
		});
}

