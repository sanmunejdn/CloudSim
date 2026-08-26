#ifndef DATA_BACKENDCOMPONENTCODECBUILTINS_H
#define DATA_BACKENDCOMPONENTCODECBUILTINS_H

/// @file BackendComponentCodecBuiltins.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 注册 Follow 等内置组件编解码

#include "BackendComponentCodecRegistry.h"
#include "CustomDeviceRobotMountComponent.h"
#include "FollowAttachmentComponent.h"
#include "RunLogger.h"

/// 注册 Follow 等内置组件编解码
inline void ensureBackendComponentCodecBuiltinsRegistered()
{
	static std::once_flag once;
	std::call_once(once,
				   []()
				   {
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

	BackendComponentCodecRegistry::instance().registerCodec(
		CustomDeviceRobotMountComponent::typeKeyStatic(),
		[](const BackendComponentPtr& component, nlohmann::json& outData)
		{
			const auto mount = std::dynamic_pointer_cast<CustomDeviceRobotMountComponent>(component);
			if (!mount)
			{
				return false;
			}
			mount->writeJson(outData);
			return true;
		},
		[](const nlohmann::json& inData)
		{
			auto mount = std::make_shared<CustomDeviceRobotMountComponent>();
			mount->readJson(inData);
			return std::static_pointer_cast<IBackendComponent>(mount);
		});

	BackendComponentCodecRegistry::instance().registerPropertyPrefix(
		"follow.", FollowAttachmentComponent::typeKeyStatic(),
		[]() { return std::static_pointer_cast<IBackendComponent>(std::make_shared<FollowAttachmentComponent>()); });

	BackendComponentCodecRegistry::instance().registerLegacyObjectKey(
		"followAttachment", FollowAttachmentComponent::typeKeyStatic(),
		[](const nlohmann::json& in)
		{
			auto follow = std::make_shared<FollowAttachmentComponent>();
			follow->readJson(in);
			return std::static_pointer_cast<IBackendComponent>(follow);
		});

	BackendComponentCodecRegistry::instance().registerDefaultPropertyRows(
		FollowAttachmentComponent::typeKeyStatic(),
		[]() { return std::static_pointer_cast<IBackendComponent>(std::make_shared<FollowAttachmentComponent>()); });
				   });
}

#endif // DATA_BACKENDCOMPONENTCODECBUILTINS_H
