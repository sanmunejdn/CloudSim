#pragma once

#include "robot_scene_global.h"

#include <json.hpp>

#include <string>

namespace RobotInstruction
{
class Base;

class ROBOT_SCENE_API AttributeBase
{
public:
	virtual ~AttributeBase() = default;

	virtual void appendRows(const Base& cmd, nlohmann::json& rows) const = 0;
	virtual bool handlesKey(const Base& cmd, const std::string& key) const = 0;
	virtual bool apply(Base& cmd, const std::string& key, const std::string& value, std::string* errMsg) const = 0;
};

class ROBOT_SCENE_API PoseAttribute final : public AttributeBase
{
public:
	void appendRows(const Base& cmd, nlohmann::json& rows) const override;
	bool handlesKey(const Base& cmd, const std::string& key) const override;
	bool apply(Base& cmd, const std::string& key, const std::string& value, std::string* errMsg) const override;
};

class ROBOT_SCENE_API EulerAttribute final : public AttributeBase
{
public:
	void appendRows(const Base& cmd, nlohmann::json& rows) const override;
	bool handlesKey(const Base& cmd, const std::string& key) const override;
	bool apply(Base& cmd, const std::string& key, const std::string& value, std::string* errMsg) const override;
};

class ROBOT_SCENE_API SpeedAttribute final : public AttributeBase
{
public:
	void appendRows(const Base& cmd, nlohmann::json& rows) const override;
	bool handlesKey(const Base& cmd, const std::string& key) const override;
	bool apply(Base& cmd, const std::string& key, const std::string& value, std::string* errMsg) const override;
};

class ROBOT_SCENE_API AccelAttribute final : public AttributeBase
{
public:
	void appendRows(const Base& cmd, nlohmann::json& rows) const override;
	bool handlesKey(const Base& cmd, const std::string& key) const override;
	bool apply(Base& cmd, const std::string& key, const std::string& value, std::string* errMsg) const override;
};

class ROBOT_SCENE_API AxisConfigAttribute final : public AttributeBase
{
public:
	void appendRows(const Base& cmd, nlohmann::json& rows) const override;
	bool handlesKey(const Base& cmd, const std::string& key) const override;
	bool apply(Base& cmd, const std::string& key, const std::string& value, std::string* errMsg) const override;
};

class ROBOT_SCENE_API BlendRadiusAttribute final : public AttributeBase
{
public:
	void appendRows(const Base& cmd, nlohmann::json& rows) const override;
	bool handlesKey(const Base& cmd, const std::string& key) const override;
	bool apply(Base& cmd, const std::string& key, const std::string& value, std::string* errMsg) const override;
};

} // namespace RobotInstruction
