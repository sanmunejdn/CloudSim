#pragma once

#include "robot_scene_global.h"

#include <json.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace RobotInstruction
{
struct ROBOT_SCENE_API Vec3
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

enum class ROBOT_SCENE_API Type
{
	PTP = 0,
	LINE,
	CUSTOM
};

class AttributeBase;

class ROBOT_SCENE_API Base
{
public:
	Base();
	virtual ~Base() = default;

	const std::string& id() const { return m_id; }
	void setId(const std::string& id) { m_id = id; }

	const std::string& controllerId() const { return m_controllerId; }
	void setControllerId(const std::string& controllerId) { m_controllerId = controllerId; }

	Type type() const { return m_type; }
	void setType(Type t) { m_type = t; }

	const std::string& name() const { return m_name; }
	void setName(const std::string& n) { m_name = n; }

	virtual bool hasPoseProperty() const { return false; }
	virtual Vec3 pose() const { return Vec3{}; }
	virtual void setPose(const Vec3& p) { (void)p; }

	virtual bool hasEulerProperty() const { return false; }
	virtual Vec3 eulerDeg() const { return Vec3{}; }
	virtual void setEulerDeg(const Vec3& e) { (void)e; }

	virtual bool hasSpeedProperty() const { return false; }
	virtual double speed() const { return 0.0; }
	virtual void setSpeed(double v) { (void)v; }

	virtual bool hasAccelProperty() const { return false; }
	virtual double accel() const { return 0.0; }
	virtual void setAccel(double v) { (void)v; }

	virtual bool hasAxisConfigProperty() const { return false; }
	virtual std::string axisConfig() const { return {}; }
	virtual void setAxisConfig(const std::string& v) { (void)v; }

	virtual bool hasBlendRadiusProperty() const { return false; }
	virtual double blendRadius() const { return 0.0; }
	virtual void setBlendRadius(double v) { (void)v; }

	nlohmann::json snapshotPropertyRows() const;
	bool applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg);

	const std::unordered_map<std::string, std::string>& extensionProperties() const { return m_extensionProperties; }
	void setExtensionProperty(const std::string& key, const std::string& value) { m_extensionProperties[key] = value; }

protected:
	void addAttribute(const std::shared_ptr<AttributeBase>& attr);

private:
	std::string m_id;
	std::string m_controllerId;
	std::string m_name;
	Type m_type = Type::CUSTOM;
	std::vector<std::shared_ptr<AttributeBase>> m_attributes;
	std::unordered_map<std::string, std::string> m_extensionProperties;
};

class ROBOT_SCENE_API PtpInstruction final : public Base
{
public:
	PtpInstruction();

	bool hasPoseProperty() const override { return true; }
	Vec3 pose() const override { return m_pose; }
	void setPose(const Vec3& p) override { m_pose = p; }

	bool hasEulerProperty() const override { return true; }
	Vec3 eulerDeg() const override { return m_eulerDeg; }
	void setEulerDeg(const Vec3& e) override { m_eulerDeg = e; }

	bool hasSpeedProperty() const override { return true; }
	double speed() const override { return m_speed; }
	void setSpeed(double v) override { m_speed = v; }

	bool hasAccelProperty() const override { return true; }
	double accel() const override { return m_accel; }
	void setAccel(double v) override { m_accel = v; }

	bool hasAxisConfigProperty() const override { return true; }
	std::string axisConfig() const override { return m_axisConfig; }
	void setAxisConfig(const std::string& v) override { m_axisConfig = v; }

private:
	Vec3 m_pose{};
	Vec3 m_eulerDeg{};
	double m_speed = 100.0;
	double m_accel = 100.0;
	std::string m_axisConfig = "AUTO";
};

class ROBOT_SCENE_API LineInstruction final : public Base
{
public:
	LineInstruction();

	bool hasPoseProperty() const override { return true; }
	Vec3 pose() const override { return m_pose; }
	void setPose(const Vec3& p) override { m_pose = p; }

	bool hasEulerProperty() const override { return true; }
	Vec3 eulerDeg() const override { return m_eulerDeg; }
	void setEulerDeg(const Vec3& e) override { m_eulerDeg = e; }

	bool hasSpeedProperty() const override { return true; }
	double speed() const override { return m_tcpSpeed; }
	void setSpeed(double v) override { m_tcpSpeed = v; }

	bool hasAccelProperty() const override { return true; }
	double accel() const override { return m_tcpAccel; }
	void setAccel(double v) override { m_tcpAccel = v; }

	bool hasBlendRadiusProperty() const override { return true; }
	double blendRadius() const override { return m_blendRadius; }
	void setBlendRadius(double v) override { m_blendRadius = v; }

private:
	Vec3 m_pose{};
	Vec3 m_eulerDeg{};
	double m_tcpSpeed = 200.0;
	double m_tcpAccel = 200.0;
	double m_blendRadius = 0.0;
};

} // namespace RobotInstruction
