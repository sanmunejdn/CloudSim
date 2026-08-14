#ifndef ROBOTSCENE_ROBOTINSTRUCTIONMODEL_H
#define ROBOTSCENE_ROBOTINSTRUCTIONMODEL_H

/// @file RobotInstructionModel.h
/// @brief RobotInstructionModel 接口

#include "robot_scene_global.h"

#include "RobotInstructionAxisConfiguration.h"
#include "RobotInstructionCondition.h"
#include "TrajectoryPipelineTypes.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <json.hpp>

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
	ARC,
	WAIT,
	IF,
	WHILE,
	SET_DO,
	SET_AO,
	PathPlan,
	DeviceAxis
};

enum class ROBOT_SCENE_API Category
{
	Motion = 0,
	Logic,
	Planning
};

enum class ROBOT_SCENE_API PathPlanPhase
{
	Draft = 0,
	RawReady,
	Applied
};

ROBOT_SCENE_API bool isPathPlanType(Type t);
ROBOT_SCENE_API bool isPlanningCategory(Category c);

ROBOT_SCENE_API Category categoryForType(Type t);
ROBOT_SCENE_API std::string typeToString(Type t);
ROBOT_SCENE_API bool typeFromString(const std::string& s, Type& out);

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
	void setType(Type t)
	{
		m_type = t;
		m_category = categoryForType(t);
	}

	Category category() const { return m_category; }

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

	virtual bool hasMotionAxisConfigurationProperty() const { return false; }
	virtual MotionAxisConfiguration motionAxisConfiguration() const { return {}; }
	virtual void setMotionAxisConfiguration(const MotionAxisConfiguration& cfg) { (void)cfg; }

	virtual bool hasBlendRadiusProperty() const { return false; }
	virtual double blendRadius() const { return 0.0; }
	virtual void setBlendRadius(double v) { (void)v; }

	virtual bool hasViaPoseProperty() const { return false; }
	virtual Vec3 viaPose() const { return Vec3{}; }
	virtual void setViaPose(const Vec3& p) { (void)p; }

	virtual bool hasViaEulerProperty() const { return false; }
	virtual Vec3 viaEulerDeg() const { return Vec3{}; }
	virtual void setViaEulerDeg(const Vec3& e) { (void)e; }

	virtual bool hasDurationProperty() const { return false; }
	virtual double durationSec() const { return 0.0; }
	virtual void setDurationSec(double v) { (void)v; }

	virtual bool hasConditionProperty() const { return false; }
	virtual const Condition& condition() const;
	virtual void setCondition(const Condition& c) { (void)c; }

	virtual bool hasIoPortProperty() const { return false; }
	virtual int ioPort() const { return 0; }
	virtual void setIoPort(int p) { (void)p; }

	virtual bool hasIoSignalNameProperty() const { return false; }
	virtual const std::string& ioSignalName() const;
	virtual void setIoSignalName(const std::string& name) { (void)name; }

	virtual bool hasIoValueProperty() const { return false; }
	virtual bool ioBoolValue() const { return false; }
	virtual void setIoBoolValue(bool v) { (void)v; }

	virtual double ioAnalogValue() const { return 0.0; }
	virtual void setIoAnalogValue(double v) { (void)v; }

	virtual bool hasDeviceAxisProperty() const { return false; }
	virtual const std::string& deviceBackendId() const;
	virtual void setDeviceBackendId(const std::string& id) { (void)id; }
	virtual int deviceAxisIndex() const { return 0; }
	virtual void setDeviceAxisIndex(int index) { (void)index; }
	virtual double deviceAxisTargetQ() const { return 0.0; }
	virtual void setDeviceAxisTargetQ(double q) { (void)q; }

	virtual const std::vector<std::shared_ptr<Base>>& nestedSteps() const;
	virtual const std::vector<std::shared_ptr<Base>>& elseSteps() const;

	nlohmann::json snapshotPropertyRows() const;
	bool applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg);

	const std::unordered_map<std::string, std::string>& extensionProperties() const { return m_extensionProperties; }
	void setExtensionProperty(const std::string& key, const std::string& value) { m_extensionProperties[key] = value; }
	void eraseExtensionProperty(const std::string& key) { (void)m_extensionProperties.erase(key); }

protected:
	void addAttribute(const std::shared_ptr<AttributeBase>& attr);

private:
	static const Condition s_emptyCondition;
	static const std::string s_emptyString;

	std::string m_id;
	std::string m_controllerId;
	std::string m_name;
	Type m_type = Type::PTP;
	Category m_category = Category::Motion;
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
	std::string axisConfig() const override { return m_axisConfiguration.preset; }
	void setAxisConfig(const std::string& v) override;

	bool hasMotionAxisConfigurationProperty() const override { return true; }
	MotionAxisConfiguration motionAxisConfiguration() const override { return m_axisConfiguration; }
	void setMotionAxisConfiguration(const MotionAxisConfiguration& cfg) override { m_axisConfiguration = cfg; }

private:
	Vec3 m_pose{};
	Vec3 m_eulerDeg{};
	double m_speed = 100.0;
	double m_accel = 100.0;
	MotionAxisConfiguration m_axisConfiguration{};
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

	bool hasAxisConfigProperty() const override { return true; }
	std::string axisConfig() const override { return m_axisConfiguration.preset; }
	void setAxisConfig(const std::string& v) override;

	bool hasMotionAxisConfigurationProperty() const override { return true; }
	MotionAxisConfiguration motionAxisConfiguration() const override { return m_axisConfiguration; }
	void setMotionAxisConfiguration(const MotionAxisConfiguration& cfg) override { m_axisConfiguration = cfg; }

private:
	Vec3 m_pose{};
	Vec3 m_eulerDeg{};
	double m_tcpSpeed = 200.0;
	double m_tcpAccel = 200.0;
	double m_blendRadius = 0.0;
	MotionAxisConfiguration m_axisConfiguration{};
};

class ROBOT_SCENE_API ArcInstruction final : public Base
{
public:
	ArcInstruction();

	bool hasPoseProperty() const override { return true; }
	Vec3 pose() const override { return m_pose; }
	void setPose(const Vec3& p) override { m_pose = p; }

	bool hasEulerProperty() const override { return true; }
	Vec3 eulerDeg() const override { return m_eulerDeg; }
	void setEulerDeg(const Vec3& e) override { m_eulerDeg = e; }

	bool hasViaPoseProperty() const override { return true; }
	Vec3 viaPose() const override { return m_viaPose; }
	void setViaPose(const Vec3& p) override { m_viaPose = p; }

	bool hasViaEulerProperty() const override { return true; }
	Vec3 viaEulerDeg() const override { return m_viaEulerDeg; }
	void setViaEulerDeg(const Vec3& e) override { m_viaEulerDeg = e; }

	bool hasSpeedProperty() const override { return true; }
	double speed() const override { return m_tcpSpeed; }
	void setSpeed(double v) override { m_tcpSpeed = v; }

	bool hasAccelProperty() const override { return true; }
	double accel() const override { return m_tcpAccel; }
	void setAccel(double v) override { m_tcpAccel = v; }

	bool hasBlendRadiusProperty() const override { return true; }
	double blendRadius() const override { return m_blendRadius; }
	void setBlendRadius(double v) override { m_blendRadius = v; }

	bool hasAxisConfigProperty() const override { return true; }
	std::string axisConfig() const override { return m_axisConfiguration.preset; }
	void setAxisConfig(const std::string& v) override;

	bool hasMotionAxisConfigurationProperty() const override { return true; }
	MotionAxisConfiguration motionAxisConfiguration() const override { return m_axisConfiguration; }
	void setMotionAxisConfiguration(const MotionAxisConfiguration& cfg) override { m_axisConfiguration = cfg; }

private:
	Vec3 m_pose{};
	Vec3 m_eulerDeg{};
	Vec3 m_viaPose{};
	Vec3 m_viaEulerDeg{};
	double m_tcpSpeed = 200.0;
	double m_tcpAccel = 200.0;
	double m_blendRadius = 0.0;
	MotionAxisConfiguration m_axisConfiguration{};
};

class ROBOT_SCENE_API WaitInstruction final : public Base
{
public:
	WaitInstruction();

	bool hasDurationProperty() const override { return true; }
	double durationSec() const override { return m_durationSec; }
	void setDurationSec(double v) override { m_durationSec = v; }

	/// Always=纯延时；Io=等到信号（durationSec>0 为超时秒，0=一直等）
	bool hasConditionProperty() const override { return true; }
	const Condition& condition() const override { return m_condition; }
	void setCondition(const Condition& c) override { m_condition = c; }

private:
	double m_durationSec = 0.0;
	Condition m_condition{};
};

class ROBOT_SCENE_API IfInstruction final : public Base
{
public:
	IfInstruction();

	bool hasConditionProperty() const override { return true; }
	const Condition& condition() const override { return m_condition; }
	void setCondition(const Condition& c) override { m_condition = c; }

	const std::vector<std::shared_ptr<Base>>& nestedSteps() const override { return m_thenSteps; }
	const std::vector<std::shared_ptr<Base>>& elseSteps() const override { return m_elseSteps; }

	std::vector<std::shared_ptr<Base>>& thenSteps() { return m_thenSteps; }
	std::vector<std::shared_ptr<Base>>& elseStepsMut() { return m_elseSteps; }

private:
	Condition m_condition{};
	std::vector<std::shared_ptr<Base>> m_thenSteps;
	std::vector<std::shared_ptr<Base>> m_elseSteps;
};

class ROBOT_SCENE_API WhileInstruction final : public Base
{
public:
	WhileInstruction();

	bool hasConditionProperty() const override { return true; }
	const Condition& condition() const override { return m_condition; }
	void setCondition(const Condition& c) override { m_condition = c; }

	const std::vector<std::shared_ptr<Base>>& nestedSteps() const override { return m_bodySteps; }

	std::vector<std::shared_ptr<Base>>& bodySteps() { return m_bodySteps; }

private:
	Condition m_condition{};
	std::vector<std::shared_ptr<Base>> m_bodySteps;
};

class ROBOT_SCENE_API SetDigitalOutputInstruction final : public Base
{
public:
	SetDigitalOutputInstruction();

	bool hasIoPortProperty() const override { return true; }
	int ioPort() const override { return m_port; }
	void setIoPort(int p) override { m_port = p; }

	bool hasIoSignalNameProperty() const override { return true; }
	const std::string& ioSignalName() const override { return m_signalName; }
	void setIoSignalName(const std::string& name) override { m_signalName = name; }

	bool hasIoValueProperty() const override { return true; }
	bool ioBoolValue() const override { return m_value; }
	void setIoBoolValue(bool v) override { m_value = v; }

private:
	int m_port = 0;
	bool m_value = false;
	std::string m_signalName;
};

class ROBOT_SCENE_API SetAnalogOutputInstruction final : public Base
{
public:
	SetAnalogOutputInstruction();

	bool hasIoPortProperty() const override { return true; }
	int ioPort() const override { return m_port; }
	void setIoPort(int p) override { m_port = p; }

	bool hasIoSignalNameProperty() const override { return true; }
	const std::string& ioSignalName() const override { return m_signalName; }
	void setIoSignalName(const std::string& name) override { m_signalName = name; }

	double ioAnalogValue() const override { return m_value; }
	void setIoAnalogValue(double v) override { m_value = v; }

private:
	int m_port = 0;
	double m_value = 0.0;
	std::string m_signalName;
};

class ROBOT_SCENE_API DeviceAxisInstruction final : public Base
{
public:
	DeviceAxisInstruction();

	bool hasDurationProperty() const override { return true; }
	double durationSec() const override { return m_durationSec; }
	void setDurationSec(double v) override { m_durationSec = v; }

	bool hasDeviceAxisProperty() const override { return true; }
	const std::string& deviceBackendId() const override { return m_deviceBackendId; }
	void setDeviceBackendId(const std::string& id) override { m_deviceBackendId = id; }
	int deviceAxisIndex() const override { return m_axisIndex; }
	void setDeviceAxisIndex(int index) override { m_axisIndex = index; }
	double deviceAxisTargetQ() const override { return m_targetQ; }
	void setDeviceAxisTargetQ(double q) override { m_targetQ = q; }

private:
	std::string m_deviceBackendId;
	int m_axisIndex = 0;
	double m_targetQ = 0.0;
	double m_durationSec = 0.0;
};

class ROBOT_SCENE_API PathPlanInstruction final : public Base
{
public:
	PathPlanInstruction();

	PathPlanPhase phase() const { return m_phase; }
	void setPhase(PathPlanPhase p) { m_phase = p; }

	const std::string& sourceFeatureJson() const { return m_sourceFeatureJson; }
	void setSourceFeatureJson(const std::string& json) { m_sourceFeatureJson = json; }
	void setSourceFeatureJson(std::string&& json) { m_sourceFeatureJson = std::move(json); }

	const std::vector<TrajectoryOpDescriptor>& pipeline() const { return m_pipeline; }
	std::vector<TrajectoryOpDescriptor>& pipelineMut() { return m_pipeline; }
	void setPipeline(std::vector<TrajectoryOpDescriptor> ops) { m_pipeline = std::move(ops); }

	const std::vector<TrajectoryOpDescriptor>& appliedHistory() const { return m_appliedHistory; }
	std::vector<TrajectoryOpDescriptor>& appliedHistoryMut() { return m_appliedHistory; }

	const std::string& outputGroupId() const { return m_outputGroupId; }
	void setOutputGroupId(const std::string& id) { m_outputGroupId = id; }

	const std::string& rawTrajectoryKey() const { return m_rawTrajectoryKey; }
	void setRawTrajectoryKey(const std::string& key) { m_rawTrajectoryKey = key; }

	int rawRevision() const { return m_rawRevision; }
	void setRawRevision(int r) { m_rawRevision = r; }
	void bumpRawRevision() { ++m_rawRevision; }

private:
	PathPlanPhase m_phase = PathPlanPhase::Draft;
	std::string m_sourceFeatureJson;
	std::vector<TrajectoryOpDescriptor> m_pipeline;
	std::vector<TrajectoryOpDescriptor> m_appliedHistory;
	std::string m_outputGroupId;
	std::string m_rawTrajectoryKey;
	int m_rawRevision = 0;
};

ROBOT_SCENE_API PathPlanInstruction* asPathPlan(Base& ins);
ROBOT_SCENE_API const PathPlanInstruction* asPathPlan(const Base& ins);
ROBOT_SCENE_API DeviceAxisInstruction* asDeviceAxis(Base& ins);
ROBOT_SCENE_API const DeviceAxisInstruction* asDeviceAxis(const Base& ins);

ROBOT_SCENE_API std::string makeInstructionId();

} // namespace RobotInstruction

#endif // ROBOTSCENE_ROBOTINSTRUCTIONMODEL_H
