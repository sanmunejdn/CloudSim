/// @file CustomDeviceRobotMountComponent.cpp
/// @brief 自定义设备机器人法兰挂载组件

#include "CustomDeviceRobotMountComponent.h"

#include "CustomDeviceBackendData.h"

#include <cstring>

namespace
{
void writeMat4Json(const BackendMat4& m, nlohmann::json& out)
{
	out = nlohmann::json::array();
	for (int i = 0; i < 16; ++i)
	{
		out.push_back(m.v[i]);
	}
}

bool readMat4Json(const nlohmann::json& in, BackendMat4& out)
{
	if (!in.is_array() || in.size() < 16)
	{
		return false;
	}
	for (int i = 0; i < 16; ++i)
	{
		if (!in[i].is_number())
		{
			return false;
		}
		out.v[i] = in[i].get<double>();
	}
	return true;
}
} // namespace

bool CustomDeviceRobotMountComponent::enabled() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_enabled;
}

void CustomDeviceRobotMountComponent::setEnabled(const bool on)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_enabled = on;
}

const std::string& CustomDeviceRobotMountComponent::robotSceneBackendId() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_robotSceneBackendId;
}

void CustomDeviceRobotMountComponent::setRobotSceneBackendId(std::string id)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_robotSceneBackendId = std::move(id);
}

const std::string& CustomDeviceRobotMountComponent::flangeLinkName() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_flangeLinkName;
}

void CustomDeviceRobotMountComponent::setFlangeLinkName(std::string name)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_flangeLinkName = std::move(name);
}

const std::string& CustomDeviceRobotMountComponent::mountFrameBackendId() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_mountFrameBackendId;
}

void CustomDeviceRobotMountComponent::setMountFrameBackendId(std::string id)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_mountFrameBackendId = std::move(id);
}

const std::string& CustomDeviceRobotMountComponent::flangeBackendId() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_flangeBackendId;
}

void CustomDeviceRobotMountComponent::setFlangeBackendId(std::string id)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_flangeBackendId = std::move(id);
}

const BackendMat4& CustomDeviceRobotMountComponent::tFlangeDevice() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_tFlangeDevice;
}

void CustomDeviceRobotMountComponent::setTFlangeDevice(const BackendMat4& m)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_tFlangeDevice = m;
}

const BackendMat4& CustomDeviceRobotMountComponent::frameInDeviceW0() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_frameInDeviceW0;
}

void CustomDeviceRobotMountComponent::setFrameInDeviceW0(const BackendMat4& m)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_frameInDeviceW0 = m;
}

const BackendMat4& CustomDeviceRobotMountComponent::toolFrameInFlange() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_toolFrameInFlange;
}

void CustomDeviceRobotMountComponent::setToolFrameInFlange(const BackendMat4& m)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_toolFrameInFlange = m;
}

bool CustomDeviceRobotMountComponent::alignsMountFrameToTcp() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_enabled && m_alignMountFrameToTcp && !m_mountFrameBackendId.empty();
}

void CustomDeviceRobotMountComponent::setAlignMountFrameToTcp(const bool on)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_alignMountFrameToTcp = on;
}

bool CustomDeviceRobotMountComponent::computeTFlangeDeviceFromWorldPoses(const BackendMat4& flangeWorld,
																	  const BackendMat4& deviceWorld,
																	  BackendMat4& outTFlangeDevice)
{
	BackendMat4 invFlange{};
	if (!backend_mat4_invert_rigid(flangeWorld, invFlange))
	{
		return false;
	}
	return backend_mat4_multiply(invFlange, deviceWorld, outTFlangeDevice);
}

bool CustomDeviceRobotMountComponent::computeFrameInDeviceFromWorldPoses(const BackendMat4& deviceWorld,
																		const BackendMat4& frameWorld,
																		BackendMat4& outFrameInDevice)
{
	BackendMat4 invDevice{};
	if (!backend_mat4_invert_rigid(deviceWorld, invDevice))
	{
		return false;
	}
	return backend_mat4_multiply(invDevice, frameWorld, outFrameInDevice);
}

bool CustomDeviceRobotMountComponent::computeEffectiveDeviceWorldForFrameTcpAlign(
	const BackendMat4& flangeWorld, const BackendMat4& toolFrameInFlange, const BackendMat4& frameInDeviceW0,
	BackendMat4& outDeviceWorld)
{
	BackendMat4 tcpWorld{};
	if (!backend_mat4_multiply(flangeWorld, toolFrameInFlange, tcpWorld))
	{
		return false;
	}
	BackendMat4 invFrameInDevice{};
	if (!backend_mat4_invert_rigid(frameInDeviceW0, invFrameInDevice))
	{
		return false;
	}
	return backend_mat4_multiply(tcpWorld, invFrameInDevice, outDeviceWorld);
}

std::shared_ptr<CustomDeviceRobotMountComponent> CustomDeviceRobotMountComponent::mountOf(
	CustomDeviceBackendData& device)
{
	return device.getComponent<CustomDeviceRobotMountComponent>();
}

std::shared_ptr<const CustomDeviceRobotMountComponent> CustomDeviceRobotMountComponent::mountOf(
	const CustomDeviceBackendData& device)
{
	return device.getComponent<CustomDeviceRobotMountComponent>();
}

CustomDeviceRobotMountComponent& CustomDeviceRobotMountComponent::ensureMount(CustomDeviceBackendData& device)
{
	if (auto existing = mountOf(device))
	{
		return *existing;
	}
	auto mount = std::make_shared<CustomDeviceRobotMountComponent>();
	device.addComponent(mount);
	return *mountOf(device);
}

void CustomDeviceRobotMountComponent::writeJson(nlohmann::json& out) const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	out = nlohmann::json::object();
	out["enabled"] = m_enabled;
	out["alignMountFrameToTcp"] = m_alignMountFrameToTcp;
	out["robotSceneBackendId"] = m_robotSceneBackendId;
	out["flangeLinkName"] = m_flangeLinkName;
	out["mountFrameBackendId"] = m_mountFrameBackendId;
	out["flangeBackendId"] = m_flangeBackendId;
	writeMat4Json(m_tFlangeDevice, out["T_flange_device"]);
	writeMat4Json(m_frameInDeviceW0, out["frameInDeviceW0"]);
	writeMat4Json(m_toolFrameInFlange, out["toolFrameInFlange"]);
}

void CustomDeviceRobotMountComponent::readJson(const nlohmann::json& in)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_enabled = in.value("enabled", false);
	m_alignMountFrameToTcp = in.value("alignMountFrameToTcp", false);
	if (in.contains("robotSceneBackendId") && in["robotSceneBackendId"].is_string())
	{
		m_robotSceneBackendId = in["robotSceneBackendId"].get<std::string>();
	}
	if (in.contains("flangeLinkName") && in["flangeLinkName"].is_string())
	{
		m_flangeLinkName = in["flangeLinkName"].get<std::string>();
	}
	if (in.contains("mountFrameBackendId") && in["mountFrameBackendId"].is_string())
	{
		m_mountFrameBackendId = in["mountFrameBackendId"].get<std::string>();
	}
	if (in.contains("flangeBackendId") && in["flangeBackendId"].is_string())
	{
		m_flangeBackendId = in["flangeBackendId"].get<std::string>();
	}
	if (in.contains("T_flange_device"))
	{
		(void)readMat4Json(in["T_flange_device"], m_tFlangeDevice);
	}
	if (in.contains("frameInDeviceW0"))
	{
		(void)readMat4Json(in["frameInDeviceW0"], m_frameInDeviceW0);
	}
	if (in.contains("toolFrameInFlange"))
	{
		(void)readMat4Json(in["toolFrameInFlange"], m_toolFrameInFlange);
	}
}
