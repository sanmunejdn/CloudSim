#ifndef DATA_CUSTOMDEVICEROBOTMOUNTCOMPONENT_H
#define DATA_CUSTOMDEVICEROBOTMOUNTCOMPONENT_H

/// @file CustomDeviceRobotMountComponent.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 自定义设备挂到机器人法兰：安装 Frame 随设备运动并与 TCP 重合

#include "data_global.h"

#include "BackendComponent.h"
#include "BackendFollowMath.h"

#include <array>
#include <mutex>
#include <string>

#include <json.hpp>

class CustomDeviceBackendData;

/// 设备根相对法兰 link 系的冻结偏移（列主序 4×4）
class DATA_EXPORT CustomDeviceRobotMountComponent : public IBackendComponent
{
public:
	static constexpr const char* typeKeyStatic() { return "CustomDeviceRobotMount"; }

	std::string componentType() const override { return typeKeyStatic(); }

	bool enabled() const;
	void setEnabled(bool on);

	// 按值返回：锁内拷贝，避免锁释放后悬挂引用
	std::string robotSceneBackendId() const;
	void setRobotSceneBackendId(std::string id);

	std::string flangeLinkName() const;
	void setFlangeLinkName(std::string name);

	std::string mountFrameBackendId() const;
	void setMountFrameBackendId(std::string id);

	/// 挂载时解析的法兰 link mesh backendId（FK 后读 worldMatrix）
	std::string flangeBackendId() const;
	void setFlangeBackendId(std::string id);

	BackendMat4 tFlangeDevice() const;
	void setTFlangeDevice(const BackendMat4& m);

	/// 安装坐标系在设备 W0 下的位姿：T_device_mountFrame（挂载时冻结）
	BackendMat4 frameInDeviceW0() const;
	void setFrameInDeviceW0(const BackendMat4& m);

	/// 挂载时刻机器人激活工具系 T_flange_tool
	BackendMat4 toolFrameInFlange() const;
	void setToolFrameInFlange(const BackendMat4& m);

	bool alignsMountFrameToTcp() const;

	void setAlignMountFrameToTcp(bool on);

	void collectReferencedBackendIds(std::vector<std::string>& out) const override;

	void writeJson(nlohmann::json& out) const;
	void readJson(const nlohmann::json& in);

	/// 挂载时刻：T_flange_device = inv(T_flange_w) * T_device_w
	static bool computeTFlangeDeviceFromWorldPoses(const BackendMat4& flangeWorld, const BackendMat4& deviceWorld,
												   BackendMat4& outTFlangeDevice);

	/// T_mount_in_device = inv(T_device_w) * T_mount_frame_w
	static bool computeFrameInDeviceFromWorldPoses(const BackendMat4& deviceWorld, const BackendMat4& frameWorld,
												 BackendMat4& outFrameInDevice);

	/// W_device = T_tcp_w * inv(T_mount_in_device)，其中 T_tcp_w = T_flange_w * T_tool
	static bool computeEffectiveDeviceWorldForFrameTcpAlign(const BackendMat4& flangeWorld,
															const BackendMat4& toolFrameInFlange,
															const BackendMat4& frameInDeviceW0,
															BackendMat4& outDeviceWorld);

	static std::shared_ptr<CustomDeviceRobotMountComponent> mountOf(CustomDeviceBackendData& device);
	static std::shared_ptr<const CustomDeviceRobotMountComponent> mountOf(const CustomDeviceBackendData& device);
	static CustomDeviceRobotMountComponent& ensureMount(CustomDeviceBackendData& device);

private:
	mutable std::mutex m_mutex;
	bool m_enabled = false;
	bool m_alignMountFrameToTcp = false;
	std::string m_robotSceneBackendId;
	std::string m_flangeLinkName;
	std::string m_mountFrameBackendId;
	std::string m_flangeBackendId;
	BackendMat4 m_tFlangeDevice = BackendMat4::identity();
	BackendMat4 m_frameInDeviceW0 = BackendMat4::identity();
	BackendMat4 m_toolFrameInFlange = BackendMat4::identity();
};

#endif // DATA_CUSTOMDEVICEROBOTMOUNTCOMPONENT_H
