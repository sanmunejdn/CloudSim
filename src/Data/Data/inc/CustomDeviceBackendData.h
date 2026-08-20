#ifndef DATA_CUSTOMDEVICEBACKENDDATA_H
#define DATA_CUSTOMDEVICEBACKENDDATA_H

/// @file CustomDeviceBackendData.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 自定义设备聚合根：Link/Joint 图为唯一持久化源；axes/q 为运行时投影

#include "BackendDataBase.h"

#include <string>
#include <vector>

enum class CustomDeviceMotionType : int
{
	Translate = 0,
	Rotate = 1
};

struct DATA_EXPORT CustomDeviceAxisConfig
{
	bool enabled = true;
	std::string displayName = "Axis";
	std::string jointName = "device_joint";
	CustomDeviceMotionType motionType = CustomDeviceMotionType::Translate;
	double lower = 0.0;
	double upper = 1000.0;
	double home = 0.0;
	double axis[3]{1.0, 0.0, 0.0};
	double originMm[3]{0.0, 0.0, 0.0};
	/// 非空：旋转中心取该 Backend 原点（Frame 或模型几何），变换到父连杆局部
	std::string motionCenterFrameBackendId;
};

struct DATA_EXPORT CustomDeviceAxisConfigSet
{
	std::vector<CustomDeviceAxisConfig> axes;
};

/// 画布上的刚体块（绑定场景几何）
struct DATA_EXPORT CustomDeviceLink
{
	std::string id;
	std::string displayName;
	std::string geometryBackendId;
	bool fixed = false;
	double canvasX = 0.0;
	double canvasY = 0.0;
	/// 相对设备 W0 的静止位姿（列主序）；空则单位阵
	double restInDeviceW0[16]{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
};

/// 父子运动副；Rest = inv(Wp)*Wc @ q=0
struct DATA_EXPORT CustomDeviceJoint
{
	std::string id;
	std::string parentLinkId;
	std::string childLinkId;
	CustomDeviceAxisConfig motion;
	double parentToChildRest[16]{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
};

/// 设备指令：命名姿态（全轴 q 快照）
struct DATA_EXPORT CustomDeviceNamedPose
{
	std::string id;
	std::string name;
	std::vector<double> q;
};

/// DI 上升沿 → 姿态；signalName 为本设备信号表 DI 名
struct DATA_EXPORT CustomDevicePoseSignalBinding
{
	std::string id;
	std::string signalName;
	std::string poseId;
	double durationSec = 1.0;
	bool enabled = true;
};

DATA_EXPORT CustomDeviceAxisConfig makeDefaultCustomDeviceTranslateAxis();
DATA_EXPORT CustomDeviceAxisConfig makeDefaultCustomDeviceRotateAxis();
DATA_EXPORT void normalizeCustomDeviceAxisConfig(CustomDeviceAxisConfig& cfg);
DATA_EXPORT void writeCustomDeviceAxisConfigSetToJson(const CustomDeviceAxisConfigSet& set, nlohmann::json& out);
DATA_EXPORT bool readCustomDeviceAxisConfigSetFromJson(const nlohmann::json& in, CustomDeviceAxisConfigSet& out);
DATA_EXPORT void writeCustomDeviceLinksToJson(const std::vector<CustomDeviceLink>& links, nlohmann::json& out);
DATA_EXPORT bool readCustomDeviceLinksFromJson(const nlohmann::json& in, std::vector<CustomDeviceLink>& out);
DATA_EXPORT void writeCustomDeviceJointsToJson(const std::vector<CustomDeviceJoint>& joints, nlohmann::json& out);
DATA_EXPORT bool readCustomDeviceJointsFromJson(const nlohmann::json& in, std::vector<CustomDeviceJoint>& out);
DATA_EXPORT void writeCustomDeviceNamedPosesToJson(const std::vector<CustomDeviceNamedPose>& poses, nlohmann::json& out);
DATA_EXPORT bool readCustomDeviceNamedPosesFromJson(const nlohmann::json& in, std::vector<CustomDeviceNamedPose>& out);
DATA_EXPORT void writeCustomDevicePoseSignalBindingsToJson(const std::vector<CustomDevicePoseSignalBinding>& bindings,
														  nlohmann::json& out);
DATA_EXPORT bool readCustomDevicePoseSignalBindingsFromJson(const nlohmann::json& in,
														   std::vector<CustomDevicePoseSignalBinding>& out);
DATA_EXPORT std::string makeCustomDevicePoseId();
DATA_EXPORT std::string makeCustomDevicePoseBindingId();


class DATA_EXPORT CustomDeviceBackendData : public BackendDataBase
{
public:
	static constexpr float kDefaultAxisLengthMm = 80.0f;

	CustomDeviceBackendData();
	~CustomDeviceBackendData() override = default;

	std::string className() const override;
	bool hasGeometry() const override;
	BackendBoundingBox geometryBounds() const override;
	std::size_t geometryElementCount() const override;
	void clearGeometry() override;

	bool hasPoseProperty() const override { return true; }
	bool hasRotationProperty() const override { return true; }

	float axisLengthMm() const { return m_axisLengthMm; }
	void setAxisLengthMm(float mm);

	const CustomDeviceAxisConfigSet& axes() const { return m_axes; }
	void setAxes(const CustomDeviceAxisConfigSet& axes);

	const std::vector<CustomDeviceLink>& links() const { return m_links; }
	void setLinks(const std::vector<CustomDeviceLink>& links);
	const std::vector<CustomDeviceJoint>& joints() const { return m_joints; }
	void setJoints(const std::vector<CustomDeviceJoint>& joints);
	/// Joint → 运行时 axes/q 投影（供轴控）
	void syncAxesFromJoints();
	bool usesLinkJointGraph() const { return !m_joints.empty() && !m_links.empty(); }

	const std::vector<double>& qValues() const { return m_q; }
	void setQValues(const std::vector<double>& q);
	void ensureQSize();

	const BackendMat4& baseWorldW0() const { return m_baseWorldW0; }
	void setBaseWorldW0(const BackendMat4& w0);
	void captureBaseWorldW0FromCurrentWorld();

	const std::vector<CustomDeviceNamedPose>& namedPoses() const { return m_namedPoses; }
	void setNamedPoses(const std::vector<CustomDeviceNamedPose>& poses);
	const CustomDeviceNamedPose* findNamedPose(const std::string& poseId) const;

	const std::vector<CustomDevicePoseSignalBinding>& poseSignalBindings() const { return m_poseSignalBindings; }
	void setPoseSignalBindings(const std::vector<CustomDevicePoseSignalBinding>& bindings);

	/// 本设备自持 IO（与 NamedSignalTable JSON 同形）
	const nlohmann::json& ioSignalsJson() const { return m_ioSignalsJson; }
	void setIoSignalsJson(nlohmann::json signalsJson);

private:
	void saveDerivedJson(nlohmann::json& out) const override;
	bool loadDerivedJson(const nlohmann::json& in, std::string* errMsg) override;

	float m_axisLengthMm = kDefaultAxisLengthMm;
	CustomDeviceAxisConfigSet m_axes;
	std::vector<CustomDeviceLink> m_links;
	std::vector<CustomDeviceJoint> m_joints;
	std::vector<double> m_q;
	BackendMat4 m_baseWorldW0{};
	bool m_baseWorldW0Valid = false;
	std::vector<CustomDeviceNamedPose> m_namedPoses;
	std::vector<CustomDevicePoseSignalBinding> m_poseSignalBindings;
	nlohmann::json m_ioSignalsJson = nlohmann::json::array();
};

#endif // DATA_CUSTOMDEVICEBACKENDDATA_H
