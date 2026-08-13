#ifndef DATA_CUSTOMDEVICEBACKENDDATA_H
#define DATA_CUSTOMDEVICEBACKENDDATA_H

/// @file CustomDeviceBackendData.h
/// @brief 自定义设备聚合根：位姿 + 单/多轴运动配置；几何在子件

#include "BackendDataBase.h"

#include <string>
#include <vector>

/// 设备局部运动类型（与外轴语义对齐，Data 层自包含以免依赖 RobotScene）
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
};

struct DATA_EXPORT CustomDeviceAxisConfigSet
{
	std::vector<CustomDeviceAxisConfig> axes;
};

DATA_EXPORT CustomDeviceAxisConfig makeDefaultCustomDeviceTranslateAxis();
DATA_EXPORT CustomDeviceAxisConfig makeDefaultCustomDeviceRotateAxis();
DATA_EXPORT void normalizeCustomDeviceAxisConfig(CustomDeviceAxisConfig& cfg);
DATA_EXPORT void writeCustomDeviceAxisConfigSetToJson(const CustomDeviceAxisConfigSet& set, nlohmann::json& out);
DATA_EXPORT bool readCustomDeviceAxisConfigSetFromJson(const nlohmann::json& in, CustomDeviceAxisConfigSet& out);

/// 自定义设备聚合根：位姿 + 运动；子件经父子/Follow 挂接
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

	const std::vector<double>& qValues() const { return m_q; }
	void setQValues(const std::vector<double>& q);
	void ensureQSize();

	/// q=0 时设备世界矩阵（列主序）
	const BackendMat4& baseWorldW0() const { return m_baseWorldW0; }
	void setBaseWorldW0(const BackendMat4& w0);
	void captureBaseWorldW0FromCurrentWorld();

private:
	void saveDerivedJson(nlohmann::json& out) const override;
	bool loadDerivedJson(const nlohmann::json& in, std::string* errMsg) override;

	float m_axisLengthMm = kDefaultAxisLengthMm;
	CustomDeviceAxisConfigSet m_axes;
	std::vector<double> m_q;
	BackendMat4 m_baseWorldW0{};
	bool m_baseWorldW0Valid = false;
};

#endif // DATA_CUSTOMDEVICEBACKENDDATA_H
