#ifndef DATA_FRAMEBACKENDDATA_H
#define DATA_FRAMEBACKENDDATA_H

/// @file FrameBackendData.h
/// @brief 命名坐标系后端：仅世界位姿 + 轴长，无实体网格

#include "BackendDataBase.h"

/// 命名坐标系后端：仅世界位姿 + 轴长，无实体网格
class DATA_EXPORT FrameBackendData : public BackendDataBase
{
public:
	static constexpr float kDefaultAxisLengthMm = 100.0f;

	FrameBackendData();
	~FrameBackendData() override = default;

	std::string className() const override;
	bool hasGeometry() const override;
	BackendBoundingBox geometryBounds() const override;
	std::size_t geometryElementCount() const override;
	void clearGeometry() override;

	bool hasPoseProperty() const override { return true; }
	bool hasRotationProperty() const override { return true; }

	float axisLengthMm() const { return m_axisLengthMm; }
	void setAxisLengthMm(float mm);

	nlohmann::json snapshotPropertyRows(const BackendDataManager* mgr = nullptr) const override;
	bool applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg,
							 const BackendDataManager* mgr = nullptr) override;

private:
	void saveDerivedJson(nlohmann::json& out) const override;
	bool loadDerivedJson(const nlohmann::json& in, std::string* errMsg) override;

	float m_axisLengthMm = kDefaultAxisLengthMm;
};

#endif // DATA_FRAMEBACKENDDATA_H
