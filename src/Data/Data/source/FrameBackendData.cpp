/// @file FrameBackendData.cpp
/// @brief Frame 后端数据

#include "pch.h"

#include "../../PropertyCore/inc/PropertyAttribute.h"
#include "BackendObjectAttribute.h"
#include "BackendTypeIdentity.h"
#include "FrameBackendData.h"
#include "RunLogger.h"

FrameBackendData::FrameBackendData()
{
	setName(backend_type::kCatalogCoordinateFrame);
	appendStandardAttributesForCapabilities(*this, m_attributes);
}

std::string FrameBackendData::className() const
{
	return backend_type::kClassFrame;
}

bool FrameBackendData::hasGeometry() const
{
	return true;
}

BackendBoundingBox FrameBackendData::geometryBounds() const
{
	const double half = static_cast<double>(m_axisLengthMm);
	BackendBoundingBox box{};
	box.min = {-half, -half, -half};
	box.max = {half, half, half};
	box.valid = true;
	return box;
}

std::size_t FrameBackendData::geometryElementCount() const
{
	return 1U;
}

void FrameBackendData::clearGeometry()
{
}

void FrameBackendData::setAxisLengthMm(const float mm)
{
	if (mm > 0.0f)
	{
		m_axisLengthMm = mm;
		bumpGeometryRevision();
		return;
	}
	RunLogger::warn("[FrameBackendData] setAxisLengthMm: ignore non-positive value.");
}

nlohmann::json FrameBackendData::snapshotPropertyRows(const BackendDataManager* mgr) const
{
	// 构造期已挂 pose/rotation attribute，须走 Pipeline，否则面板只有 pose.frame
	nlohmann::json rows = BackendDataBase::snapshotPropertyRows(mgr);
	property_core::PropertyPipeline<BackendDataBase, BackendAttributeBase>::appendRows(m_attributes, *this, rows);
	return rows;
}

bool FrameBackendData::applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg,
										   const BackendDataManager* mgr)
{
	if (property_core::PropertyPipeline<BackendDataBase, BackendAttributeBase>::apply(m_attributes, *this, key, value,
																					  errMsg))
	{
		return true;
	}
	return BackendDataBase::applyPropertyChange(key, value, errMsg, mgr);
}

void FrameBackendData::saveDerivedJson(nlohmann::json& out) const
{
	// 无文件几何；占位使工程加载走内嵌路径，不被「无 geometry/源路径」门禁丢弃
	out["geometry"] = nlohmann::json{{"kind", "frame"}};
	out["axisLengthMm"] = m_axisLengthMm;
}

bool FrameBackendData::loadDerivedJson(const nlohmann::json& in, std::string* errMsg)
{
	// C2: 与 brep 的严格检查不统一；frame 的 kind 错了也无实际危害，故仅注释说明
	(void)errMsg;
	if (in.contains("axisLengthMm") && in["axisLengthMm"].is_number())
	{
		setAxisLengthMm(static_cast<float>(in["axisLengthMm"].get<double>()));
	}
	return true;
}
