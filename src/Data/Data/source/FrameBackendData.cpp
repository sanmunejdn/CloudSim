/// @file FrameBackendData.cpp
/// @brief Frame 后端数据

#include "pch.h"

#include "BackendTypeIdentity.h"
#include "FrameBackendData.h"
#include "RunLogger.h"

FrameBackendData::FrameBackendData()
{
	setName(backend_type::kCatalogCoordinateFrame);
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

const std::vector<BackendPropertyBinding>& FrameBackendData::extraPropertyBindings() const
{
	return backend_property_binding_extras::axisLengthExtras();
}

void FrameBackendData::saveDerivedJson(nlohmann::json& out) const
{
	// 无文件几何；占位使工程加载走内嵌路径，不被「无 geometry/源路径」门禁丢弃
	out["geometry"] = nlohmann::json{{"kind", "frame"}};
	out["axisLengthMm"] = m_axisLengthMm;
}

bool FrameBackendData::loadDerivedJson(const nlohmann::json& in, std::string* errMsg)
{
	(void)errMsg;
	if (in.contains("axisLengthMm") && in["axisLengthMm"].is_number())
	{
		setAxisLengthMm(static_cast<float>(in["axisLengthMm"].get<double>()));
	}
	return true;
}
