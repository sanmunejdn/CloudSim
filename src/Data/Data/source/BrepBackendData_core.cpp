/// @file BrepBackendData_core.cpp
/// @brief B-rep 后端核心

#include "pch.h"

#include "../../PropertyCore/inc/PropertyAttribute.h"
#include "BackendObjectAttribute.h"
#include "BackendPropertyRow.h"
#include "BackendSpatial.h"
#include "BrepBackendData.h"

#include <Adapters.h>
#include <ShapeIo.h>

BrepBackendData::BrepBackendData()
{
	setName(backend_type::kClassBrepModel);
	BackendColor c;
	c.r = 0.65f;
	c.g = 0.82f;
	c.b = 0.95f;
	c.a = 1.0f;
	m_color = c;
	appendStandardAttributesForCapabilities(*this, m_attributes);
}

std::string BrepBackendData::className() const
{
	return backend_type::kClassBrepModel;
}

bool BrepBackendData::hasGeometry() const
{
	return !m_shape.isNull();
}

BackendBoundingBox BrepBackendData::geometryBounds() const
{
	return m_bounds;
}

std::size_t BrepBackendData::geometryElementCount() const
{
	return hasGeometry() ? 1U : 0U;
}

void BrepBackendData::clearGeometry()
{
	m_shape = geoalgo::ShapeHandle{};
	m_bounds = BackendBoundingBox{};
	m_faceHighlightColors.clear();
	bumpGeometryRevision();
}

void BrepBackendData::setShape(geoalgo::ShapeHandle shape)
{
	m_shape = std::move(shape);
	recomputeBounds();
	bumpGeometryRevision();
}

void BrepBackendData::shareShapeFrom(const BrepBackendData& other)
{
	m_shape = other.m_shape;
	recomputeBounds();
	bumpGeometryRevision();
}

void BrepBackendData::setColor(const BackendColor& color)
{
	m_color = color;
}

BackendColor BrepBackendData::color() const
{
	return m_color;
}

void BrepBackendData::setFaceHighlightColors(std::unordered_map<int, BackendColor> colorsByFaceIndex)
{
	m_faceHighlightColors = std::move(colorsByFaceIndex);
	// C1: 唯一调用方与 setShape 配对（setShape 内部 bump），现实安全；
	// 未来单独设高亮的新调用方会踩，故补 bump
	bumpGeometryRevision();
}

const std::unordered_map<int, BackendColor>& BrepBackendData::faceHighlightColors() const
{
	return m_faceHighlightColors;
}

void BrepBackendData::clearFaceHighlightColors()
{
	m_faceHighlightColors.clear();
	// C1: 同上，单独 clear 时也需 bump
	bumpGeometryRevision();
}

void BrepBackendData::recomputeBounds()
{
	m_bounds = BackendBoundingBox{};
	const geoalgo::ShapeHandle::BoundsMm b = m_shape.boundingBoxMm();
	if (!b.valid)
	{
		return;
	}
	m_bounds.valid = true;
	m_bounds.min.x = b.minX;
	m_bounds.min.y = b.minY;
	m_bounds.min.z = b.minZ;
	m_bounds.max.x = b.maxX;
	m_bounds.max.y = b.maxY;
	m_bounds.max.z = b.maxZ;
}

bool BrepBackendData::loadFromBrepFile(const std::string& path, std::string* errMsg)
{
	geoalgo::ShapeHandle shape;
	if (!geoalgo::readBrepFile(path, shape, errMsg))
	{
		return false;
	}
	setShape(std::move(shape));
	return true;
}

bool BrepBackendData::writeBrepFile(const std::string& path, std::string* errMsg) const
{
	return geoalgo::writeBrepFile(path, m_shape, errMsg);
}

bool BrepBackendData::writeStepFile(const std::string& path, std::string* errMsg) const
{
	return geoalgo::writeStepFile(path, m_shape, errMsg);
}

geoalgo::ShapeHandle BrepBackendData::worldShape() const
{
	if (m_shape.isNull())
	{
		return {};
	}

	// 获取 worldMatrix 并转换为 Eigen Isometry
	const BackendMat4 world = worldMatrix();
	const engine::ColMajorMat4 cm = [&world]()
	{
		engine::ColMajorMat4 out{};
		for (int i = 0; i < 16; ++i)
		{
			out[static_cast<size_t>(i)] = world.v[i];
		}
		return out;
	}();
	const Eigen::Isometry3d iso = engine::rigidTransformFromColMajor(cm).isometry();

	// 调用 GeometryAlgorithm 的变换函数
	return geoalgo::transformShape(m_shape, iso);
}

nlohmann::json BrepBackendData::snapshotPropertyRows(const BackendDataManager* mgr) const
{
	nlohmann::json rows = BackendDataBase::snapshotPropertyRows(mgr);
	property_core::PropertyPipeline<BackendDataBase, BackendAttributeBase>::appendRows(m_attributes, *this, rows);
	backend_property_json::appendRow(rows, "brep.has_shape", "B-rep", false, hasGeometry() ? "yes" : "no");
	return rows;
}

bool BrepBackendData::applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg,
										  const BackendDataManager* mgr)
{
	if (property_core::PropertyPipeline<BackendDataBase, BackendAttributeBase>::apply(m_attributes, *this, key, value,
																					  errMsg))
	{
		return true;
	}
	return BackendDataBase::applyPropertyChange(key, value, errMsg, mgr);
}

void BrepBackendData::saveDerivedJson(nlohmann::json& out) const
{
	out["geometry"] = nlohmann::json::object();
	out["geometry"]["kind"] = "brep";
	if (!m_brepSidecarRel.empty())
	{
		out["geometry"]["brepSidecar"] = m_brepSidecarRel;
	}
}

bool BrepBackendData::loadDerivedJson(const nlohmann::json& in, std::string* errMsg)
{
	const auto geomIt = in.find("geometry");
	if (geomIt == in.end() || !geomIt->is_object())
	{
		return true;
	}
	const nlohmann::json& geom = geomIt.value();
	if (geom.value("kind", std::string()) != "brep")
	{
		// 与 Mesh/PointCloud 一致：kind 不匹配视为数据损坏，报错而非静默跳过
		if (errMsg)
		{
			*errMsg = "geometry.kind mismatch, expected \"brep\".";
		}
		return false;
	}
	if (geom.contains("brepSidecar") && geom["brepSidecar"].is_string())
	{
		m_brepSidecarRel = geom["brepSidecar"].get<std::string>();
	}
	(void)errMsg;
	return true;
}
