#include "pch.h"
#include "BrepBackendData.h"
#include <ShapeIo.h>
#include "BackendObjectAttribute.h"
#include "BackendPropertyRow.h"
#include "../../PropertyCore/inc/PropertyAttribute.h"

BrepBackendData::BrepBackendData()
{
	setName("BrepModel");
	BackendColor c;
	c.r = 0.65f;
	c.g = 0.82f;
	c.b = 0.95f;
	c.a = 1.0f;
	m_color = c;
	m_attributes.push_back(makeBackendPoseAttribute());
	m_attributes.push_back(makeBackendRotationAttribute());
	m_attributes.push_back(makeBackendDisplayColorAttribute());
}

std::string BrepBackendData::className() const
{
	return "BrepModel";
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
}

void BrepBackendData::setShape(geoalgo::ShapeHandle shape)
{
	m_shape = std::move(shape);
	recomputeBounds();
}

void BrepBackendData::shareShapeFrom(const BrepBackendData& other)
{
	m_shape = other.m_shape;
	recomputeBounds();
}

void BrepBackendData::setPose(const BackendVec3& position)
{
	m_position = position;
}

BackendVec3 BrepBackendData::pose() const
{
	return m_position;
}

void BrepBackendData::setRotation(const BackendVec3& eulerDeg)
{
	m_rotation = eulerDeg;
}

BackendVec3 BrepBackendData::rotation() const
{
	return m_rotation;
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
}

const std::unordered_map<int, BackendColor>& BrepBackendData::faceHighlightColors() const
{
	return m_faceHighlightColors;
}

void BrepBackendData::clearFaceHighlightColors()
{
	m_faceHighlightColors.clear();
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
	if (property_core::PropertyPipeline<BackendDataBase, BackendAttributeBase>::apply(
			m_attributes, *this, key, value, errMsg))
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
		return true;
	}
	if (geom.contains("brepSidecar") && geom["brepSidecar"].is_string())
	{
		m_brepSidecarRel = geom["brepSidecar"].get<std::string>();
	}
	(void)errMsg;
	return true;
}
