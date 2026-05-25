#include "pch.h"
#include "MeshBackendData.h"
#include "BackendObjectAttribute.h"
#include "BackendPropertyRow.h"
#include "RunLogger.h"
#include "geometry_base64.h"
#include "../../PropertyCore/inc/PropertyAttribute.h"

MeshBackendData::MeshBackendData()
{
	setName("Model");
	BackendColor c;
	c.r = 0.65f;
	c.g = 0.82f;
	c.b = 0.95f;
	c.a = 1.0f;
	m_color = c;

	// 复用 pose/rotation/color 属性，面板行为一致
	m_attributes.push_back(makeBackendPoseAttribute());
	m_attributes.push_back(makeBackendRotationAttribute());
	m_attributes.push_back(makeBackendDisplayColorAttribute());
}

std::string MeshBackendData::className() const
{
	return "Model";
}

bool MeshBackendData::hasGeometry() const
{
	return !m_triangleSoup.empty();
}

BackendBoundingBox MeshBackendData::geometryBounds() const
{
	return m_bounds;
}

std::size_t MeshBackendData::geometryElementCount() const
{
	return m_triangleSoup.size() / 9U;
}

void MeshBackendData::clearGeometry()
{
	m_triangleSoup.clear();
	m_triangleNormals.clear();
	m_bounds = BackendBoundingBox{};
}

void MeshBackendData::setPose(const BackendVec3& position)
{
	m_position = position;
}

BackendVec3 MeshBackendData::pose() const
{
	return m_position;
}

void MeshBackendData::setRotation(const BackendVec3& eulerDeg)
{
	m_rotation = eulerDeg;
}

BackendVec3 MeshBackendData::rotation() const
{
	return m_rotation;
}

void MeshBackendData::setColor(const BackendColor& color)
{
	m_color = color;
}

BackendColor MeshBackendData::color() const
{
	return m_color;
}

nlohmann::json MeshBackendData::snapshotPropertyRows(const BackendDataManager* mgr) const
{
	nlohmann::json rows = BackendDataBase::snapshotPropertyRows(mgr);
	property_core::PropertyPipeline<BackendDataBase, BackendAttributeBase>::appendRows(m_attributes, *this, rows);

	backend_property_json::appendRow(
		rows, "mesh.triangle_count", "Triangles", false, std::to_string(geometryElementCount()));
	return rows;
}

bool MeshBackendData::applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg,
	const BackendDataManager* mgr)
{
	if (property_core::PropertyPipeline<BackendDataBase, BackendAttributeBase>::apply(
			m_attributes, *this, key, value, errMsg))
	{
		return true;
	}
	return BackendDataBase::applyPropertyChange(key, value, errMsg, mgr);
}

void MeshBackendData::setTriangleSoup(std::vector<float> xyzPerTriangleVertex)
{
	setTriangleSoupWithNormals(std::move(xyzPerTriangleVertex), {});
}

void MeshBackendData::setTriangleSoupWithNormals(std::vector<float> xyzPerTriangleVertex,
	std::vector<float> normalPerTriangleVertex)
{
	if (xyzPerTriangleVertex.size() % 9U != 0U)
	{
		clearGeometry();
		return;
	}
	if (!normalPerTriangleVertex.empty() && normalPerTriangleVertex.size() != xyzPerTriangleVertex.size())
	{
		clearGeometry();
		return;
	}
	m_triangleSoup = std::move(xyzPerTriangleVertex);
	m_triangleNormals = std::move(normalPerTriangleVertex);
	recomputeBounds();
}

void MeshBackendData::transformVerticesColumnMajorHomogeneous4x4(const double M[16])
{
	if (m_triangleSoup.size() < 3U || (m_triangleSoup.size() % 3U) != 0U)
	{
		return;
	}
	for (std::size_t i = 0; i + 2 < m_triangleSoup.size(); i += 3U)
	{
		const double x = static_cast<double>(m_triangleSoup[i]);
		const double y = static_cast<double>(m_triangleSoup[i + 1]);
		const double z = static_cast<double>(m_triangleSoup[i + 2]);
		const double nx = M[0] * x + M[4] * y + M[8] * z + M[12];
		const double ny = M[1] * x + M[5] * y + M[9] * z + M[13];
		const double nz = M[2] * x + M[6] * y + M[10] * z + M[14];
		m_triangleSoup[i] = static_cast<float>(nx);
		m_triangleSoup[i + 1] = static_cast<float>(ny);
		m_triangleSoup[i + 2] = static_cast<float>(nz);
	}
	if (m_triangleNormals.size() == m_triangleSoup.size())
	{
		for (std::size_t i = 0; i + 2 < m_triangleNormals.size(); i += 3U)
		{
			const double x = static_cast<double>(m_triangleNormals[i]);
			const double y = static_cast<double>(m_triangleNormals[i + 1]);
			const double z = static_cast<double>(m_triangleNormals[i + 2]);
			const double nx = M[0] * x + M[4] * y + M[8] * z;
			const double ny = M[1] * x + M[5] * y + M[9] * z;
			const double nz = M[2] * x + M[6] * y + M[10] * z;
			m_triangleNormals[i] = static_cast<float>(nx);
			m_triangleNormals[i + 1] = static_cast<float>(ny);
			m_triangleNormals[i + 2] = static_cast<float>(nz);
		}
	}
	recomputeBounds();
}

void MeshBackendData::recomputeBounds()
{
	m_bounds = BackendBoundingBox{};
	if (m_triangleSoup.size() < 3U)
	{
		return;
	}
	double minX = m_triangleSoup[0];
	double minY = m_triangleSoup[1];
	double minZ = m_triangleSoup[2];
	double maxX = minX;
	double maxY = minY;
	double maxZ = minZ;
	for (std::size_t i = 0; i + 2 < m_triangleSoup.size(); i += 3)
	{
		const double x = m_triangleSoup[i];
		const double y = m_triangleSoup[i + 1];
		const double z = m_triangleSoup[i + 2];
		minX = std::min(minX, x);
		minY = std::min(minY, y);
		minZ = std::min(minZ, z);
		maxX = std::max(maxX, x);
		maxY = std::max(maxY, y);
		maxZ = std::max(maxZ, z);
	}
	m_bounds.min.x = minX;
	m_bounds.min.y = minY;
	m_bounds.min.z = minZ;
	m_bounds.max.x = maxX;
	m_bounds.max.y = maxY;
	m_bounds.max.z = maxZ;
	m_bounds.valid = true;
}

bool MeshBackendData::writeProjectEmbeddedGeometry(std::string& outTriangleSoupBase64) const
{
	outTriangleSoupBase64.clear();
	if (m_triangleSoup.empty() || (m_triangleSoup.size() % 9U) != 0U)
	{
		return false;
	}
	outTriangleSoupBase64 = geometryBase64EncodeFloats(m_triangleSoup);
	return !outTriangleSoupBase64.empty();
}

bool MeshBackendData::readProjectEmbeddedGeometry(const std::string& triangleSoupBase64)
{
	std::vector<float> soup;
	if (!geometryBase64DecodeFloats(triangleSoupBase64, soup) || soup.size() < 9U || (soup.size() % 9U) != 0U)
	{
		return false;
	}
	setTriangleSoup(std::move(soup));
	return !m_triangleSoup.empty();
}

void MeshBackendData::saveDerivedJson(nlohmann::json& out) const
{
	std::string soupB64;
	if (writeProjectEmbeddedGeometry(soupB64))
	{
		out["geometry"] = nlohmann::json{
			{ "kind", "triangles" },
			{ "encoding", "float32_le" },
			{ "xyzBase64", soupB64 } };
	}
	out["mesh"] = nlohmann::json{ { "transformPivotAtOrigin", m_transformPivotAtOrigin } };
}

bool MeshBackendData::loadDerivedJson(const nlohmann::json& in, std::string* errMsg)
{
	if (in.contains("mesh") && in["mesh"].is_object())
	{
		m_transformPivotAtOrigin = in["mesh"].value("transformPivotAtOrigin", false);
	}
	if (!in.contains("geometry"))
	{
		return true;
	}
	const nlohmann::json geo = in["geometry"];
	if (!geo.is_object())
	{
		if (errMsg)
		{
			*errMsg = "Mesh geometry must be object.";
		}
		return false;
	}
	if (geo.value("kind", std::string()) != "triangles")
	{
		if (errMsg)
		{
			*errMsg = "Mesh geometry kind mismatch.";
		}
		return false;
	}
	const std::string xyzBase64 = geo.value("xyzBase64", std::string());
	if (xyzBase64.empty())
	{
		return true;
	}
	if (!readProjectEmbeddedGeometry(xyzBase64))
	{
		if (errMsg)
		{
			*errMsg = "Mesh geometry decode failed.";
		}
		return false;
	}
	return true;
}
