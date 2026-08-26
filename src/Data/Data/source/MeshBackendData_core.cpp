/// @file MeshBackendData_core.cpp
/// @brief 网格后端核心

#include "pch.h"

#include "../../PropertyCore/inc/PropertyAttribute.h"
#include "BackendObjectAttribute.h"
#include "BackendPropertyRow.h"
#include "BackendTypeIdentity.h"
#include "MeshBackendData.h"
#include "RunLogger.h"
#include "geometry_base64.h"

MeshBackendData::MeshBackendData()
{
	setName(backend_type::kCatalogModel);
	BackendColor c;
	c.r = 0.65f;
	c.g = 0.82f;
	c.b = 0.95f;
	c.a = 1.0f;
	m_color = c;

	// 按 has*Property() 声明统一追加，避免手工 push 漏推导致面板静默少行
	appendStandardAttributesForCapabilities(*this, m_attributes);
}

std::string MeshBackendData::className() const
{
	return backend_type::kClassModel;
}

bool MeshBackendData::hasGeometry() const
{
	return !m_triangleSoup.empty() || hasOverlayLineSegments();
}

BackendBoundingBox MeshBackendData::geometryBounds() const
{
	return m_bounds;
}

std::size_t MeshBackendData::geometryElementCount() const
{
	if (!m_triangleSoup.empty())
	{
		return m_triangleSoup.size() / 9U;
	}
	return m_overlayLineSegments.size() / 6U;
}

void MeshBackendData::clearGeometry()
{
	m_triangleSoup.clear();
	m_triangleNormals.clear();
	m_triangleVertexColors.clear();
	m_overlayLineSegments.clear();
	m_bounds = BackendBoundingBox{};
	bumpGeometryRevision();
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

	backend_property_json::appendRow(rows, "mesh.triangle_count", "Triangles", false,
									 std::to_string(geometryElementCount()));
	return rows;
}

bool MeshBackendData::applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg,
										  const BackendDataManager* mgr)
{
	if (property_core::PropertyPipeline<BackendDataBase, BackendAttributeBase>::apply(m_attributes, *this, key, value,
																					  errMsg))
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
		RunLogger::warn("[MeshBackendData] setTriangleSoupWithNormals: bad soup size, keep existing geometry.");
		return;
	}
	if (!normalPerTriangleVertex.empty() && normalPerTriangleVertex.size() != xyzPerTriangleVertex.size())
	{
		RunLogger::warn("[MeshBackendData] setTriangleSoupWithNormals: normals size mismatch, keep existing geometry.");
		return;
	}
	m_triangleSoup = std::move(xyzPerTriangleVertex);
	m_triangleNormals = std::move(normalPerTriangleVertex);
	m_triangleVertexColors.clear();
	recomputeBounds();
	bumpGeometryRevision();
}

void MeshBackendData::setTriangleSoupWithVertexColors(std::vector<float> xyzPerTriangleVertex,
													  std::vector<float> rgbPerTriangleVertex)
{
	if (xyzPerTriangleVertex.size() % 9U != 0U)
	{
		RunLogger::warn("[MeshBackendData] setTriangleSoupWithVertexColors: bad soup size, keep existing geometry.");
		return;
	}
	if (rgbPerTriangleVertex.size() != xyzPerTriangleVertex.size())
	{
		RunLogger::warn("[MeshBackendData] setTriangleSoupWithVertexColors: color size mismatch, keep existing geometry.");
		return;
	}
	m_triangleSoup = std::move(xyzPerTriangleVertex);
	m_triangleVertexColors = std::move(rgbPerTriangleVertex);
	m_triangleNormals.clear();
	recomputeBounds();
	bumpGeometryRevision();
}

void MeshBackendData::setOverlayLineSegments(std::vector<float> xyzLinePairs)
{
	if (xyzLinePairs.size() % 6U != 0U)
	{
		// B3: 与同文件其他 setter 对齐，非法尺寸告警
		RunLogger::warn("[MeshBackendData] setOverlayLineSegments: size not multiple of 6, cleared.");
		m_overlayLineSegments.clear();
		recomputeBounds();
		bumpGeometryRevision();
		return;
	}
	m_overlayLineSegments = std::move(xyzLinePairs);
	recomputeBounds();
	bumpGeometryRevision();
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
	bumpGeometryRevision();
}

void MeshBackendData::recomputeBounds()
{
	m_bounds = BackendBoundingBox{};
	auto expand = [&](const double x, const double y, const double z)
	{
		if (!m_bounds.valid)
		{
			m_bounds.min.x = x;
			m_bounds.min.y = y;
			m_bounds.min.z = z;
			m_bounds.max.x = x;
			m_bounds.max.y = y;
			m_bounds.max.z = z;
			m_bounds.valid = true;
			return;
		}
		m_bounds.min.x = std::min(m_bounds.min.x, x);
		m_bounds.min.y = std::min(m_bounds.min.y, y);
		m_bounds.min.z = std::min(m_bounds.min.z, z);
		m_bounds.max.x = std::max(m_bounds.max.x, x);
		m_bounds.max.y = std::max(m_bounds.max.y, y);
		m_bounds.max.z = std::max(m_bounds.max.z, z);
	};
	for (std::size_t i = 0; i + 2 < m_triangleSoup.size(); i += 3)
	{
		expand(m_triangleSoup[i], m_triangleSoup[i + 1], m_triangleSoup[i + 2]);
	}
	for (std::size_t i = 0; i + 2 < m_overlayLineSegments.size(); i += 3)
	{
		expand(m_overlayLineSegments[i], m_overlayLineSegments[i + 1], m_overlayLineSegments[i + 2]);
	}
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
		nlohmann::json geo = nlohmann::json{{"kind", "triangles"}, {"encoding", "float32_le"}, {"xyzBase64", soupB64}};
		if (hasTriangleVertexNormals())
		{
			geo["normalsBase64"] = geometryBase64EncodeFloats(m_triangleNormals);
		}
		if (hasTriangleVertexColors())
		{
			geo["vertexColorsBase64"] = geometryBase64EncodeFloats(m_triangleVertexColors);
		}
		if (hasOverlayLineSegments())
		{
			geo["overlayLinesBase64"] = geometryBase64EncodeFloats(m_overlayLineSegments);
		}
		out["geometry"] = std::move(geo);
	}
	out["mesh"] = nlohmann::json{{"transformPivotAtOrigin", m_transformPivotAtOrigin},
								 {"overlayLinesAlwaysOnTop", m_overlayLinesAlwaysOnTop}};
}

bool MeshBackendData::loadDerivedJson(const nlohmann::json& in, std::string* errMsg)
{
	if (in.contains("mesh") && in["mesh"].is_object())
	{
		m_transformPivotAtOrigin = in["mesh"].value("transformPivotAtOrigin", false);
		m_overlayLinesAlwaysOnTop = in["mesh"].value("overlayLinesAlwaysOnTop", false);
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

	auto tryLoadOptionalChannel = [&](const char* key, std::vector<float>& dest, const char* label,
									  const bool requireMatchSoupSize)
	{
		dest.clear();
		if (!geo.contains(key) || !geo[key].is_string())
		{
			return;
		}
		const std::string b64 = geo[key].get<std::string>();
		if (b64.empty())
		{
			return;
		}
		std::vector<float> decoded;
		if (!geometryBase64DecodeFloats(b64, decoded))
		{
			RunLogger::warn(std::string("[MeshBackendData] drop ") + label + ": Base64 decode failed.");
			return;
		}
		if (requireMatchSoupSize && decoded.size() != m_triangleSoup.size())
		{
			RunLogger::warn(std::string("[MeshBackendData] drop ") + label + ": size mismatch with soup.");
			return;
		}
		if (!requireMatchSoupSize && (decoded.size() < 6U || (decoded.size() % 6U) != 0U))
		{
			RunLogger::warn(std::string("[MeshBackendData] drop ") + label + ": invalid overlay line size.");
			return;
		}
		dest = std::move(decoded);
	};

	tryLoadOptionalChannel("normalsBase64", m_triangleNormals, "normalsBase64", true);
	tryLoadOptionalChannel("vertexColorsBase64", m_triangleVertexColors, "vertexColorsBase64", true);
	tryLoadOptionalChannel("overlayLinesBase64", m_overlayLineSegments, "overlayLinesBase64", false);

	recomputeBounds();
	bumpGeometryRevision();
	return true;
}
