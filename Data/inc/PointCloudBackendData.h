#pragma once

#include <memory>
#include <string>
#include <vector>

#include "BackendDataBase.h"
#include "BackendObjectAttribute.h"

/// 点云后端：交错 xyz 浮点缓冲与可选每顶点 RGBA，支持 PLY 等加载与工程内嵌序列化。
class DATA_EXPORT PointCloudBackendData : public BackendDataBase
{
public:
	PointCloudBackendData();
	~PointCloudBackendData() override = default;

	std::string className() const override;
	bool hasGeometry() const override;
	BackendBoundingBox geometryBounds() const override;
	std::size_t geometryElementCount() const override;
	void clearGeometry() override;

	void setPose(const BackendVec3& position) override;
	BackendVec3 pose() const override;
	void setRotation(const BackendVec3& eulerDeg) override;
	BackendVec3 rotation() const override;

	void setColor(const BackendColor& color) override;
	BackendColor color() const override;

	bool hasPoseProperty() const override { return true; }
	bool hasRotationProperty() const override { return true; }
	bool hasColorProperty() const override { return true; }

	void setPointCount(std::size_t count);
	void setBounds(const BackendBoundingBox& bounds);

	// Raw points: xyz interleaved (3 * N floats). Optional per-vertex RGBA (4 * N floats, 0..1).
	void setPointBuffers(std::vector<float> xyz, std::vector<float> rgbaPerVertex);
	const std::vector<float>& pointPositionsXyz() const { return m_xyz; }
	const std::vector<float>& pointVertexRgba() const { return m_rgbaVertex; }
	bool hasPerVertexColors() const { return !m_rgbaVertex.empty(); }

	// Project file embeddedGeometry (Base64 float32 LE); false if no point data to save.
	bool writeProjectEmbeddedGeometry(std::string& outXyzBase64, std::string& outRgbaPerVertexBase64) const;
	bool readProjectEmbeddedGeometry(const std::string& xyzBase64, const std::string& rgbaPerVertexBase64);

	// Load PLY point cloud (ASCII/binary) via CGAL; flexible ASCII header when CGAL needs format on line 2.
	// utf8Path should be UTF-8 (e.g. QString::toUtf8()). Face elements are skipped for rendering.
	bool readPointCloudFromPlyFile(const std::string& utf8Path, std::string* errMsg = nullptr);

	// Load from disk by extension: .ply (CGAL), .xyz (ASCII). Path should be native-encoded for std::ifstream (e.g. QFile::encodeName).
	bool loadFromFile(const std::string& path, std::string* errMsg = nullptr);

	// Project sidecar: binary PLY (float x/y/z in body to match typical clouds; binary_little_endian).
	// utf8Path should be UTF-8 (e.g. QString::toUtf8()).
	bool writePointCloudPlySidecar(const std::string& utf8Path, std::string* errMsg) const;
	bool readPointCloudPlySidecar(const std::string& utf8Path, std::string* errMsg);

	nlohmann::json snapshotPropertyRows(const BackendDataManager* mgr = nullptr) const override;
	bool applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg,
		const BackendDataManager* mgr = nullptr) override;

private:
	void recomputeBoundsFromPoints();

	BackendVec3 m_position;
	BackendVec3 m_rotation;
	BackendColor m_color;
	BackendBoundingBox m_bounds;
	std::size_t m_pointCount = 0U;
	std::vector<float> m_xyz;
	std::vector<float> m_rgbaVertex;
};

