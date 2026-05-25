#pragma once

#include <memory>
#include <string>
#include <vector>

#include "BackendDataBase.h"
#include "BackendObjectAttribute.h"

/// 点云后端：交错 xyz 浮点缓冲与可选每顶点 RGBA，支持 PLY 等加载与工程内嵌序列化
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

	void setPointBuffers(std::vector<float> xyz, std::vector<float> rgbaPerVertex);
	void setPointBuffers(std::vector<float> xyz, std::vector<float> rgbaPerVertex, std::vector<float> normalsNxNyNz);
	void setPointNormals(std::vector<float> normalsNxNyNz);
	const std::vector<float>& pointPositionsXyz() const { return m_xyz; }
	const std::vector<float>& pointVertexRgba() const { return m_rgbaVertex; }
	const std::vector<float>& pointNormalsNxNyNz() const { return m_normals; }
	bool hasPerVertexColors() const { return !m_rgbaVertex.empty(); }
	bool hasPointNormals() const { return !m_normals.empty(); }

	bool writeProjectEmbeddedGeometry(std::string& outXyzBase64, std::string& outRgbaPerVertexBase64) const;
	bool readProjectEmbeddedGeometry(const std::string& xyzBase64, const std::string& rgbaPerVertexBase64);

/// PLY 加载（CGAL）；utf8Path 为 UTF-8
	bool readPointCloudFromPlyFile(const std::string& utf8Path, std::string* errMsg = nullptr);

/// 按扩展名加载；path 用本地编码（QFile::encodeName）
	bool loadFromFile(const std::string& path, std::string* errMsg = nullptr);

	bool writePointCloudPlySidecar(const std::string& utf8Path, std::string* errMsg) const;
	bool readPointCloudPlySidecar(const std::string& utf8Path, std::string* errMsg);

	nlohmann::json snapshotPropertyRows(const BackendDataManager* mgr = nullptr) const override;
	bool applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg,
		const BackendDataManager* mgr = nullptr) override;

private:
	void recomputeBoundsFromPoints();
	void saveDerivedJson(nlohmann::json& out) const override;
	bool loadDerivedJson(const nlohmann::json& in, std::string* errMsg) override;

	BackendVec3 m_position;
	BackendVec3 m_rotation;
	BackendColor m_color;
	BackendBoundingBox m_bounds;
	std::size_t m_pointCount = 0U;
	std::vector<float> m_xyz;
	std::vector<float> m_rgbaVertex;
	std::vector<float> m_normals;
};

