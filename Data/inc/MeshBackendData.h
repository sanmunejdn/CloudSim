#pragma once

#include <memory>
#include <string>
#include <vector>

#include "BackendDataBase.h"

class BackendAttributeBase;

struct DATA_EXPORT MeshHierarchyPart
{
	std::string partPath;
	std::string parentPartPath;
	std::string displayName;
	std::vector<float> triangleSoup;
};

/// 三角网格后端：以三角形 soup（每三角形 9 个 float：三顶点 xyz）存几何，支持文件加载与工程内嵌序列化。
class DATA_EXPORT MeshBackendData : public BackendDataBase
{
public:
	MeshBackendData();
	~MeshBackendData() override = default;

	std::string className() const override;
	bool hasGeometry() const override;
	BackendBoundingBox geometryBounds() const override;
	std::size_t geometryElementCount() const override;
	void clearGeometry() override;

	// Nine floats per triangle: v0.xyz, v1.xyz, v2.xyz (model space, same frame as OSG import).
	void setTriangleSoup(std::vector<float> xyzPerTriangleVertex);
	const std::vector<float>& triangleSoup() const { return m_triangleSoup; }

	/// Apply column-major rigid 4x4 (mesh-file → link frame) to every triangle vertex xyz in \c m_triangleSoup; recomputes bounds.
	void transformVerticesColumnMajorHomogeneous4x4(const double colMajor16[16]);

	void setPose(const BackendVec3& position) override;
	BackendVec3 pose() const override;
	void setRotation(const BackendVec3& eulerDeg) override;
	BackendVec3 rotation() const override;

	void setColor(const BackendColor& color) override;
	BackendColor color() const override;

	bool hasPoseProperty() const override { return true; }
	bool hasRotationProperty() const override { return true; }
	bool hasColorProperty() const override { return true; }

	bool writeProjectEmbeddedGeometry(std::string& outTriangleSoupBase64) const;
	bool readProjectEmbeddedGeometry(const std::string& triangleSoupBase64);

	// CGAL polygon soup I/O (.obj .stl .ply .off). Path native-encoded for std::ifstream (e.g. QFile::encodeName).
	bool loadFromFile(const std::string& path, std::string* errMsg = nullptr);
	static bool loadStepHierarchyFromFile(const std::string& path, std::vector<MeshHierarchyPart>& outParts, std::string* errMsg = nullptr);
	static bool loadDxfHierarchyFromFile(const std::string& path, std::vector<MeshHierarchyPart>& outParts, std::string* errMsg = nullptr);

	nlohmann::json snapshotPropertyRows(const BackendDataManager* mgr = nullptr) const override;
	bool applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg,
		const BackendDataManager* mgr = nullptr) override;

private:
	void recomputeBounds();

	std::vector<float> m_triangleSoup;
	BackendBoundingBox m_bounds;
	BackendVec3 m_position;
	BackendVec3 m_rotation;
	BackendColor m_color;
};
