#ifndef DATA_MESHBACKENDDATA_H
#define DATA_MESHBACKENDDATA_H

/// @file MeshBackendData.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 三角网格后端：以三角形 soup（每三角形 9 个 float：三顶点 xyz）存几何，支持文件加载与工程内嵌序列化

#include "BackendDataBase.h"

#include <memory>
#include <string>
#include <vector>

class BackendAttributeBase;

struct DATA_EXPORT MeshHierarchyPart
{
	std::string partPath;
	std::string parentPartPath;
	std::string displayName;
	std::vector<float> triangleSoup;
};

/// 三角网格后端：以三角形 soup（每三角形 9 个 float：三顶点 xyz）存几何，支持文件加载与工程内嵌序列化
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

	void setTriangleSoup(std::vector<float> xyzPerTriangleVertex);
	/// 可选每三角法线（9 float，与 soup 对齐），OBJ vn 光照
	void setTriangleSoupWithNormals(std::vector<float> xyzPerTriangleVertex,
									std::vector<float> normalPerTriangleVertex);
	/// 每顶点 rgb（与 soup 同长度、同顶点顺序）
	void setTriangleSoupWithVertexColors(std::vector<float> xyzPerTriangleVertex,
										 std::vector<float> rgbPerTriangleVertex);
	/// 叠加线段（每段 6 float：起点 xyz + 终点 xyz），GL_LINES 绘制
	void setOverlayLineSegments(std::vector<float> xyzLinePairs);
	/// 叠加线始终可见（关闭深度测试）
	void setOverlayLinesAlwaysOnTop(const bool alwaysOnTop) { m_overlayLinesAlwaysOnTop = alwaysOnTop; }
	bool overlayLinesAlwaysOnTop() const { return m_overlayLinesAlwaysOnTop; }
	const std::vector<float>& overlayLineSegments() const { return m_overlayLineSegments; }
	bool hasOverlayLineSegments() const
	{
		return m_overlayLineSegments.size() >= 6U && (m_overlayLineSegments.size() % 6U) == 0U;
	}
	const std::vector<float>& triangleSoup() const { return m_triangleSoup; }
	const std::vector<float>& triangleVertexNormals() const { return m_triangleNormals; }
	const std::vector<float>& triangleVertexColors() const { return m_triangleVertexColors; }
	bool hasTriangleVertexNormals() const
	{
		return !m_triangleNormals.empty() && m_triangleNormals.size() == m_triangleSoup.size();
	}
	bool hasTriangleVertexColors() const
	{
		return !m_triangleVertexColors.empty() && m_triangleVertexColors.size() == m_triangleSoup.size();
	}

	/// 列主序 4×4（mesh 系→连杆系）烘焙到 soup 顶点并重算包围
	void transformVerticesColumnMajorHomogeneous4x4(const double colMajor16[16]);

	void setColor(const BackendColor& color) override;
	BackendColor color() const override;

	bool hasPoseProperty() const override { return true; }
	bool hasRotationProperty() const override { return true; }
	bool hasColorProperty() const override { return true; }

	bool writeProjectEmbeddedGeometry(std::string& outTriangleSoupBase64) const;
	bool readProjectEmbeddedGeometry(const std::string& triangleSoupBase64);

	/// 文件加载实现见 backend_io；此处转发兼容旧调用方
	bool loadFromFile(const std::string& path, std::string* errMsg = nullptr, int meshImportQuality = 1);
	/// 三角 soup 写 PLY（含 face 元素）；path UTF-8
	bool writeTriangleMeshPly(const std::string& utf8Path, std::string* errMsg = nullptr) const;
	/// 写入指定的三角形 soup 到 PLY 文件（用于导出世界坐标系下的网格）
	bool writeTriangleMeshPly(const std::string& utf8Path, const std::vector<float>& soupOverride,
							  std::string* errMsg = nullptr) const;

	/// 返回世界坐标系下的三角形 soup（应用 worldMatrix 变换后的副本）
	std::vector<float> worldTriangleSoup() const;
	static bool loadStepHierarchyFromFile(const std::string& path, std::vector<MeshHierarchyPart>& outParts,
										  std::string* errMsg = nullptr);
	static bool loadDxfHierarchyFromFile(const std::string& path, std::vector<MeshHierarchyPart>& outParts,
										 std::string* errMsg = nullptr);

	nlohmann::json snapshotPropertyRows(const BackendDataManager* mgr = nullptr) const override;
	bool applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg,
							 const BackendDataManager* mgr = nullptr) override;

	/// true 时以 mesh 原点为枢轴（URDF 连杆系），非 bbox 中心
	void setTransformPivotAtOrigin(bool atOrigin) { m_transformPivotAtOrigin = atOrigin; }
	bool transformPivotAtOrigin() const { return m_transformPivotAtOrigin; }

private:
	void recomputeBounds();
	void saveDerivedJson(nlohmann::json& out) const override;
	bool loadDerivedJson(const nlohmann::json& in, std::string* errMsg) override;

	std::vector<float> m_triangleSoup;
	std::vector<float> m_triangleNormals;
	std::vector<float> m_triangleVertexColors;
	std::vector<float> m_overlayLineSegments;
	bool m_overlayLinesAlwaysOnTop = false;
	BackendBoundingBox m_bounds;
	BackendColor m_color;
	bool m_transformPivotAtOrigin = false;
};

#endif // DATA_MESHBACKENDDATA_H
