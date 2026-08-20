#ifndef OSGWIDGETCORE_BREPPICKINDEX_H
#define OSGWIDGETCORE_BREPPICKINDEX_H

/// @file BrepPickIndex.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief BREP 拾取索引：导入时构建，hover 查表 + 屏幕空间边距离

#include "osgwidgetcore_global.h"

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include <osg/Matrixd>
#include <osg/Vec3f>

namespace geoalgo
{
class ShapeHandle;
struct Point3d;
struct BrepImportArtifacts;
} // namespace geoalgo

/// BREP 拾取索引：导入时构建，hover 查表 + 屏幕空间边距离
class OSGWIDGETCORE_EXPORT BrepPickIndex
{
public:
	bool empty() const;
	std::size_t triangleCount() const;

	int faceIndexForTriangle(unsigned triIndex) const;
	const std::vector<float>& faceSoupModel(int faceIndex) const;
	const std::vector<float>& edgePolylineModel(int edgeIndex) const;
	const std::vector<std::vector<int>>& faceEdgeIndices() const;

	bool build(const geoalgo::ShapeHandle& shape, std::string* errMsg = nullptr);
	bool buildFromArtifacts(const geoalgo::BrepImportArtifacts& artifacts, std::string* errMsg = nullptr);

	bool pickEdgeByScreen(int hintFaceIndex, double mouseX, double mouseY, const osg::Matrixd& mvp, int viewportWidthPx,
						  int viewportHeightPx, double hitRadiusPx,
						  const std::function<bool(const geoalgo::Point3d&, osg::Vec3f&)>& modelPointToWorld,
						  int& outEdgeIndex, double& outDistancePx, std::vector<osg::Vec3f>& outPolylineWorld) const;

private:
	std::vector<int> m_triangleFaceIndex;
	std::vector<std::vector<float>> m_faceSoupModel;
	std::vector<std::vector<float>> m_edgePolylinesModel;
	std::vector<std::vector<int>> m_faceEdgeIndices;
};

#endif // OSGWIDGETCORE_BREPPICKINDEX_H
