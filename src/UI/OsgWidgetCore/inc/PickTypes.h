#ifndef OSGWIDGETCORE_PICKTYPES_H
#define OSGWIDGETCORE_PICKTYPES_H

/// @file PickTypes.h
/// @brief PickTypes 接口

#include <cstdint>
#include <string>
#include <vector>

#include <osg/Vec3f>

enum class PickKind
{
	PointCloud,
	MeshFace,
	MeshEdge,
	BackendObject,
	GizmoAxis
};

struct PickQuery
{
	double screenX = 0.0;
	double screenY = 0.0;
	PickKind kind = PickKind::PointCloud;
	std::string scopeBackendId;
	double hitRadiusPx = 32.0;
	/// hover 走 KD 快路径，跳过全场景射线相交
	bool hoverPick = false;
};

struct PickResult
{
	bool hit = false;
	double screenDistancePx = 0.0;
	osg::Vec3f worldPoint;
	std::string backendId;
	std::uint64_t indexGeneration = 0;
	osg::Vec3f meshEdgeA;
	osg::Vec3f meshEdgeB;
	std::vector<osg::Vec3f> meshEdgePolylineWorld;
	std::vector<osg::Vec3f> meshFaceVertsWorld;
	osg::Vec3f meshNormalWorld;
	int brepFaceIndex = -1;
	int brepEdgeIndex = -1;
	bool brepNativePick = false;
	int pickedTriangleIndex = -1;
	int pointIndex = -1;
	int meshTriangleIndex = -1;
};

struct PickPreviewState
{
	bool valid = false;
	PickResult result;
};

#endif // OSGWIDGETCORE_PICKTYPES_H
