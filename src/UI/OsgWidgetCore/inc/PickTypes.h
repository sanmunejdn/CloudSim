#pragma once

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
	osg::Vec3f meshNormalWorld;
	std::vector<osg::Vec3f> meshFaceVertsWorld;
};

struct PickPreviewState
{
	bool valid = false;
	PickResult result;
};
