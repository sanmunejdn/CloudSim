#pragma once

#include "cloudsim_plugin_sdk_global.h"

#include <QString>

/// 宿主 createPrimitiveMesh 图元类型（mm）
enum class PluginPrimitiveKind
{
	Box = 0,
	Cylinder,
	Cone,
	Sphere
};

struct PluginPrimitiveMeshParams
{
	PluginPrimitiveKind kind = PluginPrimitiveKind::Box;
	double lengthMm = 100.0;
	double widthMm = 50.0;
	double heightMm = 100.0;
	double radiusMm = 30.0;
	double radiusTopMm = 0.0;
};

struct PluginPrimitiveMeshQuality
{
	int segments = 32;
	int rings = 16;
};

struct PluginVec3
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

struct PluginMeshCreateOptions
{
	QString displayName;
	QString sourcePath;
	PluginVec3 poseMm{};
	PluginVec3 rotationDeg{};
	bool selectInTree = true;
	bool resetViewToHome = true;
};

enum class PluginMeshBooleanOp
{
	Difference = 0,
	Union,
	Intersection
};

struct PluginBooleanMeshOptions
{
	PluginMeshBooleanOp op = PluginMeshBooleanOp::Difference;
	QString resultName;
	bool hideOperands = true;
	bool selectInTree = true;
	bool resetViewToHome = false;
};
