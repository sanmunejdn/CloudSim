#ifndef CLOUDSIMMESHTRAJECTORYSDK_MESHTRAJECTORYTYPES_H
#define CLOUDSIMMESHTRAJECTORYSDK_MESHTRAJECTORYTYPES_H

/// @file MeshTrajectoryTypes.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief MeshTrajectoryTypes 接口

#include "mesh_trajectory_sdk_global.h"

#include <cstddef>
#include <string>
#include <vector>

enum class MeshTrajectorySelectionMode
{
	Replace,
	Add,
	Toggle,
	Subtract
};

struct MeshTrajectorySelectionPatch
{
	std::vector<int> triangleIndices;
};

struct MeshTrajectorySessionSummary
{
	std::string backendIdUtf8;
	int triangleCount = 0;
	int selectedTriangleCount = 0;
};

#endif // CLOUDSIMMESHTRAJECTORYSDK_MESHTRAJECTORYTYPES_H
