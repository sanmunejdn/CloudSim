#pragma once

#include "data_global.h"
#include "BackendFollowMath.h"

#include <vector>

class BackendDataBase;
class BackendDataManager;

/// 世界点 = objectWorldMatrix × v_stored
DATA_EXPORT BackendMat4 objectWorldMatrix(const BackendDataBase& obj, const BackendDataManager* mgr = nullptr);
DATA_EXPORT BackendVec3 transformPointToWorld(
	const BackendDataBase& obj, const BackendVec3& vStored, const BackendDataManager* mgr = nullptr);
DATA_EXPORT BackendVec3 transformPointToStored(
	const BackendDataBase& obj, const BackendVec3& vWorld, const BackendDataManager* mgr = nullptr);

DATA_EXPORT BackendVec3 backend_mat4_transform_point(const BackendMat4& m, const BackendVec3& p);

/// 批量变换 xyz 坐标到世界系（原地修改，每 3 个 float 一个点）
DATA_EXPORT void transformXyzToWorld(std::vector<float>& xyz, const BackendMat4& worldMatrix);

/// 批量变换三角形 soup 到世界系（原地修改，每 9 个 float 一个三角形）
DATA_EXPORT void transformTriangleSoupToWorld(std::vector<float>& triangleSoup, const BackendMat4& worldMatrix);
