#pragma once

#include "data_global.h"
#include "BackendFollowMath.h"

class BackendDataBase;
class BackendDataManager;

/// 世界点 = objectWorldMatrix × v_stored
DATA_EXPORT BackendMat4 objectWorldMatrix(const BackendDataBase& obj, const BackendDataManager* mgr = nullptr);
DATA_EXPORT BackendVec3 transformPointToWorld(
	const BackendDataBase& obj, const BackendVec3& vStored, const BackendDataManager* mgr = nullptr);
DATA_EXPORT BackendVec3 transformPointToStored(
	const BackendDataBase& obj, const BackendVec3& vWorld, const BackendDataManager* mgr = nullptr);

DATA_EXPORT BackendVec3 backend_mat4_transform_point(const BackendMat4& m, const BackendVec3& p);
