#include "BackendSpatial.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"

BackendVec3 backend_mat4_transform_point(const BackendMat4& m, const BackendVec3& p)
{
	return BackendVec3{
		m.v[0] * p.x + m.v[4] * p.y + m.v[8] * p.z + m.v[12],
		m.v[1] * p.x + m.v[5] * p.y + m.v[9] * p.z + m.v[13],
		m.v[2] * p.x + m.v[6] * p.y + m.v[10] * p.z + m.v[14]};
}

BackendMat4 objectWorldMatrix(const BackendDataBase& obj, const BackendDataManager* mgr)
{
	(void)mgr;
	return obj.worldMatrix(mgr);
}

BackendVec3 transformPointToWorld(const BackendDataBase& obj, const BackendVec3& vStored, const BackendDataManager* mgr)
{
	return backend_mat4_transform_point(objectWorldMatrix(obj, mgr), vStored);
}

BackendVec3 transformPointToStored(const BackendDataBase& obj, const BackendVec3& vWorld, const BackendDataManager* mgr)
{
	BackendMat4 inv{};
	if (!backend_mat4_invert_rigid(objectWorldMatrix(obj, mgr), inv))
	{
		return vWorld;
	}
	return backend_mat4_transform_point(inv, vWorld);
}
