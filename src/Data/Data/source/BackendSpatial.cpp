#include "BackendSpatial.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"

#include <Adapters.h>

namespace
{

engine::RigidTransform rigidFromBackendMat4(const BackendMat4& m)
{
	engine::ColMajorMat4 cm{};
	for (int i = 0; i < 16; ++i)
	{
		cm[static_cast<size_t>(i)] = m.v[i];
	}
	return engine::rigidTransformFromColMajor(cm);
}

BackendMat4 backendMat4FromRigid(const engine::RigidTransform& rt)
{
	const engine::ColMajorMat4 cm = engine::colMajorFromRigidTransform(rt);
	BackendMat4 out{};
	for (int i = 0; i < 16; ++i)
	{
		out.v[i] = cm[static_cast<size_t>(i)];
	}
	return out;
}

} // namespace

BackendVec3 backend_mat4_transform_point(const BackendMat4& m, const BackendVec3& p)
{
	const engine::RigidTransform rt = rigidFromBackendMat4(m);
	const Eigen::Vector3d out =
		rt.isometry() * Eigen::Vector3d(p.x, p.y, p.z);
	return BackendVec3{out.x(), out.y(), out.z()};
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
	const engine::RigidTransform inv = rigidFromBackendMat4(objectWorldMatrix(obj, mgr)).inverse();
	const Eigen::Vector3d out =
		inv.isometry() * Eigen::Vector3d(vWorld.x, vWorld.y, vWorld.z);
	return BackendVec3{out.x(), out.y(), out.z()};
}
