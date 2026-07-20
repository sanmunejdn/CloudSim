/// @file BackendSpatial.cpp
/// @brief BackendSpatial 实现

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
	const Eigen::Vector3d out = rt.isometry() * Eigen::Vector3d(p.x, p.y, p.z);
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
	const Eigen::Vector3d out = inv.isometry() * Eigen::Vector3d(vWorld.x, vWorld.y, vWorld.z);
	return BackendVec3{out.x(), out.y(), out.z()};
}

void transformXyzToWorld(std::vector<float>& xyz, const BackendMat4& worldMatrix)
{
	if (xyz.empty())
	{
		return;
	}
	const Eigen::Isometry3d iso = rigidFromBackendMat4(worldMatrix).isometry();
	const std::size_t n = xyz.size() / 3U;
	for (std::size_t i = 0U; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		Eigen::Vector3d p(static_cast<double>(xyz[b]), static_cast<double>(xyz[b + 1U]),
						  static_cast<double>(xyz[b + 2U]));
		p = iso * p;
		xyz[b] = static_cast<float>(p.x());
		xyz[b + 1U] = static_cast<float>(p.y());
		xyz[b + 2U] = static_cast<float>(p.z());
	}
}

void transformTriangleSoupToWorld(std::vector<float>& triangleSoup, const BackendMat4& worldMatrix)
{
	if (triangleSoup.empty())
	{
		return;
	}
	const Eigen::Isometry3d iso = rigidFromBackendMat4(worldMatrix).isometry();
	const std::size_t triCount = triangleSoup.size() / 9U;
	for (std::size_t t = 0U; t < triCount; ++t)
	{
		for (int v = 0; v < 3; ++v)
		{
			const std::size_t base = t * 9U + static_cast<std::size_t>(v) * 3U;
			Eigen::Vector3d p(static_cast<double>(triangleSoup[base]), static_cast<double>(triangleSoup[base + 1U]),
							  static_cast<double>(triangleSoup[base + 2U]));
			p = iso * p;
			triangleSoup[base] = static_cast<float>(p.x());
			triangleSoup[base + 1U] = static_cast<float>(p.y());
			triangleSoup[base + 2U] = static_cast<float>(p.z());
		}
	}
}
