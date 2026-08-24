#include "CustomDeviceMat4Layout.h"

#include "Mat4Ops.h"

#include <Adapters.h>

namespace CustomDeviceMat4Layout
{
namespace
{
engine::RigidTransform rigidFromOsgPackedMat4(const double m[16])
{
	engine::ColMajorMat4 cm{};
	for (int i = 0; i < 16; ++i)
	{
		cm[static_cast<size_t>(i)] = m[i];
	}
	return engine::rigidTransformFromColMajor(cm);
}

void rigidToKinematicCoreMat4(const engine::RigidTransform& rt, double out[16])
{
	const Eigen::Isometry3d& iso = rt.isometry();
	kinematic_core::mat4IdentityColumnMajor(out);
	for (int r = 0; r < 3; ++r)
	{
		for (int c = 0; c < 3; ++c)
		{
			out[c * 4 + r] = iso.linear()(r, c);
		}
	}
	out[12] = iso.translation().x();
	out[13] = iso.translation().y();
	out[14] = iso.translation().z();
}

engine::RigidTransform rigidFromKinematicCoreMat4(const double m[16])
{
	Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
	for (int r = 0; r < 3; ++r)
	{
		for (int c = 0; c < 3; ++c)
		{
			iso.linear()(r, c) = m[c * 4 + r];
		}
	}
	iso.translation() = Eigen::Vector3d(m[12], m[13], m[14]);
	return engine::RigidTransform::fromIsometry(iso);
}
} // namespace

void osgBackendToKinematicCore(const double osgBackend[16], double kinematicCore[16])
{
	rigidToKinematicCoreMat4(rigidFromOsgPackedMat4(osgBackend), kinematicCore);
}

void kinematicCoreToOsgBackend(const double kinematicCore[16], double osgBackend[16])
{
	const engine::ColMajorMat4 cm = engine::colMajorFromRigidTransform(rigidFromKinematicCoreMat4(kinematicCore));
	for (int i = 0; i < 16; ++i)
	{
		osgBackend[i] = cm[static_cast<size_t>(i)];
	}
}

void backendMat4ToKinematicCore(const BackendMat4& src, double kinematicCore[16])
{
	osgBackendToKinematicCore(src.v, kinematicCore);
}

BackendMat4 kinematicCoreToBackendMat4(const double kinematicCore[16])
{
	BackendMat4 out = BackendMat4::identity();
	kinematicCoreToOsgBackend(kinematicCore, out.v);
	return out;
}

bool kinematicCoreInvertRigid(const double kinematicCore[16], double outKinematicCore[16])
{
	BackendMat4 inv{};
	if (!backend_mat4_invert_rigid(kinematicCoreToBackendMat4(kinematicCore), inv))
	{
		return false;
	}
	backendMat4ToKinematicCore(inv, outKinematicCore);
	return true;
}

} // namespace CustomDeviceMat4Layout
