#include "TrajectoryGeometryResolverHost.h"

#include "FeaturePickTransform.h"
#include "IRobotDocumentHost.h"
#include "IRobotOsgViewHost.h"

#include "BackendDataManager.h"
#include "BrepBackendData.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"
#include "TrajectoryGeometryResolver.h"

#include <osg/Matrixd>

namespace trajectory_geometry_host
{
namespace
{

void osgMatrixToColMajor16(const osg::Matrixd& m, double out[16])
{
	for (int col = 0; col < 4; ++col)
	{
		for (int row = 0; row < 4; ++row)
		{
			out[col * 4 + row] = m(row, col);
		}
	}
}

void adjustColMajorTranslationForModelCenter(double m[16], const double cx, const double cy, const double cz)
{
	const double rx = m[0] * cx + m[4] * cy + m[8] * cz;
	const double ry = m[1] * cx + m[5] * cy + m[9] * cz;
	const double rz = m[2] * cx + m[6] * cy + m[10] * cz;
	m[12] -= rx;
	m[13] -= ry;
	m[14] -= rz;
}

bool bakeModelPointToWorld(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	const float modelX,
	const float modelY,
	const float modelZ,
	float& outWorldX,
	float& outWorldY,
	float& outWorldZ,
	std::string* errMsg)
{
	const geoalgo::Point3d model{
		static_cast<double>(modelX),
		static_cast<double>(modelY),
		static_cast<double>(modelZ)};
	osg::Vec3f world{};
	if (!feature_pick_transform::stepModelPointToWorldMm(osg, backendId, model, world, errMsg))
	{
		return false;
	}
	outWorldX = world.x();
	outWorldY = world.y();
	outWorldZ = world.z();
	return true;
}

bool fillBrepModelToWorld(
	IRobotOsgViewHost* osg,
	const std::string& backendId,
	RobotInstruction::TrajectoryGeometrySnapshot& out,
	std::string* errMsg)
{
	const std::string xformId = osg->resolvePickScopeBackendId(backendId);
	osg::Matrixd worldMat{};
	if (!osg->getBackendRootWorldMatrix(xformId, worldMat))
	{
		if (errMsg)
		{
			*errMsg = "backend world matrix unavailable";
		}
		return false;
	}
	osgMatrixToColMajor16(worldMat, out.modelToWorldColMajor16);
	out.hasModelToWorld = true;
	return true;
}

bool resolvePointCloudSnapshot(
	IRobotOsgViewHost* osg,
	const PointCloudBackendData& data,
	const std::string& backendId,
	RobotInstruction::TrajectoryGeometrySnapshot& out,
	std::string* errMsg)
{
	const std::vector<float>& xyz = data.pointPositionsXyz();
	if (xyz.size() < 3)
	{
		if (errMsg)
		{
			*errMsg = "point cloud has no vertices";
		}
		return false;
	}
	out.kind = RobotInstruction::TrajectoryGeometryKind::PointCloud;
	out.positionsWorldMm.clear();
	out.positionsWorldMm.reserve(xyz.size());
	for (std::size_t i = 0; i + 2 < xyz.size(); i += 3)
	{
		float wx = 0.0f;
		float wy = 0.0f;
		float wz = 0.0f;
		if (!bakeModelPointToWorld(osg, backendId, xyz[i], xyz[i + 1], xyz[i + 2], wx, wy, wz, errMsg))
		{
			return false;
		}
		out.positionsWorldMm.push_back(wx);
		out.positionsWorldMm.push_back(wy);
		out.positionsWorldMm.push_back(wz);
	}
	return true;
}

bool resolveMeshSnapshot(
	IRobotOsgViewHost* osg,
	const MeshBackendData& data,
	const std::string& backendId,
	RobotInstruction::TrajectoryGeometrySnapshot& out,
	std::string* errMsg)
{
	const std::vector<float>& soup = data.triangleSoup();
	if (soup.size() < 9)
	{
		if (errMsg)
		{
			*errMsg = "mesh has no triangles";
		}
		return false;
	}
	out.kind = RobotInstruction::TrajectoryGeometryKind::TriangleMesh;
	out.triangleSoupWorldMm.clear();
	out.triangleSoupWorldMm.reserve(soup.size());
	for (std::size_t i = 0; i + 2 < soup.size(); i += 3)
	{
		float wx = 0.0f;
		float wy = 0.0f;
		float wz = 0.0f;
		if (!bakeModelPointToWorld(osg, backendId, soup[i], soup[i + 1], soup[i + 2], wx, wy, wz, errMsg))
		{
			return false;
		}
		out.triangleSoupWorldMm.push_back(wx);
		out.triangleSoupWorldMm.push_back(wy);
		out.triangleSoupWorldMm.push_back(wz);
	}
	return true;
}

bool resolveBrepSnapshot(
	IRobotOsgViewHost* osg,
	const BrepBackendData& data,
	const std::string& backendId,
	RobotInstruction::TrajectoryGeometrySnapshot& out,
	std::string* errMsg)
{
	if (data.shapeRef().isNull())
	{
		if (errMsg)
		{
			*errMsg = "brep shape is empty";
		}
		return false;
	}
	out.kind = RobotInstruction::TrajectoryGeometryKind::Brep;
	out.brepShape = data.shapeRef();
	return fillBrepModelToWorld(osg, backendId, out, errMsg);
}

} // namespace

void bindTrajectoryGeometryResolver(IRobotDocumentHost* document, IRobotOsgViewHost* osg)
{
	if (!document || !osg)
	{
		RobotInstruction::clearTrajectoryGeometryResolver();
		return;
	}
	RobotInstruction::setTrajectoryGeometryResolver(
		[document, osg](
			const std::string& backendId,
			RobotInstruction::TrajectoryGeometrySnapshot& out,
			std::string* errMsg) -> bool {
			BackendDataManager& mgr = document->backend();
			const std::shared_ptr<BackendDataBase> base = mgr.getData(backendId);
			if (!base || !base->hasGeometry())
			{
				if (errMsg)
				{
					*errMsg = "backend not found or has no geometry";
				}
				return false;
			}
			if (const auto pc = std::dynamic_pointer_cast<PointCloudBackendData>(base))
			{
				return resolvePointCloudSnapshot(osg, *pc, backendId, out, errMsg);
			}
			if (const auto mesh = std::dynamic_pointer_cast<MeshBackendData>(base))
			{
				return resolveMeshSnapshot(osg, *mesh, backendId, out, errMsg);
			}
			if (const auto brep = std::dynamic_pointer_cast<BrepBackendData>(base))
			{
				return resolveBrepSnapshot(osg, *brep, backendId, out, errMsg);
			}
			if (errMsg)
			{
				*errMsg = "unsupported geometry backend type";
			}
			return false;
		});
}

void clearTrajectoryGeometryResolverBinding()
{
	RobotInstruction::clearTrajectoryGeometryResolver();
}

} // namespace trajectory_geometry_host
