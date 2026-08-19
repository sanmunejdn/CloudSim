/// @file AssemblyMateApply.cpp
/// @brief 装配一次定位写回 worldMatrix（对齐 ICP 左乘）

#include "AssemblyMateApply.h"

#include "Adapters.h"
#include "BackendDataBase.h"
#include "BrepBackendData.h"
#include "CoreTypes.h"
#include "DocumentHost.h"
#include "DocumentHostEvents.h"
#include "IDataService.h"
#include "RigidTransform.h"
#include "ShapeQuery.h"

#include <Eigen/Geometry>
#include <memory>

namespace cloudsim::host
{
namespace
{
void setErr(QString* err, const QString& msg)
{
	if (err)
	{
		*err = msg;
	}
}

engine::RigidTransform rigidFromMat(const BackendMat4& m)
{
	engine::ColMajorMat4 cm{};
	for (int i = 0; i < 16; ++i)
	{
		cm[static_cast<size_t>(i)] = m.v[i];
	}
	return engine::rigidTransformFromColMajor(cm);
}

BackendMat4 matFromRigid(const engine::RigidTransform& rt)
{
	const engine::ColMajorMat4 cm = engine::colMajorFromRigidTransform(rt);
	BackendMat4 out{};
	for (int i = 0; i < 16; ++i)
	{
		out.v[i] = cm[static_cast<size_t>(i)];
	}
	return out;
}

std::shared_ptr<BrepBackendData> asBrep(DocumentHost& host, const std::string& id, QString* err)
{
	const auto obj = host.findObject(id);
	auto brep = std::dynamic_pointer_cast<BrepBackendData>(obj);
	if (!brep || brep->shapeRef().isNull())
	{
		setErr(err, QStringLiteral("不是有效的 B-rep 对象"));
		return {};
	}
	return brep;
}

} // namespace

bool resolveAssemblyMatePick(DocumentHost& host, const std::string& brepId, const int faceIndex, const double pickWorldX,
							 const double pickWorldY, const double pickWorldZ, AssemblyMateFaceRef& out,
							 QString* outError)
{
	out = {};
	const auto brep = asBrep(host, brepId, outError);
	if (!brep)
	{
		return false;
	}
	std::string qerr;
	if (!geoalgo::validateShapeFaceIndex(brep->shapeRef(), faceIndex, &qerr))
	{
		setErr(outError, qerr.empty() ? QStringLiteral("面索引无效") : QString::fromStdString(qerr));
		return false;
	}
	out.backendId = brepId;
	out.faceIndex = faceIndex;
	out.pickWorldMm[0] = pickWorldX;
	out.pickWorldMm[1] = pickWorldY;
	out.pickWorldMm[2] = pickWorldZ;
	return true;
}

bool snapshotBackendWorldMatrix(DocumentHost& host, const std::string& backendId, BackendMat4& outWorld)
{
	const auto obj = host.findObject(backendId);
	if (!obj)
	{
		return false;
	}
	outWorld = obj->worldMatrix();
	return true;
}

bool restoreBackendWorldMatrix(DocumentHost& host, const std::string& backendId, const BackendMat4& world,
							   QString* outError)
{
	const auto obj = host.findObject(backendId);
	if (!obj)
	{
		setErr(outError, QStringLiteral("对象不存在"));
		return false;
	}
	obj->setWorldMatrix(world);
	(void)host.syncOuterPatFromBackendId(backendId);
	return true;
}

bool applyAssemblyMate(DocumentHost& host, const AssemblyMateFaceRef& grounded, const AssemblyMateFaceRef& moving,
					   const geoalgo::AssemblyMateParams& params, const BackendMat4* movingWorldSnapshot,
					   const bool commit, QString* outError)
{
	if (grounded.backendId.empty() || moving.backendId.empty() || grounded.backendId == moving.backendId)
	{
		setErr(outError, QStringLiteral("需要两个不同的 B-rep 对象"));
		return false;
	}
	if (movingWorldSnapshot)
	{
		if (!restoreBackendWorldMatrix(host, moving.backendId, *movingWorldSnapshot, outError))
		{
			return false;
		}
	}

	const auto gBrep = asBrep(host, grounded.backendId, outError);
	const auto mBrep = asBrep(host, moving.backendId, outError);
	if (!gBrep || !mBrep)
	{
		return false;
	}

	const geoalgo::Point3d gPick{grounded.pickWorldMm[0], grounded.pickWorldMm[1], grounded.pickWorldMm[2]};
	const geoalgo::Point3d mPick{moving.pickWorldMm[0], moving.pickWorldMm[1], moving.pickWorldMm[2]};
	geoalgo::FaceMateGeom gGeom;
	geoalgo::FaceMateGeom mGeom;
	std::string err;
	if (!geoalgo::queryFaceMateGeom(gBrep->worldShape(), grounded.faceIndex, &gPick, gGeom, &err))
	{
		setErr(outError, err.empty() ? QStringLiteral("固定面几何查询失败") : QString::fromStdString(err));
		return false;
	}
	if (!geoalgo::queryFaceMateGeom(mBrep->worldShape(), moving.faceIndex, &mPick, mGeom, &err))
	{
		setErr(outError, err.empty() ? QStringLiteral("动件面几何查询失败") : QString::fromStdString(err));
		return false;
	}

	Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
	if (!geoalgo::computeAssemblyMateDelta(gGeom, mGeom, params, delta, &err))
	{
		setErr(outError, err.empty() ? QStringLiteral("配合计算失败") : QString::fromStdString(err));
		return false;
	}

	const engine::RigidTransform deltaRt = engine::RigidTransform::fromIsometry(delta);
	const engine::RigidTransform curRt = rigidFromMat(mBrep->worldMatrix());
	mBrep->setWorldMatrix(matFromRigid(deltaRt.composeColumn(curRt)));
	(void)host.syncOuterPatFromBackendId(moving.backendId);

	if (!commit)
	{
		return true;
	}

	publishPoseCommittedFromBackend(host, *mBrep);
	host.data().markFollowDirtyFromMove(QString::fromStdString(moving.backendId));
	cloudsim::core::FollowSolveContextDto ctx;
	ctx.manualPoseAuthorityBackendId = QString::fromStdString(moving.backendId);
	(void)host.data().runFollowSolveAndSync(ctx, outError);
	return true;
}

} // namespace cloudsim::host
