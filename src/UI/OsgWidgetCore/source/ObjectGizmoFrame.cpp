/// @file ObjectGizmoFrame.cpp
/// @brief ObjectGizmoFrame 实现

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "ObjectGizmoFrame.h"

#include <cmath>

#include <Adapters.h>
#include <RigidTransform.h>
#include <osg/MatrixTransform>
#include <osg/Node>
#include <osg/PositionAttitudeTransform>

namespace
{
bool readInnerOriginInOuterLocal(osg::MatrixTransform* outer, osg::Vec3d& outInnerOrigin)
{
	if (!outer || outer->getNumChildren() < 1)
	{
		return false;
	}
	osg::Node* const inner = outer->getChild(0);
	if (auto* pat = dynamic_cast<osg::PositionAttitudeTransform*>(inner))
	{
		const osg::Vec3d p = pat->getPosition();
		outInnerOrigin.set(p.x(), p.y(), p.z());
		return true;
	}
	if (auto* mt = dynamic_cast<osg::MatrixTransform*>(inner))
	{
		osg::Vec3d t;
		osg::Quat r;
		osg::Vec3d s;
		osg::Quat so;
		mt->getMatrix().decompose(t, r, s, so);
		outInnerOrigin = t;
		return true;
	}
	outInnerOrigin.set(0.0, 0.0, 0.0);
	return true;
}

osg::NodePath nodePathToSceneRootFromLeaf(const osg::Node* leaf)
{
	osg::NodePath path;
	for (const osg::Node* n = leaf; n != nullptr; n = n->getNumParents() > 0 ? n->getParent(0) : nullptr)
	{
		path.insert(path.begin(), const_cast<osg::Node*>(n));
	}
	return path;
}

osg::Vec3f worldAxis(int axisIndex)
{
	if (axisIndex == 0)
	{
		return osg::Vec3f(1.0f, 0.0f, 0.0f);
	}
	if (axisIndex == 1)
	{
		return osg::Vec3f(0.0f, 1.0f, 0.0f);
	}
	return osg::Vec3f(0.0f, 0.0f, 1.0f);
}

bool parentWorldMatrixOfOuter(osg::MatrixTransform* outer, osg::Matrixd& outParentWorld)
{
	if (!outer)
	{
		return false;
	}
	osg::NodePath pathOuter = nodePathToSceneRootFromLeaf(outer);
	if (pathOuter.size() >= 2U)
	{
		osg::NodePath parentPath;
		parentPath.reserve(pathOuter.size() - 1U);
		for (unsigned i = 0; i + 1U < pathOuter.size(); ++i)
		{
			parentPath.push_back(pathOuter[i]);
		}
		outParentWorld = osg::computeLocalToWorld(parentPath);
		return true;
	}
	outParentWorld.makeIdentity();
	return true;
}

osg::Vec3d worldDirectionToOuterParent(const osg::Vec3d& worldDir, const osg::Matrixd& parentWorld)
{
	const osg::Matrixd invParent = osg::Matrixd::inverse(parentWorld);
	return osg::Matrixd::transform3x3(worldDir, invParent);
}

engine::RigidTransform rigidFromPoseAndAttitude(const osg::Vec3f& pose, const osg::Quat& attitude)
{
	const Eigen::Quaterniond rq(attitude.w(), attitude.x(), attitude.y(), attitude.z());
	return engine::RigidTransform::fromTranslationQuat(Eigen::Vector3d(pose.x(), pose.y(), pose.z()), rq);
}

} // namespace

osg::Matrixd ObjectGizmoFrame::outerLocalMatrix(const osg::Vec3f& centerPlusPose, const osg::Quat& attitude)
{
	return engine::osgMatrixFromRigidTransform(rigidFromPoseAndAttitude(centerPlusPose, attitude));
}

bool ObjectGizmoFrame::fromOuter(osg::MatrixTransform* outer, const osg::Vec3f& modelCenter, ObjectGizmoFrame& out)
{
	if (!outer || outer->getNumChildren() < 1)
	{
		return false;
	}
	const engine::RigidTransform rt = engine::rigidTransformFromOsg(outer->getMatrix());
	const Eigen::Vector3d t = rt.translationMm();
	const Eigen::Quaterniond rq = rt.rotation().normalized();

	osg::Vec3d innerOffset;
	if (!readInnerOriginInOuterLocal(outer, innerOffset))
	{
		innerOffset.set(static_cast<double>(-modelCenter.x()), static_cast<double>(-modelCenter.y()),
						static_cast<double>(-modelCenter.z()));
	}

	out.m_innerOriginInOuterLocal.set(static_cast<float>(innerOffset.x()), static_cast<float>(innerOffset.y()),
									  static_cast<float>(innerOffset.z()));
	out.m_modelCenter.set(static_cast<float>(-innerOffset.x()), static_cast<float>(-innerOffset.y()),
						  static_cast<float>(-innerOffset.z()));
	out.m_centerPlusPose.set(static_cast<float>(t.x()), static_cast<float>(t.y()), static_cast<float>(t.z()));
	out.m_attitude.set(static_cast<float>(rq.x()), static_cast<float>(rq.y()), static_cast<float>(rq.z()),
					   static_cast<float>(rq.w()));
	return true;
}

void ObjectGizmoFrame::setFromBackend(const osg::Vec3f& poseRelativeToCenter, const osg::Quat& attitude,
									  const osg::Vec3f& modelCenter)
{
	(void)modelCenter;
	m_modelCenter.set(0.0f, 0.0f, 0.0f);
	m_innerOriginInOuterLocal.set(0.0f, 0.0f, 0.0f);
	m_centerPlusPose = poseRelativeToCenter;
	m_attitude = attitude;
}

void ObjectGizmoFrame::setCenterPlusPoseAndAttitude(const osg::Vec3f& centerPlusPose, const osg::Quat& attitude)
{
	m_centerPlusPose = centerPlusPose;
	m_attitude = attitude;
}

void ObjectGizmoFrame::applyToOuter(osg::MatrixTransform* outer) const
{
	if (!outer)
	{
		return;
	}
	outer->setMatrix(outerLocalMatrix(m_centerPlusPose, m_attitude));
}

osg::Vec3d ObjectGizmoFrame::pivotWorldFromOuter(osg::MatrixTransform* outer)
{
	if (!outer || outer->getNumChildren() < 1)
	{
		return osg::Vec3d(0.0, 0.0, 0.0);
	}
	osg::NodePath path = nodePathToSceneRootFromLeaf(outer);
	path.push_back(outer->getChild(0));
	return osg::Vec3d(0.0, 0.0, 0.0) * osg::computeLocalToWorld(path);
}

osg::Vec3d ObjectGizmoFrame::pivotInOuterParentFromOuter(osg::MatrixTransform* outer, const osg::Vec3f& modelCenter)
{
	(void)modelCenter;
	const osg::Vec3d pivotW = pivotWorldFromOuter(outer);
	osg::Matrixd parentWorld;
	if (!parentWorldMatrixOfOuter(outer, parentWorld))
	{
		return pivotW;
	}
	return pivotW * osg::Matrixd::inverse(parentWorld);
}

osg::Vec3d ObjectGizmoFrame::pivotInOuterParentFromWorld(osg::MatrixTransform* outer, const osg::Vec3d& pivotWorld)
{
	osg::Matrixd parentWorld;
	if (!parentWorldMatrixOfOuter(outer, parentWorld))
	{
		return pivotWorld;
	}
	return pivotWorld * osg::Matrixd::inverse(parentWorld);
}

osg::Vec3d ObjectGizmoFrame::pivotInOuterParentFromMembers() const
{
	return osg::Vec3d(static_cast<double>(m_centerPlusPose.x()), static_cast<double>(m_centerPlusPose.y()),
					  static_cast<double>(m_centerPlusPose.z()));
}

void ObjectGizmoFrame::setCenterPlusPoseFromPivotInOuterParent(const osg::Vec3d& pivotInOuterParent)
{
	m_centerPlusPose.set(static_cast<float>(pivotInOuterParent.x()), static_cast<float>(pivotInOuterParent.y()),
						 static_cast<float>(pivotInOuterParent.z()));
}

void ObjectGizmoFrame::translatePivotInOuterParent(const osg::Vec3d& deltaInOuterParent)
{
	m_centerPlusPose.set(static_cast<float>(static_cast<double>(m_centerPlusPose.x()) + deltaInOuterParent.x()),
						 static_cast<float>(static_cast<double>(m_centerPlusPose.y()) + deltaInOuterParent.y()),
						 static_cast<float>(static_cast<double>(m_centerPlusPose.z()) + deltaInOuterParent.z()));
}

void ObjectGizmoFrame::translateAlongWorldAxis(osg::MatrixTransform* outer, int axisIndex, double deltaWorld)
{
	if (!outer || std::abs(deltaWorld) < 1e-20)
	{
		return;
	}
	const osg::Vec3f axf = worldAxis(axisIndex);
	osg::Vec3d axisW(static_cast<double>(axf.x()), static_cast<double>(axf.y()), static_cast<double>(axf.z()));
	osg::Matrixd parentWorld;
	parentWorldMatrixOfOuter(outer, parentWorld);
	osg::Vec3d deltaParent = worldDirectionToOuterParent(axisW, parentWorld);
	const double len = deltaParent.length();
	if (len > 1e-12)
	{
		deltaParent *= (deltaWorld / len);
	}
	translatePivotInOuterParent(deltaParent);
}

void ObjectGizmoFrame::translateAlongBodyAxis(osg::MatrixTransform* outer, int axisIndex, double deltaWorld)
{
	if (!outer || std::abs(deltaWorld) < 1e-20)
	{
		return;
	}
	const osg::Vec3f loc = worldAxis(axisIndex);
	const osg::Vec3f w = m_attitude * loc;
	osg::Vec3d axisW(static_cast<double>(w.x()), static_cast<double>(w.y()), static_cast<double>(w.z()));
	translateAlongWorldDirection(outer, axisW, deltaWorld);
}

void ObjectGizmoFrame::translateAlongWorldDirection(osg::MatrixTransform* outer, const osg::Vec3d& axisWUnit,
													double deltaWorld)
{
	if (!outer || std::abs(deltaWorld) < 1e-20)
	{
		return;
	}
	const double len = axisWUnit.length();
	if (len < 1e-12)
	{
		return;
	}
	osg::Matrixd parentWorld;
	parentWorldMatrixOfOuter(outer, parentWorld);
	osg::Vec3d deltaParent = worldDirectionToOuterParent(axisWUnit / len, parentWorld);
	const double deltaLen = deltaParent.length();
	if (deltaLen > 1e-12)
	{
		deltaParent *= (deltaWorld / deltaLen);
	}
	translatePivotInOuterParent(deltaParent);
}

bool ObjectGizmoFrame::dragAxisDirectionSceneWorld(osg::MatrixTransform* outer, bool worldGizmoFrame,
												   const osg::Quat& outerAttitude, int axisIndex, osg::Vec3d& outUnit)
{
	outUnit.set(0.0, 0.0, 1.0);
	if (!outer || axisIndex < 0 || axisIndex > 2)
	{
		return false;
	}
	osg::Vec3d axisInParent;
	if (worldGizmoFrame)
	{
		axisInParent = osg::Vec3d(axisIndex == 0 ? 1.0 : 0.0, axisIndex == 1 ? 1.0 : 0.0, axisIndex == 2 ? 1.0 : 0.0);
	}
	else
	{
		const osg::Vec3f w = outerAttitude * worldAxis(axisIndex);
		axisInParent.set(static_cast<double>(w.x()), static_cast<double>(w.y()), static_cast<double>(w.z()));
	}
	osg::Matrixd parentWorld;
	if (!parentWorldMatrixOfOuter(outer, parentWorld))
	{
		return false;
	}
	outUnit = osg::Matrixd::transform3x3(axisInParent, parentWorld);
	const double len = outUnit.length();
	if (len < 1e-12)
	{
		return false;
	}
	outUnit /= len;
	return true;
}

bool ObjectGizmoFrame::dragAxisDirectionOuterParent(osg::MatrixTransform* outer, bool worldGizmoFrame,
													const osg::Quat& outerAttitude, int axisIndex, osg::Vec3d& outUnit)
{
	outUnit.set(0.0, 0.0, 1.0);
	if (!outer || axisIndex < 0 || axisIndex > 2)
	{
		return false;
	}
	if (worldGizmoFrame)
	{
		const osg::Vec3d axisScene(axisIndex == 0 ? 1.0 : 0.0, axisIndex == 1 ? 1.0 : 0.0, axisIndex == 2 ? 1.0 : 0.0);
		osg::Matrixd parentWorld;
		if (!parentWorldMatrixOfOuter(outer, parentWorld))
		{
			return false;
		}
		outUnit = worldDirectionToOuterParent(axisScene, parentWorld);
	}
	else
	{
		const osg::Vec3f w = outerAttitude * worldAxis(axisIndex);
		outUnit.set(static_cast<double>(w.x()), static_cast<double>(w.y()), static_cast<double>(w.z()));
	}
	const double len = outUnit.length();
	if (len < 1e-12)
	{
		return false;
	}
	outUnit /= len;
	return true;
}

void ObjectGizmoFrame::rotatePreMultiplyWorldAxis(int axisIndex, double deltaRad)
{
	const osg::Vec3 ax(worldAxis(axisIndex));
	const osg::Quat deltaQuat(static_cast<float>(deltaRad), ax);
	m_attitude = deltaQuat * m_attitude;
}

void ObjectGizmoFrame::rotatePostMultiplyLocalAxis(int axisIndex, double deltaRad)
{
	const osg::Vec3 ax(worldAxis(axisIndex));
	const osg::Quat deltaQuat(static_cast<float>(deltaRad), ax);
	m_attitude = m_attitude * deltaQuat;
}

void ObjectGizmoFrame::adjustCenterPlusPoseForRotationDelta(const osg::Quat& R_old, const osg::Quat& R_new)
{
	(void)R_old;
	m_attitude = R_new;
}

void ObjectGizmoFrame::setRotationKeepingPivotInOuterParent(const osg::Vec3d& pivotInOuterParent,
															const osg::Quat& newAttitude)
{
	(void)pivotInOuterParent;
	m_attitude = newAttitude;
}
