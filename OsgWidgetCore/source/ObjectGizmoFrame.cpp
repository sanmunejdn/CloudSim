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
	if (axisIndex == 0) return osg::Vec3f(1.0f, 0.0f, 0.0f);
	if (axisIndex == 1) return osg::Vec3f(0.0f, 1.0f, 0.0f);
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
} // namespace

osg::Matrixd ObjectGizmoFrame::outerLocalMatrix(const osg::Vec3f& centerPlusPose, const osg::Quat& attitude)
{
	return osg::Matrixd::translate(osg::Vec3d(centerPlusPose.x(), centerPlusPose.y(), centerPlusPose.z()))
		* osg::Matrixd::rotate(attitude);
}

bool ObjectGizmoFrame::fromOuter(osg::MatrixTransform* outer, const osg::Vec3f& modelCenter, ObjectGizmoFrame& out)
{
	if (!outer || outer->getNumChildren() < 1)
	{
		return false;
	}
	osg::Quat q;
	osg::Vec3d td;
	osg::Vec3d s;
	osg::Quat so;
	outer->getMatrix().decompose(td, q, s, so);

	osg::Node* const inner0 = outer->getChild(0);
	osg::NodePath pathToInner = nodePathToSceneRootFromLeaf(outer);
	pathToInner.push_back(inner0);
	const osg::Vec3d fileOrigin = osg::Vec3d(0.0, 0.0, 0.0) * osg::computeLocalToWorld(pathToInner);

	osg::NodePath pathOuter = nodePathToSceneRootFromLeaf(outer);
	osg::Matrixd parentWorld;
	if (pathOuter.size() >= 2U)
	{
		osg::NodePath parentPath;
		parentPath.reserve(pathOuter.size() - 1U);
		for (unsigned i = 0; i + 1U < pathOuter.size(); ++i)
		{
			parentPath.push_back(pathOuter[i]);
		}
		parentWorld = osg::computeLocalToWorld(parentPath);
	}
	else
	{
		parentWorld.makeIdentity();
	}
	const osg::Vec3d fileInOuterParent = fileOrigin * osg::Matrixd::inverse(parentWorld);
	osg::Vec3d innerOffset;
	if (!readInnerOriginInOuterLocal(outer, innerOffset))
	{
		innerOffset.set(
			static_cast<double>(-modelCenter.x()),
			static_cast<double>(-modelCenter.y()),
			static_cast<double>(-modelCenter.z()));
	}
	const osg::Vec3d rotatedOff = innerOffset * osg::Matrixd::rotate(q);
	const osg::Vec3d outerTranslate = fileInOuterParent - rotatedOff;

	out.m_innerOriginInOuterLocal.set(
		static_cast<float>(innerOffset.x()),
		static_cast<float>(innerOffset.y()),
		static_cast<float>(innerOffset.z()));
	out.m_modelCenter.set(
		static_cast<float>(-innerOffset.x()),
		static_cast<float>(-innerOffset.y()),
		static_cast<float>(-innerOffset.z()));
	out.m_centerPlusPose.set(
		static_cast<float>(outerTranslate.x()),
		static_cast<float>(outerTranslate.y()),
		static_cast<float>(outerTranslate.z()));
	out.m_attitude = q;
	return true;
}

void ObjectGizmoFrame::setFromBackend(const osg::Vec3f& poseRelativeToCenter, const osg::Quat& attitude,
	const osg::Vec3f& modelCenter)
{
	m_modelCenter = modelCenter;
	m_innerOriginInOuterLocal.set(-modelCenter.x(), -modelCenter.y(), -modelCenter.z());
	m_centerPlusPose = modelCenter + poseRelativeToCenter;
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
	const osg::Vec3d w = osg::Vec3d(0.0, 0.0, 0.0) * osg::computeLocalToWorld(path);
	return w;
}

osg::Vec3d ObjectGizmoFrame::pivotInOuterParentFromOuter(osg::MatrixTransform* outer, const osg::Vec3f& modelCenter)
{
	(void)modelCenter;
	const osg::Vec3d pivotW = pivotWorldFromOuter(outer);
	osg::NodePath pathOuter = nodePathToSceneRootFromLeaf(outer);
	osg::Matrixd parentWorld;
	if (pathOuter.size() >= 2U)
	{
		osg::NodePath parentPath;
		parentPath.reserve(pathOuter.size() - 1U);
		for (unsigned i = 0; i + 1U < pathOuter.size(); ++i)
		{
			parentPath.push_back(pathOuter[i]);
		}
		parentWorld = osg::computeLocalToWorld(parentPath);
	}
	else
	{
		parentWorld.makeIdentity();
	}
	return pivotW * osg::Matrixd::inverse(parentWorld);
}

osg::Vec3d ObjectGizmoFrame::pivotInOuterParentFromMembers() const
{
	const osg::Vec3d inner(
		static_cast<double>(m_innerOriginInOuterLocal.x()),
		static_cast<double>(m_innerOriginInOuterLocal.y()),
		static_cast<double>(m_innerOriginInOuterLocal.z()));
	const osg::Vec3d cpp(
		static_cast<double>(m_centerPlusPose.x()),
		static_cast<double>(m_centerPlusPose.y()),
		static_cast<double>(m_centerPlusPose.z()));
	return cpp + inner * osg::Matrixd::rotate(m_attitude);
}

void ObjectGizmoFrame::setCenterPlusPoseFromPivotInOuterParent(const osg::Vec3d& pivotInOuterParent)
{
	const osg::Vec3d inner(
		static_cast<double>(m_innerOriginInOuterLocal.x()),
		static_cast<double>(m_innerOriginInOuterLocal.y()),
		static_cast<double>(m_innerOriginInOuterLocal.z()));
	const osg::Vec3d T_new = pivotInOuterParent - inner * osg::Matrixd::rotate(m_attitude);
	m_centerPlusPose.set(
		static_cast<float>(T_new.x()),
		static_cast<float>(T_new.y()),
		static_cast<float>(T_new.z()));
}

void ObjectGizmoFrame::translatePivotInOuterParent(const osg::Vec3d& deltaInOuterParent)
{
	osg::Vec3d pivot = pivotInOuterParentFromMembers();
	pivot += deltaInOuterParent;
	setCenterPlusPoseFromPivotInOuterParent(pivot);
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
	const osg::Vec3d rBodyD(
		static_cast<double>(m_innerOriginInOuterLocal.x()),
		static_cast<double>(m_innerOriginInOuterLocal.y()),
		static_cast<double>(m_innerOriginInOuterLocal.z()));
	const osg::Matrixd R_old_m = osg::Matrixd::rotate(R_old);
	const osg::Matrixd R_new_m = osg::Matrixd::rotate(R_new);
	const osg::Vec3d T_old_d(
		static_cast<double>(m_centerPlusPose.x()),
		static_cast<double>(m_centerPlusPose.y()),
		static_cast<double>(m_centerPlusPose.z()));
	const osg::Vec3d wOld = rBodyD * R_old_m;
	const osg::Vec3d wNew = rBodyD * R_new_m;
	const osg::Vec3d T_new_d = T_old_d + wOld - wNew;
	m_centerPlusPose.set(static_cast<float>(T_new_d.x()), static_cast<float>(T_new_d.y()), static_cast<float>(T_new_d.z()));
	m_attitude = R_new;
}

void ObjectGizmoFrame::setRotationKeepingPivotInOuterParent(const osg::Vec3d& pivotInOuterParent,
	const osg::Quat& newAttitude)
{
	const osg::Vec3d rBodyD(
		static_cast<double>(m_innerOriginInOuterLocal.x()),
		static_cast<double>(m_innerOriginInOuterLocal.y()),
		static_cast<double>(m_innerOriginInOuterLocal.z()));
	const osg::Matrixd R_new_m = osg::Matrixd::rotate(newAttitude);
	const osg::Vec3d T_new_d = pivotInOuterParent - rBodyD * R_new_m;
	m_centerPlusPose.set(static_cast<float>(T_new_d.x()), static_cast<float>(T_new_d.y()), static_cast<float>(T_new_d.z()));
	m_attitude = newAttitude;
}
