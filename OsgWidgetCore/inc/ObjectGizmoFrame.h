#pragma once

#include "osgwidgetcore_global.h"

#include <osg/Matrixd>
#include <osg/MatrixTransform>
#include <osg/Quat>
#include <osg/Vec3f>

/// Single source of truth for object gizmo pose: outer branch uses \c T(center+pose) * R
/// (same convention as \c MeshBackendVisual::buildOuterBranch). Row-vector OSG convention.
class OSGWIDGETCORE_EXPORT ObjectGizmoFrame
{
public:
	ObjectGizmoFrame() = default;

	const osg::Vec3f& modelCenter() const { return m_modelCenter; }
	const osg::Vec3f& centerPlusPose() const { return m_centerPlusPose; }
	const osg::Quat& attitude() const { return m_attitude; }

	osg::Vec3f backendPoseRelativeToCenter() const { return m_centerPlusPose - m_modelCenter; }

	/// Recover center+pose and R from outer \c MatrixTransform (child0 = inner PAT at \c -modelCenter).
	static bool fromOuter(osg::MatrixTransform* outer, const osg::Vec3f& modelCenter, ObjectGizmoFrame& out);

	void setFromBackend(const osg::Vec3f& poseRelativeToCenter, const osg::Quat& attitude, const osg::Vec3f& modelCenter);

	void setCenterPlusPoseAndAttitude(const osg::Vec3f& centerPlusPose, const osg::Quat& attitude);

	void applyToOuter(osg::MatrixTransform* outer) const;

	/// File origin (inner local 0,0,0) in world space.
	static osg::Vec3d pivotWorldFromOuter(osg::MatrixTransform* outer);

	/// File origin in the coordinate system of outer's parent (for \c T_new = pivotParent - (-center)*R_new).
	static osg::Vec3d pivotInOuterParentFromOuter(osg::MatrixTransform* outer, const osg::Vec3f& modelCenter);

	static osg::Matrixd outerLocalMatrix(const osg::Vec3f& centerPlusPose, const osg::Quat& attitude);

	/// Move inner-origin pivot in outer-parent space (row-vector \c T(cpp)*R); \a outer supplies parent chain.
	void translateAlongWorldAxis(osg::MatrixTransform* outer, int axisIndex, double deltaWorld);
	void translateAlongBodyAxis(osg::MatrixTransform* outer, int axisIndex, double deltaWorld);

	void rotatePreMultiplyWorldAxis(int axisIndex, double deltaRad);
	void rotatePostMultiplyLocalAxis(int axisIndex, double deltaRad);

	/// Row-vector body offset pivot: \c T_new = T_old + (-c)*R_old - (-c)*R_new.
	void adjustCenterPlusPoseForRotationDelta(const osg::Quat& R_old, const osg::Quat& R_new);

	/// Keep pivot fixed in outer-parent space: \c T_new = pivotInParent - (-center)*R_new (row-vector).
	void setRotationKeepingPivotInOuterParent(const osg::Vec3d& pivotInOuterParent, const osg::Quat& newAttitude);

private:
	osg::Vec3d pivotInOuterParentFromMembers() const;
	void setCenterPlusPoseFromPivotInOuterParent(const osg::Vec3d& pivotInOuterParent);
	void translatePivotInOuterParent(const osg::Vec3d& deltaInOuterParent);

	/// Inner PAT translation in outer local space (= \c -modelCenter for standard \c buildOuterBranch).
	osg::Vec3f m_innerOriginInOuterLocal{};
	osg::Vec3f m_modelCenter{};
	osg::Vec3f m_centerPlusPose{};
	osg::Quat m_attitude{};
};
