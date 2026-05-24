#pragma once

#include "osgwidgetcore_global.h"

#include <osg/Matrixd>
#include <osg/MatrixTransform>
#include <osg/Quat>
#include <osg/Vec3f>

/// 对象变换罗盘位姿单一来源：外层 \c T(center+pose)*R（与 \c MeshBackendVisual::buildOuterBranch 一致，行向量 OSG）
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
	/// World pivot → outer-parent space (recompute each rotate frame from frozen world pivot).
	static osg::Vec3d pivotInOuterParentFromWorld(osg::MatrixTransform* outer, const osg::Vec3d& pivotWorld);

	static osg::Matrixd outerLocalMatrix(const osg::Vec3f& centerPlusPose, const osg::Quat& attitude);

	/// Move inner-origin pivot in outer-parent space (row-vector \c T(cpp)*R); \a outer supplies parent chain.
	void translateAlongWorldAxis(osg::MatrixTransform* outer, int axisIndex, double deltaWorld);
	void translateAlongBodyAxis(osg::MatrixTransform* outer, int axisIndex, double deltaWorld);
	/// Move pivot along a unit world direction (screen-drag axis from \c gizmoCompassUnitAxisWorld).
	void translateAlongWorldDirection(osg::MatrixTransform* outer, const osg::Vec3d& axisWUnit, double deltaWorld);

	/// Pivot of file origin in outer-parent space (matches \c setRotationKeepingPivotInOuterParent).
	osg::Vec3d pivotInOuterParentFromMembers() const;

	/// 罗盘环法向 / 屏幕旋转：场景世界单位方向（与 \c computeGizmoPivotWorld 同坐标系）。
	static bool dragAxisDirectionSceneWorld(osg::MatrixTransform* outer, bool worldGizmoFrame,
		const osg::Quat& outerAttitude, int axisIndex, osg::Vec3d& outUnit);
	/// 写入 \c outer 局部姿态的四元数：outer 父节点坐标系下的旋转轴。
	static bool dragAxisDirectionOuterParent(osg::MatrixTransform* outer, bool worldGizmoFrame,
		const osg::Quat& outerAttitude, int axisIndex, osg::Vec3d& outUnit);

	void rotatePreMultiplyWorldAxis(int axisIndex, double deltaRad);
	void rotatePostMultiplyLocalAxis(int axisIndex, double deltaRad);

	/// Keep file-origin pivot in outer-parent: \c (inner+trans_new)*R_new = (inner+trans_old)*R_old.
	void adjustCenterPlusPoseForRotationDelta(const osg::Quat& R_old, const osg::Quat& R_new);

	/// Keep pivot fixed: solve \c trans from \c (inner+trans)*R = pivotInOuterParent.
	void setRotationKeepingPivotInOuterParent(const osg::Vec3d& pivotInOuterParent, const osg::Quat& newAttitude);

private:
	void setCenterPlusPoseFromPivotInOuterParent(const osg::Vec3d& pivotInOuterParent);
	void translatePivotInOuterParent(const osg::Vec3d& deltaInOuterParent);

	/// Inner PAT translation in outer local space (= \c -modelCenter for standard \c buildOuterBranch).
	osg::Vec3f m_innerOriginInOuterLocal{};
	osg::Vec3f m_modelCenter{};
	osg::Vec3f m_centerPlusPose{};
	osg::Quat m_attitude{};
};
