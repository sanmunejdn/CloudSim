#ifndef OSGWIDGETCORE_OBJECTGIZMOFRAME_H
#define OSGWIDGETCORE_OBJECTGIZMOFRAME_H

/// @file ObjectGizmoFrame.h
/// @brief 对象变换罗盘位姿单一来源：pose=模型原点在外层父系坐标；矩阵见 engine::osgMatrixFromRigidTransform

#include "osgwidgetcore_global.h"

#include <osg/MatrixTransform>
#include <osg/Matrixd>
#include <osg/Quat>
#include <osg/Vec3f>

/// 对象变换罗盘位姿单一来源：pose=模型原点在外层父系坐标；矩阵见 engine::osgMatrixFromRigidTransform
class OSGWIDGETCORE_EXPORT ObjectGizmoFrame
{
public:
	ObjectGizmoFrame() = default;

	const osg::Vec3f& modelCenter() const { return m_modelCenter; }
	const osg::Vec3f& centerPlusPose() const { return m_centerPlusPose; }
	const osg::Quat& attitude() const { return m_attitude; }

	osg::Vec3f backendPoseRelativeToCenter() const { return m_centerPlusPose; }

	/// 从外层 MatrixTransform 还原 center+pose 与 R（child0 为 -modelCenter 内层 PAT）
	static bool fromOuter(osg::MatrixTransform* outer, const osg::Vec3f& modelCenter, ObjectGizmoFrame& out);

	void setFromBackend(const osg::Vec3f& poseRelativeToCenter, const osg::Quat& attitude,
						const osg::Vec3f& modelCenter);

	void setCenterPlusPoseAndAttitude(const osg::Vec3f& centerPlusPose, const osg::Quat& attitude);

	void applyToOuter(osg::MatrixTransform* outer) const;

	/// 文件原点（内层 0,0,0）世界坐标
	static osg::Vec3d pivotWorldFromOuter(osg::MatrixTransform* outer);

	/// 文件原点在外层父系（T_new = pivotParent - (-center)*R_new）
	static osg::Vec3d pivotInOuterParentFromOuter(osg::MatrixTransform* outer, const osg::Vec3f& modelCenter);
	/// 世界枢轴 → 外层父系（每旋转帧由冻结世界枢轴重算）
	static osg::Vec3d pivotInOuterParentFromWorld(osg::MatrixTransform* outer, const osg::Vec3d& pivotWorld);

	static osg::Matrixd outerLocalMatrix(const osg::Vec3f& centerPlusPose, const osg::Quat& attitude);

	/// 外层父系平移模型原点（pose）
	void translateAlongWorldAxis(osg::MatrixTransform* outer, int axisIndex, double deltaWorld);
	void translateAlongBodyAxis(osg::MatrixTransform* outer, int axisIndex, double deltaWorld);
	/// 沿世界单位方向平移枢轴（屏幕拖拽轴）
	void translateAlongWorldDirection(osg::MatrixTransform* outer, const osg::Vec3d& axisWUnit, double deltaWorld);

	/// 文件原点枢轴在外层父系（同 setRotationKeepingPivotInOuterParent）
	osg::Vec3d pivotInOuterParentFromMembers() const;

	/// 罗盘环法向 / 屏幕旋转：场景世界单位方向（与 computeGizmoPivotWorld 同坐标系）
	static bool dragAxisDirectionSceneWorld(osg::MatrixTransform* outer, bool worldGizmoFrame,
											const osg::Quat& outerAttitude, int axisIndex, osg::Vec3d& outUnit);
	/// 写入 outer 局部姿态四元数：outer 父系旋转轴
	static bool dragAxisDirectionOuterParent(osg::MatrixTransform* outer, bool worldGizmoFrame,
											 const osg::Quat& outerAttitude, int axisIndex, osg::Vec3d& outUnit);

	void rotatePreMultiplyWorldAxis(int axisIndex, double deltaRad);
	void rotatePostMultiplyLocalAxis(int axisIndex, double deltaRad);

	/// 绕模型原点旋转：仅更新 attitude，pose 不变
	void adjustCenterPlusPoseForRotationDelta(const osg::Quat& R_old, const osg::Quat& R_new);

	/// 绕模型原点旋转：仅更新 attitude
	void setRotationKeepingPivotInOuterParent(const osg::Vec3d& pivotInOuterParent, const osg::Quat& newAttitude);

private:
	void setCenterPlusPoseFromPivotInOuterParent(const osg::Vec3d& pivotInOuterParent);
	void translatePivotInOuterParent(const osg::Vec3d& deltaInOuterParent);

	/// 内层 PAT 在外层局部平移（标准 buildOuterBranch 为 -modelCenter）
	osg::Vec3f m_innerOriginInOuterLocal{};
	osg::Vec3f m_modelCenter{};
	osg::Vec3f m_centerPlusPose{};
	osg::Quat m_attitude{};
};

#endif // OSGWIDGETCORE_OBJECTGIZMOFRAME_H
