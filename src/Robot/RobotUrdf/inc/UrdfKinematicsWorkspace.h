#ifndef ROBOTURDF_URDFKINEMATICSWORKSPACE_H
#define ROBOTURDF_URDFKINEMATICSWORKSPACE_H

/// @file UrdfKinematicsWorkspace.h
/// @brief FK/J/DLS 热路径预分配缓冲，避免每步反复分配

#include "robot_urdf_global.h"

#include <QVector>
#include <cstddef>
#include <vector>

namespace UrdfRobotLoader
{
struct ROBOT_URDF_API UrdfJacCol
{
	bool prismatic = false;
	double px = 0.0;
	double py = 0.0;
	double pz = 0.0;
	double zx = 0.0;
	double zy = 0.0;
	double zz = 1.0;
};

/// 调用方持有或 thread_local；容量不足才扩容
struct ROBOT_URDF_API UrdfKinematicsWorkspace
{
	QVector<double> qRad;
	std::vector<UrdfJacCol> jacCols;
	std::vector<int> bfsLinkIds;
	std::vector<double> bfsMat16; // 每节点 16 doubles（Mat4）
	std::vector<double> J;
	std::vector<double> e;
	std::vector<double> jtj;
	std::vector<double> jte;

	void ensureCapacity(int nJoints, int nLinks, int taskDim)
	{
		const int nJ = nJoints > 0 ? nJoints : 1;
		const int nL = nLinks > 0 ? nLinks : 1;
		const int td = taskDim > 0 ? taskDim : 6;
		if (qRad.size() < nJ)
		{
			qRad.resize(nJ);
		}
		if (static_cast<int>(jacCols.capacity()) < nJ)
		{
			jacCols.reserve(static_cast<size_t>(nJ));
		}
		if (static_cast<int>(bfsLinkIds.capacity()) < nL)
		{
			bfsLinkIds.reserve(static_cast<size_t>(nL));
		}
		const size_t matNeed = static_cast<size_t>(nL) * 16u;
		if (bfsMat16.capacity() < matNeed)
		{
			bfsMat16.reserve(matNeed);
		}
		const size_t jNeed = static_cast<size_t>(td * nJ);
		if (J.capacity() < jNeed)
		{
			J.reserve(jNeed);
		}
		if (e.capacity() < static_cast<size_t>(td))
		{
			e.reserve(static_cast<size_t>(td));
		}
		const size_t jtjNeed = static_cast<size_t>(nJ * nJ);
		if (jtj.capacity() < jtjNeed)
		{
			jtj.reserve(jtjNeed);
		}
		if (jte.capacity() < static_cast<size_t>(nJ))
		{
			jte.reserve(static_cast<size_t>(nJ));
		}
	}
};

ROBOT_URDF_API UrdfKinematicsWorkspace& threadLocalKinematicsWorkspace();

} // namespace UrdfRobotLoader

#endif // ROBOTURDF_URDFKINEMATICSWORKSPACE_H
