#ifndef ROBOTPATHPLANNING_ROBOTPATHPLANNING_H
#define ROBOTPATHPLANNING_ROBOTPATHPLANNING_H

/// @file RobotPathPlanning.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 关节空间避障路径规划（MoveIt/OMPL 风格，CloudSim 适配）

#include "robot_path_planning_global.h"

#include "BackendFollowMath.h"
#include "CollisionWorld.h"

#include <QString>
#include <QHash>

#include <osg/Matrixd>

#include <string>
#include <vector>

namespace robot_path
{

struct ROBOT_PATH_PLANNING_API TcpPose
{
	double transMm[3]{0.0, 0.0, 0.0};
	/// xyzw，与 UrdfPoseIkTarget 一致
	double quatXyzw[4]{0.0, 0.0, 0.0, 1.0};
};

struct ROBOT_PATH_PLANNING_API PlanRequest
{
	QString urdfPath;
	QString flangeLinkName;
	std::vector<double> startJointRad;
	TcpPose goalToolInBase{};
	BackendMat4 T_flange_tool = BackendMat4::identity();
	/// URDF 基座 → 场景世界（无绑定位姿时的兜底）
	BackendMat4 T_world_urdfBase = BackendMat4::identity();
	/// 起点画面位姿（nearStart 时写回）
	QHash<QString, BackendMat4> linkWorldAtStart;
	/// 与 applyPerLinkRobotBasePlacement 同源：M=m0*inv(T0)*Tq*P（OSG 矩阵，勿再经 Backend 往返）
	QHash<QString, osg::Matrixd> fkMeshWorldT0;
	QHash<QString, osg::Matrixd> outerWorldAtBindByBackendId;
	osg::Matrixd robotBasePlacementWorld = osg::Matrixd::identity();
	/// 非拥有；须含 robotLink + scene，由 BackendCollisionSync 填充
	collision::CollisionWorld* world = nullptr;
	/// link 名 → CollisionBodyId（robotLink）
	QHash<QString, collision::CollisionBodyId> linkBodies;
	/// per-link 导入时 mesh 顶点已在连杆系
	bool meshVerticesInLinkFrame = false;

	struct Options
	{
		/// BITstar：路径长度 anytime 最优；RRTConnect 仅作连通兜底
		std::string plannerId = "BITstar";
		double planningTimeSec = 10.0;
		double longestValidSegmentRad = 0.05;
		double securityMarginMm = 1.0;
		bool useOrientation = true;
		/// false 时仅检限位（对应 Dock「启用碰撞检测」关闭）
		bool checkCollision = true;
		/// 固定种子，同起终点可复现
		unsigned int rngSeed = 42u;
	} options;
};

struct ROBOT_PATH_PLANNING_API PathResult
{
	bool ok = false;
	std::string plannerName;
	std::string errMsg;
	std::vector<std::vector<double>> jointTrajectoryRad;
	std::vector<TcpPose> tcpPoses;
	double pathLengthRad = 0.0;
	double pathLengthTcpMm = 0.0;
};

/// TCP 位姿目标 → 关节空间 OMPL/RRT 规划；输出双轨路径
ROBOT_PATH_PLANNING_API bool planToTcpPose(const PlanRequest& req, PathResult& out);

ROBOT_PATH_PLANNING_API bool runSelfTest(std::vector<std::string>& failures);

} // namespace robot_path

#endif // ROBOTPATHPLANNING_ROBOTPATHPLANNING_H
