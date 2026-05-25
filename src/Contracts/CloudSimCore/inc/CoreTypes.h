#pragma once

#include "cloudsim_core_global.h"

#include <QJsonObject>
#include <QString>
#include <QVector>

#include <array>

namespace cloudsim::core {

using ObjectId = QString;
using Mat4 = std::array<double, 16>;

/// 三维向量
struct Vec3
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

/// 位姿 DTO
struct PoseDto
{
	Vec3 positionMm;
	Vec3 eulerDeg;
};

/// 颜色 DTO
struct ColorDto
{
	float r = 1.f;
	float g = 1.f;
	float b = 1.f;
	float a = 1.f;
};

/// 包围盒 DTO
struct BBoxDto
{
	Vec3 min;
	Vec3 max;
	bool valid = false;
};

/// 属性行 DTO
struct PropertyRowDto
{
	QString key;
	QString labelEn;
	bool editable = true;
	QString value;
};

/// 注册对象元数据
struct RegisterObjectDto
{
	QString className;
	QString name;
	ObjectId parentId;
};

/// 导入选项
struct ImportOptionsDto
{
	bool quietUi = false;
	bool resetViewToHome = true;
	/// 后端目录类型
	QString catalogTypeName = QStringLiteral("Model");
	bool isPointCloud = false;
	ObjectId parentId;
	/// 工程稳定 id
	ObjectId persistedId;
};

/// 规划结果
struct PlanResultDto
{
	bool ok = false;
	QString error;
	QVector<double> jointTargetsRad;
};

/// 运动指令
struct MotionInstructionDto
{
	QString instructionType;
	PoseDto targetPose;
	QString jointRadCsv;
	QJsonObject axisConfiguration;
	QJsonObject extensions;
};

/// 规划上下文
struct PlanContextDto
{
	QVector<double> seedJointRad;
	QString urdfPath;
	QString tcpLinkName;
	Mat4 toolFrame = identityMat4();
	QJsonObject extensions;

	static Mat4 identityMat4()
	{
		Mat4 m{};
		m[0] = m[5] = m[10] = m[15] = 1.0;
		return m;
	}
};

/// URDF 注册结果
struct RobotRegistrationDto
{
	bool ok = false;
	QString error;
	ObjectId sceneRootBackendId;
	QVector<QString> warnings;
	int linkCount = 0;
	int jointCount = 0;
	QString sourceDisplayName;
};

} // namespace cloudsim::core
