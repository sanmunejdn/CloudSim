#pragma once

#include "cloudsim_core_global.h"

#include <QJsonObject>
#include <QString>
#include <QVector>

#include <array>

namespace cloudsim::core {

using ObjectId = QString;
using Mat4 = std::array<double, 16>;

struct Vec3
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

struct PoseDto
{
	Vec3 positionMm;
	Vec3 eulerDeg;
};

struct ColorDto
{
	float r = 1.f;
	float g = 1.f;
	float b = 1.f;
	float a = 1.f;
};

struct BBoxDto
{
	Vec3 min;
	Vec3 max;
	bool valid = false;
};

struct PropertyRowDto
{
	QString key;
	QString labelEn;
	bool editable = true;
	QString value;
};

struct RegisterObjectDto
{
	QString className;
	QString name;
	ObjectId parentId;
};

struct ImportOptionsDto
{
	bool quietUi = false;
	bool resetViewToHome = true;
};

struct PlanResultDto
{
	bool ok = false;
	QString error;
	QVector<double> jointTargetsRad;
};

struct MotionInstructionDto
{
	QString instructionType;
	PoseDto targetPose;
	QString jointRadCsv;
	QJsonObject axisConfiguration;
	QJsonObject extensions;
};

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

struct RobotRegistrationDto
{
	bool ok = false;
	QString error;
	ObjectId sceneRootBackendId;
};

} // namespace cloudsim::core
