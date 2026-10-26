#ifndef ROBOTSCENE_CEERULE_H
#define ROBOTSCENE_CEERULE_H

/// @file CeeRule.h
/// @brief 薄 CEE 规则：条件 → 动作列表

#include "robot_scene_global.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QString>
#include <QVector>

struct ROBOT_SCENE_API CeeCondition
{
	enum class Type
	{
		RisingEdge = 0,
		LevelHigh,
		LevelLow
	};

	Type type = Type::RisingEdge;
	QString ownerId;
	QString signal;
};

struct ROBOT_SCENE_API CeeAction
{
	enum class Type
	{
		GoNamedPose = 0,
		StartRobotProgram,
		SetDo,
		StopLane
	};

	Type type = Type::SetDo;
	QString deviceId;
	QString poseId;
	double durationSec = 1.0;
	int instanceIndex = 0;
	QString programId;
	QString ownerId;
	QString signal;
	bool value = true;
	QString participant;
};

struct ROBOT_SCENE_API CeeRule
{
	QString id;
	bool enabled = true;
	CeeCondition when;
	QVector<CeeAction> actions;
};

ROBOT_SCENE_API QJsonObject ceeRuleToJson(const CeeRule& rule);
ROBOT_SCENE_API bool ceeRuleFromJson(const QJsonObject& obj, CeeRule* out, QString* err = nullptr);
ROBOT_SCENE_API QJsonArray ceeRulesToJson(const QVector<CeeRule>& rules);
ROBOT_SCENE_API bool ceeRulesFromJson(const QJsonArray& arr, QVector<CeeRule>* out, QString* err = nullptr);

#endif // ROBOTSCENE_CEERULE_H
