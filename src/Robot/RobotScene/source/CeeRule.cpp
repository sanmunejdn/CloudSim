/// @file CeeRule.cpp
/// @brief 薄 CEE 规则 JSON

#include "CeeRule.h"

#include <QUuid>

namespace
{
QString conditionTypeToString(CeeCondition::Type t)
{
	switch (t)
	{
	case CeeCondition::Type::LevelHigh:
		return QStringLiteral("levelHigh");
	case CeeCondition::Type::LevelLow:
		return QStringLiteral("levelLow");
	case CeeCondition::Type::RisingEdge:
	default:
		return QStringLiteral("risingEdge");
	}
}

bool conditionTypeFromString(const QString& s, CeeCondition::Type* out)
{
	if (!out)
	{
		return false;
	}
	if (s.compare(QStringLiteral("levelHigh"), Qt::CaseInsensitive) == 0)
	{
		*out = CeeCondition::Type::LevelHigh;
		return true;
	}
	if (s.compare(QStringLiteral("levelLow"), Qt::CaseInsensitive) == 0)
	{
		*out = CeeCondition::Type::LevelLow;
		return true;
	}
	if (s.compare(QStringLiteral("risingEdge"), Qt::CaseInsensitive) == 0 || s.isEmpty())
	{
		*out = CeeCondition::Type::RisingEdge;
		return true;
	}
	return false;
}

QString actionTypeToString(CeeAction::Type t)
{
	switch (t)
	{
	case CeeAction::Type::GoNamedPose:
		return QStringLiteral("goNamedPose");
	case CeeAction::Type::StartRobotProgram:
		return QStringLiteral("startRobotProgram");
	case CeeAction::Type::StopLane:
		return QStringLiteral("stopLane");
	case CeeAction::Type::SetDo:
	default:
		return QStringLiteral("setDo");
	}
}

bool actionTypeFromString(const QString& s, CeeAction::Type* out)
{
	if (!out)
	{
		return false;
	}
	if (s.compare(QStringLiteral("goNamedPose"), Qt::CaseInsensitive) == 0)
	{
		*out = CeeAction::Type::GoNamedPose;
		return true;
	}
	if (s.compare(QStringLiteral("startRobotProgram"), Qt::CaseInsensitive) == 0)
	{
		*out = CeeAction::Type::StartRobotProgram;
		return true;
	}
	if (s.compare(QStringLiteral("stopLane"), Qt::CaseInsensitive) == 0)
	{
		*out = CeeAction::Type::StopLane;
		return true;
	}
	if (s.compare(QStringLiteral("setDo"), Qt::CaseInsensitive) == 0)
	{
		*out = CeeAction::Type::SetDo;
		return true;
	}
	return false;
}
} // namespace

QJsonObject ceeRuleToJson(const CeeRule& rule)
{
	QJsonObject when;
	when.insert(QStringLiteral("type"), conditionTypeToString(rule.when.type));
	when.insert(QStringLiteral("ownerId"), rule.when.ownerId);
	when.insert(QStringLiteral("signal"), rule.when.signal);

	QJsonArray doArr;
	for (const CeeAction& a : rule.actions)
	{
		QJsonObject o;
		o.insert(QStringLiteral("type"), actionTypeToString(a.type));
		switch (a.type)
		{
		case CeeAction::Type::GoNamedPose:
			o.insert(QStringLiteral("deviceId"), a.deviceId);
			o.insert(QStringLiteral("poseId"), a.poseId);
			o.insert(QStringLiteral("durationSec"), a.durationSec);
			break;
		case CeeAction::Type::StartRobotProgram:
			o.insert(QStringLiteral("instanceIndex"), a.instanceIndex);
			o.insert(QStringLiteral("programId"), a.programId);
			break;
		case CeeAction::Type::SetDo:
			o.insert(QStringLiteral("ownerId"), a.ownerId);
			o.insert(QStringLiteral("signal"), a.signal);
			o.insert(QStringLiteral("value"), a.value);
			break;
		case CeeAction::Type::StopLane:
			o.insert(QStringLiteral("participant"), a.participant);
			break;
		}
		doArr.append(o);
	}

	QJsonObject root;
	root.insert(QStringLiteral("id"), rule.id);
	root.insert(QStringLiteral("enabled"), rule.enabled);
	root.insert(QStringLiteral("when"), when);
	root.insert(QStringLiteral("do"), doArr);
	return root;
}

bool ceeRuleFromJson(const QJsonObject& obj, CeeRule* out, QString* err)
{
	if (!out)
	{
		return false;
	}
	CeeRule rule;
	rule.id = obj.value(QStringLiteral("id")).toString();
	if (rule.id.isEmpty())
	{
		rule.id = QUuid::createUuid().toString(QUuid::WithoutBraces);
	}
	rule.enabled = obj.value(QStringLiteral("enabled")).toBool(true);
	const QJsonObject when = obj.value(QStringLiteral("when")).toObject();
	if (!conditionTypeFromString(when.value(QStringLiteral("type")).toString(), &rule.when.type))
	{
		if (err)
		{
			*err = QStringLiteral("ceeRule when.type invalid");
		}
		return false;
	}
	rule.when.ownerId = when.value(QStringLiteral("ownerId")).toString();
	rule.when.signal = when.value(QStringLiteral("signal")).toString();

	const QJsonArray doArr = obj.value(QStringLiteral("do")).toArray();
	for (const QJsonValue& v : doArr)
	{
		const QJsonObject o = v.toObject();
		CeeAction a;
		if (!actionTypeFromString(o.value(QStringLiteral("type")).toString(), &a.type))
		{
			if (err)
			{
				*err = QStringLiteral("ceeRule action.type invalid");
			}
			return false;
		}
		a.deviceId = o.value(QStringLiteral("deviceId")).toString();
		a.poseId = o.value(QStringLiteral("poseId")).toString();
		a.durationSec = o.value(QStringLiteral("durationSec")).toDouble(1.0);
		a.instanceIndex = o.value(QStringLiteral("instanceIndex")).toInt(0);
		a.programId = o.value(QStringLiteral("programId")).toString();
		a.ownerId = o.value(QStringLiteral("ownerId")).toString();
		a.signal = o.value(QStringLiteral("signal")).toString();
		a.value = o.value(QStringLiteral("value")).toBool(true);
		a.participant = o.value(QStringLiteral("participant")).toString();
		rule.actions.push_back(a);
	}
	*out = rule;
	return true;
}

QJsonArray ceeRulesToJson(const QVector<CeeRule>& rules)
{
	QJsonArray arr;
	for (const CeeRule& r : rules)
	{
		arr.append(ceeRuleToJson(r));
	}
	return arr;
}

bool ceeRulesFromJson(const QJsonArray& arr, QVector<CeeRule>* out, QString* err)
{
	if (!out)
	{
		return false;
	}
	QVector<CeeRule> rules;
	rules.reserve(arr.size());
	for (const QJsonValue& v : arr)
	{
		CeeRule r;
		if (!ceeRuleFromJson(v.toObject(), &r, err))
		{
			return false;
		}
		rules.push_back(r);
	}
	*out = rules;
	return true;
}
