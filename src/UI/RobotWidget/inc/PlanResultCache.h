#pragma once

#include "RobotInstructionController.h"

#include <QHash>
#include <QString>

/// PlanResult 缓存，key = instructionId + fingerprint；仅 UI 线程访问
class PlanResultCache
{
public:
	const RobotInstruction::PlanResult* fetch(const QString& instructionId, const QString& fingerprint) const;

	void store(const QString& instructionId, const QString& fingerprint, const RobotInstruction::PlanResult& result);

	void invalidateByInstruction(const QString& instructionId);
	void invalidateAll();

	int size() const { return m_entries.size(); }

private:
	static QString makeKey(const QString& id, const QString& fp);

	struct Entry
	{
		QString instructionId;
		QString fingerprint;
		RobotInstruction::PlanResult result;
	};

	QHash<QString, Entry> m_entries;
};
