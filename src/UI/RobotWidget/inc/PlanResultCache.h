/// @file PlanResultCache.h
/// @brief PlanResult 缓存，key = instructionId + fingerprint；仅 UI 线程访问；有界淘汰

#ifndef ROBOTWIDGET_PLANRESULTCACHE_H
#define ROBOTWIDGET_PLANRESULTCACHE_H

#include "RobotInstructionController.h"

#include <QHash>
#include <QList>
#include <QString>
#include <cstddef>

/// PlanResult 缓存，key = instructionId + fingerprint；仅 UI 线程访问
class PlanResultCache
{
public:
	static constexpr int kDefaultMaxEntries = 384;

	const RobotInstruction::PlanResult* fetch(const QString& instructionId, const QString& fingerprint) const;

	void store(const QString& instructionId, const QString& fingerprint, const RobotInstruction::PlanResult& result,
			   size_t motionIndex = 0);

	void invalidateByInstruction(const QString& instructionId);
	void invalidateAll();

	/// 淘汰落后播放游标过远的条目，控制万级路点内存
	void evictFarBehind(size_t currentMotionIndex, size_t keepBehind = 64);

	int size() const { return m_entries.size(); }
	void setMaxEntries(int maxEntries) { m_maxEntries = maxEntries > 16 ? maxEntries : 16; }

private:
	static QString makeKey(const QString& id, const QString& fp);
	void evictOverflow();

	struct Entry
	{
		QString instructionId;
		QString fingerprint;
		size_t motionIndex = 0;
		RobotInstruction::PlanResult result;
	};

	QHash<QString, Entry> m_entries;
	QList<QString> m_fifoKeys;
	int m_maxEntries = kDefaultMaxEntries;
};

#endif // ROBOTWIDGET_PLANRESULTCACHE_H
