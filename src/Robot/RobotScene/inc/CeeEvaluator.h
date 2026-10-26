#ifndef ROBOTSCENE_CEEEVALUATOR_H
#define ROBOTSCENE_CEEEVALUATOR_H

/// @file CeeEvaluator.h
/// @brief 薄 CEE：按规则对 DI 求值并派发动作

#include "robot_scene_global.h"

#include "CeeRule.h"

#include <QHash>
#include <QString>
#include <QVector>

class ICeeActionHandler;
class IIoSignalNetwork;
class IoPhase;
class SimulationKernel;

class ROBOT_SCENE_API CeeEvaluator
{
public:
	void setRules(const QVector<CeeRule>& rules);
	const QVector<CeeRule>& rules() const { return m_rules; }

	void resetEdgeState();

	/// 在 IoPhase::swapAll 之后调用；读锁存 DI，写 DO / stopLane，经 handler 启运动
	void evaluate(IIoSignalNetwork* network, IoPhase& ioPhase, SimulationKernel& kernel, ICeeActionHandler* handler);

private:
	bool readDi(IIoSignalNetwork* network, IoPhase& ioPhase, const QString& ownerId, const QString& signal,
				bool* outValue) const;
	void applyAction(const CeeAction& action, IIoSignalNetwork* network, IoPhase& ioPhase, SimulationKernel& kernel,
					 ICeeActionHandler* handler);

	QVector<CeeRule> m_rules;
	/// ownerId|signal → 上一拍 DI
	QHash<QString, bool> m_lastDi;
};

ROBOT_SCENE_API bool runCeeEvaluatorSelfTest(QString* errOut = nullptr);

#endif // ROBOTSCENE_CEEEVALUATOR_H
