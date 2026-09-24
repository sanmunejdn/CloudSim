#ifndef ROBOTSCENE_CONTROLLERMANAGER_H
#define ROBOTSCENE_CONTROLLERMANAGER_H

/// @file ControllerManager.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 外置控制器 TCP 服务端（127.0.0.1:19620）

#include "robot_scene_global.h"

#include "ControlContext.h"

#include <QObject>
#include <QString>
#include <QVector>
#include <memory>
#include <string>

class IRobotBackendPoseSink;

/// listen + 按 tick 应答 STEP；与指令回放互斥由调用方保证
/// 无 Q_OBJECT：RobotScene 未接 MOC，信号改由调用方轮询 hasClient
class ROBOT_SCENE_API ControllerManager : public QObject
{
public:
	explicit ControllerManager(QObject* parent = nullptr);
	~ControllerManager() override;

	ControlContext& context() { return m_context; }
	const ControlContext& context() const { return m_context; }

	bool startListening(quint16 port = 19620);
	void stopListening();
	bool isListening() const { return m_listening; }
	bool hasClient() const;

	void setDocument(IRobotSimulationDocument* doc);
	void setEnabled(bool on);
	bool isEnabled() const { return m_enabled; }

	/// 非阻塞收包；有 STEP 则写入 ControlContext pending
	void pollIncoming();

	/// 应用 pending 并回 STEP_REPLY；无 pending 则 no-op
	bool tickApply(IRobotBackendPoseSink* osg, QVector<double>& aggregatedJointAnglesRad);

	void notifySimulationStopped(const QString& reason = QStringLiteral("simulation_stopped"));

	qint64 simTimeMs() const { return m_simTimeMs; }
	void resetSimTime() { m_simTimeMs = 0; }

	QString lastError() const { return m_lastError; }

	static constexpr int kSimDtMs = 16;
	static constexpr quint16 kBasePort = 19620;

	/// 绑定本端口对应的机器人实例；HELLO 下标必须一致（P6）
	void setBoundRobotInstanceIndex(int instanceIndex);
	int boundRobotInstanceIndex() const { return m_boundRobotInstanceIndex; }
	quint16 listenPort() const { return m_listenPort; }

private:
	struct Impl;
	std::unique_ptr<Impl> m_impl;
	ControlContext m_context;
	IRobotSimulationDocument* m_doc = nullptr;
	bool m_enabled = false;
	bool m_listening = false;
	bool m_stepAwaitingReply = false;
	int m_pendingDtMs = 16;
	qint64 m_simTimeMs = 0;
	QString m_lastError;
	int m_boundRobotInstanceIndex = 0;
	quint16 m_listenPort = kBasePort;
	int m_sessionProtocolVer = 1;

	bool sendJsonLine(const std::string& line);
	void handleLine(const std::string& line);
	void closeClient();
	bool trySolveStepPose(const std::vector<double>& tcpMm, const std::vector<double>& eulerDeg,
						  QVector<double>& outJointRad, QString& errOut);
};

#endif // ROBOTSCENE_CONTROLLERMANAGER_H
