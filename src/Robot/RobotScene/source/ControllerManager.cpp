/// @file ControllerManager.cpp
/// @brief 外置控制器 TCP 服务端实现

#include "ControllerManager.h"

#include "RobotTeachIk.h"
#include "UrdfRobotLoader.h"

#include <RigidTransform.h>
#include <json.hpp>

#include <QDebug>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <vector>

#include <winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

using json = nlohmann::json;

namespace
{
bool g_wsaStarted = false;

bool ensureWsa()
{
	if (g_wsaStarted)
		return true;
	WSADATA wsa{};
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
		return false;
	g_wsaStarted = true;
	return true;
}

QString resolveIkLinkName(const QString& urdfPath)
{
	QString preferred;
	if (UrdfRobotLoader::loadPrimaryTerminalLinkName(urdfPath, preferred, nullptr) && !preferred.isEmpty())
		return preferred;
	QStringList childLinks;
	if (UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, childLinks, nullptr) && !childLinks.isEmpty())
		return childLinks.back();
	return QString();
}
} // namespace

struct ControllerManager::Impl
{
	SOCKET listenSock = INVALID_SOCKET;
	SOCKET clientSock = INVALID_SOCKET;
	std::string recvBuf;
};

ControllerManager::ControllerManager(QObject* parent)
	: QObject(parent)
	, m_impl(std::make_unique<Impl>())
{
}

ControllerManager::~ControllerManager()
{
	stopListening();
}

bool ControllerManager::hasClient() const
{
	return m_impl && m_impl->clientSock != INVALID_SOCKET;
}

void ControllerManager::setDocument(IRobotSimulationDocument* doc)
{
	m_doc = doc;
}

void ControllerManager::setEnabled(bool on)
{
	m_enabled = on;
	if (!on)
		notifySimulationStopped(QStringLiteral("external_controller_disabled"));
}

void ControllerManager::setBoundRobotInstanceIndex(int instanceIndex)
{
	m_boundRobotInstanceIndex = instanceIndex;
	m_context.setRobotInstanceIndex(instanceIndex);
}

bool ControllerManager::startListening(quint16 port)
{
	stopListening();
	if (!ensureWsa())
	{
		m_lastError = QStringLiteral("WSAStartup failed");
		return false;
	}

	SOCKET ls = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (ls == INVALID_SOCKET)
	{
		m_lastError = QStringLiteral("listen socket failed");
		return false;
	}

	BOOL yes = TRUE;
	setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&yes), sizeof(yes));

	u_long nonBlock = 1;
	ioctlsocket(ls, FIONBIO, &nonBlock);

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);

	if (bind(ls, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
	{
		closesocket(ls);
		m_lastError = QStringLiteral("bind 127.0.0.1:%1 failed").arg(port);
		return false;
	}
	if (listen(ls, 1) != 0)
	{
		closesocket(ls);
		m_lastError = QStringLiteral("listen failed");
		return false;
	}

	m_impl->listenSock = ls;
	m_listening = true;
	m_listenPort = port;
	m_lastError.clear();
	return true;
}

void ControllerManager::stopListening()
{
	if (!m_impl)
		return;
	closeClient();
	if (m_impl->listenSock != INVALID_SOCKET)
	{
		closesocket(m_impl->listenSock);
		m_impl->listenSock = INVALID_SOCKET;
	}
	m_listening = false;
	m_stepAwaitingReply = false;
}

void ControllerManager::closeClient()
{
	if (!m_impl)
		return;
	if (m_impl->clientSock != INVALID_SOCKET)
	{
		// 断连只关 socket：关节保持末姿态，不复位
		closesocket(m_impl->clientSock);
		m_impl->clientSock = INVALID_SOCKET;
	}
	m_impl->recvBuf.clear();
	m_stepAwaitingReply = false;
}

bool ControllerManager::sendJsonLine(const std::string& line)
{
	if (!hasClient())
		return false;
	std::string payload = line;
	if (payload.empty() || payload.back() != '\n')
		payload.push_back('\n');
	int sent = 0;
	const int total = static_cast<int>(payload.size());
	while (sent < total)
	{
		const int n = send(m_impl->clientSock, payload.data() + sent, total - sent, 0);
		if (n <= 0)
		{
			closeClient();
			return false;
		}
		sent += n;
	}
	return true;
}

void ControllerManager::notifySimulationStopped(const QString& reason)
{
	if (!hasClient())
		return;
	json j;
	j["type"] = "QUIT";
	j["protocolVer"] = m_sessionProtocolVer >= 2 ? 2 : 1;
	j["reason"] = reason.toStdString();
	sendJsonLine(j.dump());
	closeClient();
}

bool ControllerManager::trySolveStepPose(const std::vector<double>& tcpMm, const std::vector<double>& eulerDeg,
										 QVector<double>& outJointRad, QString& errOut)
{
	outJointRad.clear();
	if (!m_doc || tcpMm.size() != 3 || eulerDeg.size() != 3)
	{
		errOut = QStringLiteral("bad pose arrays");
		return false;
	}
	const int idx = m_context.robotInstanceIndex();
	const QString urdf = m_doc->robotUrdfAbsolutePathForInstance(idx);
	if (urdf.isEmpty())
	{
		errOut = QStringLiteral("no urdf");
		return false;
	}
	const QString ikLink = resolveIkLinkName(urdf);
	if (ikLink.isEmpty())
	{
		errOut = QStringLiteral("no ik link");
		return false;
	}

	QVector<double> seed = m_context.sensorSnapshot();
	if (seed.size() != m_context.jointCount())
	{
		if (!m_doc->robotLocalJointAnglesForInstance(idx, seed) || seed.size() != m_context.jointCount())
		{
			seed = QVector<double>(m_context.jointCount(), 0.0);
		}
	}

	RobotTeachIk::TeachIkContext ctx;
	ctx.urdfPath = urdf;
	ctx.ikLinkName = ikLink;
	ctx.useOrientation = true;
	ctx.T_flange_tool = BackendMat4::identity();
	ctx.maxIkIterations = 80;
	ctx.T_base_target = engine::RigidTransform::fromTranslationEulerDeg(
		tcpMm[0], tcpMm[1], tcpMm[2], eulerDeg[0], eulerDeg[1], eulerDeg[2]);
	ctx.seedJointRad.clear();
	ctx.seedJointRad.reserve(static_cast<size_t>(seed.size()));
	for (double v : seed)
		ctx.seedJointRad.push_back(v);

	const RobotTeachIk::TeachIkResult ik = RobotTeachIk::solveTeachIk(ctx);
	if (!ik.ok || static_cast<int>(ik.jointRad.size()) != m_context.jointCount())
	{
		errOut = ik.error.empty() ? QStringLiteral("IK failed") : QString::fromStdString(ik.error);
		return false;
	}
	outJointRad.reserve(static_cast<int>(ik.jointRad.size()));
	for (double v : ik.jointRad)
		outJointRad.append(v);
	return true;
}

void ControllerManager::handleLine(const std::string& line)
{
	json j;
	try
	{
		j = json::parse(line);
	}
	catch (...)
	{
		json err;
		err["type"] = "ERROR";
		err["protocolVer"] = m_sessionProtocolVer;
		err["code"] = "INTERNAL";
		err["message"] = "invalid json";
		sendJsonLine(err.dump());
		return;
	}

	const int ver = j.value("protocolVer", 0);
	const std::string type = j.value("type", "");
	if (type != "GOODBYE" && ver != 1 && ver != 2)
	{
		json err;
		err["type"] = "ERROR";
		err["protocolVer"] = 1;
		err["code"] = "PROTOCOL_MISMATCH";
		err["message"] = "protocolVer must be 1 or 2";
		sendJsonLine(err.dump());
		return;
	}

	if (type == "GOODBYE")
	{
		closeClient();
		return;
	}

	if (!m_enabled)
	{
		json err;
		err["type"] = "ERROR";
		err["protocolVer"] = ver == 2 ? 2 : 1;
		err["code"] = "NOT_READY";
		err["message"] = "ExternalController disabled";
		sendJsonLine(err.dump());
		return;
	}

	const int replyVer = (ver == 2) ? 2 : 1;

	if (type == "HELLO")
	{
		const int idx = j.value("robotInstanceIndex", 0);
		if (idx != m_boundRobotInstanceIndex)
		{
			json err;
			err["type"] = "ERROR";
			err["protocolVer"] = replyVer;
			err["code"] = "BAD_ROBOT_INDEX";
			err["message"] = "robotInstanceIndex does not match this listen port";
			sendJsonLine(err.dump());
			return;
		}
		m_context.setRobotInstanceIndex(idx);
		m_sessionProtocolVer = replyVer;
		if (!m_doc || !m_context.sampleJointMetaFromDocument(m_doc))
		{
			json err;
			err["type"] = "ERROR";
			err["protocolVer"] = replyVer;
			err["code"] = "BAD_ROBOT_INDEX";
			err["message"] = "no robot joints";
			sendJsonLine(err.dump());
			return;
		}
		json ack;
		ack["type"] = "HELLO_ACK";
		ack["protocolVer"] = replyVer;
		ack["simDtMs"] = kSimDtMs;
		ack["jointCount"] = m_context.jointCount();
		json names = json::array();
		for (const QString& n : m_context.jointNames())
			names.push_back(n.toStdString());
		ack["jointNames"] = names;
		json lower = json::array();
		json upper = json::array();
		for (double v : m_context.jointLowerRad())
			lower.push_back(v);
		for (double v : m_context.jointUpperRad())
			upper.push_back(v);
		ack["jointLowerRad"] = lower;
		ack["jointUpperRad"] = upper;
		if (replyVer >= 2)
			ack["supportsStepPose"] = true;
		m_simTimeMs = 0;
		{
			QVector<double> actual;
			if (m_doc->robotLocalJointAnglesForInstance(idx, actual) &&
				actual.size() == m_context.jointCount())
				m_context.setSensorSnapshot(actual);
		}
		sendJsonLine(ack.dump());
		return;
	}

	if (type == "STEP")
	{
		const int dtMs = j.value("dtMs", kSimDtMs);
		if (dtMs <= 0 || (dtMs % kSimDtMs) != 0)
		{
			json err;
			err["type"] = "ERROR";
			err["protocolVer"] = replyVer;
			err["code"] = "BAD_DT";
			err["message"] = "dtMs must be a positive multiple of simDtMs";
			sendJsonLine(err.dump());
			return;
		}
		std::vector<double> targets;
		if (j.contains("targetJointRad") && j["targetJointRad"].is_array())
		{
			for (const auto& v : j["targetJointRad"])
				targets.push_back(v.get<double>());
		}
		if (static_cast<int>(targets.size()) != m_context.jointCount())
		{
			json err;
			err["type"] = "ERROR";
			err["protocolVer"] = replyVer;
			err["code"] = "BAD_JOINT_COUNT";
			err["message"] = "targetJointRad length mismatch";
			sendJsonLine(err.dump());
			return;
		}
		const QVector<double> lo = m_context.jointLowerRad();
		const QVector<double> hi = m_context.jointUpperRad();
		if (lo.size() == m_context.jointCount() && hi.size() == m_context.jointCount())
		{
			for (int i = 0; i < static_cast<int>(targets.size()); ++i)
			{
				if (targets[static_cast<size_t>(i)] < lo[i] || targets[static_cast<size_t>(i)] > hi[i])
				{
					json err;
					err["type"] = "ERROR";
					err["protocolVer"] = replyVer;
					err["code"] = "OUT_OF_LIMITS";
					err["message"] = "targetJointRad out of joint limits";
					sendJsonLine(err.dump());
					return;
				}
			}
		}
		QVector<double> q;
		q.reserve(static_cast<int>(targets.size()));
		for (double v : targets)
			q.append(v);
		m_context.setPendingTargets(q);
		m_pendingDtMs = dtMs;
		m_stepAwaitingReply = true;
		return;
	}

	if (type == "STEP_POSE")
	{
		if (ver != 2)
		{
			json err;
			err["type"] = "ERROR";
			err["protocolVer"] = replyVer;
			err["code"] = "PROTOCOL_MISMATCH";
			err["message"] = "STEP_POSE requires protocolVer=2";
			sendJsonLine(err.dump());
			return;
		}
		const int dtMs = j.value("dtMs", kSimDtMs);
		if (dtMs <= 0 || (dtMs % kSimDtMs) != 0)
		{
			json err;
			err["type"] = "ERROR";
			err["protocolVer"] = 2;
			err["code"] = "BAD_DT";
			err["message"] = "dtMs must be a positive multiple of simDtMs";
			sendJsonLine(err.dump());
			return;
		}
		const std::string frame = j.value("frame", "robot_base");
		if (frame != "robot_base")
		{
			json err;
			err["type"] = "ERROR";
			err["protocolVer"] = 2;
			err["code"] = "BAD_FRAME";
			err["message"] = "only frame=robot_base is supported";
			sendJsonLine(err.dump());
			return;
		}
		std::vector<double> tcpMm;
		std::vector<double> eulerDeg;
		if (j.contains("targetTcpMm") && j["targetTcpMm"].is_array())
		{
			for (const auto& v : j["targetTcpMm"])
				tcpMm.push_back(v.get<double>());
		}
		if (j.contains("targetEulerDeg") && j["targetEulerDeg"].is_array())
		{
			for (const auto& v : j["targetEulerDeg"])
				eulerDeg.push_back(v.get<double>());
		}
		if (tcpMm.size() != 3 || eulerDeg.size() != 3)
		{
			json err;
			err["type"] = "ERROR";
			err["protocolVer"] = 2;
			err["code"] = "INTERNAL";
			err["message"] = "targetTcpMm/targetEulerDeg must be length 3";
			sendJsonLine(err.dump());
			return;
		}

		QVector<double> q;
		QString ikErr;
		if (!trySolveStepPose(tcpMm, eulerDeg, q, ikErr))
		{
			json err;
			err["type"] = "ERROR";
			err["protocolVer"] = 2;
			err["code"] = "IK_FAILED";
			err["message"] = ikErr.toStdString();
			sendJsonLine(err.dump());
			return;
		}
		const QVector<double> lo = m_context.jointLowerRad();
		const QVector<double> hi = m_context.jointUpperRad();
		if (lo.size() == q.size() && hi.size() == q.size())
		{
			for (int i = 0; i < q.size(); ++i)
			{
				if (q[i] < lo[i] || q[i] > hi[i])
				{
					json err;
					err["type"] = "ERROR";
					err["protocolVer"] = 2;
					err["code"] = "OUT_OF_LIMITS";
					err["message"] = "IK solution out of joint limits";
					sendJsonLine(err.dump());
					return;
				}
			}
		}
		m_context.setPendingTargets(q);
		m_pendingDtMs = dtMs;
		m_stepAwaitingReply = true;
		return;
	}

	json err;
	err["type"] = "ERROR";
	err["protocolVer"] = replyVer;
	err["code"] = "INTERNAL";
	err["message"] = "unknown type";
	sendJsonLine(err.dump());
}

void ControllerManager::pollIncoming()
{
	if (!m_listening || !m_impl)
		return;

	if (m_impl->listenSock != INVALID_SOCKET && m_impl->clientSock == INVALID_SOCKET)
	{
		sockaddr_in ca{};
		int calen = sizeof(ca);
		SOCKET cs = accept(m_impl->listenSock, reinterpret_cast<sockaddr*>(&ca), &calen);
		if (cs != INVALID_SOCKET)
		{
			u_long nonBlock = 1;
			ioctlsocket(cs, FIONBIO, &nonBlock);
			m_impl->clientSock = cs;
			m_impl->recvBuf.clear();
		}
	}

	if (m_impl->clientSock == INVALID_SOCKET)
		return;

	char buf[4096];
	for (;;)
	{
		const int n = recv(m_impl->clientSock, buf, sizeof(buf), 0);
		if (n > 0)
		{
			m_impl->recvBuf.append(buf, buf + n);
			continue;
		}
		if (n == 0)
		{
			closeClient();
			return;
		}
		const int err = WSAGetLastError();
		if (err == WSAEWOULDBLOCK)
			break;
		closeClient();
		return;
	}

	for (;;)
	{
		const auto pos = m_impl->recvBuf.find('\n');
		if (pos == std::string::npos)
			break;
		std::string line = m_impl->recvBuf.substr(0, pos);
		m_impl->recvBuf.erase(0, pos + 1);
		if (!line.empty() && line.back() == '\r')
			line.pop_back();
		if (!line.empty())
			handleLine(line);
	}
}

bool ControllerManager::tickApply(IRobotBackendPoseSink* osg, QVector<double>& aggregatedJointAnglesRad)
{
	pollIncoming();
	// 无 STEP：保持末姿态空转，不改关节、不推 simTime
	if (!m_stepAwaitingReply || !m_context.hasPendingTargets())
		return false;
	if (!m_doc || !osg)
		return false;

	if (!m_context.applyPendingToScene(m_doc, osg, aggregatedJointAnglesRad))
	{
		json err;
		err["type"] = "ERROR";
		err["protocolVer"] = m_sessionProtocolVer;
		err["code"] = "INTERNAL";
		err["message"] = "applyPending failed";
		sendJsonLine(err.dump());
		m_stepAwaitingReply = false;
		return false;
	}

	m_simTimeMs += m_pendingDtMs;
	const QVector<double> snap = m_context.sensorSnapshot();
	json reply;
	reply["type"] = "STEP_REPLY";
	reply["protocolVer"] = m_sessionProtocolVer;
	reply["simTimeMs"] = static_cast<int>(m_simTimeMs);
	json arr = json::array();
	for (double v : snap)
		arr.push_back(v);
	reply["actualJointRad"] = arr;
	json sensors;
	sensors["jointPosition"] = arr;
	reply["sensors"] = sensors;
	sendJsonLine(reply.dump());
	m_stepAwaitingReply = false;
	return true;
}
