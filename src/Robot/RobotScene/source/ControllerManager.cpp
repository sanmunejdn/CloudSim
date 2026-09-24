/// @file ControllerManager.cpp
/// @brief 外置控制器 TCP 服务端实现

#include "ControllerManager.h"

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
	j["protocolVer"] = 1;
	j["reason"] = reason.toStdString();
	sendJsonLine(j.dump());
	closeClient();
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
		err["protocolVer"] = 1;
		err["code"] = "INTERNAL";
		err["message"] = "invalid json";
		sendJsonLine(err.dump());
		return;
	}

	const int ver = j.value("protocolVer", 0);
	const std::string type = j.value("type", "");
	if (ver != 1 && type != "GOODBYE")
	{
		json err;
		err["type"] = "ERROR";
		err["protocolVer"] = 1;
		err["code"] = "PROTOCOL_MISMATCH";
		err["message"] = "protocolVer must be 1";
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
		err["protocolVer"] = 1;
		err["code"] = "NOT_READY";
		err["message"] = "ExternalController disabled";
		sendJsonLine(err.dump());
		return;
	}

	if (type == "HELLO")
	{
		const int idx = j.value("robotInstanceIndex", 0);
		m_context.setRobotInstanceIndex(idx);
		if (!m_doc || !m_context.sampleJointMetaFromDocument(m_doc))
		{
			json err;
			err["type"] = "ERROR";
			err["protocolVer"] = 1;
			err["code"] = "BAD_ROBOT_INDEX";
			err["message"] = "no robot joints";
			sendJsonLine(err.dump());
			return;
		}
		json ack;
		ack["type"] = "HELLO_ACK";
		ack["protocolVer"] = 1;
		ack["simDtMs"] = 16;
		ack["jointCount"] = m_context.jointCount();
		json names = json::array();
		for (const QString& n : m_context.jointNames())
			names.push_back(n.toStdString());
		ack["jointNames"] = names;
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
		const int dtMs = j.value("dtMs", 16);
		if (dtMs > 0 && (dtMs % 16) != 0)
		{
			qWarning("ControllerManager: STEP.dtMs=%d is not a multiple of simDtMs=16; applying frame anyway",
					 dtMs);
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
			err["protocolVer"] = 1;
			err["code"] = "BAD_JOINT_COUNT";
			err["message"] = "targetJointRad length mismatch";
			sendJsonLine(err.dump());
			return;
		}
		QVector<double> q;
		q.reserve(static_cast<int>(targets.size()));
		for (double v : targets)
			q.append(v);
		m_context.setPendingTargets(q);
		m_pendingDtMs = dtMs > 0 ? dtMs : 16;
		m_stepAwaitingReply = true;
		return;
	}

	if (type == "STEP_POSE")
	{
		json err;
		err["type"] = "ERROR";
		err["protocolVer"] = 1;
		err["code"] = "NOT_READY";
		err["message"] = "STEP_POSE reserved for protocolVer=2 (see docs/features/仿真外置控制器/P3_立项说明.md)";
		sendJsonLine(err.dump());
		return;
	}

	json err;
	err["type"] = "ERROR";
	err["protocolVer"] = 1;
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
		err["protocolVer"] = 1;
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
	reply["protocolVer"] = 1;
	reply["simTimeMs"] = static_cast<int>(m_simTimeMs);
	json arr = json::array();
	for (double v : snap)
		arr.push_back(v);
	reply["actualJointRad"] = arr;
	sendJsonLine(reply.dump());
	m_stepAwaitingReply = false;
	return true;
}
