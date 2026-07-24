/// @file RobotMotionClientImpl.cpp
/// @brief 经 localhost TCP JSON 与 RobotCommBridge 通讯

#include "RobotMotionClientImpl.h"

#include <json.hpp>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

#include <chrono>
#include <cstring>
#include <sstream>

#pragma comment(lib, "ws2_32.lib")

using json = nlohmann::json;

namespace
{
bool g_wsaStarted = false;

std::string brandToString(RobotCommBrand b)
{
	switch (b)
	{
	case RobotCommBrand::Abb:
		return "abb";
	case RobotCommBrand::Kuka:
		return "kuka";
	case RobotCommBrand::Fanuc:
	default:
		return "fanuc";
	}
}

} // namespace

RobotMotionClientImpl::RobotMotionClientImpl() = default;

RobotMotionClientImpl::~RobotMotionClientImpl()
{
	disconnectBridge();
}

bool RobotMotionClientImpl::ensureWinsock()
{
	if (g_wsaStarted)
		return true;
	WSADATA wsa{};
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		setError("WSAStartup failed");
		return false;
	}
	g_wsaStarted = true;
	return true;
}

void RobotMotionClientImpl::setError(const std::string& msg)
{
	m_lastError = msg;
}

std::string RobotMotionClientImpl::lastError() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_lastError;
}

bool RobotMotionClientImpl::isBridgeConnected() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_bridgeConnected;
}

bool RobotMotionClientImpl::isRobotConnected() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_robotConnected;
}

bool RobotMotionClientImpl::connectBridge(const RobotCommBridgeEndpoint& endpoint)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	disconnectBridge();
	if (!ensureWinsock())
		return false;

	SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (s == INVALID_SOCKET)
	{
		setError("socket create failed");
		return false;
	}

	DWORD timeout = static_cast<DWORD>(endpoint.timeoutMs > 0 ? endpoint.timeoutMs : 3000);
	setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout));
	setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout));

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_port = htons(endpoint.port);
	if (inet_pton(AF_INET, endpoint.host.c_str(), &addr.sin_addr) != 1)
	{
		closesocket(s);
		setError("invalid bridge host");
		return false;
	}

	if (connect(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
	{
		closesocket(s);
		setError("bridge connect failed");
		return false;
	}

	m_socket = reinterpret_cast<void*>(s);
	m_bridgeConnected = true;
	m_timeoutMs = endpoint.timeoutMs > 0 ? endpoint.timeoutMs : 3000;
	m_lastError.clear();
	return true;
}

void RobotMotionClientImpl::disconnectBridge()
{
	// 可能已在持锁路径调用；仅清理套接字
	if (m_socket)
	{
		SOCKET s = reinterpret_cast<SOCKET>(m_socket);
		closesocket(s);
		m_socket = nullptr;
	}
	m_bridgeConnected = false;
	m_robotConnected = false;
}

bool RobotMotionClientImpl::sendRequest(const std::string& jsonLine, std::string& responseLine)
{
	if (!m_socket || !m_bridgeConnected)
	{
		setError("bridge not connected");
		return false;
	}

	SOCKET s = reinterpret_cast<SOCKET>(m_socket);
	std::string payload = jsonLine;
	if (payload.empty() || payload.back() != '\n')
		payload.push_back('\n');

	int sent = 0;
	const int total = static_cast<int>(payload.size());
	while (sent < total)
	{
		const int n = ::send(s, payload.data() + sent, total - sent, 0);
		if (n <= 0)
		{
			setError("send failed");
			m_bridgeConnected = false;
			return false;
		}
		sent += n;
	}

	responseLine.clear();
	char buf[4096];
	while (responseLine.find('\n') == std::string::npos)
	{
		const int n = ::recv(s, buf, sizeof(buf), 0);
		if (n <= 0)
		{
			setError("recv failed or timeout");
			m_bridgeConnected = false;
			return false;
		}
		responseLine.append(buf, buf + n);
		if (responseLine.size() > 1024 * 1024)
		{
			setError("response too large");
			return false;
		}
	}
	const auto pos = responseLine.find('\n');
	responseLine.resize(pos);
	return true;
}

bool RobotMotionClientImpl::ping()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	json req{{"cmd", "ping"}, {"id", m_nextId++}};
	std::string resp;
	if (!sendRequest(req.dump(), resp))
		return false;
	try
	{
		const auto j = json::parse(resp);
		if (!j.value("ok", false))
		{
			setError(j.value("message", "ping failed"));
			return false;
		}
		return true;
	}
	catch (const std::exception& ex)
	{
		setError(std::string("ping parse: ") + ex.what());
		return false;
	}
}

bool RobotMotionClientImpl::connectRobot(const RobotCommConnectConfig& config)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	json req{
		{"cmd", "connect"},
		{"id", m_nextId++},
		{"brand", brandToString(config.brand)},
		{"host", config.robotHost},
		{"port", config.robotPort},
		{"user", config.user},
		{"password", config.password},
		{"mechUnit", config.mechUnit},
		{"fanucPoseAddr", config.fanucPoseAddr},
		{"fanucJointAddr", config.fanucJointAddr},
		{"fanucPoseLen", config.fanucPoseLen},
		{"fanucJointLen", config.fanucJointLen},
		{"kukaJointVar", config.kukaJointVar},
		{"kukaPoseVar", config.kukaPoseVar},
	};
	std::string resp;
	if (!sendRequest(req.dump(), resp))
		return false;
	try
	{
		const auto j = json::parse(resp);
		if (!j.value("ok", false))
		{
			setError(j.value("message", "robot connect failed"));
			m_robotConnected = false;
			return false;
		}
		m_robotConnected = true;
		m_lastError.clear();
		return true;
	}
	catch (const std::exception& ex)
	{
		setError(std::string("connect parse: ") + ex.what());
		return false;
	}
}

bool RobotMotionClientImpl::disconnectRobot()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	json req{{"cmd", "disconnect"}, {"id", m_nextId++}};
	std::string resp;
	if (!sendRequest(req.dump(), resp))
	{
		m_robotConnected = false;
		return false;
	}
	m_robotConnected = false;
	try
	{
		const auto j = json::parse(resp);
		if (!j.value("ok", false))
		{
			setError(j.value("message", "disconnect failed"));
			return false;
		}
		return true;
	}
	catch (...)
	{
		return true;
	}
}

bool RobotMotionClientImpl::getFeedback(RobotFeedback& out)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	json req{{"cmd", "get_feedback"}, {"id", m_nextId++}};
	std::string resp;
	if (!sendRequest(req.dump(), resp))
		return false;
	try
	{
		const auto j = json::parse(resp);
		if (!j.value("ok", false))
		{
			setError(j.value("message", "get_feedback failed"));
			return false;
		}
		out = RobotFeedback{};
		out.hasJoints = j.value("hasJoints", false);
		out.hasPose = j.value("hasPose", false);
		out.controllerState = j.value("controllerState", "");
		out.timestampMs = j.value("timestampMs", static_cast<int64_t>(0));
		if (j.contains("jointRad") && j["jointRad"].is_array())
		{
			for (const auto& v : j["jointRad"])
				out.jointRad.push_back(v.get<double>());
			out.hasJoints = !out.jointRad.empty();
		}
		if (j.contains("toolPoseInBase") && j["toolPoseInBase"].is_object())
		{
			const auto& p = j["toolPoseInBase"];
			if (p.contains("positionMm") && p["positionMm"].is_array() && p["positionMm"].size() >= 3)
			{
				out.toolPoseInBase.positionMm[0] = p["positionMm"][0].get<double>();
				out.toolPoseInBase.positionMm[1] = p["positionMm"][1].get<double>();
				out.toolPoseInBase.positionMm[2] = p["positionMm"][2].get<double>();
			}
			if (p.contains("eulerDeg") && p["eulerDeg"].is_array() && p["eulerDeg"].size() >= 3)
			{
				out.toolPoseInBase.eulerDeg[0] = p["eulerDeg"][0].get<double>();
				out.toolPoseInBase.eulerDeg[1] = p["eulerDeg"][1].get<double>();
				out.toolPoseInBase.eulerDeg[2] = p["eulerDeg"][2].get<double>();
			}
			out.hasPose = true;
		}
		m_lastError.clear();
		return out.hasJoints || out.hasPose;
	}
	catch (const std::exception& ex)
	{
		setError(std::string("feedback parse: ") + ex.what());
		return false;
	}
}
