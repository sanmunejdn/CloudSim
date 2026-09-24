/// @file ControllerClientImpl.cpp
/// @brief localhost TCP JSON 客户端（CONSENSUS protocolVer=1）

#include "ControllerClientImpl.h"

#include <json.hpp>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <cstring>
#include <sstream>

#include <winsock2.h>
#include <ws2tcpip.h>

#pragma comment(lib, "ws2_32.lib")

using json = nlohmann::json;

namespace
{
bool g_wsaStarted = false;
}

ControllerClientImpl::ControllerClientImpl() = default;

ControllerClientImpl::~ControllerClientImpl()
{
	disconnectHost();
}

bool ControllerClientImpl::ensureWinsock()
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

void ControllerClientImpl::setError(const std::string& msg)
{
	m_lastError = msg;
}

std::string ControllerClientImpl::lastError() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_lastError;
}

bool ControllerClientImpl::isConnected() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_connected;
}

bool ControllerClientImpl::connectHost(const ControllerEndpoint& endpoint)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	disconnectHost();
	if (!ensureWinsock())
		return false;

	SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (s == INVALID_SOCKET)
	{
		setError("socket create failed");
		return false;
	}

	DWORD timeout = static_cast<DWORD>(endpoint.timeoutMs > 0 ? endpoint.timeoutMs : 5000);
	setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout));
	setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout));

	sockaddr_in addr{};
	addr.sin_family = AF_INET;
	addr.sin_port = htons(endpoint.port);
	if (inet_pton(AF_INET, endpoint.host.c_str(), &addr.sin_addr) != 1)
	{
		closesocket(s);
		setError("invalid host");
		return false;
	}

	if (connect(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
	{
		closesocket(s);
		setError("connect failed");
		return false;
	}

	m_socket = reinterpret_cast<void*>(s);
	m_connected = true;
	m_timeoutMs = endpoint.timeoutMs > 0 ? endpoint.timeoutMs : 5000;
	m_lastError.clear();
	return true;
}

void ControllerClientImpl::disconnectHost()
{
	if (m_socket)
	{
		SOCKET s = reinterpret_cast<SOCKET>(m_socket);
		closesocket(s);
		m_socket = nullptr;
	}
	m_connected = false;
	m_jointCount = 0;
}

bool ControllerClientImpl::sendLine(const std::string& jsonLine, std::string& responseLine)
{
	if (!m_socket || !m_connected)
	{
		setError("not connected");
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
		const int n = send(s, payload.data() + sent, total - sent, 0);
		if (n <= 0)
		{
			setError("send failed");
			return false;
		}
		sent += n;
	}

	responseLine.clear();
	char buf[4096];
	while (responseLine.find('\n') == std::string::npos)
	{
		const int n = recv(s, buf, sizeof(buf), 0);
		if (n <= 0)
		{
			setError("recv failed");
			return false;
		}
		responseLine.append(buf, buf + n);
	}
	const auto pos = responseLine.find('\n');
	responseLine.resize(pos);
	if (!responseLine.empty() && responseLine.back() == '\r')
		responseLine.pop_back();
	return true;
}

bool ControllerClientImpl::hello(int robotInstanceIndex, ControllerHelloAck& outAck)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	json req;
	req["type"] = "HELLO";
	req["protocolVer"] = CLOUDSIM_CONTROLLER_PROTOCOL_VER;
	req["robotInstanceIndex"] = robotInstanceIndex;

	std::string resp;
	if (!sendLine(req.dump(), resp))
		return false;

	json j;
	try
	{
		j = json::parse(resp);
	}
	catch (...)
	{
		setError("HELLO_ACK parse failed");
		return false;
	}

	const std::string type = j.value("type", "");
	if (type == "ERROR")
	{
		setError(j.value("message", j.value("code", "ERROR")));
		return false;
	}
	if (type != "HELLO_ACK")
	{
		setError("unexpected type: " + type);
		return false;
	}
	if (j.value("protocolVer", 0) != CLOUDSIM_CONTROLLER_PROTOCOL_VER)
	{
		setError("PROTOCOL_MISMATCH");
		return false;
	}

	outAck.simDtMs = j.value("simDtMs", 16);
	outAck.jointCount = j.value("jointCount", 0);
	outAck.jointNames.clear();
	if (j.contains("jointNames") && j["jointNames"].is_array())
	{
		for (const auto& n : j["jointNames"])
			outAck.jointNames.push_back(n.get<std::string>());
	}
	m_jointCount = outAck.jointCount;
	m_lastError.clear();
	return outAck.jointCount > 0;
}

bool ControllerClientImpl::step(int dtMs, const std::vector<double>& targetJointRad, ControllerStepReply& outReply)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	if (m_jointCount > 0 && static_cast<int>(targetJointRad.size()) != m_jointCount)
	{
		setError("BAD_JOINT_COUNT");
		return false;
	}

	json req;
	req["type"] = "STEP";
	req["protocolVer"] = CLOUDSIM_CONTROLLER_PROTOCOL_VER;
	req["dtMs"] = dtMs;
	req["targetJointRad"] = targetJointRad;

	std::string resp;
	if (!sendLine(req.dump(), resp))
		return false;

	json j;
	try
	{
		j = json::parse(resp);
	}
	catch (...)
	{
		setError("STEP_REPLY parse failed");
		return false;
	}

	const std::string type = j.value("type", "");
	outReply = ControllerStepReply{};
	if (type == "QUIT")
	{
		outReply.quit = true;
		outReply.quitReason = j.value("reason", "");
		m_lastError.clear();
		return true;
	}
	if (type == "ERROR")
	{
		setError(j.value("message", j.value("code", "ERROR")));
		return false;
	}
	if (type != "STEP_REPLY")
	{
		setError("unexpected type: " + type);
		return false;
	}

	outReply.simTimeMs = j.value("simTimeMs", 0);
	outReply.actualJointRad.clear();
	if (j.contains("actualJointRad") && j["actualJointRad"].is_array())
	{
		for (const auto& v : j["actualJointRad"])
			outReply.actualJointRad.push_back(v.get<double>());
	}
	m_lastError.clear();
	return true;
}

bool ControllerClientImpl::goodbye()
{
	std::lock_guard<std::mutex> lock(m_mutex);
	if (!m_connected)
		return true;
	json req;
	req["type"] = "GOODBYE";
	req["protocolVer"] = CLOUDSIM_CONTROLLER_PROTOCOL_VER;
	std::string resp;
	(void)sendLine(req.dump(), resp);
	disconnectHost();
	return true;
}
