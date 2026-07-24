/// @file RealRobotPoseSource.cpp
/// @brief TCP 读一行 JSON 末端位姿：{"x","y","z","rx","ry","rz"} mm/deg

#include "IRobotPoseSource.h"

#define WIN32_LEAN_AND_MEAN
#include <WinSock2.h>
#include <WS2tcpip.h>

#include <cstdio>
#include <cstring>
#include <string>

#pragma comment(lib, "Ws2_32.lib")

namespace industrial_camera
{
namespace
{

bool parsePoseJson(const std::string& line, Pose6d& out)
{
	auto findNum = [&](const char* key, double& v) -> bool {
		const std::string k = std::string("\"") + key + "\"";
		const auto pos = line.find(k);
		if (pos == std::string::npos)
			return false;
		const auto colon = line.find(':', pos);
		if (colon == std::string::npos)
			return false;
		v = std::atof(line.c_str() + colon + 1);
		return true;
	};
	return findNum("x", out.x) && findNum("y", out.y) && findNum("z", out.z) && findNum("rx", out.rxDeg)
		   && findNum("ry", out.ryDeg) && findNum("rz", out.rzDeg);
}

class RealRobotPoseSource final : public IRobotPoseSource
{
public:
	explicit RealRobotPoseSource(RealRobotPoseConfig cfg)
		: cfg_(std::move(cfg))
	{
		WSADATA wsa{};
		WSAStartup(MAKEWORD(2, 2), &wsa);
	}

	~RealRobotPoseSource() override { WSACleanup(); }

	bool getCurrentPose(Pose6d& out) override
	{
		lastError_.clear();
		SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (s == INVALID_SOCKET)
		{
			lastError_ = "socket 创建失败";
			return false;
		}
		DWORD tv = static_cast<DWORD>(cfg_.timeoutMs);
		setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&tv), sizeof(tv));
		setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&tv), sizeof(tv));

		sockaddr_in addr{};
		addr.sin_family = AF_INET;
		addr.sin_port = htons(static_cast<u_short>(cfg_.port));
		if (inet_pton(AF_INET, cfg_.host.c_str(), &addr.sin_addr) != 1)
		{
			lastError_ = "无效主机: " + cfg_.host;
			closesocket(s);
			return false;
		}
		if (connect(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0)
		{
			lastError_ = "连接机器人位姿服务失败 " + cfg_.host + ":" + std::to_string(cfg_.port);
			closesocket(s);
			return false;
		}
		// 请求一行
		const char* req = "GET_POSE\n";
		send(s, req, static_cast<int>(std::strlen(req)), 0);
		char buf[512]{};
		const int n = recv(s, buf, sizeof(buf) - 1, 0);
		closesocket(s);
		if (n <= 0)
		{
			lastError_ = "未收到位姿数据";
			return false;
		}
		buf[n] = 0;
		if (!parsePoseJson(buf, out))
		{
			lastError_ = "JSON 解析失败，期望 {\"x\",\"y\",\"z\",\"rx\",\"ry\",\"rz\"}";
			return false;
		}
		return true;
	}

	std::string lastError() const override { return lastError_; }

private:
	RealRobotPoseConfig cfg_;
	std::string lastError_;
};

} // namespace

std::unique_ptr<IRobotPoseSource> createRealRobotPoseSource(const RealRobotPoseConfig& cfg)
{
	return std::make_unique<RealRobotPoseSource>(cfg);
}

} // namespace industrial_camera
