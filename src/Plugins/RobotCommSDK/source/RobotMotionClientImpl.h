#ifndef ROBOTCOMMSDK_ROBOTMOTIONCLIENTIMPL_H
#define ROBOTCOMMSDK_ROBOTMOTIONCLIENTIMPL_H

#include "IRobotMotionClient.h"

#include <mutex>
#include <string>

class RobotMotionClientImpl final : public IRobotMotionClient
{
public:
	RobotMotionClientImpl();
	~RobotMotionClientImpl() override;

	bool connectBridge(const RobotCommBridgeEndpoint& endpoint) override;
	void disconnectBridge() override;
	bool isBridgeConnected() const override;

	bool connectRobot(const RobotCommConnectConfig& config) override;
	bool disconnectRobot() override;
	bool isRobotConnected() const override;

	bool ping() override;
	bool getFeedback(RobotFeedback& out) override;

	std::string lastError() const override;

private:
	bool ensureWinsock();
	bool sendRequest(const std::string& jsonLine, std::string& responseLine);
	void setError(const std::string& msg);

	mutable std::mutex m_mutex;
	void* m_socket = nullptr; // SOCKET as void* to avoid winsock in header
	bool m_bridgeConnected = false;
	bool m_robotConnected = false;
	std::string m_lastError;
	int m_nextId = 1;
	int m_timeoutMs = 3000;
};

#endif // ROBOTCOMMSDK_ROBOTMOTIONCLIENTIMPL_H
