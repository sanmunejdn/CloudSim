#ifndef CLOUDSIMCONTROLLERSDK_CONTROLLERCLIENTIMPL_H
#define CLOUDSIMCONTROLLERSDK_CONTROLLERCLIENTIMPL_H

/// @file ControllerClientImpl.h
/// @brief ControllerClientImpl

#include "IControllerClient.h"

#include <mutex>
#include <string>

class ControllerClientImpl final : public IControllerClient
{
public:
	ControllerClientImpl();
	~ControllerClientImpl() override;

	bool connectHost(const ControllerEndpoint& endpoint) override;
	void disconnectHost() override;
	bool isConnected() const override;

	bool hello(int robotInstanceIndex, ControllerHelloAck& outAck) override;
	bool stepPose(int dtMs, const double tcpMm[3], const double eulerDeg[3], const std::string& frame,
				  ControllerStepReply& outReply) override;
	bool step(int dtMs, const std::vector<double>& targetJointRad, ControllerStepReply& outReply) override;
	bool goodbye() override;

	std::string lastError() const override;

private:
	bool ensureWinsock();
	bool sendLine(const std::string& jsonLine, std::string& responseLine);
	void setError(const std::string& msg);

	mutable std::mutex m_mutex;
	void* m_socket = nullptr;
	bool m_connected = false;
	std::string m_lastError;
	int m_timeoutMs = 5000;
	int m_jointCount = 0;
	int m_protocolVer = CLOUDSIM_CONTROLLER_PROTOCOL_VER;
};

#endif // CLOUDSIMCONTROLLERSDK_CONTROLLERCLIENTIMPL_H
