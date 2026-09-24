/// @file main.cpp
/// @brief CloudSim 外置控制器 C++ 样例：正弦关节 STEP

#include "IControllerClient.h"

#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

namespace
{
ControllerEndpoint endpointFromEnv()
{
	ControllerEndpoint ep;
	const char* raw = std::getenv("CLOUDSIM_HOST");
	if (!raw || !*raw)
		return ep;
	const char* colon = std::strrchr(raw, ':');
	if (!colon || colon == raw)
	{
		ep.host = raw;
		return ep;
	}
	ep.host.assign(raw, colon);
	ep.port = static_cast<uint16_t>(std::atoi(colon + 1));
	if (ep.port == 0)
		ep.port = CLOUDSIM_CONTROLLER_DEFAULT_PORT;
	return ep;
}

int robotIndexFromEnv()
{
	const char* raw = std::getenv("CLOUDSIM_ROBOT_INDEX");
	if (!raw || !*raw)
		return 0;
	return std::atoi(raw);
}
} // namespace

int main()
{
	auto client = createControllerClient();
	const ControllerEndpoint ep = endpointFromEnv();
	if (!client->connectHost(ep))
	{
		std::fprintf(stderr, "connect failed: %s\n", client->lastError().c_str());
		return 1;
	}
	ControllerHelloAck ack;
	if (!client->hello(robotIndexFromEnv(), ack))
	{
		std::fprintf(stderr, "hello failed: %s\n", client->lastError().c_str());
		return 2;
	}
	const int n = ack.jointCount;
	const int dt = ack.simDtMs > 0 ? ack.simDtMs : 16;
	std::fprintf(stdout, "jointCount=%d simDtMs=%d lower=%zu upper=%zu\n", n, dt, ack.jointLowerRad.size(),
				 ack.jointUpperRad.size());
	for (int k = 0; k < 2000; ++k)
	{
		std::vector<double> q(static_cast<size_t>(n), 0.0);
		for (int i = 0; i < n; ++i)
			q[static_cast<size_t>(i)] = 0.3 * std::sin(0.05 * k + 0.4 * i);
		ControllerStepReply reply;
		if (!client->step(dt, q, reply))
		{
			std::fprintf(stderr, "step failed: %s\n", client->lastError().c_str());
			break;
		}
		if (k % 50 == 0)
		{
			const auto& jp =
				reply.sensorJointPosition.empty() ? reply.actualJointRad : reply.sensorJointPosition;
			std::fprintf(stdout, "simTimeMs=%d sensors.jointPosition[0]=%.3f\n", reply.simTimeMs,
						 jp.empty() ? 0.0 : jp.front());
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(dt));
	}
	client->goodbye();
	return 0;
}
