/// @file main.cpp
/// @brief CloudSim 外置控制器 C++ 样例：正弦关节 STEP

#include "IControllerClient.h"

#include <cmath>
#include <chrono>
#include <cstdio>
#include <thread>
#include <vector>

int main()
{
	auto client = createControllerClient();
	ControllerEndpoint ep;
	ep.host = "127.0.0.1";
	ep.port = 19620;
	if (!client->connectHost(ep))
	{
		std::fprintf(stderr, "connect failed: %s\n", client->lastError().c_str());
		return 1;
	}
	ControllerHelloAck ack;
	if (!client->hello(0, ack))
	{
		std::fprintf(stderr, "hello failed: %s\n", client->lastError().c_str());
		return 2;
	}
	const int n = ack.jointCount;
	const int dt = ack.simDtMs > 0 ? ack.simDtMs : 16;
	std::fprintf(stdout, "jointCount=%d simDtMs=%d\n", n, dt);
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
			std::fprintf(stdout, "simTimeMs=%d\n", reply.simTimeMs);
		std::this_thread::sleep_for(std::chrono::milliseconds(dt));
	}
	client->goodbye();
	return 0;
}
