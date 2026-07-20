#ifndef PLCCOMMSDK_PLCCOMMCLIENTIMPL_H
#define PLCCOMMSDK_PLCCOMMCLIENTIMPL_H

/// @file PlcCommClientImpl.h
/// @brief PlcCommClientImpl 接口

#include "IPlcCommClient.h"
#include "PlcTagHandle.h"

#include <cstdint>
#include <map>
#include <mutex>
#include <string>

class PlcCommClientImpl final : public IPlcCommClient
{
public:
	PlcCommClientImpl() = default;
	~PlcCommClientImpl() override;

	bool connect(const PlcConnectionConfig& config) override;
	void disconnect() override;
	bool isConnected() const override;

	int addTag(const PlcTagSpec& spec) override;
	void removeTag(int handle) override;

	bool readTag(int handle, PlcTagValue& out) override;
	bool writeTag(int handle, const PlcTagValue& value) override;

	std::string lastError() const override;

private:
	static constexpr int kDefaultTimeoutMs = 10000;
	static constexpr int kMinTimeoutMs = 1000;
	static constexpr int kMaxTimeoutMs = 120000;

	int effectiveTimeoutMs() const;
	bool waitForTagStatus(int32_t tagId, int timeoutMs);
	bool probeModbusConnection(const PlcConnectionConfig& config);
	void setErrorFromStatus(int status);
	PlcTagHandle* findTag(int handle);

	mutable std::mutex mutex_;
	PlcConnectionConfig config_{};
	bool connected_ = false;
	int nextHandle_ = 1;
	std::map<int, PlcTagHandle> tags_;
	std::string lastError_;
};

#endif // PLCCOMMSDK_PLCCOMMCLIENTIMPL_H
