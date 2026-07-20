/// @file PlcCommClientImpl.cpp
/// @brief PlcCommClientImpl 实现

#include "PlcCommClientImpl.h"

#include "PlcTagStringBuilder.h"

#include <chrono>
#include <thread>
#include <utility>

#include <libplctag.h>

namespace
{
constexpr int kStatusPollIntervalMs = 20;

} // namespace

PlcCommClientImpl::~PlcCommClientImpl()
{
	disconnect();
}

bool PlcCommClientImpl::connect(const PlcConnectionConfig& config)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (config.gateway.empty())
	{
		lastError_ = "gateway 为空";
		return false;
	}
	if (config.protocol == PlcProtocol::AbEip && config.path.empty())
	{
		lastError_ = "AB EIP 需要 path";
		return false;
	}

	tags_.clear();
	nextHandle_ = 1;
	connected_ = false;
	lastError_.clear();

	if (config.protocol == PlcProtocol::ModbusTcp)
	{
		if (!probeModbusConnection(config))
		{
			return false;
		}
	}

	config_ = config;
	connected_ = true;
	return true;
}

void PlcCommClientImpl::disconnect()
{
	std::lock_guard<std::mutex> lock(mutex_);
	tags_.clear();
	connected_ = false;
	plc_tag_shutdown();
	lastError_.clear();
}

bool PlcCommClientImpl::isConnected() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return connected_;
}

int PlcCommClientImpl::addTag(const PlcTagSpec& spec)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!connected_)
	{
		lastError_ = "未连接";
		return -1;
	}
	if (spec.name.empty())
	{
		lastError_ = "标签名为空";
		return -1;
	}

	std::string attrib;
	std::string buildError;
	if (!buildPlcTagAttributeString(config_, spec, &attrib, &buildError))
	{
		lastError_ = buildError.empty() ? "标签属性串无效" : buildError;
		return -1;
	}

	const int createTimeout = effectiveTimeoutMs();
	PlcTagHandle handle(attrib, createTimeout);
	if (!handle.valid())
	{
		setErrorFromStatus(handle.lastStatus());
		lastError_ += " (";
		lastError_ += attrib;
		lastError_ += ")";
		return -1;
	}
	if (!waitForTagStatus(handle.id(), createTimeout))
	{
		lastError_ += " (";
		lastError_ += attrib;
		lastError_ += ")";
		return -1;
	}

	const int id = nextHandle_++;
	tags_.emplace(id, std::move(handle));
	lastError_.clear();
	return id;
}

void PlcCommClientImpl::removeTag(int handle)
{
	std::lock_guard<std::mutex> lock(mutex_);
	tags_.erase(handle);
}

bool PlcCommClientImpl::readTag(int handle, PlcTagValue& out)
{
	std::lock_guard<std::mutex> lock(mutex_);
	PlcTagHandle* tag = findTag(handle);
	if (!tag)
	{
		lastError_ = "无效句柄";
		return false;
	}

	const int ioTimeout = effectiveTimeoutMs();
	const int rc = plc_tag_read(tag->id(), ioTimeout);
	if (rc != PLCTAG_STATUS_OK)
	{
		setErrorFromStatus(rc);
		return false;
	}

	const int size = plc_tag_get_size(tag->id());
	if (size < 0)
	{
		setErrorFromStatus(size);
		return false;
	}

	out.data.resize(static_cast<size_t>(size));
	if (size > 0)
	{
		const int rawRc = plc_tag_get_raw_bytes(tag->id(), 0, out.data.data(), size);
		if (rawRc != PLCTAG_STATUS_OK)
		{
			setErrorFromStatus(rawRc);
			out.data.clear();
			return false;
		}
	}
	lastError_.clear();
	return true;
}

bool PlcCommClientImpl::writeTag(int handle, const PlcTagValue& value)
{
	std::lock_guard<std::mutex> lock(mutex_);
	PlcTagHandle* tag = findTag(handle);
	if (!tag)
	{
		lastError_ = "无效句柄";
		return false;
	}

	if (!value.data.empty())
	{
		const int rawRc = plc_tag_set_raw_bytes(tag->id(), 0, const_cast<uint8_t*>(value.data.data()),
												static_cast<int>(value.data.size()));
		if (rawRc != PLCTAG_STATUS_OK)
		{
			setErrorFromStatus(rawRc);
			return false;
		}
	}

	const int ioTimeout = effectiveTimeoutMs();
	const int rc = plc_tag_write(tag->id(), ioTimeout);
	if (rc != PLCTAG_STATUS_OK)
	{
		setErrorFromStatus(rc);
		return false;
	}
	lastError_.clear();
	return true;
}

std::string PlcCommClientImpl::lastError() const
{
	std::lock_guard<std::mutex> lock(mutex_);
	return lastError_;
}

int PlcCommClientImpl::effectiveTimeoutMs() const
{
	if (config_.timeoutMs < kMinTimeoutMs)
	{
		return kDefaultTimeoutMs;
	}
	if (config_.timeoutMs > kMaxTimeoutMs)
	{
		return kMaxTimeoutMs;
	}
	return config_.timeoutMs;
}

bool PlcCommClientImpl::waitForTagStatus(int32_t tagId, int timeoutMs)
{
	const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);

	for (;;)
	{
		const int status = plc_tag_status(tagId);
		if (status != PLCTAG_STATUS_PENDING)
		{
			if (status == PLCTAG_STATUS_OK)
			{
				lastError_.clear();
				return true;
			}
			setErrorFromStatus(status);
			return false;
		}
		if (std::chrono::steady_clock::now() >= deadline)
		{
			setErrorFromStatus(PLCTAG_ERR_TIMEOUT);
			return false;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(kStatusPollIntervalMs));
	}
}

bool PlcCommClientImpl::probeModbusConnection(const PlcConnectionConfig& config)
{
	PlcTagSpec probe;
	probe.name = "hr0";
	probe.elemCount = 1;

	std::string attrib;
	std::string buildError;
	if (!buildPlcTagAttributeString(config, probe, &attrib, &buildError))
	{
		lastError_ = buildError.empty() ? "探测标签无效" : buildError;
		return false;
	}

	const int timeout = config.timeoutMs < kMinTimeoutMs ? kDefaultTimeoutMs : config.timeoutMs;
	PlcTagHandle probeTag(attrib, timeout);
	if (!probeTag.valid())
	{
		setErrorFromStatus(probeTag.lastStatus());
		lastError_ += "；连接探测失败 (";
		lastError_ += attrib;
		lastError_ += ")";
		return false;
	}
	if (!waitForTagStatus(probeTag.id(), timeout))
	{
		lastError_ += " (";
		lastError_ += attrib;
		lastError_ += ")";
		return false;
	}

	// 建连以 create+status 为准；读寄存器在 Slave 未映射 hr0 时也会超时，不能作为连通判据
	lastError_.clear();
	return true;
}

void PlcCommClientImpl::setErrorFromStatus(int status)
{
	if (status < 0)
	{
		const char* msg = plc_tag_decode_error(status);
		lastError_ = msg ? msg : "未知错误";
		if (status == PLCTAG_ERR_TIMEOUT)
		{
			lastError_ += "；请检查 IP/端口/单元 ID、Modbus Slave 是否已 Connect、本机可试 127.0.0.1";
		}
	}
	else
	{
		lastError_.clear();
	}
}

PlcTagHandle* PlcCommClientImpl::findTag(int handle)
{
	const auto it = tags_.find(handle);
	if (it == tags_.end())
	{
		return nullptr;
	}
	return &it->second;
}
