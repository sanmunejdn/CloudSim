/// @file Progress.cpp
/// @brief 进度总线实现

#include "Progress.h"

#include <atomic>
#include <mutex>

namespace
{
std::mutex gMutex;
Progress::UiSink gUiSink;
std::atomic<std::uint64_t> gNextId{0};
std::atomic<bool> gReady{false};

void emitEvent(const Progress::Event& ev)
{
	Progress::UiSink sinkCopy;
	{
		std::lock_guard<std::mutex> lock(gMutex);
		sinkCopy = gUiSink;
	}
	if (sinkCopy)
	{
		sinkCopy(ev);
	}
}

double normalizeFraction(double fraction01)
{
	if (fraction01 < 0.0)
	{
		return fraction01;
	}
	// 兼容误用 0–100 的调用方
	if (fraction01 > 1.5)
	{
		fraction01 *= 0.01;
	}
	if (fraction01 > 1.0)
	{
		return 1.0;
	}
	return fraction01;
}
} // namespace

namespace Progress
{
void initialize()
{
	gReady.store(true, std::memory_order_release);
}

void shutdown()
{
	clearUiSink();
	gReady.store(false, std::memory_order_release);
}

void setUiSink(UiSink sink)
{
	std::lock_guard<std::mutex> lock(gMutex);
	gUiSink = std::move(sink);
}

void clearUiSink()
{
	std::lock_guard<std::mutex> lock(gMutex);
	gUiSink = {};
}

std::uint64_t begin(std::string title, bool cancellable)
{
	const std::uint64_t id = ++gNextId;
	announceStarted(id, std::move(title), cancellable);
	return id;
}

void announceStarted(std::uint64_t id, std::string title, bool cancellable)
{
	if (id == 0)
	{
		return;
	}
	// 避免 JobSystem id 与自增 id 撞车时后续 begin 复用
	const std::uint64_t cur = gNextId.load(std::memory_order_relaxed);
	if (id > cur)
	{
		gNextId.store(id, std::memory_order_relaxed);
	}
	Event ev;
	ev.kind = Kind::Started;
	ev.id = id;
	ev.title = std::move(title);
	ev.cancellable = cancellable;
	emitEvent(ev);
}

void report(std::uint64_t id, double fraction01, std::string_view message)
{
	if (id == 0)
	{
		return;
	}
	Event ev;
	ev.kind = Kind::Update;
	ev.id = id;
	ev.fraction = normalizeFraction(fraction01);
	ev.message.assign(message.begin(), message.end());
	emitEvent(ev);
}

void end(std::uint64_t id, bool ok, std::string_view error)
{
	if (id == 0)
	{
		return;
	}
	Event ev;
	ev.kind = Kind::Finished;
	ev.id = id;
	ev.success = ok;
	ev.error.assign(error.begin(), error.end());
	emitEvent(ev);
}

Scope::Scope(std::string title, bool cancellable) : m_id(begin(std::move(title), cancellable)) {}

Scope::~Scope()
{
	if (!m_ended && m_id != 0)
	{
		end(m_id, m_ok, m_error);
		m_ended = true;
	}
}

void Scope::report(double fraction01, std::string_view message)
{
	Progress::report(m_id, fraction01, message);
}

void Scope::setSuccess(bool ok, std::string_view error)
{
	m_ok = ok;
	m_error.assign(error.begin(), error.end());
	if (!m_ended && m_id != 0)
	{
		end(m_id, m_ok, m_error);
		m_ended = true;
	}
}
} // namespace Progress
