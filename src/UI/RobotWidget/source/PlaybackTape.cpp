/// @file PlaybackTape.cpp

#include "PlaybackTape.h"

#include <algorithm>
#include <cmath>

void PlaybackTape::clear()
{
	m_frames.clear();
}

void PlaybackTape::setCapacityHint(int maxFrames)
{
	m_maxFrames = std::max(1, maxFrames);
}

void PlaybackTape::push(const TapeFrame& frame)
{
	if (!m_frames.isEmpty() && frame.t + 1e-9 < m_frames.back().t)
	{
		// 时钟回绕后重建
		m_frames.clear();
	}
	m_frames.push_back(frame);
	if (m_frames.size() > m_maxFrames)
	{
		const int drop = m_frames.size() - m_maxFrames;
		m_frames.erase(m_frames.begin(), m_frames.begin() + drop);
	}
}

double PlaybackTape::minT() const
{
	return m_frames.isEmpty() ? 0.0 : m_frames.front().t;
}

double PlaybackTape::maxT() const
{
	return m_frames.isEmpty() ? 0.0 : m_frames.back().t;
}

bool PlaybackTape::sampleAt(double t, TapeFrame* out) const
{
	if (!out || m_frames.isEmpty())
	{
		return false;
	}
	// 钳到已录区间，避免首帧非 0 / 拖到两端误报无采样
	if (t <= m_frames.front().t)
	{
		*out = m_frames.front();
		return true;
	}
	if (t >= m_frames.back().t)
	{
		*out = m_frames.back();
		return true;
	}
	auto it = std::lower_bound(m_frames.begin(), m_frames.end(), t,
							   [](const TapeFrame& f, double tt) { return f.t < tt; });
	if (it == m_frames.end())
	{
		*out = m_frames.back();
		return true;
	}
	if (it == m_frames.begin())
	{
		*out = *it;
		return true;
	}
	const TapeFrame& b = *it;
	const TapeFrame& a = *(it - 1);
	*out = (std::abs(b.t - t) < std::abs(a.t - t)) ? b : a;
	return true;
}
