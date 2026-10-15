#ifndef ROBOTSCENE_LOGICALCLOCK_H
#define ROBOTSCENE_LOGICALCLOCK_H

/// @file LogicalClock.h
/// @brief 仿真逻辑时钟（与 wall QTimer 解耦）

#include "robot_scene_global.h"

#include <algorithm>
#include <cstdint>

class ROBOT_SCENE_API LogicalClock
{
public:
	std::int64_t tickIndex() const { return m_tickIndex; }
	double simTimeSec() const { return m_simTimeSec; }
	double playbackRate() const { return m_playbackRate; }

	void setPlaybackRate(double rate)
	{
		m_playbackRate = std::clamp(rate, 0.1, 10.0);
	}

	void reset()
	{
		m_tickIndex = 0;
		m_simTimeSec = 0.0;
	}

	/// 推进一拍，返回本拍逻辑 dt
	double advance(double dtWallSec)
	{
		const double dt = std::max(0.0, dtWallSec) * m_playbackRate;
		m_simTimeSec += dt;
		++m_tickIndex;
		return dt;
	}

private:
	std::int64_t m_tickIndex = 0;
	double m_simTimeSec = 0.0;
	double m_playbackRate = 1.0;
};

#endif // ROBOTSCENE_LOGICALCLOCK_H
