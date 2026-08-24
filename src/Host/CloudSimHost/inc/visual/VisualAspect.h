#ifndef CLOUDSIMHOST_VISUALASPECT_H
#define CLOUDSIMHOST_VISUALASPECT_H

/// @file VisualAspect.h
/// @brief OSG 同步面（位姿/外观/可见性/几何等）

#include <cstdint>

namespace cloudsim::host
{
enum class VisualAspect : std::uint32_t
{
	None = 0,
	Transform = 1u << 0,
	Appearance = 1u << 1,
	Visibility = 1u << 2,
	Geometry = 1u << 3,
	Selection = 1u << 4,
	Hierarchy = 1u << 5
};

constexpr VisualAspect operator|(VisualAspect a, VisualAspect b)
{
	return static_cast<VisualAspect>(static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b));
}

constexpr VisualAspect operator&(VisualAspect a, VisualAspect b)
{
	return static_cast<VisualAspect>(static_cast<std::uint32_t>(a) & static_cast<std::uint32_t>(b));
}

constexpr bool hasVisualAspect(VisualAspect set, VisualAspect flag)
{
	return (static_cast<std::uint32_t>(set & flag)) != 0u;
}

enum class VisualChangeReason : std::uint8_t
{
	PropertyCommit,
	FollowSolve,
	FkWrite,
	Import,
	Selection,
	Manual
};

enum class FlushPolicy : std::uint8_t
{
	Immediate,
	CoalescePerId
};

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_VISUALASPECT_H
