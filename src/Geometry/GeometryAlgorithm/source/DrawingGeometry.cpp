/// @file DrawingGeometry.cpp
/// @brief 图元 → 折线（优先已离散点，避免错误解析重采样）

#include "DrawingGeometry.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>
#include <utility>

namespace geoalgo
{
namespace
{

/// 仅整圆按圆心+半径去重；弧段不可并键，否则同圆多弧只剩一段 → 缺线
void dedupeEntitiesInPlace(std::vector<DrawingEntity>& ents)
{
	if (ents.size() < 2)
		return;
	std::unordered_set<long long> seen;
	seen.reserve(ents.size() * 2);
	std::vector<DrawingEntity> kept;
	kept.reserve(ents.size());
	auto q = [](double v, double inv) { return static_cast<long long>(std::llround(v * inv)); };
	for (DrawingEntity& e : ents)
	{
		long long key = 0;
		bool useKey = false;
		if (e.kind == DrawingEntityKind::Circle)
		{
			const double inv = 100.0;
			key = (q(e.data[0], inv) << 42) ^ (q(e.data[1], inv) << 21) ^ q(e.data[3], inv);
			key ^= (e.hidden ? 1LL : 0LL) << 60;
			useKey = true;
		}
		else if (e.kind == DrawingEntityKind::Line || e.polylineXy.size() >= 6)
		{
			const double inv = 50.0;
			float x0 = 0, y0 = 0, x1 = 0, y1 = 0;
			if (e.kind == DrawingEntityKind::Line)
			{
				x0 = static_cast<float>(e.data[0]);
				y0 = static_cast<float>(e.data[1]);
				x1 = static_cast<float>(e.data[3]);
				y1 = static_cast<float>(e.data[4]);
			}
			else
			{
				x0 = e.polylineXy[0];
				y0 = e.polylineXy[1];
				x1 = e.polylineXy[e.polylineXy.size() - 3];
				y1 = e.polylineXy[e.polylineXy.size() - 2];
			}
			long long a = (q(x0, inv) << 32) ^ q(y0, inv);
			long long b = (q(x1, inv) << 32) ^ q(y1, inv);
			if (a > b)
				std::swap(a, b);
			// 中点防「同端点不同路径」误杀
			float xm = 0, ym = 0;
			if (e.polylineXy.size() >= 9)
			{
				const std::size_t mid = (e.polylineXy.size() / 3 / 2) * 3;
				xm = e.polylineXy[mid];
				ym = e.polylineXy[mid + 1];
			}
			else
			{
				xm = 0.5f * (x0 + x1);
				ym = 0.5f * (y0 + y1);
			}
			key = a ^ (b << 1) ^ (q(xm, inv) << 16) ^ q(ym, inv);
			key ^= (e.hidden ? 1LL : 0LL) << 60;
			useKey = true;
		}
		if (useKey)
		{
			if (seen.find(key) != seen.end())
				continue;
			seen.insert(key);
		}
		kept.push_back(std::move(e));
	}
	ents.swap(kept);
}

bool polylineFromStored(const DrawingEntity& e, Polyline3d& out)
{
	if (e.polylineXy.size() < 6)
		return false;
	out.xyz = e.polylineXy;
	return true;
}

bool entityToPolyline(const DrawingEntity& e, Polyline3d& out)
{
	// 离散点是权威几何；解析采样仅兜底
	if (polylineFromStored(e, out))
		return true;
	out.xyz.clear();
	if (e.kind == DrawingEntityKind::Line)
	{
		out.xyz = {static_cast<float>(e.data[0]), static_cast<float>(e.data[1]), 0.f,
				   static_cast<float>(e.data[3]), static_cast<float>(e.data[4]), 0.f};
		return true;
	}
	return false;
}

} // namespace

void drawingEntitiesToPolylines(const std::vector<DrawingEntity>& ents, std::vector<Polyline3d>& visible,
								std::vector<Polyline3d>& hidden)
{
	visible.clear();
	hidden.clear();
	std::vector<DrawingEntity> local = ents;
	dedupeEntitiesInPlace(local);
	visible.reserve(local.size());
	hidden.reserve(local.size() / 4);
	for (const DrawingEntity& e : local)
	{
		Polyline3d poly;
		if (!entityToPolyline(e, poly))
			continue;
		if (e.hidden)
			hidden.push_back(std::move(poly));
		else
			visible.push_back(std::move(poly));
	}
}

} // namespace geoalgo
