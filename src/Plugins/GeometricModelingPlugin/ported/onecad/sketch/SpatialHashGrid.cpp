#include "SpatialHashGrid.h"

#include "Sketch.h"
#include "SketchArc.h"
#include "SketchCircle.h"
#include "SketchEllipse.h"
#include "SketchLine.h"
#include "SketchPoint.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace onecad::core::sketch {

namespace {

bool computeBoundingCircle(const SketchEntity& entity,
                           const Sketch& sketch,
                           Vec2d& center,
                           double& radius)
{
    switch (entity.type()) {
        case EntityType::Point: {
            const auto* point = static_cast<const SketchPoint*>(&entity);
            center = {point->x(), point->y()};
            radius = 0.0;
            return true;
        }
        case EntityType::Line: {
            const auto* line = static_cast<const SketchLine*>(&entity);
            const auto* start = sketch.getEntityAs<SketchPoint>(line->startPointId());
            const auto* end = sketch.getEntityAs<SketchPoint>(line->endPointId());
            if (!start || !end) {
                return false;
            }

            const double sx = start->x();
            const double sy = start->y();
            const double ex = end->x();
            const double ey = end->y();
            center = {(sx + ex) * 0.5, (sy + ey) * 0.5};
            radius = 0.5 * std::hypot(ex - sx, ey - sy);
            return true;
        }
        case EntityType::Arc: {
            const auto* arc = static_cast<const SketchArc*>(&entity);
            const auto* centerPoint = sketch.getEntityAs<SketchPoint>(arc->centerPointId());
            if (!centerPoint) {
                return false;
            }
            center = {centerPoint->x(), centerPoint->y()};
            radius = std::max(0.0, arc->radius());
            return true;
        }
        case EntityType::Circle: {
            const auto* circle = static_cast<const SketchCircle*>(&entity);
            const auto* centerPoint = sketch.getEntityAs<SketchPoint>(circle->centerPointId());
            if (!centerPoint) {
                return false;
            }
            center = {centerPoint->x(), centerPoint->y()};
            radius = std::max(0.0, circle->radius());
            return true;
        }
        case EntityType::Ellipse: {
            const auto* ellipse = static_cast<const SketchEllipse*>(&entity);
            const auto* centerPoint = sketch.getEntityAs<SketchPoint>(ellipse->centerPointId());
            if (!centerPoint) {
                return false;
            }
            center = {centerPoint->x(), centerPoint->y()};
            radius = std::max(0.0, ellipse->majorRadius());
            return true;
        }
        default:
            break;
    }

    const BoundingBox2d bounds = entity.bounds();
    if (bounds.isEmpty()) {
        return false;
    }

    const double cx = (bounds.minX + bounds.maxX) * 0.5;
    const double cy = (bounds.minY + bounds.maxY) * 0.5;
    center = {cx, cy};
    radius = 0.5 * std::hypot(bounds.maxX - bounds.minX, bounds.maxY - bounds.minY);
    return true;
}

} // namespace

SpatialHashGrid::SpatialHashGrid(double cellSize)
    : cellSize_(cellSize)
{
    if (cellSize_ <= 0.0) {
        throw std::invalid_argument("SpatialHashGrid cell size must be positive");
    }
}

void SpatialHashGrid::clear() {
    cells_.clear();
}

void SpatialHashGrid::insert(const EntityID& id, const Vec2d& center, double radius) {
    const double safeRadius = std::max(0.0, radius);
    const int minCellX = static_cast<int>(std::floor((center.x - safeRadius) / cellSize_));
    const int maxCellX = static_cast<int>(std::floor((center.x + safeRadius) / cellSize_));
    const int minCellY = static_cast<int>(std::floor((center.y - safeRadius) / cellSize_));
    const int maxCellY = static_cast<int>(std::floor((center.y + safeRadius) / cellSize_));

    for (int cellX = minCellX; cellX <= maxCellX; ++cellX) {
        for (int cellY = minCellY; cellY <= maxCellY; ++cellY) {
            cells_[hashCell(cellX, cellY)].push_back(id);
        }
    }
}

void SpatialHashGrid::rebuild(const Sketch& sketch) {
    clear();

    for (const auto& entity : sketch.getAllEntities()) {
        Vec2d center;
        double radius = 0.0;
        if (!computeBoundingCircle(*entity, sketch, center, radius)) {
            continue;
        }
        insert(entity->id(), center, radius);
    }
}

std::vector<EntityID> SpatialHashGrid::query(const Vec2d& center, double radius) const {
    std::vector<EntityID> candidates;
    if (cells_.empty()) {
        return candidates;
    }

    const double safeRadius = std::max(0.0, radius);
    const int minCellX = static_cast<int>(std::floor((center.x - safeRadius) / cellSize_));
    const int maxCellX = static_cast<int>(std::floor((center.x + safeRadius) / cellSize_));
    const int minCellY = static_cast<int>(std::floor((center.y - safeRadius) / cellSize_));
    const int maxCellY = static_cast<int>(std::floor((center.y + safeRadius) / cellSize_));

    std::unordered_set<EntityID> unique;
    for (int cellX = minCellX; cellX <= maxCellX; ++cellX) {
        for (int cellY = minCellY; cellY <= maxCellY; ++cellY) {
            const long long key = hashCell(cellX, cellY);
            const auto it = cells_.find(key);
            if (it == cells_.end()) {
                continue;
            }
            unique.insert(it->second.begin(), it->second.end());
        }
    }

    candidates.reserve(unique.size());
    for (const EntityID& id : unique) {
        candidates.push_back(id);
    }
    return candidates;
}

bool SpatialHashGrid::empty() const {
    return cells_.empty();
}

long long SpatialHashGrid::hashCell(int cellX, int cellY) {
    const long long x = static_cast<long long>(cellX);
    const long long y = static_cast<long long>(cellY);
    return (x * 73856093LL) ^ (y * 19349663LL);
}

} // namespace onecad::core::sketch
