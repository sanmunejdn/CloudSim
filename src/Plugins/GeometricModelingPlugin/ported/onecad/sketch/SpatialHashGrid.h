#ifndef ONECAD_CORE_SKETCH_SPATIAL_HASH_GRID_H
#define ONECAD_CORE_SKETCH_SPATIAL_HASH_GRID_H

#include "SketchTypes.h"

#include <unordered_map>
#include <vector>

namespace onecad::core::sketch {

class Sketch;

class SpatialHashGrid {
public:
    explicit SpatialHashGrid(double cellSize = constants::SNAP_RADIUS_MM);

    void clear();
    void insert(const EntityID& id, const Vec2d& center, double radius);
    void rebuild(const Sketch& sketch);
    std::vector<EntityID> query(const Vec2d& center, double radius) const;
    bool empty() const;

private:
    double cellSize_;
    std::unordered_map<long long, std::vector<EntityID>> cells_;

    static long long hashCell(int cellX, int cellY);
};

} // namespace onecad::core::sketch

#endif // ONECAD_CORE_SKETCH_SPATIAL_HASH_GRID_H
