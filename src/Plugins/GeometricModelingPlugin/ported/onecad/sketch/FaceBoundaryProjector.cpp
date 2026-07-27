#include "FaceBoundaryProjector.h"

#include "SketchArc.h"
#include "SketchCircle.h"
#include "SketchLine.h"
#include "SketchPoint.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <GeomAbs_CurveType.hxx>
#include <QLoggingCategory>
#include <QString>
#include <TopAbs.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <unordered_set>
#include <vector>

namespace onecad::core::sketch {

Q_LOGGING_CATEGORY(logFaceBoundaryProjector, "onecad.core.faceboundary")

namespace {

constexpr double kPointEpsilon = 1e-9;
constexpr double kAngleTolerance = 1e-4;

double distanceSquared(const Vec2d& a, const Vec2d& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return dx * dx + dy * dy;
}

bool nearlyEqual(double a, double b, double tolerance) {
    return std::abs(a - b) <= tolerance;
}

bool samePoint(const Vec2d& a, const Vec2d& b, double tolerance) {
    return distanceSquared(a, b) <= tolerance * tolerance;
}

double normalizeAngle(double angle) {
    const double twoPi = 2.0 * std::numbers::pi_v<double>;
    angle = std::fmod(angle, twoPi);
    if (angle <= -std::numbers::pi_v<double>) {
        angle += twoPi;
    } else if (angle > std::numbers::pi_v<double>) {
        angle -= twoPi;
    }
    return angle;
}

double normalizeAnglePositive(double angle) {
    const double twoPi = 2.0 * std::numbers::pi_v<double>;
    angle = std::fmod(angle, twoPi);
    if (angle < 0.0) {
        angle += twoPi;
    }
    return angle;
}

bool sameAngle(double a, double b, double tolerance) {
    return std::abs(normalizeAngle(a - b)) <= tolerance;
}

std::optional<Vec2d> getPointPosition(const Sketch& sketch, const EntityID& pointId) {
    const auto* point = sketch.getEntityAs<SketchPoint>(pointId);
    if (!point) {
        return std::nullopt;
    }
    return Vec2d{point->position().X(), point->position().Y()};
}

struct PointResolveResult {
    EntityID id;
    bool created = false;
};

PointResolveResult findOrCreatePoint(Sketch& sketch,
                                     const Vec2d& position,
                                     double tolerance,
                                     FaceBoundaryProjector::ProjectionResult& outStats) {
    for (const auto& entity : sketch.getAllEntities()) {
        if (!entity) {
            continue;
        }
        const auto* point = dynamic_cast<const SketchPoint*>(entity.get());
        if (!point) {
            continue;
        }
        const Vec2d existing{point->position().X(), point->position().Y()};
        if (samePoint(existing, position, tolerance)) {
            return PointResolveResult{point->id(), false};
        }
    }

    EntityID pointId = sketch.addPoint(position.x, position.y, false);
    if (!pointId.empty()) {
        sketch.setEntityReferenceLocked(pointId, true);
        outStats.insertedPoints++;
        return PointResolveResult{pointId, true};
    }
    return {};
}

bool lineExists(const Sketch& sketch,
                const EntityID& startId,
                const EntityID& endId,
                double tolerance) {
    const auto startPos = getPointPosition(sketch, startId);
    const auto endPos = getPointPosition(sketch, endId);
    if (!startPos || !endPos) {
        return false;
    }

    for (const auto& entity : sketch.getAllEntities()) {
        if (!entity) {
            continue;
        }
        const auto* line = dynamic_cast<const SketchLine*>(entity.get());
        if (!line) {
            continue;
        }

        const auto aPos = getPointPosition(sketch, line->startPointId());
        const auto bPos = getPointPosition(sketch, line->endPointId());
        if (!aPos || !bPos) {
            continue;
        }

        const bool direct = samePoint(*startPos, *aPos, tolerance) &&
                            samePoint(*endPos, *bPos, tolerance);
        const bool reversed = samePoint(*startPos, *bPos, tolerance) &&
                              samePoint(*endPos, *aPos, tolerance);
        if (direct || reversed) {
            return true;
        }
    }

    return false;
}

bool circleExists(const Sketch& sketch,
                  const Vec2d& center,
                  double radius,
                  double pointTolerance,
                  double radiusTolerance) {
    for (const auto& entity : sketch.getAllEntities()) {
        if (!entity) {
            continue;
        }
        const auto* circle = dynamic_cast<const SketchCircle*>(entity.get());
        if (!circle) {
            continue;
        }
        const auto centerPos = getPointPosition(sketch, circle->centerPointId());
        if (!centerPos) {
            continue;
        }
        if (samePoint(*centerPos, center, pointTolerance) &&
            nearlyEqual(circle->radius(), radius, radiusTolerance)) {
            return true;
        }
    }
    return false;
}

bool arcExists(const Sketch& sketch,
               const Vec2d& center,
               double radius,
               double startAngle,
               double endAngle,
               double pointTolerance,
               double radiusTolerance) {
    for (const auto& entity : sketch.getAllEntities()) {
        if (!entity) {
            continue;
        }
        const auto* arc = dynamic_cast<const SketchArc*>(entity.get());
        if (!arc) {
            continue;
        }
        const auto centerPos = getPointPosition(sketch, arc->centerPointId());
        if (!centerPos) {
            continue;
        }
        if (!samePoint(*centerPos, center, pointTolerance)) {
            continue;
        }
        if (!nearlyEqual(arc->radius(), radius, radiusTolerance)) {
            continue;
        }
        if (sameAngle(arc->startAngle(), startAngle, kAngleTolerance) &&
            sameAngle(arc->endAngle(), endAngle, kAngleTolerance)) {
            return true;
        }
    }
    return false;
}

struct SegmentResult {
    bool valid = false;
    EntityID startPointId;
    EntityID endPointId;
};

SegmentResult addLineSegment(Sketch& sketch,
                             const Vec2d& start,
                             const Vec2d& end,
                             const FaceBoundaryProjector::Options& options,
                             FaceBoundaryProjector::ProjectionResult& outStats,
                             std::unordered_set<EntityID>& pointsToFix) {
    if (distanceSquared(start, end) <= kPointEpsilon * kPointEpsilon) {
        return {};
    }

    const auto startRes = findOrCreatePoint(sketch, start, options.pointMergeTolerance, outStats);
    const auto endRes = findOrCreatePoint(sketch, end, options.pointMergeTolerance, outStats);
    if (startRes.id.empty() || endRes.id.empty()) {
        return {};
    }

    if (startRes.created) {
        pointsToFix.insert(startRes.id);
    }
    if (endRes.created) {
        pointsToFix.insert(endRes.id);
    }

    if (!lineExists(sketch, startRes.id, endRes.id, options.pointMergeTolerance)) {
        const EntityID lineId = sketch.addLine(startRes.id, endRes.id, false);
        if (!lineId.empty()) {
            sketch.setEntityReferenceLocked(lineId, true);
            outStats.insertedCurves++;
        }
    }

    return SegmentResult{true, startRes.id, endRes.id};
}

std::vector<Vec2d> sampleEdgePolyline(const BRepAdaptor_Curve& curve,
                                      const Sketch& sketch,
                                      int segments) {
    std::vector<Vec2d> points;
    const int segmentCount = std::max(2, segments);
    points.reserve(static_cast<size_t>(segmentCount) + 1);

    const double first = curve.FirstParameter();
    const double last = curve.LastParameter();
    for (int i = 0; i <= segmentCount; ++i) {
        const double t = first + (last - first) * (static_cast<double>(i) / segmentCount);
        const gp_Pnt point3d = curve.Value(t);
        points.push_back(sketch.toSketch({point3d.X(), point3d.Y(), point3d.Z()}));
    }

    return points;
}

} // namespace

FaceBoundaryProjector::ProjectionResult FaceBoundaryProjector::projectFaceBoundaries(
    const TopoDS_Face& face,
    Sketch& sketch,
    const Options& options) {
    ProjectionResult result;
    qCDebug(logFaceBoundaryProjector) << "projectFaceBoundaries:start"
                                      << "existingEntityCount=" << sketch.getAllEntities().size()
                                      << "preferExactPrimitives=" << options.preferExactPrimitives
                                      << "lockInsertedBoundaryPoints=" << options.lockInsertedBoundaryPoints
                                      << "fallbackSegmentsPerCurve=" << options.fallbackSegmentsPerCurve;

    if (face.IsNull()) {
        result.errorMessage = "Face is null";
        qCWarning(logFaceBoundaryProjector) << "projectFaceBoundaries:face-null";
        return result;
    }

    std::unordered_set<EntityID> pointsToFix;
    int wireCount = 0;

    for (TopExp_Explorer wireExp(face, TopAbs_WIRE); wireExp.More(); wireExp.Next()) {
        ++wireCount;
        const TopoDS_Wire wire = TopoDS::Wire(wireExp.Current());
        const bool wireIsClosed = wire.Closed();
        BRepTools_WireExplorer edgeExp(wire, face);

        EntityID firstPointId;
        EntityID lastPointId;
        int wireCurveCount = 0;
        bool wireClosedByCircle = false;
        qCDebug(logFaceBoundaryProjector) << "projectFaceBoundaries:wire-start"
                                          << "wireIndex=" << wireCount
                                          << "wireClosed=" << wireIsClosed;

        while (edgeExp.More()) {
            const TopoDS_Edge edge = edgeExp.Current();
            BRepAdaptor_Curve curve(edge);
            const GeomAbs_CurveType curveType = curve.GetType();

            if (curveType == GeomAbs_Line) {
                const gp_Pnt p1 = curve.Value(curve.FirstParameter());
                const gp_Pnt p2 = curve.Value(curve.LastParameter());
                const Vec2d s = sketch.toSketch({p1.X(), p1.Y(), p1.Z()});
                const Vec2d e = sketch.toSketch({p2.X(), p2.Y(), p2.Z()});
                const auto seg = addLineSegment(sketch, s, e, options, result, pointsToFix);
                if (seg.valid) {
                    if (firstPointId.empty()) {
                        firstPointId = seg.startPointId;
                    }
                    lastPointId = seg.endPointId;
                    wireCurveCount++;
                }
            } else if (curveType == GeomAbs_Circle && options.preferExactPrimitives) {
                const gp_Circ circle = curve.Circle();
                const double radius = circle.Radius();
                if (radius > kPointEpsilon) {
                    const double first = curve.FirstParameter();
                    const double last = curve.LastParameter();
                    const double span = std::abs(last - first);
                    const bool isFullCircle = edge.Closed() ||
                                              nearlyEqual(span, 2.0 * std::numbers::pi_v<double>, 1e-3);

                    if (isFullCircle) {
                        const gp_Pnt center3d = circle.Location();
                        const Vec2d center = sketch.toSketch({center3d.X(), center3d.Y(), center3d.Z()});
                        const auto centerRes = findOrCreatePoint(
                            sketch, center, options.pointMergeTolerance, result);
                        if (!centerRes.id.empty()) {
                            if (centerRes.created) {
                                pointsToFix.insert(centerRes.id);
                            }
                            if (!circleExists(sketch, center, radius,
                                              options.pointMergeTolerance,
                                              options.radiusTolerance)) {
                                const EntityID circleId = sketch.addCircle(centerRes.id, radius, false);
                                if (!circleId.empty()) {
                                    sketch.setEntityReferenceLocked(circleId, true);
                                    result.insertedCurves++;
                                }
                            }
                            wireCurveCount++;
                            wireClosedByCircle = true;
                        }
                    } else {
                        bool insertedExactArc = false;
                        const gp_Pnt center3d = circle.Location();
                        const Vec2d center = sketch.toSketch({center3d.X(), center3d.Y(), center3d.Z()});
                        const auto centerRes = findOrCreatePoint(
                            sketch, center, options.pointMergeTolerance, result);

                        if (!centerRes.id.empty()) {
                            if (centerRes.created) {
                                pointsToFix.insert(centerRes.id);
                            }

                            const gp_Pnt start3d = curve.Value(first);
                            const gp_Pnt end3d = curve.Value(last);
                            const Vec2d start = sketch.toSketch({start3d.X(), start3d.Y(), start3d.Z()});
                            const Vec2d end = sketch.toSketch({end3d.X(), end3d.Y(), end3d.Z()});

                            double startAngle = std::atan2(start.y - center.y, start.x - center.x);
                            double endAngle = std::atan2(end.y - center.y, end.x - center.x);

                            bool useFallback = false;
                            bool ccw = true;
                            gp_Pnt mid3d;
                            gp_Vec tangent3d;
                            try {
                                const double mid = first + 0.5 * (last - first);
                                curve.D1(mid, mid3d, tangent3d);
                            } catch (...) {
                                useFallback = true;
                            }

                            if (!useFallback) {
                                const Vec2d mid = sketch.toSketch({mid3d.X(), mid3d.Y(), mid3d.Z()});
                                const Vec2d radial{mid.x - center.x, mid.y - center.y};
                                const auto& plane = sketch.getPlane();
                                const double tangentX = tangent3d.X() * plane.xAxis.x +
                                                        tangent3d.Y() * plane.xAxis.y +
                                                        tangent3d.Z() * plane.xAxis.z;
                                const double tangentY = tangent3d.X() * plane.yAxis.x +
                                                        tangent3d.Y() * plane.yAxis.y +
                                                        tangent3d.Z() * plane.yAxis.z;
                                const double cross = radial.x * tangentY - radial.y * tangentX;
                                if (std::abs(cross) <= 1e-9) {
                                    useFallback = true;
                                } else {
                                    ccw = cross > 0.0;
                                }
                            }

                            if (!useFallback) {
                                double arcStart = startAngle;
                                double arcEnd = endAngle;
                                if (!ccw) {
                                    std::swap(arcStart, arcEnd);
                                }

                                const double sweep = normalizeAnglePositive(arcEnd - arcStart);
                                if (sweep <= kPointEpsilon ||
                                    nearlyEqual(sweep, 2.0 * std::numbers::pi_v<double>, 1e-3)) {
                                    useFallback = true;
                                } else {
                                    if (!arcExists(sketch, center, radius,
                                                   arcStart, arcEnd,
                                                   options.pointMergeTolerance,
                                                   options.radiusTolerance)) {
                                        const EntityID arcId = sketch.addArc(
                                            centerRes.id, radius, arcStart, arcEnd, false);
                                        if (!arcId.empty()) {
                                            sketch.setEntityReferenceLocked(arcId, true);
                                            result.insertedCurves++;
                                        }
                                    }
                                    wireCurveCount++;
                                    insertedExactArc = true;
                                }
                            }
                        }

                        if (!insertedExactArc) {
                            const auto sampled = sampleEdgePolyline(curve, sketch, options.fallbackSegmentsPerCurve);
                            for (size_t i = 0; i + 1 < sampled.size(); ++i) {
                                const auto seg = addLineSegment(sketch, sampled[i], sampled[i + 1],
                                                                options, result, pointsToFix);
                                if (!seg.valid) {
                                    continue;
                                }
                                if (firstPointId.empty()) {
                                    firstPointId = seg.startPointId;
                                }
                                lastPointId = seg.endPointId;
                                wireCurveCount++;
                            }
                        }
                    }
                }
            } else {
                const auto sampled = sampleEdgePolyline(curve, sketch, options.fallbackSegmentsPerCurve);
                for (size_t i = 0; i + 1 < sampled.size(); ++i) {
                    const auto seg = addLineSegment(sketch, sampled[i], sampled[i + 1],
                                                    options, result, pointsToFix);
                    if (!seg.valid) {
                        continue;
                    }
                    if (firstPointId.empty()) {
                        firstPointId = seg.startPointId;
                    }
                    lastPointId = seg.endPointId;
                    wireCurveCount++;
                }
            }

            edgeExp.Next();
        }

        if (wireClosedByCircle) {
            result.hasClosedBoundary = true;
        } else if (wireCurveCount >= 3 &&
                   ((wireIsClosed && !firstPointId.empty()) ||
                    (!firstPointId.empty() && firstPointId == lastPointId))) {
            result.hasClosedBoundary = true;
        }
        qCDebug(logFaceBoundaryProjector) << "projectFaceBoundaries:wire-done"
                                          << "wireIndex=" << wireCount
                                          << "wireCurveCount=" << wireCurveCount
                                          << "wireClosedByCircle=" << wireClosedByCircle
                                          << "firstPointId=" << QString::fromStdString(firstPointId)
                                          << "lastPointId=" << QString::fromStdString(lastPointId);
    }

    if (options.lockInsertedBoundaryPoints) {
        for (const auto& pointId : pointsToFix) {
            if (!sketch.hasFixedConstraint(pointId)) {
                const ConstraintID fixedId = sketch.addFixed(pointId);
                if (!fixedId.empty()) {
                    result.insertedFixedConstraints++;
                }
            }
        }
    }

    result.success = result.hasClosedBoundary || result.insertedCurves > 0;
    if (!result.success && result.errorMessage.empty()) {
        result.errorMessage = "No host face boundary geometry was projected";
        qCWarning(logFaceBoundaryProjector) << "projectFaceBoundaries:no-geometry-projected"
                                            << "entityCount=" << sketch.getAllEntities().size();
    }

    qCInfo(logFaceBoundaryProjector) << "projectFaceBoundaries:done"
                                     << "entityCount=" << sketch.getAllEntities().size()
                                     << "wireCount=" << wireCount
                                     << "success=" << result.success
                                     << "hasClosedBoundary=" << result.hasClosedBoundary
                                     << "insertedPoints=" << result.insertedPoints
                                     << "insertedCurves=" << result.insertedCurves
                                     << "insertedFixedConstraints=" << result.insertedFixedConstraints;
    return result;
}

} // namespace onecad::core::sketch
