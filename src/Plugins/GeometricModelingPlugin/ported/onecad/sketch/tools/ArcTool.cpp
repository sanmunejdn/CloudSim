#include "ArcTool.h"
#include "../Sketch.h"
#include "../SketchRenderer.h"
#include "../IntersectionManager.h"
#include "../SketchArc.h"
#include "../SketchPoint.h"
#include "../SketchLine.h"
#include "../constraints/Constraints.h"

#include <cmath>
#include <cstdio>
#include <numbers>

namespace onecad::core::sketch::tools {

namespace {
constexpr double PI = std::numbers::pi;
constexpr double TWO_PI = 2.0 * std::numbers::pi;

// Normalize angle to [0, 2π)
double normalizeAngle(double angle) {
    angle = std::fmod(angle, TWO_PI);
    if (angle < 0) {
        angle += TWO_PI;
    }
    return angle;
}

// Calculate angle from center to point
double angleToPoint(const Vec2d& center, const Vec2d& point) {
    return std::atan2(point.y - center.y, point.x - center.x);
}

} // anonymous namespace

ArcTool::ArcTool() = default;

std::string ArcTool::name() const {
    return "Arc";
}

std::optional<Vec2d> ArcTool::getReferencePoint() const {
    if (state_ == State::FirstClick) {
        return startPoint_;
    } else if (state_ == State::Drawing && previewValid_) {
        return previewCenter_;
    }
    return std::nullopt;
}

std::vector<Vec2d> ArcTool::getCommittedAnchorPoints() const {
    if (state_ == State::FirstClick) {
        return {startPoint_};
    }
    if (state_ == State::Drawing) {
        return {startPoint_, middlePoint_};
    }
    return {};
}

void ArcTool::onMousePress(const Vec2d& pos, Qt::MouseButton button) {
    if (button == Qt::RightButton) {
        cancel();
        return;
    }

    if (button != Qt::LeftButton) {
        return;
    }

    arcCreated_ = false;

    if (state_ == State::Idle) {
        // First click - start point
        startPoint_ = pos;
        currentPoint_ = pos;
        startPointId_.clear();
        if (snapResult_.snapped && !snapResult_.pointId.empty()) {
            startPointId_ = snapResult_.pointId;
        }
        state_ = State::FirstClick;
        previewValid_ = false;
    } else if (state_ == State::FirstClick) {
        // Second click - middle point (point on arc)
        double dx = pos.x - startPoint_.x;
        double dy = pos.y - startPoint_.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < constants::MIN_GEOMETRY_SIZE) {
            return; // Too close to start
        }

        middlePoint_ = pos;
        middlePointId_.clear();
        if (snapResult_.snapped && !snapResult_.pointId.empty()) {
            middlePointId_ = snapResult_.pointId;
        }
        state_ = State::Drawing;
        updatePreviewArc();
    } else if (state_ == State::Drawing) {
        // Third click - end point, create arc
        if (!sketch_) {
            return;
        }

        // Calculate arc from three points
        Vec2d center;
        double radius, startAngle, endAngle;

        if (!calculateArcFromThreePoints(startPoint_, middlePoint_, currentPoint_,
                                          center, radius, startAngle, endAngle)) {
            // Points are collinear or otherwise invalid
            return;
        }

        if (radius < constants::MIN_GEOMETRY_SIZE) {
            return; // Arc too small
        }

        // Create center point
        EntityID centerPointId = sketch_->addPoint(center.x, center.y);
        if (centerPointId.empty()) {
            return;
        }

        // Create arc
        EntityID arcId = sketch_->addArc(centerPointId, radius, startAngle, endAngle);

        if (!arcId.empty()) {
            // Process intersections - split existing entities at intersection points
            if (snapManager_) {
                IntersectionManager intersectionMgr;
                intersectionMgr.processIntersections(arcId, *sketch_, *snapManager_);
            }

            // Apply point-on-curve constraint for start point if snapped
            if (!startPointId_.empty()) {
                sketch_->addPointOnCurve(startPointId_, arcId, CurvePosition::Start);
            }

            applyInferredConstraints(arcId, centerPointId);

            // Reset for next arc
            cancel();
            // Set flag after cancel() so it persists for external consumers
            arcCreated_ = true;
        }
    }
}

void ArcTool::onMouseMove(const Vec2d& pos) {
    currentPoint_ = pos;

    if (state_ == State::Drawing) {
        updatePreviewArc();
    }
}

void ArcTool::onMouseRelease(const Vec2d& /*pos*/, Qt::MouseButton /*button*/) {
    // Arc tool uses click-click-click, nothing on release
}

void ArcTool::onKeyPress(Qt::Key key) {
    if (key == Qt::Key_Escape) {
        cancel();
    }
}

void ArcTool::cancel() {
    state_ = State::Idle;
    startPointId_.clear();
    middlePointId_.clear();
    previewValid_ = false;
    arcCreated_ = false;
    inferredConstraints_.clear();
}

void ArcTool::render(SketchRenderer& renderer) {
    if (state_ == State::FirstClick) {
        // Show line from start to current (guides user to middle point)
        renderer.setPreviewLine(startPoint_, currentPoint_);
        renderer.clearPreviewDimensions();
    } else if (state_ == State::Drawing && previewValid_) {
        // Show preview arc
        renderer.setPreviewArc(previewCenter_, previewRadius_,
                               previewStartAngle_, previewEndAngle_);

        // Show radius dimension
        char buffer[32];
        std::snprintf(buffer, sizeof(buffer), "R%.2f", previewRadius_);

        Vec2d dimPos = {
            previewCenter_.x + previewRadius_ * 0.7,
            previewCenter_.y + previewRadius_ * 0.7
        };
        renderer.setPreviewDimensions({{dimPos, std::string(buffer)}});
    } else {
        renderer.clearPreview();
        renderer.clearPreviewDimensions();
    }
}

bool ArcTool::calculateArcFromThreePoints(const Vec2d& p1, const Vec2d& p2, const Vec2d& p3,
                                           Vec2d& center, double& radius,
                                           double& startAngle, double& endAngle) const {
    // Calculate circle through 3 points using perpendicular bisector method
    // https://en.wikipedia.org/wiki/Circumscribed_circle#Cartesian_coordinates

    double ax = p1.x, ay = p1.y;
    double bx = p2.x, by = p2.y;
    double cx = p3.x, cy = p3.y;

    double d = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));

    if (std::abs(d) < 1e-10) {
        // Points are collinear
        return false;
    }

    double ax2 = ax * ax;
    double ay2 = ay * ay;
    double bx2 = bx * bx;
    double by2 = by * by;
    double cx2 = cx * cx;
    double cy2 = cy * cy;

    double ux = ((ax2 + ay2) * (by - cy) + (bx2 + by2) * (cy - ay) + (cx2 + cy2) * (ay - by)) / d;
    double uy = ((ax2 + ay2) * (cx - bx) + (bx2 + by2) * (ax - cx) + (cx2 + cy2) * (bx - ax)) / d;

    center = {ux, uy};
    radius = std::sqrt((ax - ux) * (ax - ux) + (ay - uy) * (ay - uy));

    // Calculate angles
    double angle1 = angleToPoint(center, p1);
    double angle2 = angleToPoint(center, p2);
    double angle3 = angleToPoint(center, p3);

    // Determine arc direction: we want p2 to be between p1 and p3
    // Check if going CCW from p1 through p2 to p3 works
    double a1 = normalizeAngle(angle1);
    double a2 = normalizeAngle(angle2);
    double a3 = normalizeAngle(angle3);

    // Check if p2 is between p1 and p3 going CCW
    // Assumes inputs are already normalized to [0, 2π)
    auto isBetweenCCW = [](double start, double mid, double end) {
        if (start <= end) {
            return mid >= start && mid <= end;
        } else {
            return mid >= start || mid <= end;
        }
    };

    if (isBetweenCCW(a1, a2, a3)) {
        // CCW from p1 to p3 through p2
        startAngle = angle1;
        endAngle = angle3;
    } else {
        // CW direction - swap to make CCW
        startAngle = angle3;
        endAngle = angle1;
    }

    return true;
}

void ArcTool::updatePreviewArc() {
    previewValid_ = calculateArcFromThreePoints(startPoint_, middlePoint_, currentPoint_,
                                                 previewCenter_, previewRadius_,
                                                 previewStartAngle_, previewEndAngle_);
}

void ArcTool::applyInferredConstraints(EntityID arcId, EntityID centerPointId) {
    if (!autoConstrainer_ || !autoConstrainer_->isEnabled()) {
        return;
    }

    // Get arc geometry
    auto* arc = sketch_->getEntityAs<SketchArc>(arcId);
    auto* centerPt = sketch_->getEntityAs<SketchPoint>(centerPointId);
    if (!arc || !centerPt) {
        return;
    }

    Vec2d center = {centerPt->position().X(), centerPt->position().Y()};

    // Build drawing context
    DrawingContext context;
    context.startPoint = startPoint_;
    context.currentPoint = currentPoint_;
    context.activeEntity = arcId;

    // Infer constraints
    auto constraints = autoConstrainer_->inferArcConstraints(
        center, arc->radius(), arc->startAngle(), arc->endAngle(),
        arcId, *sketch_, context);

    // Filter and apply
    auto toApply = autoConstrainer_->filterForAutoApply(constraints);
    for (const auto& c : toApply) {
        if (c.type == ConstraintType::Tangent && c.entity2) {
            sketch_->addConstraint(
                std::make_unique<constraints::TangentConstraint>(*c.entity2, arcId));
        }
    }
}

} // namespace onecad::core::sketch::tools
