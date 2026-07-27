#include "LineTool.h"
#include "../Sketch.h"
#include "../SketchRenderer.h"
#include "../AutoConstrainer.h"
#include "../IntersectionManager.h"
#include "../SketchLine.h"
#include "../SketchPoint.h"

#include <QLoggingCategory>
#include <QString>

#include <cstdio>
#include <string>
#include <cmath>
#include <numbers>

namespace onecad::core::sketch::tools {

Q_LOGGING_CATEGORY(logLineTool, "onecad.core.sketchtool.line")

LineTool::LineTool() = default;

std::string LineTool::name() const {
    return "Line";
}

std::optional<Vec2d> LineTool::getReferencePoint() const {
    if (state_ == State::FirstClick) {
        return startPoint_;
    }
    return std::nullopt;
}

std::vector<Vec2d> LineTool::getCommittedAnchorPoints() const {
    if (state_ == State::FirstClick) {
        return {startPoint_};
    }
    return {};
}

void LineTool::onMousePress(const Vec2d& pos, Qt::MouseButton button) {
    qCDebug(logLineTool) << "onMousePress" << "state=" << static_cast<int>(state_)
                         << "pos=" << pos.x << pos.y << "button=" << static_cast<int>(button)
                         << "snapType=" << static_cast<int>(snapResult_.type)
                         << "snapped=" << snapResult_.snapped;
    if (button == Qt::RightButton) {
        // Right-click finishes polyline
        cancel();
        return;
    }

    if (button != Qt::LeftButton) {
        return;
    }

    lineCreated_ = false;
    lastRejectReason_ = RejectReason::None;

    if (state_ == State::Idle) {
        // First click - record start point
        startPoint_ = pos;
        currentPoint_ = pos;
        hasLengthLock_ = false;
        lockedLength_ = 0.0;
        hasAngleLock_ = false;
        lockedAngleDeg_ = 0.0;
        startPointId_.clear();
        if (snapResult_.snapped && !snapResult_.pointId.empty()) {
            startPointId_ = snapResult_.pointId;
        }
        state_ = State::FirstClick;
    } else if (state_ == State::FirstClick) {
        // Second click - create line
        if (!sketch_) {
            return;
        }

        const bool useDraftLockedEndpoint = hasLengthLock_ || hasAngleLock_;
        Vec2d endPoint = useDraftLockedEndpoint ? currentPoint_ : pos;

        // Check minimum length to avoid degenerate geometry
        double dx = endPoint.x - startPoint_.x;
        double dy = endPoint.y - startPoint_.y;
        double length = std::sqrt(dx * dx + dy * dy);

        if (length < constants::MIN_GEOMETRY_SIZE) {
            qCDebug(logLineTool) << "reject:too-short" << "length=" << length;
            lastRejectReason_ = RejectReason::TooShort;
            return;
        }

        EntityID startId = startPointId_;
        if (startId.empty()) {
            startId = sketch_->addPoint(startPoint_.x, startPoint_.y);
        }

        EntityID endId;
        if (!useDraftLockedEndpoint && snapResult_.snapped && !snapResult_.pointId.empty()) {
            endId = snapResult_.pointId;
        } else {
            endId = sketch_->addPoint(endPoint.x, endPoint.y);
        }

        if (startId.empty() || endId.empty()) {
            qCWarning(logLineTool) << "reject:invalid-endpoints"
                                  << "startId=" << QString::fromStdString(startId)
                                  << "endId=" << QString::fromStdString(endId);
            lastRejectReason_ = RejectReason::InvalidEndpoints;
            return;
        }

        // Guard against guide-driven endpoint collapse: if a guide snap reused
        // the start point ID but the geometric end position is distinct,
        // materialize a new endpoint at click position.
        if (startId == endId) {
            if (snapResult_.snapped && snapResult_.hasGuide) {
                endId = sketch_->addPoint(endPoint.x, endPoint.y);
            }
            if (endId.empty() || startId == endId) {
                qCDebug(logLineTool) << "reject:same-endpoint-after-guide";
                lastRejectReason_ = RejectReason::SameEndpoint;
                return;
            }
        }

        if (startId.empty() || endId.empty()) {
            lastRejectReason_ = RejectReason::InvalidEndpoints;
            return;
        }

        EntityID lineId = sketch_->addLine(startId, endId);

        qCDebug(logLineTool) << "line-create-attempt"
                             << "startId=" << QString::fromStdString(startId)
                             << "endId=" << QString::fromStdString(endId)
                             << "lineId=" << QString::fromStdString(lineId);

        if (!lineId.empty()) {
            lineCreated_ = true;

            // Process intersections - split existing entities at intersection points
            if (snapManager_) {
                IntersectionManager intersectionMgr;
                intersectionMgr.processIntersections(lineId, *sketch_, *snapManager_);
            }

            // Get the end point for polyline continuation
            // The line stores startPointId and endPointId
            auto* line = sketch_->getEntityAs<SketchLine>(lineId);
            if (line) {
                lastPointId_ = line->endPointId();
                startPointId_ = line->endPointId();

                // Apply inferred constraints
                if (autoConstrainer_ && autoConstrainer_->isEnabled()) {
                    DrawingContext context;
                    context.activeEntity = lineId;
                    context.previousEntity = lastCreatedLineId_;
                    context.startPoint = startPoint_;
                    context.currentPoint = endPoint;
                    context.isPolylineMode = !lastCreatedLineId_.empty();

                    auto constraints = autoConstrainer_->inferLineConstraints(
                        startPoint_, endPoint, lineId, *sketch_, context);

                    // Filter and apply high-confidence constraints
                    auto toApply = autoConstrainer_->filterForAutoApply(constraints);
                    qCDebug(logLineTool) << "auto-constraints"
                                         << "inferred=" << constraints.size()
                                         << "toApply=" << toApply.size()
                                         << "lineId=" << QString::fromStdString(lineId);
                    applyInferredConstraints(toApply, lineId);
                }

                lastCreatedLineId_ = lineId;
            }

            if (length > constants::MIN_GEOMETRY_SIZE) {
                fallbackDirection_ = {dx / length, dy / length};
            }

            // Continue polyline: new start = old end
            startPoint_ = endPoint;
            currentPoint_ = endPoint;
            hasLengthLock_ = false;
            lockedLength_ = 0.0;
            hasAngleLock_ = false;
            lockedAngleDeg_ = 0.0;
            // Stay in FirstClick state for polyline mode
            lastRejectReason_ = RejectReason::None;
        }
    }
}

void LineTool::onMouseMove(const Vec2d& pos) {
    if (state_ == State::FirstClick) {
        updateCurrentPointFromDraftLocks(pos);
    } else {
        currentPoint_ = pos;
    }

    // Update preview constraints
    updateInferredConstraints();
}

void LineTool::updateInferredConstraints() {
    qCDebug(logLineTool) << "updateInferredConstraints"
                         << "state=" << static_cast<int>(state_)
                         << "hasAutoConstrainer=" << (autoConstrainer_ != nullptr)
                         << "hasSketch=" << (sketch_ != nullptr);
    inferredConstraints_.clear();

    if (state_ != State::FirstClick || !autoConstrainer_ || !sketch_) {
        return;
    }

    DrawingContext context;
    context.activeEntity = {};  // Line not created yet
    context.previousEntity = lastCreatedLineId_;
    context.startPoint = startPoint_;
    context.currentPoint = currentPoint_;
    context.isPolylineMode = !lastCreatedLineId_.empty();

    // Use empty lineId since line doesn't exist yet
    inferredConstraints_ = autoConstrainer_->inferLineConstraints(
        startPoint_, currentPoint_, {}, *sketch_, context);
    qCDebug(logLineTool) << "updateInferredConstraints:done"
                         << "count=" << inferredConstraints_.size();
}

void LineTool::applyInferredConstraints(const std::vector<InferredConstraint>& constraints,
                                         EntityID lineId) {
    qCDebug(logLineTool) << "applyInferredConstraints"
                         << "lineId=" << QString::fromStdString(lineId)
                         << "count=" << constraints.size();
    if (!sketch_ || lineId.empty()) return;

    auto* line = sketch_->getEntityAs<SketchLine>(lineId);
    if (!line) return;

    for (const auto& constraint : constraints) {
        qCDebug(logLineTool) << "applyInferredConstraints:item"
                             << "type=" << static_cast<int>(constraint.type)
                             << "entity1=" << QString::fromStdString(constraint.entity1)
                             << "entity2=" << QString::fromStdString(constraint.entity2.value_or(std::string{}))
                             << "confidence=" << constraint.confidence;
        switch (constraint.type) {
            case ConstraintType::Horizontal:
                sketch_->addHorizontal(lineId);
                break;

            case ConstraintType::Vertical:
                sketch_->addVertical(lineId);
                break;

            case ConstraintType::Perpendicular:
                if (constraint.entity2) {
                    sketch_->addPerpendicular(lineId, *constraint.entity2);
                }
                break;

            case ConstraintType::Parallel:
                if (constraint.entity2) {
                    sketch_->addParallel(lineId, *constraint.entity2);
                }
                break;

            case ConstraintType::Coincident:
                if (!constraint.entity1.empty()) {
                    const auto* existing = sketch_->getEntityAs<SketchPoint>(constraint.entity1);
                    const auto* startPt = sketch_->getEntityAs<SketchPoint>(line->startPointId());
                    const auto* endPt = sketch_->getEntityAs<SketchPoint>(line->endPointId());
                    if (!existing || !startPt || !endPt) {
                        break;
                    }

                    if (constraint.entity1 == line->startPointId() ||
                        constraint.entity1 == line->endPointId()) {
                        break;
                    }

                    Vec2d existingPos{existing->position().X(), existing->position().Y()};
                    Vec2d startPos{startPt->position().X(), startPt->position().Y()};
                    Vec2d endPos{endPt->position().X(), endPt->position().Y()};
                    double tolerance = autoConstrainer_
                        ? autoConstrainer_->getConfig().coincidenceTolerance
                        : constants::SNAP_RADIUS_MM;

                    double startDist = std::hypot(existingPos.x - startPos.x, existingPos.y - startPos.y);
                    double endDist = std::hypot(existingPos.x - endPos.x, existingPos.y - endPos.y);

                    if (startDist <= tolerance) {
                        sketch_->addCoincident(line->startPointId(), constraint.entity1);
                    } else if (endDist <= tolerance) {
                        sketch_->addCoincident(line->endPointId(), constraint.entity1);
                    }
                }
                break;

            default:
                // Other constraint types not applicable to lines
                break;
        }
    }
}

void LineTool::onMouseRelease(const Vec2d& /*pos*/, Qt::MouseButton /*button*/) {
    // Line tool uses click-click, not drag, so nothing on release
}

void LineTool::onKeyPress(Qt::Key key) {
    if (key == Qt::Key_Escape) {
        cancel();
    }
}

PreviewDimensionApplyResult LineTool::applyPreviewDimensionValue(const std::string& id, double value) {
    if (state_ != State::FirstClick) {
        return {false, "Set the line start point first"};
    }
    if (!std::isfinite(value)) {
        return {false, "Value must be finite"};
    }

    if (id == "line_length") {
        if (value <= constants::MIN_GEOMETRY_SIZE) {
            return {false, "Length must be greater than minimum geometry size"};
        }
        hasLengthLock_ = true;
        lockedLength_ = value;
        updateCurrentPointFromDraftLocks(currentPoint_);
        updateInferredConstraints();
        return {true, {}};
    }

    if (id == "line_angle") {
        hasAngleLock_ = true;
        lockedAngleDeg_ = normalizeAngleDegrees(value);
        updateCurrentPointFromDraftLocks(currentPoint_);
        updateInferredConstraints();
        return {true, {}};
    }

    return {false, "Unknown line draft parameter"};
}

void LineTool::cancel() {
    state_ = State::Idle;
    startPointId_.clear();
    lastPointId_.clear();
    lastCreatedLineId_.clear();
    hasLengthLock_ = false;
    lockedLength_ = 0.0;
    hasAngleLock_ = false;
    lockedAngleDeg_ = 0.0;
    fallbackDirection_ = {1.0, 0.0};
    lineCreated_ = false;
    lastRejectReason_ = RejectReason::None;
    inferredConstraints_.clear();
}

void LineTool::render(SketchRenderer& renderer) {
    if (state_ == State::FirstClick) {
        // Show preview line from start to current mouse position
        renderer.setPreviewLine(startPoint_, currentPoint_);

        // Calculate length for dimension
        double dx = currentPoint_.x - startPoint_.x;
        double dy = currentPoint_.y - startPoint_.y;
        double length = std::sqrt(dx * dx + dy * dy);

        if (length > constants::MIN_GEOMETRY_SIZE) {
            std::vector<SketchRenderer::PreviewDimension> dims;
            dims.reserve(2);

            char buffer[32];
            std::snprintf(buffer, sizeof(buffer), "%.2f", length);
            Vec2d midPoint = {
                (startPoint_.x + currentPoint_.x) * 0.5,
                (startPoint_.y + currentPoint_.y) * 0.5
            };
            dims.push_back({midPoint, std::string(buffer), "line_length", length, "mm"});

            const double angleDeg =
                normalizeAngleDegrees(std::atan2(dy, dx) * 180.0 / std::numbers::pi);
            char angleBuffer[32];
            std::snprintf(angleBuffer, sizeof(angleBuffer), "%.1f\xC2\xB0", angleDeg);

            Vec2d perp{-dy / length, dx / length};
            Vec2d anglePos = {
                startPoint_.x + dx * 0.35 + perp.x * 2.0,
                startPoint_.y + dy * 0.35 + perp.y * 2.0
            };
            dims.push_back({anglePos, std::string(angleBuffer), "line_angle", angleDeg, "\xC2\xB0"});

            renderer.setPreviewDimensions(dims);
        } else {
            renderer.clearPreviewDimensions();
        }
    } else {
        renderer.clearPreview();
    }
}

void LineTool::updateCurrentPointFromDraftLocks(const Vec2d& cursorPos) {
    if (state_ != State::FirstClick) {
        return;
    }

    if (!hasLengthLock_ && !hasAngleLock_) {
        currentPoint_ = cursorPos;
        return;
    }

    Vec2d direction = fallbackDirection_;
    const double dx = cursorPos.x - startPoint_.x;
    const double dy = cursorPos.y - startPoint_.y;
    const double rawLength = std::sqrt(dx * dx + dy * dy);

    if (rawLength > 1e-9) {
        direction = {dx / rawLength, dy / rawLength};
    }

    if (hasAngleLock_) {
        const double radians = lockedAngleDeg_ * std::numbers::pi / 180.0;
        direction = {std::cos(radians), std::sin(radians)};
    }

    const double directionLength = std::sqrt(direction.x * direction.x + direction.y * direction.y);
    if (directionLength > 1e-9) {
        direction.x /= directionLength;
        direction.y /= directionLength;
        fallbackDirection_ = direction;
    } else {
        direction = {1.0, 0.0};
        fallbackDirection_ = direction;
    }

    double length = rawLength;
    if (hasLengthLock_) {
        length = lockedLength_;
    }

    currentPoint_.x = startPoint_.x + direction.x * length;
    currentPoint_.y = startPoint_.y + direction.y * length;
}

double LineTool::normalizeAngleDegrees(double angleDegrees) {
    if (!std::isfinite(angleDegrees)) {
        return 0.0;
    }
    double normalized = std::fmod(angleDegrees, 360.0);
    if (normalized <= -180.0) {
        normalized += 360.0;
    } else if (normalized > 180.0) {
        normalized -= 360.0;
    }
    return normalized;
}

} // namespace onecad::core::sketch::tools
