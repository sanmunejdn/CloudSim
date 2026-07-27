#include "ConstraintSolver.h"
#include "../constraints/Constraints.h"
#include "../SketchPoint.h"
#include "../SketchLine.h"
#include "../SketchArc.h"
#include "../SketchCircle.h"
#include "../SketchConstraint.h"

#include <GCS.h>

#include <QLoggingCategory>
#include <QString>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace onecad::core::sketch {

Q_LOGGING_CATEGORY(logConstraintSolver, "onecad.core.constraintsolver")

namespace {

constexpr double kDragSubstepThreshold = 100.0;
constexpr int kMaxDragSubsteps = 10;

int computeDragSubsteps(double maxDeltaMagnitude) {
    if (maxDeltaMagnitude <= kDragSubstepThreshold) {
        return 1;
    }
    const double rawSteps = std::ceil(maxDeltaMagnitude / kDragSubstepThreshold);
    return std::min(kMaxDragSubsteps, std::max(1, static_cast<int>(rawSteps)));
}

double* coordPtr(SketchPoint* point, int coordIndex) {
    gp_XY& coords = point->position().ChangeCoord();
    return &coords.ChangeCoord(coordIndex);
}

GCS::Point makePoint(SketchPoint* point) {
    return GCS::Point(coordPtr(point, 1), coordPtr(point, 2));
}

bool lineEndpoints(const std::unordered_map<EntityID, SketchPoint*>& pointsById,
                   const SketchLine* line,
                   SketchPoint*& start,
                   SketchPoint*& end) {
    if (!line) {
        return false;
    }

    auto startIt = pointsById.find(line->startPointId());
    auto endIt = pointsById.find(line->endPointId());
    if (startIt == pointsById.end() || endIt == pointsById.end()) {
        return false;
    }

    start = startIt->second;
    end = endIt->second;
    return start && end;
}

bool circleCenter(const std::unordered_map<EntityID, SketchPoint*>& pointsById,
                  const SketchCircle* circle,
                  SketchPoint*& center) {
    if (!circle) {
        return false;
    }

    auto centerIt = pointsById.find(circle->centerPointId());
    if (centerIt == pointsById.end()) {
        return false;
    }

    center = centerIt->second;
    return center != nullptr;
}

bool arcCenter(const std::unordered_map<EntityID, SketchPoint*>& pointsById,
               const SketchArc* arc,
               SketchPoint*& center) {
    if (!arc) {
        return false;
    }

    auto centerIt = pointsById.find(arc->centerPointId());
    if (centerIt == pointsById.end()) {
        return false;
    }

    center = centerIt->second;
    return center != nullptr;
}

GCS::Line makeLine(SketchPoint* start, SketchPoint* end) {
    GCS::Line line;
    line.p1 = makePoint(start);
    line.p2 = makePoint(end);
    return line;
}

GCS::Circle makeCircle(SketchPoint* center, SketchCircle* circle) {
    GCS::Circle gcsCircle;
    gcsCircle.center = makePoint(center);
    gcsCircle.rad = &circle->radius();
    return gcsCircle;
}

GCS::Arc makeArc(SketchPoint* center, SketchArc* arc) {
    GCS::Arc gcsArc;
    gcsArc.center = makePoint(center);
    gcsArc.rad = &arc->radius();
    gcsArc.startAngle = &arc->startAngle();
    gcsArc.endAngle = &arc->endAngle();
    return gcsArc;
}

GCS::Algorithm toGcsAlgorithm(SolverConfig::Algorithm algorithm) {
    switch (algorithm) {
        case SolverConfig::Algorithm::LevenbergMarquardt:
            return GCS::LevenbergMarquardt;
        case SolverConfig::Algorithm::DogLeg:
            return GCS::DogLeg;
        case SolverConfig::Algorithm::BFGS:
            return GCS::BFGS;
    }
    return GCS::DogLeg;
}

SolverResult::Status toSolverStatus(int gcsStatus) {
    switch (gcsStatus) {
        case GCS::Success:
            return SolverResult::Status::Success;
        case GCS::Converged:
            return SolverResult::Status::PartialSuccess;
        case GCS::Failed:
            return SolverResult::Status::Diverged;
        case GCS::SuccessfulSolutionInvalid:
            return SolverResult::Status::InvalidInput;
        default:
            return SolverResult::Status::InternalError;
    }
}

} // namespace

ConstraintSolver::ConstraintSolver()
    : config_(),
      gcsSystem_(std::make_unique<GCS::System>()) {
    configureSystem();
}

ConstraintSolver::ConstraintSolver(const SolverConfig& config)
    : config_(config),
      gcsSystem_(std::make_unique<GCS::System>()) {
    configureSystem();
}

ConstraintSolver::~ConstraintSolver() = default;

void ConstraintSolver::setConfig(const SolverConfig& config) {
    config_ = config;
    configureSystem();
}

void ConstraintSolver::clear() {
    qCDebug(logConstraintSolver) << "clear"
                                 << "entities(points,lines,arcs,circles)="
                                 << pointsById_.size() << linesById_.size() << arcsById_.size() << circlesById_.size()
                                 << "constraints=" << constraints_.size();
    entityToGcsId_.clear();
    constraintToGcsTag_.clear();
    gcsTagToConstraint_.clear();
    pointsById_.clear();
    linesById_.clear();
    arcsById_.clear();
    circlesById_.clear();
    constraints_.clear();
    parameterBackup_.clear();
    parameters_.clear();
    drivenParameters_.clear();
    nextEntityTag_ = 1;
    nextConstraintTag_ = 1;

    if (!gcsSystem_) {
        gcsSystem_ = std::make_unique<GCS::System>();
    }
    gcsSystem_->clear();
    configureSystem();
}

void ConstraintSolver::addPoint(SketchPoint* point) {
    if (!point) {
        return;
    }
    if (pointsById_.find(point->id()) != pointsById_.end()) {
        return;
    }

    pointsById_[point->id()] = point;
    entityToGcsId_[point->id()] = nextEntityTag_++;
    parameters_.push_back(coordPtr(point, 1));
    parameters_.push_back(coordPtr(point, 2));
}

void ConstraintSolver::addLine(SketchLine* line) {
    if (!line) {
        return;
    }
    if (linesById_.find(line->id()) != linesById_.end()) {
        return;
    }
    linesById_[line->id()] = line;
    entityToGcsId_[line->id()] = nextEntityTag_++;
}

void ConstraintSolver::addArc(SketchArc* arc) {
    if (!arc) {
        return;
    }
    if (arcsById_.find(arc->id()) != arcsById_.end()) {
        return;
    }
    arcsById_[arc->id()] = arc;
    entityToGcsId_[arc->id()] = nextEntityTag_++;
    parameters_.push_back(&arc->radius());
    parameters_.push_back(&arc->startAngle());
    parameters_.push_back(&arc->endAngle());
}

void ConstraintSolver::addCircle(SketchCircle* circle) {
    if (!circle) {
        return;
    }
    if (circlesById_.find(circle->id()) != circlesById_.end()) {
        return;
    }
    circlesById_[circle->id()] = circle;
    entityToGcsId_[circle->id()] = nextEntityTag_++;
    parameters_.push_back(&circle->radius());
}

bool ConstraintSolver::addConstraint(SketchConstraint* constraint) {
    if (!constraint || !gcsSystem_) {
        qCWarning(logConstraintSolver) << "addConstraint: invalid input"
                                      << "constraintNull=" << (constraint == nullptr)
                                      << "gcsNull=" << (gcsSystem_ == nullptr);
        return false;
    }

    qCDebug(logConstraintSolver) << "addConstraint:start"
                                 << "constraintId=" << QString::fromStdString(constraint->id())
                                 << "type=" << static_cast<int>(constraint->type());

    int tagId = nextConstraintTag_;
    if (!translateConstraint(constraint, tagId)) {
        qCWarning(logConstraintSolver) << "addConstraint: translation failed"
                                      << "constraintId=" << QString::fromStdString(constraint->id())
                                      << "tagId=" << tagId;
        return false;
    }

    constraints_.push_back(constraint);
    constraintToGcsTag_[constraint->id()] = tagId;
    gcsTagToConstraint_[tagId] = constraint->id();
    nextConstraintTag_++;
    gcsSystem_->invalidatedDiagnosis();
    qCDebug(logConstraintSolver) << "addConstraint:done"
                                 << "constraintId=" << QString::fromStdString(constraint->id())
                                 << "tagId=" << tagId
                                 << "totalConstraints=" << constraints_.size();
    return true;
}

void ConstraintSolver::removeEntity(EntityID id) {
    entityToGcsId_.erase(id);
    pointsById_.erase(id);
    linesById_.erase(id);
    arcsById_.erase(id);
    circlesById_.erase(id);

    parameters_.clear();
    for (const auto& [pointId, point] : pointsById_) {
        parameters_.push_back(coordPtr(point, 1));
        parameters_.push_back(coordPtr(point, 2));
    }
    for (const auto& [arcId, arc] : arcsById_) {
        parameters_.push_back(&arc->radius());
        parameters_.push_back(&arc->startAngle());
        parameters_.push_back(&arc->endAngle());
    }
    for (const auto& [circleId, circle] : circlesById_) {
        parameters_.push_back(&circle->radius());
    }
}

void ConstraintSolver::removeConstraint(ConstraintID id) {
    auto tagIt = constraintToGcsTag_.find(id);
    if (tagIt != constraintToGcsTag_.end()) {
        if (gcsSystem_) {
            gcsSystem_->clearByTag(tagIt->second);
            gcsSystem_->invalidatedDiagnosis();
        }
        gcsTagToConstraint_.erase(tagIt->second);
        constraintToGcsTag_.erase(tagIt);
    }

    constraints_.erase(std::remove_if(constraints_.begin(), constraints_.end(),
                                      [&](const SketchConstraint* c) {
                                          return c && c->id() == id;
                                      }),
                       constraints_.end());
}

SolverResult ConstraintSolver::solve() {
    SolverResult result;
    auto start = std::chrono::steady_clock::now();

    qCDebug(logConstraintSolver) << "solve:start"
                                 << "points=" << pointsById_.size()
                                 << "lines=" << linesById_.size()
                                 << "arcs=" << arcsById_.size()
                                 << "circles=" << circlesById_.size()
                                 << "constraints=" << constraints_.size()
                                 << "parameters=" << parameters_.size()
                                 << "algorithm=" << static_cast<int>(config_.algorithm);

    if (!gcsSystem_) {
        result.success = false;
        result.status = SolverResult::Status::InternalError;
        result.errorMessage = "PlaneGCS system not available";
        qCCritical(logConstraintSolver) << "solve:missing-gcs-system";
        return result;
    }

    backupParameters();

    gcsSystem_->declareUnknowns(parameters_);
    gcsSystem_->declareDrivenParams(drivenParameters_);

    GCS::Algorithm alg = toGcsAlgorithm(config_.algorithm);
    gcsSystem_->initSolution(alg);

    int status = gcsSystem_->solve(true, alg, false);
    if (status == GCS::Failed && config_.algorithm == SolverConfig::Algorithm::DogLeg) {
        qCWarning(logConstraintSolver) << "solve:dogleg-failed-fallback-to-lm";
        status = gcsSystem_->solve(true, GCS::LevenbergMarquardt, false);
    }

    result.status = toSolverStatus(status);
    result.success = (status == GCS::Success || status == GCS::Converged);

    if (result.success) {
        gcsSystem_->applySolution();
    } else {
        gcsSystem_->undoSolution();
        restoreParameters();
    }

    std::vector<int> conflictingTags;
    gcsSystem_->getConflicting(conflictingTags);
    for (int tag : conflictingTags) {
        auto it = gcsTagToConstraint_.find(tag);
        if (it != gcsTagToConstraint_.end()) {
            result.conflictingConstraints.push_back(it->second);
        }
    }

    if (config_.detectRedundant) {
        std::vector<int> redundantTags;
        gcsSystem_->getRedundant(redundantTags);
        for (int tag : redundantTags) {
            auto it = gcsTagToConstraint_.find(tag);
            if (it != gcsTagToConstraint_.end()) {
                result.redundantConstraints.push_back(it->second);
            }
        }
        if (!result.redundantConstraints.empty() && result.success) {
            result.status = SolverResult::Status::Redundant;
        }
    }

    auto end = std::chrono::steady_clock::now();
    result.solveTime = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    totalSolves_++;
    if (result.success) {
        successfulSolves_++;
    }
    totalSolveTime_ += result.solveTime;

    if (config_.timeoutMs > 0) {
        auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(result.solveTime).count();
        if (elapsedMs > config_.timeoutMs) {
            result.status = SolverResult::Status::Timeout;
            result.success = false;
            gcsSystem_->undoSolution();
            restoreParameters();
        }
    }

    return result;
}

SolverResult ConstraintSolver::solveWithDrag(EntityID pointId, const Vec2d& targetPos,
                                             const std::unordered_set<EntityID>& pointIdsToFix) {
    qCDebug(logConstraintSolver) << "solveWithDrag:start"
                                 << "pointId=" << QString::fromStdString(pointId)
                                 << "target=" << targetPos.x << targetPos.y
                                 << "fixedPoints=" << pointIdsToFix.size();
    auto draggedIt = pointsById_.find(pointId);
    if (draggedIt == pointsById_.end() || !draggedIt->second) {
        SolverResult result;
        result.success = false;
        result.status = SolverResult::Status::InvalidInput;
        result.errorMessage = "Dragged point not found";
        return result;
    }

    if (!gcsSystem_) {
        SolverResult result;
        result.success = false;
        result.status = SolverResult::Status::InternalError;
        result.errorMessage = "PlaneGCS system not available";
        return result;
    }

    const Vec2d startPos{draggedIt->second->position().X(), draggedIt->second->position().Y()};
    const double deltaX = targetPos.x - startPos.x;
    const double deltaY = targetPos.y - startPos.y;
    const double deltaMagnitude = std::sqrt((deltaX * deltaX) + (deltaY * deltaY));
    const int substeps = computeDragSubsteps(deltaMagnitude);
    if (substeps > 1) {
        qCInfo(logConstraintSolver) << "solveWithDrag:substep-triggered"
                                    << "pointId=" << QString::fromStdString(pointId)
                                    << "delta=" << deltaMagnitude
                                    << "steps=" << substeps;
    }

    const ConstraintSolver::DragSolveSnapshot snapshot = captureDragSolveSnapshot();
    SolverResult lastResult;
    for (int step = 1; step <= substeps; ++step) {
        const double t = static_cast<double>(step) / static_cast<double>(substeps);
        const Vec2d intermediateTarget{
            .x = startPos.x + (deltaX * t),
            .y = startPos.y + (deltaY * t),
        };
        lastResult = solveWithDragSingleStep(pointId, intermediateTarget, pointIdsToFix);
        if (!lastResult.success) {
            restoreDragSolveSnapshot(snapshot);
            if (lastResult.errorMessage.empty()) {
                lastResult.errorMessage = "Drag rejected by constraints";
            }
            return lastResult;
        }
    }

    return lastResult;
}

SolverResult ConstraintSolver::solveWithGroupDrag(
    const std::unordered_map<EntityID, Vec2d>& targetPositions) {
    qCDebug(logConstraintSolver) << "solveWithGroupDrag:start"
                                 << "targetPoints=" << targetPositions.size();

    if (!gcsSystem_) {
        SolverResult result;
        result.success = false;
        result.status = SolverResult::Status::InternalError;
        result.errorMessage = "PlaneGCS system not available";
        return result;
    }

    if (targetPositions.empty()) {
        SolverResult result;
        result.success = false;
        result.status = SolverResult::Status::InvalidInput;
        result.errorMessage = "No group drag targets";
        return result;
    }

    std::unordered_map<EntityID, Vec2d> startPositions;
    startPositions.reserve(targetPositions.size());
    double maxDeltaMagnitude = 0.0;
    for (const auto& [pointId, targetPos] : targetPositions) {
        auto pointIt = pointsById_.find(pointId);
        if (pointIt == pointsById_.end() || !pointIt->second) {
            SolverResult result;
            result.success = false;
            result.status = SolverResult::Status::InvalidInput;
            result.errorMessage = "Group drag point not found";
            return result;
        }
        const Vec2d startPos{pointIt->second->position().X(), pointIt->second->position().Y()};
        startPositions[pointId] = startPos;
        const double dx = targetPos.x - startPos.x;
        const double dy = targetPos.y - startPos.y;
        maxDeltaMagnitude = std::max(maxDeltaMagnitude, std::sqrt((dx * dx) + (dy * dy)));
    }

    const int substeps = computeDragSubsteps(maxDeltaMagnitude);
    if (substeps > 1) {
        qCInfo(logConstraintSolver) << "solveWithGroupDrag:substep-triggered"
                                    << "targetPoints=" << targetPositions.size()
                                    << "maxDelta=" << maxDeltaMagnitude
                                    << "steps=" << substeps;
    }

    const ConstraintSolver::DragSolveSnapshot snapshot = captureDragSolveSnapshot();
    SolverResult lastResult;
    std::unordered_map<EntityID, Vec2d> intermediateTargets;
    intermediateTargets.reserve(targetPositions.size());
    for (int step = 1; step <= substeps; ++step) {
        intermediateTargets.clear();
        const double t = static_cast<double>(step) / static_cast<double>(substeps);
        for (const auto& [pointId, targetPos] : targetPositions) {
            const Vec2d& startPos = startPositions.at(pointId);
            intermediateTargets[pointId] = Vec2d{
                .x = startPos.x + ((targetPos.x - startPos.x) * t),
                .y = startPos.y + ((targetPos.y - startPos.y) * t),
            };
        }

        lastResult = solveWithGroupDragSingleStep(intermediateTargets);
        if (!lastResult.success) {
            restoreDragSolveSnapshot(snapshot);
            if (lastResult.errorMessage.empty()) {
                lastResult.errorMessage = "Group drag rejected by constraints";
            }
            return lastResult;
        }
    }

    return lastResult;
}

ConstraintSolver::DragSolveSnapshot ConstraintSolver::captureDragSolveSnapshot() const {
    ConstraintSolver::DragSolveSnapshot snapshot;
    snapshot.pointPositions.reserve(pointsById_.size());
    for (const auto& [id, point] : pointsById_) {
        if (!point) {
            continue;
        }
        snapshot.pointPositions[id] = Vec2d{.x = point->position().X(), .y = point->position().Y()};
    }

    snapshot.arcStates.reserve(arcsById_.size());
    for (const auto& [id, arc] : arcsById_) {
        if (!arc) {
            continue;
        }
        snapshot.arcStates[id] = ConstraintSolver::DragSolveSnapshot::ArcState{
            .radius = arc->radius(),
            .startAngle = arc->startAngle(),
            .endAngle = arc->endAngle(),
        };
    }

    snapshot.circleRadii.reserve(circlesById_.size());
    for (const auto& [id, circle] : circlesById_) {
        if (!circle) {
            continue;
        }
        snapshot.circleRadii[id] = circle->radius();
    }

    return snapshot;
}

void ConstraintSolver::restoreDragSolveSnapshot(const ConstraintSolver::DragSolveSnapshot& snapshot) {
    for (const auto& [id, pos] : snapshot.pointPositions) {
        auto pointIt = pointsById_.find(id);
        if (pointIt == pointsById_.end() || !pointIt->second) {
            continue;
        }
        pointIt->second->setPosition(pos.x, pos.y);
    }

    for (const auto& [id, arcState] : snapshot.arcStates) {
        auto arcIt = arcsById_.find(id);
        if (arcIt == arcsById_.end() || !arcIt->second) {
            continue;
        }
        arcIt->second->setRadius(arcState.radius);
        arcIt->second->setStartAngle(arcState.startAngle);
        arcIt->second->setEndAngle(arcState.endAngle);
    }

    for (const auto& [id, radius] : snapshot.circleRadii) {
        auto circleIt = circlesById_.find(id);
        if (circleIt == circlesById_.end() || !circleIt->second) {
            continue;
        }
        circleIt->second->setRadius(radius);
    }

    if (gcsSystem_) {
        gcsSystem_->clearByTag(-1);
        gcsSystem_->invalidatedDiagnosis();
    }
}

SolverResult ConstraintSolver::solveWithDragSingleStep(
    EntityID pointId,
    const Vec2d& targetPos,
    const std::unordered_set<EntityID>& pointIdsToFix) {
    auto it = pointsById_.find(pointId);
    if (it == pointsById_.end() || !it->second) {
        SolverResult result;
        result.success = false;
        result.status = SolverResult::Status::InvalidInput;
        result.errorMessage = "Dragged point not found";
        return result;
    }

    if (!gcsSystem_) {
        SolverResult result;
        result.success = false;
        result.status = SolverResult::Status::InternalError;
        result.errorMessage = "PlaneGCS system not available";
        return result;
    }

    constexpr int dragTag = -1;
    gcsSystem_->clearByTag(dragTag);

    struct FixedCoord {
        double x;
        double y;
    };
    std::unordered_map<EntityID, FixedCoord> fixedPositions;
    const bool fixAllOtherPoints = pointIdsToFix.empty();
    for (const auto& [id, point] : pointsById_) {
        if (id == pointId || !point) {
            continue;
        }
        if (!fixAllOtherPoints && pointIdsToFix.find(id) == pointIdsToFix.end()) {
            continue;
        }
        fixedPositions[id] = {point->position().X(), point->position().Y()};
    }
    for (auto& [id, coord] : fixedPositions) {
        auto pointIt = pointsById_.find(id);
        if (pointIt == pointsById_.end() || !pointIt->second) {
            continue;
        }
        GCS::Point gcsPoint = makePoint(pointIt->second);
        gcsSystem_->addConstraintCoordinateX(gcsPoint, &coord.x, dragTag, true);
        gcsSystem_->addConstraintCoordinateY(gcsPoint, &coord.y, dragTag, true);
    }

    double targetX = targetPos.x;
    double targetY = targetPos.y;
    GCS::Point dragPoint = makePoint(it->second);
    gcsSystem_->addConstraintCoordinateX(dragPoint, &targetX, dragTag, true);
    gcsSystem_->addConstraintCoordinateY(dragPoint, &targetY, dragTag, true);

    SolverResult result = solve();

    gcsSystem_->clearByTag(dragTag);
    gcsSystem_->invalidatedDiagnosis();

    return result;
}

SolverResult ConstraintSolver::solveWithGroupDragSingleStep(
    const std::unordered_map<EntityID, Vec2d>& targetPositions) {
    if (!gcsSystem_) {
        SolverResult result;
        result.success = false;
        result.status = SolverResult::Status::InternalError;
        result.errorMessage = "PlaneGCS system not available";
        return result;
    }

    if (targetPositions.empty()) {
        SolverResult result;
        result.success = false;
        result.status = SolverResult::Status::InvalidInput;
        result.errorMessage = "No group drag targets";
        return result;
    }

    constexpr int dragTag = -1;
    gcsSystem_->clearByTag(dragTag);

    std::vector<std::pair<GCS::Point, std::pair<double, double>>> tempTargets;
    tempTargets.reserve(targetPositions.size());
    for (const auto& [pointId, targetPos] : targetPositions) {
        auto pointIt = pointsById_.find(pointId);
        if (pointIt == pointsById_.end() || !pointIt->second) {
            SolverResult result;
            result.success = false;
            result.status = SolverResult::Status::InvalidInput;
            result.errorMessage = "Group drag point not found";
            gcsSystem_->clearByTag(dragTag);
            gcsSystem_->invalidatedDiagnosis();
            return result;
        }
        tempTargets.emplace_back(makePoint(pointIt->second),
                                 std::make_pair(targetPos.x, targetPos.y));
    }

    for (auto& [point, target] : tempTargets) {
        gcsSystem_->addConstraintCoordinateX(point, &target.first, dragTag, true);
        gcsSystem_->addConstraintCoordinateY(point, &target.second, dragTag, true);
    }

    SolverResult result = solve();
    if (!result.success && result.errorMessage.empty()) {
        result.errorMessage = "Group drag rejected by constraints";
    }

    gcsSystem_->clearByTag(dragTag);
    gcsSystem_->invalidatedDiagnosis();

    return result;
}

void ConstraintSolver::applySolution() {
    if (gcsSystem_) {
        gcsSystem_->applySolution();
    }
}

void ConstraintSolver::revertSolution() {
    if (gcsSystem_) {
        gcsSystem_->undoSolution();
    }
    restoreParameters();
}
std::vector<ConstraintID> ConstraintSolver::findRedundantConstraints() const {
    std::vector<ConstraintID> result;
    if (!gcsSystem_) {
        return result;
    }

    std::vector<int> redundantTags;
    gcsSystem_->getRedundant(redundantTags);
    for (int tag : redundantTags) {
        auto it = gcsTagToConstraint_.find(tag);
        if (it != gcsTagToConstraint_.end()) {
            result.push_back(it->second);
        }
    }
    return result;
}

bool ConstraintSolver::isSolvable() const {
    if (!gcsSystem_) {
        return false;
    }
    return !gcsSystem_->hasConflicting();
}

int ConstraintSolver::diagnose() {
    if (!gcsSystem_) {
        return -1;
    }
    gcsSystem_->declareUnknowns(parameters_);
    gcsSystem_->declareDrivenParams(drivenParameters_);
    return gcsSystem_->diagnose(toGcsAlgorithm(config_.algorithm));
}

bool ConstraintSolver::hasConflicting() const {
    return gcsSystem_ && gcsSystem_->hasConflicting();
}

bool ConstraintSolver::hasRedundant() const {
    return gcsSystem_ && gcsSystem_->hasRedundant();
}
void ConstraintSolver::backupParameters() {
    parameterBackup_.clear();

    for (const auto& [id, point] : pointsById_) {
        if (!point) {
            continue;
        }
        ParameterBackup backup;
        backup.entityId = id;
        backup.type = EntityType::Point;
        backup.values = {point->x(), point->y()};
        parameterBackup_.push_back(std::move(backup));
    }

    for (const auto& [id, arc] : arcsById_) {
        if (!arc) {
            continue;
        }
        ParameterBackup backup;
        backup.entityId = id;
        backup.type = EntityType::Arc;
        backup.values = {arc->radius(), arc->startAngle(), arc->endAngle()};
        parameterBackup_.push_back(std::move(backup));
    }

    for (const auto& [id, circle] : circlesById_) {
        if (!circle) {
            continue;
        }
        ParameterBackup backup;
        backup.entityId = id;
        backup.type = EntityType::Circle;
        backup.values = {circle->radius()};
        parameterBackup_.push_back(std::move(backup));
    }
}

void ConstraintSolver::restoreParameters() {
    for (const auto& backup : parameterBackup_) {
        switch (backup.type) {
            case EntityType::Point: {
                auto it = pointsById_.find(backup.entityId);
                if (it != pointsById_.end() && it->second && backup.values.size() >= 2) {
                    it->second->setPosition(backup.values[0], backup.values[1]);
                }
                break;
            }
            case EntityType::Arc: {
                auto it = arcsById_.find(backup.entityId);
                if (it != arcsById_.end() && it->second && backup.values.size() >= 3) {
                    it->second->setRadius(backup.values[0]);
                    it->second->setStartAngle(backup.values[1]);
                    it->second->setEndAngle(backup.values[2]);
                }
                break;
            }
            case EntityType::Circle: {
                auto it = circlesById_.find(backup.entityId);
                if (it != circlesById_.end() && it->second && !backup.values.empty()) {
                    it->second->setRadius(backup.values[0]);
                }
                break;
            }
            default:
                break;
        }
    }
}

bool ConstraintSolver::translateConstraint(SketchConstraint* constraint, int tagId) {
    if (!constraint || !gcsSystem_) {
        return false;
    }

    using namespace onecad::core::sketch::constraints;

    auto getPoint = [&](const EntityID& id) -> SketchPoint* {
        auto it = pointsById_.find(id);
        return it != pointsById_.end() ? it->second : nullptr;
    };
    auto getLine = [&](const EntityID& id) -> SketchLine* {
        auto it = linesById_.find(id);
        return it != linesById_.end() ? it->second : nullptr;
    };
    auto getCircle = [&](const EntityID& id) -> SketchCircle* {
        auto it = circlesById_.find(id);
        return it != circlesById_.end() ? it->second : nullptr;
    };
    auto getArc = [&](const EntityID& id) -> SketchArc* {
        auto it = arcsById_.find(id);
        return it != arcsById_.end() ? it->second : nullptr;
    };

    if (auto* coincident = dynamic_cast<CoincidentConstraint*>(constraint)) {
        auto* p1 = getPoint(coincident->point1());
        auto* p2 = getPoint(coincident->point2());
        if (!p1 || !p2) {
            return false;
        }
        auto gp1 = makePoint(p1);
        auto gp2 = makePoint(p2);
        gcsSystem_->addConstraintP2PCoincident(gp1, gp2, tagId, true);
        return true;
    }

    if (auto* horizontal = dynamic_cast<HorizontalConstraint*>(constraint)) {
        auto* line = getLine(horizontal->lineId());
        if (!line) {
            return false;
        }
        SketchPoint* start = nullptr;
        SketchPoint* end = nullptr;
        if (!lineEndpoints(pointsById_, line, start, end)) {
            return false;
        }
        auto gp1 = makePoint(start);
        auto gp2 = makePoint(end);
        gcsSystem_->addConstraintHorizontal(gp1, gp2, tagId, true);
        return true;
    }

    if (auto* vertical = dynamic_cast<VerticalConstraint*>(constraint)) {
        auto* line = getLine(vertical->lineId());
        if (!line) {
            return false;
        }
        SketchPoint* start = nullptr;
        SketchPoint* end = nullptr;
        if (!lineEndpoints(pointsById_, line, start, end)) {
            return false;
        }
        auto gp1 = makePoint(start);
        auto gp2 = makePoint(end);
        gcsSystem_->addConstraintVertical(gp1, gp2, tagId, true);
        return true;
    }

    if (auto* parallel = dynamic_cast<ParallelConstraint*>(constraint)) {
        auto* line1 = getLine(parallel->line1());
        auto* line2 = getLine(parallel->line2());
        if (!line1 || !line2) {
            return false;
        }
        SketchPoint* l1s = nullptr;
        SketchPoint* l1e = nullptr;
        SketchPoint* l2s = nullptr;
        SketchPoint* l2e = nullptr;
        if (!lineEndpoints(pointsById_, line1, l1s, l1e) ||
            !lineEndpoints(pointsById_, line2, l2s, l2e)) {
            return false;
        }
        GCS::Line l1 = makeLine(l1s, l1e);
        GCS::Line l2 = makeLine(l2s, l2e);
        gcsSystem_->addConstraintParallel(l1, l2, tagId, true);
        return true;
    }

    if (auto* perpendicular = dynamic_cast<PerpendicularConstraint*>(constraint)) {
        auto* line1 = getLine(perpendicular->line1());
        auto* line2 = getLine(perpendicular->line2());
        if (!line1 || !line2) {
            return false;
        }
        SketchPoint* l1s = nullptr;
        SketchPoint* l1e = nullptr;
        SketchPoint* l2s = nullptr;
        SketchPoint* l2e = nullptr;
        if (!lineEndpoints(pointsById_, line1, l1s, l1e) ||
            !lineEndpoints(pointsById_, line2, l2s, l2e)) {
            return false;
        }
        GCS::Line l1 = makeLine(l1s, l1e);
        GCS::Line l2 = makeLine(l2s, l2e);
        gcsSystem_->addConstraintPerpendicular(l1, l2, tagId, true);
        return true;
    }

    if (auto* distance = dynamic_cast<DistanceConstraint*>(constraint)) {
        auto* p1 = getPoint(distance->entity1());
        auto* p2 = getPoint(distance->entity2());
        auto* line1 = getLine(distance->entity1());
        auto* line2 = getLine(distance->entity2());

        if (p1 && p2) {
            auto gp1 = makePoint(p1);
            auto gp2 = makePoint(p2);
            gcsSystem_->addConstraintP2PDistance(gp1, gp2, distance->valuePtr(), tagId, true);
            return true;
        }

        if (p1 && line2) {
            SketchPoint* l2s = nullptr;
            SketchPoint* l2e = nullptr;
            if (!lineEndpoints(pointsById_, line2, l2s, l2e)) {
                return false;
            }
            GCS::Line line = makeLine(l2s, l2e);
            auto gp1 = makePoint(p1);
            gcsSystem_->addConstraintP2LDistance(gp1, line, distance->valuePtr(), tagId, true);
            return true;
        }

        if (p2 && line1) {
            SketchPoint* l1s = nullptr;
            SketchPoint* l1e = nullptr;
            if (!lineEndpoints(pointsById_, line1, l1s, l1e)) {
                return false;
            }
            GCS::Line line = makeLine(l1s, l1e);
            auto gp2 = makePoint(p2);
            gcsSystem_->addConstraintP2LDistance(gp2, line, distance->valuePtr(), tagId, true);
            return true;
        }

        if (line1 && line2) {
            SketchPoint* l1s = nullptr;
            SketchPoint* l1e = nullptr;
            SketchPoint* l2s = nullptr;
            SketchPoint* l2e = nullptr;
            if (!lineEndpoints(pointsById_, line1, l1s, l1e) ||
                !lineEndpoints(pointsById_, line2, l2s, l2e)) {
                return false;
            }
            GCS::Line line = makeLine(l2s, l2e);
            auto gp1 = makePoint(l1s);
            gcsSystem_->addConstraintP2LDistance(gp1, line, distance->valuePtr(), tagId, true);
            return true;
        }

        return false;
    }

    if (auto* horizontalDistance = dynamic_cast<HorizontalDistanceConstraint*>(constraint)) {
        auto* p1 = getPoint(horizontalDistance->point1());
        auto* p2 = getPoint(horizontalDistance->point2());
        if (!p1 || !p2) {
            return false;
        }
        gcsSystem_->addConstraintDifference(coordPtr(p1, 1), coordPtr(p2, 1),
                                            horizontalDistance->valuePtr(), tagId, true);
        return true;
    }

    if (auto* verticalDistance = dynamic_cast<VerticalDistanceConstraint*>(constraint)) {
        auto* p1 = getPoint(verticalDistance->point1());
        auto* p2 = getPoint(verticalDistance->point2());
        if (!p1 || !p2) {
            return false;
        }
        gcsSystem_->addConstraintDifference(coordPtr(p1, 2), coordPtr(p2, 2),
                                            verticalDistance->valuePtr(), tagId, true);
        return true;
    }

    if (auto* angle = dynamic_cast<AngleConstraint*>(constraint)) {
        auto* line1 = getLine(angle->line1());
        auto* line2 = getLine(angle->line2());
        if (!line1 || !line2) {
            return false;
        }
        SketchPoint* l1s = nullptr;
        SketchPoint* l1e = nullptr;
        SketchPoint* l2s = nullptr;
        SketchPoint* l2e = nullptr;
        if (!lineEndpoints(pointsById_, line1, l1s, l1e) ||
            !lineEndpoints(pointsById_, line2, l2s, l2e)) {
            return false;
        }
        GCS::Line l1 = makeLine(l1s, l1e);
        GCS::Line l2 = makeLine(l2s, l2e);
        gcsSystem_->addConstraintL2LAngle(l1, l2, angle->valuePtr(), tagId, true);
        return true;
    }

    if (auto* radius = dynamic_cast<RadiusConstraint*>(constraint)) {
        auto* circle = getCircle(radius->entityId());
        if (circle) {
            SketchPoint* center = nullptr;
            if (!circleCenter(pointsById_, circle, center)) {
                return false;
            }
            GCS::Circle circleObj = makeCircle(center, circle);
            gcsSystem_->addConstraintCircleRadius(circleObj, radius->valuePtr(), tagId, true);
            return true;
        }

        auto* arc = getArc(radius->entityId());
        if (arc) {
            SketchPoint* center = nullptr;
            if (!arcCenter(pointsById_, arc, center)) {
                return false;
            }
            GCS::Arc arcObj = makeArc(center, arc);
            gcsSystem_->addConstraintArcRadius(arcObj, radius->valuePtr(), tagId, true);
            return true;
        }

        return false;
    }

    if (auto* diameter = dynamic_cast<DiameterConstraint*>(constraint)) {
        auto* circle = getCircle(diameter->entityId());
        if (circle) {
            SketchPoint* center = nullptr;
            if (!circleCenter(pointsById_, circle, center)) {
                return false;
            }
            GCS::Circle circleObj = makeCircle(center, circle);
            gcsSystem_->addConstraintCircleDiameter(circleObj, diameter->valuePtr(), tagId, true);
            return true;
        }

        auto* arc = getArc(diameter->entityId());
        if (arc) {
            SketchPoint* center = nullptr;
            if (!arcCenter(pointsById_, arc, center)) {
                return false;
            }
            GCS::Arc arcObj = makeArc(center, arc);
            gcsSystem_->addConstraintArcDiameter(arcObj, diameter->valuePtr(), tagId, true);
            return true;
        }

        return false;
    }

    if (auto* concentric = dynamic_cast<ConcentricConstraint*>(constraint)) {
        auto centerPointFor = [&](const EntityID& entityId) -> SketchPoint* {
            SketchPoint* center = nullptr;
            if (auto* circle = getCircle(entityId)) {
                return circleCenter(pointsById_, circle, center) ? center : nullptr;
            }
            if (auto* arc = getArc(entityId)) {
                return arcCenter(pointsById_, arc, center) ? center : nullptr;
            }
            return nullptr;
        };

        auto* c1 = centerPointFor(concentric->entity1());
        auto* c2 = centerPointFor(concentric->entity2());
        if (!c1 || !c2) {
            return false;
        }
        auto gp1 = makePoint(c1);
        auto gp2 = makePoint(c2);
        gcsSystem_->addConstraintP2PCoincident(gp1, gp2, tagId, true);
        return true;
    }

    if (auto* tangent = dynamic_cast<TangentConstraint*>(constraint)) {
        auto* line1 = getLine(tangent->entity1());
        auto* line2 = getLine(tangent->entity2());
        auto* circle1 = getCircle(tangent->entity1());
        auto* circle2 = getCircle(tangent->entity2());
        auto* arc1 = getArc(tangent->entity1());
        auto* arc2 = getArc(tangent->entity2());

        if (line1 && circle2) {
            SketchPoint* l1s = nullptr;
            SketchPoint* l1e = nullptr;
            if (!lineEndpoints(pointsById_, line1, l1s, l1e)) {
                return false;
            }
            SketchPoint* center = nullptr;
            if (!circleCenter(pointsById_, circle2, center)) {
                return false;
            }
            GCS::Line line = makeLine(l1s, l1e);
            GCS::Circle circle = makeCircle(center, circle2);
            gcsSystem_->addConstraintTangent(line, circle, tagId, true);
            return true;
        }

        if (line2 && circle1) {
            SketchPoint* l2s = nullptr;
            SketchPoint* l2e = nullptr;
            if (!lineEndpoints(pointsById_, line2, l2s, l2e)) {
                return false;
            }
            SketchPoint* center = nullptr;
            if (!circleCenter(pointsById_, circle1, center)) {
                return false;
            }
            GCS::Line line = makeLine(l2s, l2e);
            GCS::Circle circle = makeCircle(center, circle1);
            gcsSystem_->addConstraintTangent(line, circle, tagId, true);
            return true;
        }

        if (line1 && arc2) {
            SketchPoint* l1s = nullptr;
            SketchPoint* l1e = nullptr;
            if (!lineEndpoints(pointsById_, line1, l1s, l1e)) {
                return false;
            }
            SketchPoint* center = nullptr;
            if (!arcCenter(pointsById_, arc2, center)) {
                return false;
            }
            GCS::Line line = makeLine(l1s, l1e);
            GCS::Arc arc = makeArc(center, arc2);
            gcsSystem_->addConstraintTangent(line, arc, tagId, true);
            return true;
        }

        if (line2 && arc1) {
            SketchPoint* l2s = nullptr;
            SketchPoint* l2e = nullptr;
            if (!lineEndpoints(pointsById_, line2, l2s, l2e)) {
                return false;
            }
            SketchPoint* center = nullptr;
            if (!arcCenter(pointsById_, arc1, center)) {
                return false;
            }
            GCS::Line line = makeLine(l2s, l2e);
            GCS::Arc arc = makeArc(center, arc1);
            gcsSystem_->addConstraintTangent(line, arc, tagId, true);
            return true;
        }

        if (circle1 && circle2) {
            SketchPoint* c1 = nullptr;
            SketchPoint* c2 = nullptr;
            if (!circleCenter(pointsById_, circle1, c1) ||
                !circleCenter(pointsById_, circle2, c2)) {
                return false;
            }
            GCS::Circle circleObj1 = makeCircle(c1, circle1);
            GCS::Circle circleObj2 = makeCircle(c2, circle2);
            gcsSystem_->addConstraintTangent(circleObj1, circleObj2, tagId, true);
            return true;
        }

        if (arc1 && arc2) {
            SketchPoint* c1 = nullptr;
            SketchPoint* c2 = nullptr;
            if (!arcCenter(pointsById_, arc1, c1) ||
                !arcCenter(pointsById_, arc2, c2)) {
                return false;
            }
            GCS::Arc arcObj1 = makeArc(c1, arc1);
            GCS::Arc arcObj2 = makeArc(c2, arc2);
            gcsSystem_->addConstraintTangent(arcObj1, arcObj2, tagId, true);
            return true;
        }

        if (circle1 && arc2) {
            SketchPoint* c1 = nullptr;
            SketchPoint* c2 = nullptr;
            if (!circleCenter(pointsById_, circle1, c1) ||
                !arcCenter(pointsById_, arc2, c2)) {
                return false;
            }
            GCS::Circle circle = makeCircle(c1, circle1);
            GCS::Arc arc = makeArc(c2, arc2);
            gcsSystem_->addConstraintTangent(circle, arc, tagId, true);
            return true;
        }

        if (arc1 && circle2) {
            SketchPoint* c1 = nullptr;
            SketchPoint* c2 = nullptr;
            if (!arcCenter(pointsById_, arc1, c1) ||
                !circleCenter(pointsById_, circle2, c2)) {
                return false;
            }
            GCS::Arc arc = makeArc(c1, arc1);
            GCS::Circle circle = makeCircle(c2, circle2);
            gcsSystem_->addConstraintTangent(circle, arc, tagId, true);
            return true;
        }

        return false;
    }

    if (auto* pointOnCurve = dynamic_cast<PointOnCurveConstraint*>(constraint)) {
        auto* point = getPoint(pointOnCurve->pointId());
        if (!point) {
            return false;
        }

        auto pointObj = makePoint(point);

        if (auto* line = getLine(pointOnCurve->curveId())) {
            SketchPoint* start = nullptr;
            SketchPoint* end = nullptr;
            if (!lineEndpoints(pointsById_, line, start, end)) {
                return false;
            }

            if (pointOnCurve->position() == CurvePosition::Start) {
                auto startObj = makePoint(start);
                gcsSystem_->addConstraintP2PCoincident(pointObj, startObj, tagId, true);
                return true;
            }
            if (pointOnCurve->position() == CurvePosition::End) {
                auto endObj = makePoint(end);
                gcsSystem_->addConstraintP2PCoincident(pointObj, endObj, tagId, true);
                return true;
            }

            GCS::Line lineObj = makeLine(start, end);
            gcsSystem_->addConstraintPointOnLine(pointObj, lineObj, tagId, true);
            return true;
        }

        if (pointOnCurve->position() != CurvePosition::Arbitrary) {
            return false;
        }

        if (auto* circle = getCircle(pointOnCurve->curveId())) {
            SketchPoint* center = nullptr;
            if (!circleCenter(pointsById_, circle, center)) {
                return false;
            }
            GCS::Circle circleObj = makeCircle(center, circle);
            gcsSystem_->addConstraintPointOnCircle(pointObj, circleObj, tagId, true);
            return true;
        }

        if (auto* arc = getArc(pointOnCurve->curveId())) {
            SketchPoint* center = nullptr;
            if (!arcCenter(pointsById_, arc, center)) {
                return false;
            }
            GCS::Arc arcObj = makeArc(center, arc);
            gcsSystem_->addConstraintPointOnArc(pointObj, arcObj, tagId, true);
            return true;
        }

        return false;
    }

    if (auto* fixed = dynamic_cast<FixedConstraint*>(constraint)) {
        auto* p = getPoint(fixed->pointId());
        if (!p) {
            return false;
        }
        auto gp = makePoint(p);
        // Use const_cast to get mutable pointer to constraint's stored values
        double* xPtr = const_cast<double*>(&fixed->fixedXRef());
        double* yPtr = const_cast<double*>(&fixed->fixedYRef());
        gcsSystem_->addConstraintCoordinateX(gp, xPtr, tagId, true);
        gcsSystem_->addConstraintCoordinateY(gp, yPtr, tagId, true);
        return true;
    }

    if (auto* midpoint = dynamic_cast<MidpointConstraint*>(constraint)) {
        auto* p = getPoint(midpoint->pointId());
        auto* line = getLine(midpoint->lineId());
        if (!p || !line) {
            return false;
        }
        SketchPoint* start = nullptr;
        SketchPoint* end = nullptr;
        if (!lineEndpoints(pointsById_, line, start, end)) {
            return false;
        }
        auto gp = makePoint(p);
        GCS::Line gcsLine = makeLine(start, end);
        // Midpoint = point on line AND on perpendicular bisector
        gcsSystem_->addConstraintPointOnLine(gp, gcsLine, tagId, true);
        gcsSystem_->addConstraintPointOnPerpBisector(gp, gcsLine, tagId, true);
        return true;
    }

    if (auto* symmetric = dynamic_cast<SymmetricConstraint*>(constraint)) {
        auto* p1 = getPoint(symmetric->point1());
        auto* p2 = getPoint(symmetric->point2());
        auto* axis = getLine(symmetric->axisLine());
        if (!p1 || !p2 || !axis) {
            return false;
        }
        SketchPoint* axisStart = nullptr;
        SketchPoint* axisEnd = nullptr;
        if (!lineEndpoints(pointsById_, axis, axisStart, axisEnd)) {
            return false;
        }
        auto gp1 = makePoint(p1);
        auto gp2 = makePoint(p2);
        GCS::Line axisLine = makeLine(axisStart, axisEnd);
        gcsSystem_->addConstraintP2PSymmetric(gp1, gp2, axisLine, tagId, true);
        return true;
    }

    if (auto* equal = dynamic_cast<EqualConstraint*>(constraint)) {
        auto* line1 = getLine(equal->entity1());
        auto* line2 = getLine(equal->entity2());
        if (line1 && line2) {
            SketchPoint* l1s = nullptr;
            SketchPoint* l1e = nullptr;
            SketchPoint* l2s = nullptr;
            SketchPoint* l2e = nullptr;
            if (!lineEndpoints(pointsById_, line1, l1s, l1e) ||
                !lineEndpoints(pointsById_, line2, l2s, l2e)) {
                return false;
            }
            GCS::Line l1 = makeLine(l1s, l1e);
            GCS::Line l2 = makeLine(l2s, l2e);
            gcsSystem_->addConstraintEqualLength(l1, l2, tagId, true);
            return true;
        }

        auto* circle1 = getCircle(equal->entity1());
        auto* circle2 = getCircle(equal->entity2());
        auto* arc1 = getArc(equal->entity1());
        auto* arc2 = getArc(equal->entity2());

        if (circle1 && circle2) {
            SketchPoint* c1 = nullptr;
            SketchPoint* c2 = nullptr;
            if (!circleCenter(pointsById_, circle1, c1) ||
                !circleCenter(pointsById_, circle2, c2)) {
                return false;
            }
            GCS::Circle circleObj1 = makeCircle(c1, circle1);
            GCS::Circle circleObj2 = makeCircle(c2, circle2);
            gcsSystem_->addConstraintEqualRadius(circleObj1, circleObj2, tagId, true);
            return true;
        }

        if (circle1 && arc2) {
            SketchPoint* c1 = nullptr;
            SketchPoint* c2 = nullptr;
            if (!circleCenter(pointsById_, circle1, c1) ||
                !arcCenter(pointsById_, arc2, c2)) {
                return false;
            }
            GCS::Circle circle = makeCircle(c1, circle1);
            GCS::Arc arc = makeArc(c2, arc2);
            gcsSystem_->addConstraintEqualRadius(circle, arc, tagId, true);
            return true;
        }

        if (arc1 && arc2) {
            SketchPoint* c1 = nullptr;
            SketchPoint* c2 = nullptr;
            if (!arcCenter(pointsById_, arc1, c1) ||
                !arcCenter(pointsById_, arc2, c2)) {
                return false;
            }
            GCS::Arc arcObj1 = makeArc(c1, arc1);
            GCS::Arc arcObj2 = makeArc(c2, arc2);
            gcsSystem_->addConstraintEqualRadius(arcObj1, arcObj2, tagId, true);
            return true;
        }

        if (arc1 && circle2) {
            SketchPoint* c1 = nullptr;
            SketchPoint* c2 = nullptr;
            if (!arcCenter(pointsById_, arc1, c1) ||
                !circleCenter(pointsById_, circle2, c2)) {
                return false;
            }
            GCS::Arc arc = makeArc(c1, arc1);
            GCS::Circle circle = makeCircle(c2, circle2);
            gcsSystem_->addConstraintEqualRadius(circle, arc, tagId, true);
            return true;
        }

        return false;
    }

    return false;
}

void ConstraintSolver::configureSystem() {
    if (!gcsSystem_) {
        return;
    }
    gcsSystem_->setConvergence(config_.tolerance);
    gcsSystem_->setMaxIterations(config_.maxIterations);
    gcsSystem_->setConvergenceRedundant(config_.tolerance);
    gcsSystem_->setMaxIterationsRedundant(config_.maxIterations);
}

} // namespace onecad::core::sketch
