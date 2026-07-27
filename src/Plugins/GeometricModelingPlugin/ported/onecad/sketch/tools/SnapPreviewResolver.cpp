#include "SnapPreviewResolver.h"

#include "../Sketch.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace onecad::core::sketch::tools {

namespace {

constexpr double kGuideLengthEpsSq = 1e-12;
constexpr double kGuideCollinearCrossEps = 0.01;
constexpr double kGuideDistanceTieEps = 1e-9;
constexpr double kGuideIntersectionMatchEps = 1e-5;
constexpr double kHvGuideAngleThresholdRad = 7.5 * (3.14159265358979323846 / 180.0);
constexpr double kAxisReferenceEpsSq = 1e-12;

bool sameResolvedSnap(const SnapResult& a, const SnapResult& b) {
    return a.snapped == b.snapped
        && a.type == b.type
        && std::abs(a.position.x - b.position.x) <= 1e-9
        && std::abs(a.position.y - b.position.y) <= 1e-9
        && a.entityId == b.entityId
        && a.secondEntityId == b.secondEntityId
        && a.pointId == b.pointId
        && a.hasGuide == b.hasGuide
        && std::abs(a.guideOrigin.x - b.guideOrigin.x) <= 1e-9
        && std::abs(a.guideOrigin.y - b.guideOrigin.y) <= 1e-9
        && a.gridKind == b.gridKind
        && a.guideIntersectionKind == b.guideIntersectionKind
        && a.hintText == b.hintText;
}

bool isGuideSuppressedPointSnapType(SnapType type) {
    switch (type) {
        case SnapType::Vertex:
        case SnapType::Endpoint:
        case SnapType::Midpoint:
        case SnapType::Center:
        case SnapType::Quadrant:
            return true;
        default:
            return false;
    }
}

bool isAxisInferenceType(SnapType type) {
    return type == SnapType::Horizontal || type == SnapType::Vertical;
}

bool isRenderableGuideType(SnapType type) {
    return type != SnapType::Grid;
}

bool passesAxisInferenceGate(const SnapResult& snap, std::optional<Vec2d> referencePoint) {
    if (!isAxisInferenceType(snap.type)) {
        return true;
    }
    if (!referencePoint.has_value()) {
        return false;
    }

    const double dx = snap.position.x - referencePoint->x;
    const double dy = snap.position.y - referencePoint->y;
    const double lenSq = (dx * dx) + (dy * dy);
    if (lenSq <= kAxisReferenceEpsSq) {
        return false;
    }

    const double maxOffAxis = std::sqrt(lenSq) * std::sin(kHvGuideAngleThresholdRad);
    if (snap.type == SnapType::Horizontal) {
        return std::abs(dy) <= maxOffAxis;
    }
    return std::abs(dx) <= maxOffAxis;
}

bool isValidGuideCandidate(const SnapResult& snap, bool includeNonRenderableGuides) {
    if (!snap.snapped || !snap.hasGuide || snap.type == SnapType::Intersection) {
        return false;
    }
    if (!includeNonRenderableGuides && !isRenderableGuideType(snap.type)) {
        return false;
    }
    const double dx = snap.position.x - snap.guideOrigin.x;
    const double dy = snap.position.y - snap.guideOrigin.y;
    return (dx * dx + dy * dy) > kGuideLengthEpsSq;
}

std::optional<Vec2d> infiniteLineIntersection(const GuideSegment& a, const GuideSegment& b) {
    const double d1x = a.target.x - a.origin.x;
    const double d1y = a.target.y - a.origin.y;
    const double d2x = b.target.x - b.origin.x;
    const double d2y = b.target.y - b.origin.y;
    const double cross = d1x * d2y - d1y * d2x;
    if (std::abs(cross) < 1e-12) {
        return std::nullopt;
    }
    const double dx = b.origin.x - a.origin.x;
    const double dy = b.origin.y - a.origin.y;
    const double t = (dx * d2y - dy * d2x) / cross;
    return Vec2d{a.origin.x + t * d1x, a.origin.y + t * d1y};
}

bool samePosition(const Vec2d& a, const Vec2d& b, double eps = SnapResult::kOverlapEps) {
    return std::abs(a.x - b.x) <= eps && std::abs(a.y - b.y) <= eps;
}

bool isCollinear(const GuideSegment& a, const GuideSegment& b) {
    double dirAx = a.target.x - a.origin.x;
    double dirAy = a.target.y - a.origin.y;
    double dirBx = b.target.x - b.origin.x;
    double dirBy = b.target.y - b.origin.y;
    const double lenA = std::sqrt((dirAx * dirAx) + (dirAy * dirAy));
    const double lenB = std::sqrt((dirBx * dirBx) + (dirBy * dirBy));
    if (lenA < 1e-6 || lenB < 1e-6) {
        return true;
    }
    dirAx /= lenA;
    dirAy /= lenA;
    dirBx /= lenB;
    dirBy /= lenB;
    const double cross = std::abs((dirAx * dirBy) - (dirAy * dirBx));
    return cross < kGuideCollinearCrossEps;
}

struct GuideCandidate {
    GuideSegment segment;
    double distance = 0.0;
    size_t sourceIndex = 0;
};

bool isGuideIntersectionSnap(const SnapResult& snap) {
    return snap.snapped && snap.type == SnapType::Intersection && snap.hasGuide;
}

std::optional<size_t> pickNearestGuideCandidateIndex(
    const std::vector<GuideCandidate>& candidates) {
    if (candidates.empty()) {
        return std::nullopt;
    }

    size_t bestIndex = 0;
    for (size_t i = 1; i < candidates.size(); ++i) {
        const auto& best = candidates[bestIndex];
        const auto& candidate = candidates[i];
        if (candidate.distance < best.distance ||
            (std::abs(candidate.distance - best.distance) <= kGuideDistanceTieEps &&
             candidate.sourceIndex < best.sourceIndex)) {
            bestIndex = i;
        }
    }
    return bestIndex;
}

std::optional<std::pair<size_t, size_t>> pickGuidePairForIntersection(
    const std::vector<GuideCandidate>& candidates,
    const std::optional<Vec2d>& intersectionPos) {
    std::optional<std::pair<size_t, size_t>> bestPair;
    double bestPairMetric = std::numeric_limits<double>::max();

    for (size_t i = 0; i < candidates.size(); ++i) {
        for (size_t j = i + 1; j < candidates.size(); ++j) {
            if (isCollinear(candidates[i].segment, candidates[j].segment)) {
                continue;
            }

            const auto intersection =
                infiniteLineIntersection(candidates[i].segment, candidates[j].segment);
            if (!intersection.has_value()) {
                continue;
            }

            if (intersectionPos.has_value() &&
                !samePosition(*intersection, *intersectionPos, kGuideIntersectionMatchEps)) {
                continue;
            }

            const double metric = candidates[i].distance + candidates[j].distance;
            if (!bestPair.has_value() ||
                metric < bestPairMetric ||
                (std::abs(metric - bestPairMetric) <= kGuideDistanceTieEps &&
                 std::pair<size_t, size_t>{candidates[i].sourceIndex, candidates[j].sourceIndex} <
                     std::pair<size_t, size_t>{
                         candidates[bestPair->first].sourceIndex,
                         candidates[bestPair->second].sourceIndex})) {
                bestPairMetric = metric;
                bestPair = std::pair<size_t, size_t>{i, j};
            }
        }
    }

    return bestPair;
}

bool hasResolvableGuidePairAt(const std::vector<GuideCandidate>& candidates, const Vec2d& pos) {
    return pickGuidePairForIntersection(candidates, pos).has_value();
}

bool hasNonGridGuidePairAt(const std::vector<SnapResult>& allSnaps,
                           const std::vector<GuideCandidate>& candidates,
                           const Vec2d& pos) {
    for (size_t i = 0; i < candidates.size(); ++i) {
        for (size_t j = i + 1; j < candidates.size(); ++j) {
            if (isCollinear(candidates[i].segment, candidates[j].segment)) {
                continue;
            }

            const auto intersection =
                infiniteLineIntersection(candidates[i].segment, candidates[j].segment);
            if (!intersection.has_value()) {
                continue;
            }
            if (!samePosition(*intersection, pos, kGuideIntersectionMatchEps)) {
                continue;
            }

            const auto typeA = allSnaps[candidates[i].sourceIndex].type;
            const auto typeB = allSnaps[candidates[j].sourceIndex].type;
            if (typeA == SnapType::Grid && typeB == SnapType::Grid) {
                continue;
            }
            return true;
        }
    }
    return false;
}

SnapResult selectBestSnapDeterministically(const SnapManager& snapManager,
                                           const Vec2d& pos,
                                           const std::vector<SnapResult>& snaps,
                                           const Sketch& sketch) {
    return snapManager.selectBestSnapFromCandidates(pos, sketch, snaps);
}

std::vector<GuideCandidate> collectGuideCandidates(
    const std::vector<SnapResult>& allSnaps,
    bool includeNonRenderableGuides) {
    std::vector<GuideCandidate> candidates;
    candidates.reserve(allSnaps.size());
    for (size_t i = 0; i < allSnaps.size(); ++i) {
        const auto& snap = allSnaps[i];
        if (!isValidGuideCandidate(snap, includeNonRenderableGuides)) {
            continue;
        }
        candidates.push_back({
            .segment = {snap.guideOrigin, snap.position},
            .distance = snap.distance,
            .sourceIndex = i
        });
    }
    return candidates;
}

std::vector<SnapResult> filterSnapsForInputContext(const std::vector<SnapResult>& allSnaps,
                                                   std::optional<Vec2d> referencePoint) {
    std::vector<SnapResult> filtered;
    filtered.reserve(allSnaps.size());

    for (const auto& snap : allSnaps) {
        if (!snap.snapped) {
            continue;
        }
        if (snap.type != SnapType::Intersection || !snap.hasGuide) {
            if (!passesAxisInferenceGate(snap, referencePoint)) {
                continue;
            }
        }
        filtered.push_back(snap);
    }

    if (filtered.empty()) {
        return filtered;
    }

    const auto allGuideCandidates = collectGuideCandidates(filtered, true);
    std::vector<SnapResult> stabilized;
    stabilized.reserve(filtered.size());
    for (const auto& snap : filtered) {
        if (snap.type == SnapType::Intersection && snap.hasGuide &&
            !hasNonGridGuidePairAt(filtered, allGuideCandidates, snap.position)) {
            continue;
        }
        stabilized.push_back(snap);
    }

    return stabilized;
}

std::optional<size_t> pickNearestGuideIntersectionSnapIndex(
    const std::vector<SnapResult>& allSnaps,
    const std::vector<GuideCandidate>& candidates) {
    std::optional<size_t> bestIndex;
    for (size_t i = 0; i < allSnaps.size(); ++i) {
        const auto& snap = allSnaps[i];
        if (!isGuideIntersectionSnap(snap)) {
            continue;
        }
        if (!hasResolvableGuidePairAt(candidates, snap.position)) {
            continue;
        }

        if (!bestIndex.has_value() ||
            snap.distance < allSnaps[*bestIndex].distance ||
            (std::abs(snap.distance - allSnaps[*bestIndex].distance) <= kGuideDistanceTieEps &&
             i < *bestIndex)) {
            bestIndex = i;
        }
    }
    return bestIndex;
}

} // namespace

SnapResult applyGuideFirstSnapPolicy(const SnapResult& fallbackSnap,
                                     const std::vector<SnapResult>& allSnaps) {
    if (fallbackSnap.snapped && isGuideSuppressedPointSnapType(fallbackSnap.type)) {
        return fallbackSnap;
    }

    const auto guideCandidates = collectGuideCandidates(allSnaps, false);
    const auto bestSingleGuide = pickNearestGuideCandidateIndex(guideCandidates);
    const auto bestGuideIntersection =
        pickNearestGuideIntersectionSnapIndex(allSnaps, guideCandidates);

    if (bestSingleGuide.has_value() && bestGuideIntersection.has_value()) {
        const auto& single = allSnaps[guideCandidates[*bestSingleGuide].sourceIndex];
        const auto& crossing = allSnaps[*bestGuideIntersection];

        const double delta = single.distance - crossing.distance;
        if (std::abs(delta) <= kGuideDistanceTieEps || delta < 0.0) {
            return single;
        }
        return crossing;
    }
    if (bestGuideIntersection.has_value()) {
        return allSnaps[*bestGuideIntersection];
    }
    if (bestSingleGuide.has_value()) {
        return allSnaps[guideCandidates[*bestSingleGuide].sourceIndex];
    }
    return fallbackSnap;
}

std::vector<GuideSegment> buildActiveGuidesForSnap(const SnapResult& resolvedSnap,
                                                   const std::vector<SnapResult>& allSnaps) {
    std::vector<GuideSegment> activeGuides;
    if (!resolvedSnap.snapped || allSnaps.empty() ||
        isGuideSuppressedPointSnapType(resolvedSnap.type)) {
        return activeGuides;
    }

    const auto candidates = collectGuideCandidates(allSnaps, false);
    if (candidates.empty()) {
        return activeGuides;
    }

    if (resolvedSnap.type == SnapType::Intersection && resolvedSnap.hasGuide) {
        auto pair = pickGuidePairForIntersection(candidates, resolvedSnap.position);
        if (!pair.has_value()) {
            pair = pickGuidePairForIntersection(candidates, std::nullopt);
        }
        if (pair.has_value()) {
            activeGuides.push_back(candidates[pair->first].segment);
            activeGuides.push_back(candidates[pair->second].segment);
            return activeGuides;
        }

        // Defensive fallback: if a guide-cross snap cannot be mapped to a pair,
        // keep a single nearest guide to avoid empty/flickering preview.
        const auto nearestGuide = pickNearestGuideCandidateIndex(candidates);
        if (nearestGuide.has_value()) {
            activeGuides.push_back(candidates[*nearestGuide].segment);
        }
        return activeGuides;
    }

    const auto nearestGuide = pickNearestGuideCandidateIndex(candidates);
    if (nearestGuide.has_value()) {
        activeGuides.push_back(candidates[*nearestGuide].segment);
    }
    return activeGuides;
}

SnapInputResolution resolveSnapForInputEvent(
    const SnapManager& snapManager,
    const Vec2d& pos,
    const Sketch& sketch,
    const std::unordered_set<EntityID>& excludeFromSnap,
    std::optional<Vec2d> referencePoint,
    bool preferGuide,
    bool collectGuideData) {
    SnapInputResolution resolution;
    resolution.allSnaps = snapManager.findAllSnaps(pos, sketch, excludeFromSnap, referencePoint);
    resolution.allSnaps = filterSnapsForInputContext(resolution.allSnaps, referencePoint);
    resolution.bestSnap = selectBestSnapDeterministically(
        snapManager,
        pos,
        resolution.allSnaps,
        sketch);
    resolution.gridConflict = snapManager.lastGridConflictDetected();
    resolution.allowPreviewCommitMismatch = resolution.gridConflict;
    resolution.resolvedSnap = resolution.bestSnap;

    if (resolution.allSnaps.empty()) {
        return resolution;
    }

    const bool requiresGuideResolution = preferGuide || collectGuideData;
    if (!requiresGuideResolution) {
        return resolution;
    }

    const SnapResult guideResolvedSnap =
        applyGuideFirstSnapPolicy(resolution.bestSnap, resolution.allSnaps);

#ifndef NDEBUG
    bool hasGuideCandidate = false;
    for (const auto& snap : resolution.allSnaps) {
        if (snap.snapped && snap.hasGuide) {
            hasGuideCandidate = true;
            break;
        }
    }
    if (!hasGuideCandidate) {
        assert(sameResolvedSnap(guideResolvedSnap, resolution.bestSnap));
    }
#endif

    resolution.resolvedSnap = preferGuide ? guideResolvedSnap : resolution.bestSnap;

    if (collectGuideData) {
        resolution.activeGuides =
            buildActiveGuidesForSnap(resolution.resolvedSnap, resolution.allSnaps);
    }

    return resolution;
}

} // namespace onecad::core::sketch::tools
