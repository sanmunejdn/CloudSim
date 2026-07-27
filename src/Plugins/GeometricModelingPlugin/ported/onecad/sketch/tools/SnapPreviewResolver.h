/**
 * @file SnapPreviewResolver.h
 * @brief Shared snap-preview resolution helpers for tools and viewport drag.
 */

#ifndef ONECAD_CORE_SKETCH_TOOLS_SNAP_PREVIEW_RESOLVER_H
#define ONECAD_CORE_SKETCH_TOOLS_SNAP_PREVIEW_RESOLVER_H

#include "../SnapManager.h"

#include <optional>
#include <unordered_set>
#include <vector>

namespace onecad::core::sketch {

class Sketch;

namespace tools {

/**
 * @brief Minimal guide segment data independent of renderer internals.
 */
struct GuideSegment {
    Vec2d origin;
    Vec2d target;
};

/**
 * @brief Complete snap resolution payload for an input event.
 */
struct SnapInputResolution {
    SnapResult bestSnap;
    SnapResult resolvedSnap;
    std::vector<SnapResult> allSnaps;
    std::vector<GuideSegment> activeGuides;
    bool gridConflict = false;
    bool allowPreviewCommitMismatch = false;
};

/**
 * @brief Apply guide-first preview policy while preserving point/endpoint priority.
 */
SnapResult applyGuideFirstSnapPolicy(const SnapResult& fallbackSnap,
                                     const std::vector<SnapResult>& allSnaps);

/**
 * @brief Build renderer-ready guide segments from snap candidates.
 */
std::vector<GuideSegment> buildActiveGuidesForSnap(
    const SnapResult& resolvedSnap,
    const std::vector<SnapResult>& allSnaps);

/**
 * @brief Resolve snap winner and optional preview guide data for one input event.
 */
SnapInputResolution resolveSnapForInputEvent(
    const SnapManager& snapManager,
    const Vec2d& pos,
    const Sketch& sketch,
    const std::unordered_set<EntityID>& excludeFromSnap = {},
    std::optional<Vec2d> referencePoint = std::nullopt,
    bool preferGuide = false,
    bool collectGuideData = false);

} // namespace tools
} // namespace onecad::core::sketch

#endif // ONECAD_CORE_SKETCH_TOOLS_SNAP_PREVIEW_RESOLVER_H
