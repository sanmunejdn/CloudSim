/**
 * @file RegenerationEngine.h
 * @brief Engine for regenerating bodies from operation history.
 *
 * Replays operations in dependency order to rebuild geometry.
 * Supports full and partial regeneration, preview mode, and error handling.
 */
#ifndef ONECAD_APP_HISTORY_REGENERATIONENGINE_H
#define ONECAD_APP_HISTORY_REGENERATIONENGINE_H

#include "DependencyGraph.h"
#include "../document/OperationRecord.h"

#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>

#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace onecad::app {
class Document;
}

namespace onecad::app::history {

// ─────────────────────────────────────────────────────────────────────────────
// Result Types
// ─────────────────────────────────────────────────────────────────────────────

enum class RegenStatus {
    Success,         // All operations succeeded
    PartialFailure,  // Some ops failed, others succeeded
    CriticalFailure  // Unrecoverable error
};

struct FailedOp {
    std::string opId;
    OperationType type;
    std::string errorMessage;
    std::vector<std::string> affectedDownstream;
};

struct RegenResult {
    RegenStatus status = RegenStatus::Success;
    std::vector<std::string> succeededOps;
    std::vector<FailedOp> failedOps;
    std::vector<std::string> skippedOps;  // Suppressed or downstream of failed
};

// ─────────────────────────────────────────────────────────────────────────────
// Regeneration Engine
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Engine for regenerating bodies from operation history.
 *
 * Usage:
 *   RegenerationEngine engine(document);
 *   auto result = engine.regenerateAll();
 *   if (result.status == RegenStatus::PartialFailure) {
 *       // Handle failed ops
 *   }
 */
class RegenerationEngine {
public:
    explicit RegenerationEngine(Document* doc);

    /**
     * @brief Regenerate all bodies from operation history.
     *
     * Clears all existing bodies, then replays operations in dependency order.
     */
    RegenResult regenerateAll();

    /**
     * @brief Regenerate a prefix of operations [0, appliedCount).
     */
    RegenResult regenerateToAppliedCount(std::size_t appliedCount);

    /**
     * @brief Regenerate from a specific operation onwards.
     *
     * Rebuilds the specified op and all downstream dependents.
     * NOT production-wired: only exercised by proto_regeneration
     * (testDirtyFlagSkipsCleanBranch). Kept as the harness for incremental
     * DependencyGraph semantics and as a future partial-regen perf lever.
     * Unlike regenerateAll it performs no stale-body cleanup.
     */
    RegenResult regenerateFrom(const std::string& opId);

    /**
     * @brief Set progress callback for long operations.
     */
    using ProgressCallback = std::function<void(int current, int total, const std::string& opId)>;
    void setProgressCallback(ProgressCallback cb) { progressCallback_ = std::move(cb); }

    /**
     * @brief Access dependency graph (for queries and suppression).
     */
    DependencyGraph& graph() { return graph_; }
    const DependencyGraph& graph() const { return graph_; }

    // ─────────────────────────────────────────────────────────────────────────
    // Shape Resolution
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @brief Resolve an edge ID to its TopoDS_Edge.
     */
    std::optional<TopoDS_Shape> resolveEdge(const std::string& edgeId) const;

    /**
     * @brief Resolve a face ID to its TopoDS_Face.
     */
    std::optional<TopoDS_Shape> resolveFace(const std::string& faceId) const;

    /**
     * @brief Resolve a body ID to its TopoDS_Shape.
     */
    std::optional<TopoDS_Shape> resolveBody(const std::string& bodyId) const;

private:
    // ─────────────────────────────────────────────────────────────────────────
    // Operation Executors
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @brief Execute a single operation.
     * @return true on success, false on failure (error in errorOut).
     */
    bool executeOperation(const OperationRecord& op, std::string& errorOut);

    /**
     * @brief Build extrude geometry.
     */
    TopoDS_Shape buildExtrude(const OperationRecord& op, std::string& errorOut);

    /**
     * @brief Build revolve geometry.
     */
    TopoDS_Shape buildRevolve(const OperationRecord& op, std::string& errorOut);

    /**
     * @brief Build fillet geometry.
     */
    TopoDS_Shape buildFillet(const OperationRecord& op, std::string& errorOut);

    /**
     * @brief Build chamfer geometry.
     */
    TopoDS_Shape buildChamfer(const OperationRecord& op, std::string& errorOut);

    /**
     * @brief Build shell geometry.
     */
    TopoDS_Shape buildShell(const OperationRecord& op, std::string& errorOut);

    /**
     * @brief Build boolean geometry.
     */
    TopoDS_Shape buildBoolean(const OperationRecord& op, std::string& errorOut);

    /**
     * @brief Build linear pattern geometry.
     */
    TopoDS_Shape buildLinearPattern(const OperationRecord& op, std::string& errorOut);

    /**
     * @brief Build circular pattern geometry.
     */
    TopoDS_Shape buildCircularPattern(const OperationRecord& op, std::string& errorOut);

    /**
     * @brief Build loft geometry.
     */
    TopoDS_Shape buildLoft(const OperationRecord& op, std::string& errorOut);

    /**
     * @brief Build sweep geometry.
     */
    TopoDS_Shape buildSweep(const OperationRecord& op, std::string& errorOut);

    /**
     * @brief Build mirror body geometry.
     */
    TopoDS_Shape buildMirrorBody(const OperationRecord& op, std::string& errorOut);

    // ─────────────────────────────────────────────────────────────────────────
    // Input Resolution
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @brief Resolve face input (sketch region or body face) to TopoDS_Face.
     */
    std::optional<TopoDS_Face> resolveFaceInput(const OperationInput& input, std::string& errorOut);

    /**
     * @brief Build face from sketch region.
     */
    std::optional<TopoDS_Face> buildFaceFromSketchRegion(const std::string& sketchId,
                                                         const std::string& regionId,
                                                         std::string& errorOut);

    /**
     * @brief Resolve legacy host body from a sketch-region input's attached sketch metadata.
     */
    std::string resolveLegacySketchHostBodyId(const OperationInput& input) const;

    /**
     * @brief Resolve boolean target body in priority order:
     * params.targetBodyId -> FaceRef.bodyId -> sketch host body.
     */
    std::string resolveBooleanTargetBodyId(const OperationRecord& op,
                                           const std::string& explicitTargetBodyId) const;

    /**
     * @brief Add or update a body in the document.
     */
    bool applyBodyResult(const std::string& bodyId, const TopoDS_Shape& shape,
                         const std::string& opId, std::string& errorOut);

    Document* doc_;
    DependencyGraph graph_;
    ProgressCallback progressCallback_;
};

} // namespace onecad::app::history

#endif // ONECAD_APP_HISTORY_REGENERATIONENGINE_H
