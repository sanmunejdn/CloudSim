/**
 * @file DependencyGraph.h
 * @brief Dependency graph for parametric feature history.
 *
 * Tracks relationships between operations (which ops depend on which bodies/sketches)
 * and provides topological sort for regeneration order.
 */
#ifndef ONECAD_APP_HISTORY_DEPENDENCYGRAPH_H
#define ONECAD_APP_HISTORY_DEPENDENCYGRAPH_H

#include "../document/OperationRecord.h"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace onecad::app::history {

/**
 * @brief Node in the dependency graph representing a single operation.
 */
struct FeatureNode {
    std::string opId;
    OperationType type = OperationType::Extrude;

    // Input dependencies (what this op reads)
    std::unordered_set<std::string> inputSketchIds;
    std::unordered_set<std::string> inputBodyIds;
    std::unordered_set<std::string> inputEdgeIds;   // ElementMap IDs
    std::unordered_set<std::string> inputFaceIds;   // ElementMap IDs

    // Output (what this op produces/modifies)
    std::unordered_set<std::string> outputBodyIds;

    // State flags
    bool suppressed = false;
    bool failed = false;
    std::string failureReason;
};

/**
 * @brief Directed acyclic graph of operation dependencies.
 *
 * Used by RegenerationEngine to determine execution order and cascade updates.
 */
class DependencyGraph {
public:
    DependencyGraph() = default;

    /**
     * @brief Clear all nodes and edges.
     */
    void clear();

    /**
     * @brief Build graph from operation list (replaces existing).
     */
    void rebuildFromOperations(const std::vector<OperationRecord>& ops);

    /**
     * @brief Add a single operation to the graph.
     */
    void addOperation(const OperationRecord& op);

    /**
     * @brief Remove an operation from the graph.
     */
    void removeOperation(const std::string& opId);

    // ─────────────────────────────────────────────────────────────────────────
    // Queries
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @brief Get node by operation ID.
     * @return Pointer to node or nullptr if not found.
     */
    const FeatureNode* getNode(const std::string& opId) const;
    FeatureNode* getNode(const std::string& opId);

    /**
     * @brief Get topologically sorted list of operation IDs.
     *
     * Operations are ordered such that all dependencies come before dependents.
     * Uses Kahn's algorithm. Returns empty if graph has a cycle.
     */
    std::vector<std::string> topologicalSort() const;

    /**
     * @brief Get all operations that depend on this operation (downstream).
     */
    std::vector<std::string> getDownstream(const std::string& opId) const;

    /**
     * @brief Get all operations that this operation depends on (upstream).
     */
    std::vector<std::string> getUpstream(const std::string& opId) const;

    /**
     * @brief Most recent operation (creation order) that outputs the given body, or "".
     */
    std::string bodyProducer(const std::string& bodyId) const;

    /**
     * @brief True if @p bodyId is a valid (non-time-travel) reference target for @p opId:
     * it is a base body (no producer), has a producer strictly before opId, or is produced
     * by opId itself with no later producer. False if only produced by a later operation.
     */
    bool producesBefore(const std::string& bodyId, const std::string& opId) const;

    /**
     * @brief Get all operation IDs in creation order.
     */
    std::vector<std::string> getAllOpIds() const { return creationOrder_; }

    /**
     * @brief Check if graph contains a cycle (invalid state).
     */
    bool hasCycle() const;

    /**
     * @brief Get number of operations in graph.
     */
    size_t size() const { return nodes_.size(); }

    /**
     * @brief Check if graph is empty.
     */
    bool empty() const { return nodes_.empty(); }

    // ─────────────────────────────────────────────────────────────────────────
    // Suppression (for rollback)
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @brief Set suppression state for an operation.
     */
    void setSuppressed(const std::string& opId, bool suppressed);

    /**
     * @brief Check if operation is suppressed.
     */
    bool isSuppressed(const std::string& opId) const;

    /**
     * @brief Suppress all downstream operations from given op.
     */
    void suppressDownstream(const std::string& opId);

    /**
     * @brief Get snapshot of all suppression states.
     */
    std::unordered_map<std::string, bool> getSuppressionState() const;

    /**
     * @brief Restore suppression states from snapshot.
     */
    void setSuppressionState(const std::unordered_map<std::string, bool>& state);

    // ─────────────────────────────────────────────────────────────────────────
    // Failure Tracking
    // ─────────────────────────────────────────────────────────────────────────

    /**
     * @brief Mark operation as failed/succeeded.
     */
    void setFailed(const std::string& opId, bool failed, const std::string& reason = {});

    /**
     * @brief Check if operation is marked as failed.
     */
    bool isFailed(const std::string& opId) const;

    /**
     * @brief Get failure reason for an operation.
     */
    std::string getFailureReason(const std::string& opId) const;

    /**
     * @brief Get list of all failed operation IDs.
     */
    std::vector<std::string> getFailedOps() const;

    /**
     * @brief Clear all failure states.
     */
    void clearFailures();

private:
    /**
     * @brief Extract dependencies from an operation record.
     */
    void extractDependencies(const OperationRecord& op, FeatureNode& node);

    /**
     * @brief Rebuild edge maps from node dependencies.
     */
    void rebuildEdges();

    /**
     * @brief DFS helper for collecting downstream operations.
     */
    void collectDownstream(const std::string& opId,
                           std::unordered_set<std::string>& visited,
                           std::vector<std::string>& result) const;

    /**
     * @brief DFS helper for collecting upstream operations.
     */
    void collectUpstream(const std::string& opId,
                         std::unordered_set<std::string>& visited,
                         std::vector<std::string>& result) const;

    // Node storage
    std::unordered_map<std::string, FeatureNode> nodes_;

    // Edge maps (opId -> set of connected opIds)
    std::unordered_map<std::string, std::unordered_set<std::string>> forwardEdges_;  // opId -> downstream
    std::unordered_map<std::string, std::unordered_set<std::string>> backwardEdges_; // opId -> upstream

    // Creation order (for deterministic iteration)
    std::vector<std::string> creationOrder_;

    // Map: bodyId -> opId that produces it
    std::unordered_map<std::string, std::string> bodyProducers_;
};

} // namespace onecad::app::history

#endif // ONECAD_APP_HISTORY_DEPENDENCYGRAPH_H
