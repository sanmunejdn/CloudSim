/**
 * @file DependencyGraph.cpp
 * @brief Implementation of DependencyGraph.
 */
#include "DependencyGraph.h"

#include <QLoggingCategory>
#include <QString>

#include <algorithm>
#include <queue>

namespace onecad::app::history {

Q_LOGGING_CATEGORY(logDependencyGraph, "onecad.app.history.dependency")

void DependencyGraph::clear() {
    nodes_.clear();
    forwardEdges_.clear();
    backwardEdges_.clear();
    creationOrder_.clear();
    bodyProducers_.clear();
}

void DependencyGraph::rebuildFromOperations(const std::vector<OperationRecord>& ops) {
    qCDebug(logDependencyGraph) << "rebuildFromOperations:start" << "operationCount=" << ops.size();
    clear();
    for (const auto& op : ops) {
        FeatureNode node;
        node.opId = op.opId;
        node.type = op.type;

        extractDependencies(op, node);
        for (const auto& bodyId : op.resultBodyIds) {
            node.outputBodyIds.insert(bodyId);
        }

        nodes_[op.opId] = std::move(node);
        creationOrder_.push_back(op.opId);
    }
    rebuildEdges();
    qCDebug(logDependencyGraph) << "rebuildFromOperations:done"
                                << "nodeCount=" << nodes_.size()
                                << "forwardEdgeCount=" << forwardEdges_.size()
                                << "backwardEdgeCount=" << backwardEdges_.size();
}

void DependencyGraph::addOperation(const OperationRecord& op) {
    qCDebug(logDependencyGraph) << "addOperation"
                                << "opId=" << QString::fromStdString(op.opId)
                                << "type=" << static_cast<int>(op.type)
                                << "outputs=" << op.resultBodyIds.size();
    FeatureNode node;
    node.opId = op.opId;
    node.type = op.type;

    // Extract dependencies from operation
    extractDependencies(op, node);

    // Store output bodies
    for (const auto& bodyId : op.resultBodyIds) {
        node.outputBodyIds.insert(bodyId);
    }

    // Add node
    nodes_[op.opId] = std::move(node);
    creationOrder_.push_back(op.opId);

    // Rebuild edges
    rebuildEdges();
}

void DependencyGraph::removeOperation(const std::string& opId) {
    auto it = nodes_.find(opId);
    if (it == nodes_.end()) {
        return;
    }

    // Remove from body producers
    for (const auto& bodyId : it->second.outputBodyIds) {
        bodyProducers_.erase(bodyId);
    }

    // Remove node
    nodes_.erase(it);

    // Remove from creation order
    creationOrder_.erase(
        std::remove(creationOrder_.begin(), creationOrder_.end(), opId),
        creationOrder_.end());

    // Rebuild edges
    rebuildEdges();
}

const FeatureNode* DependencyGraph::getNode(const std::string& opId) const {
    auto it = nodes_.find(opId);
    return (it != nodes_.end()) ? &it->second : nullptr;
}

FeatureNode* DependencyGraph::getNode(const std::string& opId) {
    auto it = nodes_.find(opId);
    return (it != nodes_.end()) ? &it->second : nullptr;
}

std::vector<std::string> DependencyGraph::topologicalSort() const {
    // Kahn's algorithm for topological sort
    std::unordered_map<std::string, int> inDegree;
    std::unordered_map<std::string, std::size_t> creationIndex;
    creationIndex.reserve(creationOrder_.size());
    for (std::size_t i = 0; i < creationOrder_.size(); ++i) {
        creationIndex[creationOrder_[i]] = i;
    }

    // Initialize in-degrees
    for (const auto& [opId, _] : nodes_) {
        inDegree[opId] = 0;
    }

    // Count in-degrees from backward edges
    for (const auto& [opId, upstreams] : backwardEdges_) {
        inDegree[opId] = static_cast<int>(upstreams.size());
    }

    // Queue nodes with zero in-degree in creation order
    auto cmp = [&](const std::string& a, const std::string& b) {
        auto ia = creationIndex.find(a);
        auto ib = creationIndex.find(b);
        const std::size_t indexA = (ia != creationIndex.end()) ? ia->second : creationOrder_.size();
        const std::size_t indexB = (ib != creationIndex.end()) ? ib->second : creationOrder_.size();
        return indexA > indexB;
    };
    std::priority_queue<std::string, std::vector<std::string>, decltype(cmp)> queue(cmp);
    for (const auto& [opId, degree] : inDegree) {
        if (degree == 0) {
            queue.push(opId);
        }
    }

    std::vector<std::string> result;
    result.reserve(nodes_.size());

    while (!queue.empty()) {
        std::string current = queue.top();
        queue.pop();
        result.push_back(current);

        // Decrement in-degree of downstream nodes
        auto fwdIt = forwardEdges_.find(current);
        if (fwdIt != forwardEdges_.end()) {
            for (const auto& downstream : fwdIt->second) {
                if (--inDegree[downstream] == 0) {
                    queue.push(downstream);
                }
            }
        }
    }

    // If result size != nodes size, there's a cycle
    if (result.size() != nodes_.size()) {
        return {};  // Empty indicates cycle
    }

    return result;
}

std::vector<std::string> DependencyGraph::getDownstream(const std::string& opId) const {
    std::vector<std::string> result;
    std::unordered_set<std::string> visited;
    collectDownstream(opId, visited, result);
    return result;
}

std::vector<std::string> DependencyGraph::getUpstream(const std::string& opId) const {
    std::vector<std::string> result;
    std::unordered_set<std::string> visited;
    collectUpstream(opId, visited, result);
    return result;
}

bool DependencyGraph::hasCycle() const {
    return topologicalSort().empty() && !nodes_.empty();
}

void DependencyGraph::setSuppressed(const std::string& opId, bool suppressed) {
    auto* node = getNode(opId);
    if (node) {
        node->suppressed = suppressed;
    }
}

bool DependencyGraph::isSuppressed(const std::string& opId) const {
    const auto* node = getNode(opId);
    return node ? node->suppressed : false;
}

void DependencyGraph::suppressDownstream(const std::string& opId) {
    for (const auto& downstreamId : getDownstream(opId)) {
        setSuppressed(downstreamId, true);
    }
}

std::unordered_map<std::string, bool> DependencyGraph::getSuppressionState() const {
    std::unordered_map<std::string, bool> state;
    for (const auto& [opId, node] : nodes_) {
        state[opId] = node.suppressed;
    }
    return state;
}

void DependencyGraph::setSuppressionState(const std::unordered_map<std::string, bool>& state) {
    for (const auto& [opId, suppressed] : state) {
        setSuppressed(opId, suppressed);
    }
}

void DependencyGraph::setFailed(const std::string& opId, bool failed, const std::string& reason) {
    auto* node = getNode(opId);
    if (node) {
        node->failed = failed;
        node->failureReason = reason;
    }
}

bool DependencyGraph::isFailed(const std::string& opId) const {
    const auto* node = getNode(opId);
    return node ? node->failed : false;
}

std::string DependencyGraph::getFailureReason(const std::string& opId) const {
    const auto* node = getNode(opId);
    return node ? node->failureReason : std::string{};
}

std::vector<std::string> DependencyGraph::getFailedOps() const {
    std::vector<std::string> result;
    for (const auto& [opId, node] : nodes_) {
        if (node.failed) {
            result.push_back(opId);
        }
    }
    return result;
}

void DependencyGraph::clearFailures() {
    for (auto& [opId, node] : nodes_) {
        node.failed = false;
        node.failureReason.clear();
    }
}

void DependencyGraph::extractDependencies(const OperationRecord& op, FeatureNode& node) {
    // Extract from input variant
    if (std::holds_alternative<SketchRegionRef>(op.input)) {
        const auto& ref = std::get<SketchRegionRef>(op.input);
        node.inputSketchIds.insert(ref.sketchId);
    } else if (std::holds_alternative<FaceRef>(op.input)) {
        const auto& ref = std::get<FaceRef>(op.input);
        node.inputBodyIds.insert(ref.bodyId);
        node.inputFaceIds.insert(ref.faceId);
        for (const auto& patchFaceId : ref.patchFaceIds) {
            node.inputFaceIds.insert(patchFaceId);
        }
    } else if (std::holds_alternative<BodyRef>(op.input)) {
        const auto& ref = std::get<BodyRef>(op.input);
        node.inputBodyIds.insert(ref.bodyId);
    }

    // Extract from params variant
    if (std::holds_alternative<ExtrudeParams>(op.params)) {
        const auto& p = std::get<ExtrudeParams>(op.params);
        if (p.booleanMode != BooleanMode::NewBody && !p.targetBodyId.empty()) {
            node.inputBodyIds.insert(p.targetBodyId);
            qCDebug(logDependencyGraph) << "extractDependencies:extrude-target-body"
                                        << "opId=" << QString::fromStdString(op.opId)
                                        << "targetBodyId=" << QString::fromStdString(p.targetBodyId)
                                        << "mode=" << static_cast<int>(p.booleanMode);
        }
    } else if (std::holds_alternative<RevolveParams>(op.params)) {
        const auto& p = std::get<RevolveParams>(op.params);
        if (p.booleanMode != BooleanMode::NewBody && !p.targetBodyId.empty()) {
            node.inputBodyIds.insert(p.targetBodyId);
            qCDebug(logDependencyGraph) << "extractDependencies:revolve-target-body"
                                        << "opId=" << QString::fromStdString(op.opId)
                                        << "targetBodyId=" << QString::fromStdString(p.targetBodyId)
                                        << "mode=" << static_cast<int>(p.booleanMode);
        }
        if (std::holds_alternative<SketchLineRef>(p.axis)) {
            const auto& axis = std::get<SketchLineRef>(p.axis);
            node.inputSketchIds.insert(axis.sketchId);
        } else if (std::holds_alternative<EdgeRef>(p.axis)) {
            const auto& axis = std::get<EdgeRef>(p.axis);
            node.inputBodyIds.insert(axis.bodyId);
            node.inputEdgeIds.insert(axis.edgeId);
        }
    } else if (std::holds_alternative<FilletChamferParams>(op.params)) {
        const auto& p = std::get<FilletChamferParams>(op.params);
        for (const auto& edgeId : p.edgeIds) {
            node.inputEdgeIds.insert(edgeId);
        }
    } else if (std::holds_alternative<ShellParams>(op.params)) {
        const auto& p = std::get<ShellParams>(op.params);
        for (const auto& faceId : p.openFaceIds) {
            node.inputFaceIds.insert(faceId);
        }
    } else if (std::holds_alternative<BooleanParams>(op.params)) {
        const auto& p = std::get<BooleanParams>(op.params);
        node.inputBodyIds.insert(p.targetBodyId);
        node.inputBodyIds.insert(p.toolBodyId);
    } else if (std::holds_alternative<LinearPatternParams>(op.params)) {
        const auto& p = std::get<LinearPatternParams>(op.params);
        if (!p.sourceBodyId.empty()) {
            node.inputBodyIds.insert(p.sourceBodyId);
        }
    } else if (std::holds_alternative<CircularPatternParams>(op.params)) {
        const auto& p = std::get<CircularPatternParams>(op.params);
        if (!p.sourceBodyId.empty()) {
            node.inputBodyIds.insert(p.sourceBodyId);
        }
    } else if (std::holds_alternative<SweepParams>(op.params)) {
        const auto& p = std::get<SweepParams>(op.params);
        if (!p.profileSketchId.empty()) {
            node.inputSketchIds.insert(p.profileSketchId);
        }
        if (!p.pathSketchId.empty()) {
            node.inputSketchIds.insert(p.pathSketchId);
        }
        if (!p.pathEdgeId.empty()) {
            node.inputEdgeIds.insert(p.pathEdgeId);
        }
    }
}

void DependencyGraph::rebuildEdges() {
    forwardEdges_.clear();
    backwardEdges_.clear();
    bodyProducers_.clear();

    // Walk creation order so dependencies target the most recent producer.
    for (const auto& opId : creationOrder_) {
        auto nodeIt = nodes_.find(opId);
        if (nodeIt == nodes_.end()) {
            continue;
        }
        const FeatureNode& node = nodeIt->second;

        auto linkToProducer = [&](const std::string& producedBodyId) {
            auto it = bodyProducers_.find(producedBodyId);
            if (it != bodyProducers_.end() && it->second != opId) {
                forwardEdges_[it->second].insert(opId);
                backwardEdges_[opId].insert(it->second);
            }
        };

        for (const auto& inputBodyId : node.inputBodyIds) {
            linkToProducer(inputBodyId);
        }

        // Face/edge inputs also depend on the body that produced them. Element ids are
        // "<bodyId>/<...>", so the owning body is the prefix before the first '/'. This
        // makes fillet/chamfer/shell that consume a feature's faces/edges regenerate when
        // that upstream feature is edited, even if the body isn't an explicit input.
        auto ownerBodyOf = [](const std::string& elemId) -> std::string {
            const auto pos = elemId.find('/');
            return pos == std::string::npos ? std::string{} : elemId.substr(0, pos);
        };
        for (const auto& faceId : node.inputFaceIds) {
            const std::string owner = ownerBodyOf(faceId);
            if (!owner.empty()) {
                linkToProducer(owner);
            }
        }
        for (const auto& edgeId : node.inputEdgeIds) {
            const std::string owner = ownerBodyOf(edgeId);
            if (!owner.empty()) {
                linkToProducer(owner);
            }
        }

        for (const auto& bodyId : node.outputBodyIds) {
            bodyProducers_[bodyId] = opId;
        }
    }
}

std::string DependencyGraph::bodyProducer(const std::string& bodyId) const {
    auto it = bodyProducers_.find(bodyId);
    return it == bodyProducers_.end() ? std::string{} : it->second;
}

bool DependencyGraph::producesBefore(const std::string& bodyId, const std::string& opId) const {
    // Locate opId's position in creation order.
    int opIndex = -1;
    for (std::size_t i = 0; i < creationOrder_.size(); ++i) {
        if (creationOrder_[i] == opId) {
            opIndex = static_cast<int>(i);
            break;
        }
    }
    if (opIndex < 0) {
        return true;  // op not tracked yet — don't block
    }

    bool producedBefore = false;
    bool producedAfter = false;
    bool producedByOp = false;
    for (std::size_t i = 0; i < creationOrder_.size(); ++i) {
        const FeatureNode* node = getNode(creationOrder_[i]);
        if (!node || node->outputBodyIds.find(bodyId) == node->outputBodyIds.end()) {
            continue;
        }
        if (static_cast<int>(i) < opIndex) {
            producedBefore = true;
        } else if (static_cast<int>(i) == opIndex) {
            producedByOp = true;
        } else {
            producedAfter = true;
        }
    }

    if (!producedBefore && !producedByOp && !producedAfter) {
        return true;  // base/external body: no operation produces it
    }
    if (producedBefore) {
        return true;  // a valid upstream producer exists
    }
    if (producedByOp && !producedAfter) {
        return true;  // op modifies a body it also produces, with no later producer
    }
    return false;  // only produced by a later op -> time-travel dependency
}

void DependencyGraph::collectDownstream(const std::string& opId,
                                        std::unordered_set<std::string>& visited,
                                        std::vector<std::string>& result) const {
    auto it = forwardEdges_.find(opId);
    if (it == forwardEdges_.end()) {
        return;
    }

    for (const auto& downstream : it->second) {
        if (visited.insert(downstream).second) {
            result.push_back(downstream);
            collectDownstream(downstream, visited, result);
        }
    }
}

void DependencyGraph::collectUpstream(const std::string& opId,
                                      std::unordered_set<std::string>& visited,
                                      std::vector<std::string>& result) const {
    auto it = backwardEdges_.find(opId);
    if (it == backwardEdges_.end()) {
        return;
    }

    for (const auto& upstream : it->second) {
        if (visited.insert(upstream).second) {
            result.push_back(upstream);
            collectUpstream(upstream, visited, result);
        }
    }
}

} // namespace onecad::app::history
