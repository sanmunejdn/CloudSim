/**
 * @file RegenerationEngine.cpp
 * @brief Implementation of RegenerationEngine.
 */
#include "RegenerationEngine.h"

#include "../document/Document.h"
#include "../../core/loop/RegionUtils.h"
#include "../../core/loop/FaceBuilder.h"
#include "../../core/modeling/FacePatchResolver.h"
#include "../../core/modeling/EdgeChainer.h"
#include "../../core/sketch/Sketch.h"
#include "../../core/sketch/SketchLine.h"
#include "../../core/sketch/SketchPoint.h"

#include <QLoggingCategory>
#include <QString>

#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepOffsetAPI_MakeThickSolid.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRep_Builder.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepOffsetAPI_DraftAngle.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <Standard_Failure.hxx>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace onecad::app::history {

Q_LOGGING_CATEGORY(logRegen, "onecad.app.history.regeneration")

namespace {
constexpr double kDraftAngleEpsilon = 1e-4;
constexpr double kSideFaceDotThreshold = 0.9;
constexpr double kMinValue = 1e-3;
constexpr double kMinAngleDeg = 1e-3;
// Max descriptor score accepted when re-matching a stale face/edge reference. The score
// scale is ~O(mm) for geometric terms, so ~1mm of drift is the most a re-match may
// silently absorb; anything looser resolved deleted geometry onto its nearest neighbor.
// (ElementMap::resolveWithFallback additionally requires a uniqueness margin.)
constexpr double kRematchRejectScore = 1.0;

std::string inputSourceSummary(const OperationInput& input) {
    if (std::holds_alternative<SketchRegionRef>(input)) {
        const auto& ref = std::get<SketchRegionRef>(input);
        return "SketchRegionRef(" + ref.sketchId + "," + ref.regionId + ")";
    }
    if (std::holds_alternative<FaceRef>(input)) {
        const auto& ref = std::get<FaceRef>(input);
        return "FaceRef(" + ref.bodyId + "," + ref.faceId +
               ",patch=" + std::to_string(ref.patchFaceIds.size()) + ")";
    }
    if (std::holds_alternative<BodyRef>(input)) {
        const auto& ref = std::get<BodyRef>(input);
        return "BodyRef(" + ref.bodyId + ")";
    }
    return "None";
}

std::string joinResultBodyIds(const std::vector<std::string>& ids) {
    if (ids.empty()) {
        return "<none>";
    }
    std::string out;
    for (const auto& id : ids) {
        if (!out.empty()) {
            out += ",";
        }
        out += id;
    }
    return out;
}

bool operationCreatesResultBody(const OperationRecord& op) {
    if (op.type == OperationType::Extrude && std::holds_alternative<ExtrudeParams>(op.params)) {
        return std::get<ExtrudeParams>(op.params).booleanMode == BooleanMode::NewBody;
    }
    if (op.type == OperationType::Revolve && std::holds_alternative<RevolveParams>(op.params)) {
        return std::get<RevolveParams>(op.params).booleanMode == BooleanMode::NewBody;
    }
    return false;
}

std::vector<std::string> sortedUniqueStrings(const std::vector<std::string>& values) {
    std::vector<std::string> out = values;
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

bool faceBelongsToBody(const TopoDS_Shape& body, const TopoDS_Face& face) {
    if (body.IsNull() || face.IsNull()) {
        return false;
    }
    for (TopExp_Explorer exp(body, TopAbs_FACE); exp.More(); exp.Next()) {
        if (exp.Current().IsSame(face)) {
            return true;
        }
    }
    return false;
}

bool checkedShapeResult(const TopoDS_Shape& shape,
                        const std::string& context,
                        std::string& errorOut) {
    if (shape.IsNull()) {
        errorOut = context + " produced null shape";
        return false;
    }
    BRepCheck_Analyzer analyzer(shape);
    if (!analyzer.IsValid()) {
        errorOut = context + " produced invalid shape";
        return false;
    }
    return true;
}

TopoDS_Shape checkedBooleanResult(const TopoDS_Shape& target,
                                  const TopoDS_Shape& tool,
                                  BooleanMode mode,
                                  const std::string& context,
                                  std::string& errorOut) {
    if (target.IsNull() || tool.IsNull()) {
        errorOut = context + " boolean input is null";
        return {};
    }

    // OCCT boolean algorithms can raise Standard_Failure from their constructors on
    // degenerate inputs; a boolean failure must never escape as an app crash.
    try {
        TopoDS_Shape result;
        if (mode == BooleanMode::Add) {
            BRepAlgoAPI_Fuse fuse(target, tool);
            if (!fuse.IsDone()) {
                errorOut = context + " fuse failed";
                return {};
            }
            result = fuse.Shape();
        } else if (mode == BooleanMode::Cut) {
            BRepAlgoAPI_Cut cut(target, tool);
            if (!cut.IsDone()) {
                errorOut = context + " cut failed";
                return {};
            }
            result = cut.Shape();
        } else if (mode == BooleanMode::Intersect) {
            BRepAlgoAPI_Common common(target, tool);
            if (!common.IsDone()) {
                errorOut = context + " intersect failed";
                return {};
            }
            result = common.Shape();
        } else {
            errorOut = context + " unsupported boolean mode";
            return {};
        }

        if (!checkedShapeResult(result, context + " boolean", errorOut)) {
            return {};
        }
        return result;
    } catch (const Standard_Failure& failure) {
        errorOut = context + " boolean raised: " +
                   std::string(failure.GetMessageString() ? failure.GetMessageString() : "OCCT failure");
        return {};
    } catch (const std::exception& ex) {
        errorOut = context + " boolean raised: " + std::string(ex.what());
        return {};
    } catch (...) {
        errorOut = context + " boolean raised an unknown exception";
        return {};
    }
}

bool planarFacePlaneAndNormal(const TopoDS_Face& face, gp_Pln& planeOut, gp_Dir& normalOut) {
    try {
        if (face.IsNull()) {
            return false;
        }
        BRepAdaptor_Surface surface(face, true);
        if (surface.GetType() != GeomAbs_Plane) {
            return false;
        }
        planeOut = surface.Plane();
        normalOut = planeOut.Axis().Direction();
        if (face.Orientation() == TopAbs_REVERSED) {
            normalOut.Reverse();
        }
        return true;
    } catch (...) {
        return false;
    }
}

// Nearest planar face of `body` crossed by a ray from `origin` along `dir` (forward).
// Returns the positive distance, or -1 if no face lies ahead. Used for "To Next".
double distanceToNextPlanarFace(const gp_Pnt& origin, const gp_Dir& dir, const TopoDS_Shape& body) {
    double best = -1.0;
    for (TopExp_Explorer exp(body, TopAbs_FACE); exp.More(); exp.Next()) {
        gp_Pln pln;
        gp_Dir n;
        if (!planarFacePlaneAndNormal(TopoDS::Face(exp.Current()), pln, n)) {
            continue;
        }
        const double denom = dir.Dot(n);
        if (std::abs(denom) < 1e-7) {
            continue;  // ray parallel to the face plane
        }
        const double t = gp_Vec(origin, pln.Location()).Dot(gp_Vec(n)) / denom;
        if (t > 1e-4 && (best < 0.0 || t < best)) {
            best = t;
        }
    }
    return best;
}

std::optional<core::modeling::FacePatchSelection> resolveFacePatchSelection(
    const FaceRef& faceRef,
    const TopoDS_Shape& body,
    const kernel::elementmap::ElementMap& elementMap,
    std::string& errorOut,
    bool& usedLegacyFallbackOut) {
    usedLegacyFallbackOut = false;
    if (body.IsNull()) {
        errorOut = "Face owner body is null: " + faceRef.bodyId;
        return std::nullopt;
    }

    if (!faceRef.patchFaceIds.empty()) {
        std::unordered_map<std::string, TopoDS_Face> facesById;
        std::vector<std::string> memberIds = sortedUniqueStrings(faceRef.patchFaceIds);
        facesById.reserve(memberIds.size());

        for (const auto& memberId : memberIds) {
            const auto* entry = elementMap.find(kernel::elementmap::ElementId::From(memberId));
            if (!entry || entry->kind != kernel::elementmap::ElementKind::Face || entry->shape.IsNull()) {
                errorOut = "Face patch member not found: " + memberId;
                return std::nullopt;
            }
            TopoDS_Face memberFace = TopoDS::Face(entry->shape);
            if (!faceBelongsToBody(body, memberFace)) {
                errorOut = "Face patch member not on body: " + memberId;
                return std::nullopt;
            }
            facesById.emplace(memberId, memberFace);
        }

        if (facesById.empty()) {
            errorOut = "Face patch is empty";
            return std::nullopt;
        }

        core::modeling::FacePatchSelection patch;
        patch.memberFaceIds = std::move(memberIds);
        patch.leaderFaceId = patch.memberFaceIds.front();
        patch.memberFaces.reserve(patch.memberFaceIds.size());
        for (const auto& memberId : patch.memberFaceIds) {
            patch.memberFaces.push_back(facesById.at(memberId));
        }
        return patch;
    }

    auto legacyPatch = core::modeling::FacePatchResolver::resolveFromSeedFaceId(
        body, elementMap, faceRef.faceId);
    if (!legacyPatch) {
        errorOut = "Face not found: " + faceRef.faceId;
        return std::nullopt;
    }
    usedLegacyFallbackOut = true;
    return legacyPatch;
}
} // namespace

RegenerationEngine::RegenerationEngine(Document* doc)
    : doc_(doc), graph_() {
    qCDebug(logRegen) << "RegenerationEngine:ctor" << "hasDocument=" << (doc_ != nullptr);
    if (doc_) {
        graph_.rebuildFromOperations(doc_->operations());
        for (const auto& op : doc_->operations()) {
            if (doc_->isOperationSuppressed(op.opId)) {
                graph_.setSuppressed(op.opId, true);
            }
        }
    }
}

RegenResult RegenerationEngine::regenerateAll() {
    if (!doc_) {
        RegenResult result;
        result.status = RegenStatus::CriticalFailure;
        return result;
    }
    return regenerateToAppliedCount(doc_->operations().size());
}

RegenResult RegenerationEngine::regenerateToAppliedCount(std::size_t appliedCount) {
    RegenResult result;
    qCInfo(logRegen) << "regenerateAll:start";

    if (!doc_) {
        qCCritical(logRegen) << "regenerateAll:no-document";
        result.status = RegenStatus::CriticalFailure;
        return result;
    }

    const auto& operations = doc_->operations();
    const std::size_t boundedAppliedCount = std::min(appliedCount, operations.size());
    std::vector<OperationRecord> appliedOps;
    appliedOps.reserve(boundedAppliedCount);
    for (std::size_t i = 0; i < boundedAppliedCount; ++i) {
        appliedOps.push_back(operations[i]);
    }

    // Rebuild dependency graph from current operations
    graph_.rebuildFromOperations(appliedOps);
    for (const auto& op : appliedOps) {
        if (doc_->isOperationSuppressed(op.opId)) {
            graph_.setSuppressed(op.opId, true);
        }
    }
    graph_.clearFailures();
    doc_->clearOperationFailures();

    // Get topological sort order
    std::vector<std::string> order = graph_.topologicalSort();
    if (order.empty() && graph_.size() > 0) {
        // Cycle detected
        qCCritical(logRegen) << "regenerateAll:dependency-cycle-detected"
                             << "graphSize=" << graph_.size();
        result.status = RegenStatus::CriticalFailure;
        return result;
    }

    std::unordered_set<std::string> expectedBodies;
    expectedBodies.reserve(appliedOps.size());
    for (const auto& op : appliedOps) {
        for (const auto& bodyId : op.resultBodyIds) {
            expectedBodies.insert(bodyId);
        }
    }
    for (const auto& bodyId : doc_->baseBodyIds()) {
        expectedBodies.insert(bodyId);
    }

    std::unordered_set<std::string> replayOutputBodyIds;
    for (const auto& op : appliedOps) {
        if (!operationCreatesResultBody(op)) {
            continue;
        }
        for (const auto& bodyId : op.resultBodyIds) {
            if (bodyId.empty() || doc_->isBaseBody(bodyId)) {
                continue;
            }
            replayOutputBodyIds.insert(bodyId);
        }
    }
    qCDebug(logRegen) << "regenerateAll:pre-reset"
                      << "bodyCount=" << replayOutputBodyIds.size();
    for (const auto& bodyId : replayOutputBodyIds) {
        const bool bodyExisted = doc_->getBodyShape(bodyId) != nullptr;
        const bool bodyRemoved = doc_->removeBodyPreserveElementMap(bodyId);
        doc_->elementMap().removeElementsForBody(bodyId);
        qCDebug(logRegen) << "regenerateAll:pre-reset-body"
                          << "bodyId=" << QString::fromStdString(bodyId)
                          << "bodyExisted=" << bodyExisted
                          << "bodyRemoved=" << bodyRemoved;
    }

    // Execute operations in order
    const int total = static_cast<int>(order.size());
    int current = 0;
    std::unordered_set<std::string> updatedBodies;
    for (const auto& bodyId : doc_->baseBodyIds()) {
        updatedBodies.insert(bodyId);
    }

    for (const auto& opId : order) {
        ++current;
        if (progressCallback_) {
            progressCallback_(current, total, opId);
        }

        // Skip suppressed operations
        if (graph_.isSuppressed(opId)) {
            qCDebug(logRegen) << "regenerateAll:skip-suppressed"
                              << QString::fromStdString(opId);
            result.skippedOps.push_back(opId);
            doc_->clearOperationFailed(opId);
            continue;
        }

        // Find the operation record
        const OperationRecord* opRecord = doc_->findOperation(opId);

        if (!opRecord) {
            qCWarning(logRegen) << "regenerateAll:missing-operation-record"
                                << QString::fromStdString(opId);
            result.skippedOps.push_back(opId);
            continue;
        }

        // Check if any upstream dependency failed
        bool upstreamFailed = false;
        for (const auto& upstreamId : graph_.getUpstream(opId)) {
            if (graph_.isFailed(upstreamId)) {
                upstreamFailed = true;
                break;
            }
        }

        if (upstreamFailed) {
            qCWarning(logRegen) << "regenerateAll:skip-upstream-failed"
                                << QString::fromStdString(opId);
            graph_.setFailed(opId, true, "Upstream operation failed");
            doc_->setOperationFailed(opId, "Upstream operation failed");
            result.skippedOps.push_back(opId);
            continue;
        }

        // Execute the operation
        std::string errorMsg;
        bool success = executeOperation(*opRecord, errorMsg);

        if (success) {
            qCDebug(logRegen) << "regenerateAll:operation-succeeded"
                              << QString::fromStdString(opId);
            result.succeededOps.push_back(opId);
            doc_->clearOperationFailed(opId);
            for (const auto& bodyId : opRecord->resultBodyIds) {
                updatedBodies.insert(bodyId);
            }
        } else {
            qCWarning(logRegen) << "regenerateAll:operation-failed"
                                << "opId=" << QString::fromStdString(opId)
                                << "error=" << QString::fromStdString(errorMsg)
                                << "source=" << QString::fromStdString(inputSourceSummary(opRecord->input))
                                << "resultBodies=" << QString::fromStdString(
                                       joinResultBodyIds(opRecord->resultBodyIds));
            graph_.setFailed(opId, true, errorMsg);
            doc_->setOperationFailed(opId, errorMsg);
            FailedOp failedOp;
            failedOp.opId = opId;
            failedOp.type = opRecord->type;
            failedOp.errorMessage = errorMsg;
            failedOp.affectedDownstream = graph_.getDownstream(opId);
            result.failedOps.push_back(std::move(failedOp));
        }
    }

    // Determine overall status
    if (result.failedOps.empty()) {
        result.status = RegenStatus::Success;
    } else if (!result.succeededOps.empty()) {
        result.status = RegenStatus::PartialFailure;
    } else {
        result.status = RegenStatus::CriticalFailure;
    }

    // Remove bodies not produced during this regeneration.
    for (const auto& bodyId : doc_->getBodyIds()) {
        if (expectedBodies.find(bodyId) == expectedBodies.end()) {
            doc_->removeBody(bodyId);
            continue;
        }
        if (updatedBodies.find(bodyId) == updatedBodies.end()) {
            doc_->removeBodyPreserveElementMap(bodyId);
        }
    }

    // Re-derive datum frames from the regenerated geometry: OffsetFromFace /
    // AngledFromEdge datums must follow upstream edits (previously recompute
    // only ran at creation, so their frames went permanently stale).
    for (const auto& datumId : doc_->getDatumPlaneIds()) {
        if (!doc_->recomputeDatumPlane(datumId)) {
            qCWarning(logRegen) << "regenerateAll:datum-recompute-failed"
                                << "datumId=" << QString::fromStdString(datumId);
        }
    }

    qCInfo(logRegen) << "regenerateAll:done"
                     << "status=" << static_cast<int>(result.status)
                     << "succeeded=" << result.succeededOps.size()
                     << "failed=" << result.failedOps.size()
                     << "skipped=" << result.skippedOps.size();

    return result;
}

RegenResult RegenerationEngine::regenerateFrom(const std::string& opId) {
    RegenResult result;

    if (!doc_) {
        result.status = RegenStatus::CriticalFailure;
        return result;
    }

    // Ensure graph is up to date
    graph_.rebuildFromOperations(doc_->operations());
    for (const auto& op : doc_->operations()) {
        if (doc_->isOperationSuppressed(op.opId)) {
            graph_.setSuppressed(op.opId, true);
        }
    }

    // Get ops that need regeneration: the op itself + all downstream
    std::vector<std::string> opsToRegen = {opId};
    auto downstream = graph_.getDownstream(opId);
    opsToRegen.insert(opsToRegen.end(), downstream.begin(), downstream.end());

    // Clear failure states for affected ops
    for (const auto& id : opsToRegen) {
        graph_.setFailed(id, false);
        doc_->clearOperationFailed(id);
    }

    // Get topological order of all ops
    std::vector<std::string> fullOrder = graph_.topologicalSort();
    if (fullOrder.empty() && graph_.size() > 0) {
        result.status = RegenStatus::CriticalFailure;
        return result;
    }

    // Execute only affected ops in topological order
    std::unordered_set<std::string> affectedSet(opsToRegen.begin(), opsToRegen.end());
    const int total = static_cast<int>(opsToRegen.size());
    int current = 0;
    std::unordered_set<std::string> updatedBodies;

    for (const auto& currentOpId : fullOrder) {
        if (affectedSet.find(currentOpId) == affectedSet.end()) {
            continue;
        }

        ++current;
        if (progressCallback_) {
            progressCallback_(current, total, currentOpId);
        }

        if (graph_.isSuppressed(currentOpId)) {
            result.skippedOps.push_back(currentOpId);
            doc_->clearOperationFailed(currentOpId);
            continue;
        }

        const OperationRecord* opRecord = doc_->findOperation(currentOpId);

        if (!opRecord) {
            result.skippedOps.push_back(currentOpId);
            continue;
        }

        bool upstreamFailed = false;
        for (const auto& upId : graph_.getUpstream(currentOpId)) {
            if (graph_.isFailed(upId)) {
                upstreamFailed = true;
                break;
            }
        }

        if (upstreamFailed) {
            graph_.setFailed(currentOpId, true, "Upstream operation failed");
            doc_->setOperationFailed(currentOpId, "Upstream operation failed");
            result.skippedOps.push_back(currentOpId);
            continue;
        }

        std::string errorMsg;
        bool success = executeOperation(*opRecord, errorMsg);

        if (success) {
            result.succeededOps.push_back(currentOpId);
            doc_->clearOperationFailed(currentOpId);
            for (const auto& bodyId : opRecord->resultBodyIds) {
                updatedBodies.insert(bodyId);
            }
        } else {
            graph_.setFailed(currentOpId, true, errorMsg);
            doc_->setOperationFailed(currentOpId, errorMsg);
            FailedOp failedOp;
            failedOp.opId = currentOpId;
            failedOp.type = opRecord->type;
            failedOp.errorMessage = errorMsg;
            failedOp.affectedDownstream = graph_.getDownstream(currentOpId);
            result.failedOps.push_back(std::move(failedOp));
        }
    }

    if (result.failedOps.empty()) {
        result.status = RegenStatus::Success;
    } else if (!result.succeededOps.empty()) {
        result.status = RegenStatus::PartialFailure;
    } else {
        result.status = RegenStatus::CriticalFailure;
    }

    return result;
}

std::optional<TopoDS_Shape> RegenerationEngine::resolveEdge(const std::string& edgeId) const {
    if (!doc_) {
        return std::nullopt;
    }

    const auto* entry = doc_->elementMap().find(kernel::elementmap::ElementId{edgeId});
    if (entry && entry->kind == kernel::elementmap::ElementKind::Edge && !entry->shape.IsNull()) {
        return entry->shape;
    }

    // Broken-reference fallback: re-match a stale edge id by descriptor (see ElementMap).
    double rematchScore = 0.0;
    TopoDS_Shape recovered = doc_->elementMap().resolveWithFallback(
        kernel::elementmap::ElementId{edgeId}, kRematchRejectScore, rematchScore);
    if (!recovered.IsNull()) {
        qCWarning(logRegen) << "resolveEdge:reference-remapped"
                            << "edgeId=" << QString::fromStdString(edgeId)
                            << "score=" << rematchScore;
        return recovered;
    }
    return std::nullopt;
}

std::optional<TopoDS_Shape> RegenerationEngine::resolveFace(const std::string& faceId) const {
    if (!doc_) {
        return std::nullopt;
    }

    const auto* entry = doc_->elementMap().find(kernel::elementmap::ElementId{faceId});
    if (entry && entry->kind == kernel::elementmap::ElementKind::Face && !entry->shape.IsNull()) {
        return entry->shape;
    }

    // Broken-reference fallback: re-match a stale face id by descriptor (see ElementMap).
    double rematchScore = 0.0;
    TopoDS_Shape recovered = doc_->elementMap().resolveWithFallback(
        kernel::elementmap::ElementId{faceId}, kRematchRejectScore, rematchScore);
    if (!recovered.IsNull()) {
        qCWarning(logRegen) << "resolveFace:reference-remapped"
                            << "faceId=" << QString::fromStdString(faceId)
                            << "score=" << rematchScore;
        return recovered;
    }
    return std::nullopt;
}

std::optional<TopoDS_Shape> RegenerationEngine::resolveBody(const std::string& bodyId) const {
    if (!doc_) {
        return std::nullopt;
    }

    const TopoDS_Shape* shape = doc_->getBodyShape(bodyId);
    if (!shape || shape->IsNull()) {
        return std::nullopt;
    }

    return *shape;
}

bool RegenerationEngine::executeOperation(const OperationRecord& op, std::string& errorOut) {
    qCDebug(logRegen) << "executeOperation:start"
                      << "opId=" << QString::fromStdString(op.opId)
                      << "type=" << static_cast<int>(op.type)
                      << "outputs=" << op.resultBodyIds.size();
    // Outer safety net: no OCCT/std exception leaked by any builder (current or
    // future) may escape the regeneration loop — a modeling failure is an op
    // failure, never an app crash.
    try {
    TopoDS_Shape result;

    switch (op.type) {
    case OperationType::Extrude:
        result = buildExtrude(op, errorOut);
        break;
    case OperationType::Revolve:
        result = buildRevolve(op, errorOut);
        break;
    case OperationType::Fillet:
        result = buildFillet(op, errorOut);
        break;
    case OperationType::Chamfer:
        result = buildChamfer(op, errorOut);
        break;
    case OperationType::Shell:
        result = buildShell(op, errorOut);
        break;
    case OperationType::Boolean:
        result = buildBoolean(op, errorOut);
        break;
    case OperationType::LinearPattern:
        result = buildLinearPattern(op, errorOut);
        break;
    case OperationType::CircularPattern:
        result = buildCircularPattern(op, errorOut);
        break;
    case OperationType::Loft:
        result = buildLoft(op, errorOut);
        break;
    case OperationType::Sweep:
        result = buildSweep(op, errorOut);
        break;
    case OperationType::MirrorBody:
        result = buildMirrorBody(op, errorOut);
        break;
    default:
        errorOut = "Unknown operation type";
        return false;
    }

    if (result.IsNull()) {
        if (errorOut.empty()) {
            errorOut = "Operation produced null shape";
        }
        qCWarning(logRegen) << "executeOperation:null-shape"
                            << "opId=" << QString::fromStdString(op.opId)
                            << "error=" << QString::fromStdString(errorOut);
        return false;
    }

    // Apply result to document
    for (const auto& bodyId : op.resultBodyIds) {
        if (!applyBodyResult(bodyId, result, op.opId, errorOut)) {
            qCWarning(logRegen) << "executeOperation:apply-body-failed"
                                << "opId=" << QString::fromStdString(op.opId)
                                << "bodyId=" << QString::fromStdString(bodyId)
                                << "error=" << QString::fromStdString(errorOut);
            return false;
        }
    }

    qCDebug(logRegen) << "executeOperation:done"
                      << "opId=" << QString::fromStdString(op.opId);
    return true;
    } catch (const Standard_Failure& failure) {
        errorOut = "Operation raised: " +
                   std::string(failure.GetMessageString() ? failure.GetMessageString()
                                                          : "OCCT failure");
    } catch (const std::exception& ex) {
        errorOut = "Operation raised: " + std::string(ex.what());
    } catch (...) {
        errorOut = "Operation raised an unknown exception";
    }
    qCCritical(logRegen) << "executeOperation:exception"
                         << "opId=" << QString::fromStdString(op.opId)
                         << "type=" << static_cast<int>(op.type)
                         << "error=" << QString::fromStdString(errorOut);
    return false;
}

TopoDS_Shape RegenerationEngine::buildExtrude(const OperationRecord& op, std::string& errorOut) {
    try {
    if (!std::holds_alternative<ExtrudeParams>(op.params)) {
        errorOut = "Invalid params for extrude";
        return {};
    }

    const auto& params = std::get<ExtrudeParams>(op.params);
    // Only Blind/Symmetric are driven by the literal distance; ThroughAll/ToFace/ToNext
    // compute their own extent, so a zero `distance` is valid for those.
    const bool distanceDriven = !params.twoDirections &&
        (params.extrudeMode == ExtrudeMode::Blind || params.extrudeMode == ExtrudeMode::Symmetric);
    if (distanceDriven && std::abs(params.distance) < kMinValue) {
        errorOut = "Extrude distance too small";
        return {};
    }

    // Extrude is a strictly sketch-region feature (face/body push-pull was removed;
    // the user creates a sketch on the face first — see the auto-sketch-on-face path).
    TopoDS_Face baseFace;
    if (std::holds_alternative<SketchRegionRef>(op.input)) {
        auto faceOpt = resolveFaceInput(op.input, errorOut);
        if (!faceOpt) {
            return {};
        }
        baseFace = *faceOpt;
    } else {
        errorOut = "Extrude requires a sketch region input";
        return {};
    }

    // Direction from the planar profile face.
    gp_Pln plane;
    gp_Dir direction(0.0, 0.0, 1.0);
    if (!planarFacePlaneAndNormal(baseFace, plane, direction)) {
        errorOut = "Only planar faces supported for extrusion";
        return {};
    }

    TopoDS_Shape profileShape = baseFace;

    // Resolve the signed distance to extrude along a reference direction for a given end
    // condition. Blind uses the literal distance; ThroughAll a large extent; ToFace
    // projects onto refDir to reach a target planar face; ToNext stops at the nearest
    // planar face of the target body along refDir. (Planar-target approach — robust and
    // avoids the BRepFeat_MakePrism fragility; curved-target up-to-surface is future work.)
    const gp_Pnt prismOrigin = plane.Location();
    auto effectiveDistance = [&](ExtrudeMode mode, double blind, const std::string& faceTargetId,
                                 const gp_Dir& refDir, std::string& err) -> std::optional<double> {
        switch (mode) {
            case ExtrudeMode::Blind:
                return blind;
            case ExtrudeMode::ThroughAll: {
                // Extent that provably crosses the boolean target: max projection
                // of the target's bbox corners onto refDir, plus a margin. Falls
                // back to a large constant when no target resolves (NewBody
                // through-all keeps prior behavior).
                const double sign = blind >= 0.0 ? 1.0 : -1.0;
                const std::string throughBodyId = resolveBooleanTargetBodyId(op, params.targetBodyId);
                if (!throughBodyId.empty()) {
                    if (auto bodyOpt = resolveBody(throughBodyId)) {
                        Bnd_Box box;
                        BRepBndLib::Add(*bodyOpt, box);
                        if (!box.IsVoid()) {
                            Standard_Real xmin = 0.0, ymin = 0.0, zmin = 0.0;
                            Standard_Real xmax = 0.0, ymax = 0.0, zmax = 0.0;
                            box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
                            double maxProj = 0.0;
                            for (int corner = 0; corner < 8; ++corner) {
                                const gp_Pnt p((corner & 1) ? xmax : xmin,
                                               (corner & 2) ? ymax : ymin,
                                               (corner & 4) ? zmax : zmin);
                                maxProj = std::max(maxProj,
                                                   gp_Vec(prismOrigin, p).Dot(gp_Vec(refDir)));
                            }
                            const double diag = gp_Pnt(xmin, ymin, zmin).Distance(gp_Pnt(xmax, ymax, zmax));
                            return sign * (std::max(maxProj, kMinValue) + 0.01 * diag + 1.0);
                        }
                    }
                }
                qCWarning(logRegen) << "buildExtrude:through-all-fallback-extent"
                                    << "opId=" << QString::fromStdString(op.opId);
                return sign * 1.0e5;
            }
            case ExtrudeMode::ToFace: {
                auto faceOpt = resolveFace(faceTargetId);
                if (!faceOpt) {
                    err = "To Face target face not found: " + faceTargetId;
                    return std::nullopt;
                }
                gp_Pln targetPln;
                gp_Dir targetN;
                if (!planarFacePlaneAndNormal(TopoDS::Face(*faceOpt), targetPln, targetN)) {
                    err = "To Face target face is not planar";
                    return std::nullopt;
                }
                const double d = gp_Vec(prismOrigin, targetPln.Location()).Dot(gp_Vec(refDir));
                if (std::abs(d) < kMinValue) {
                    err = "To Face target coincides with the sketch plane";
                    return std::nullopt;
                }
                return d;
            }
            case ExtrudeMode::ToNext: {
                const std::string nextBodyId = resolveBooleanTargetBodyId(op, params.targetBodyId);
                if (nextBodyId.empty()) {
                    err = "To Next requires an existing target body";
                    return std::nullopt;
                }
                auto bodyOpt = resolveBody(nextBodyId);
                if (!bodyOpt) {
                    err = "To Next target body not found: " + nextBodyId;
                    return std::nullopt;
                }
                const double d = distanceToNextPlanarFace(prismOrigin, refDir, *bodyOpt);
                if (d <= 0.0) {
                    err = "To Next: no face found ahead along the extrude direction";
                    return std::nullopt;
                }
                return d;
            }
            case ExtrudeMode::Symmetric:
                return blind;  // handled separately below
        }
        return blind;
    };

    auto makePrism = [&](const gp_Dir& refDir, double signedDistance, std::string& err) -> TopoDS_Shape {
        gp_Vec prismVec(refDir.X() * signedDistance, refDir.Y() * signedDistance, refDir.Z() * signedDistance);
        BRepPrimAPI_MakePrism prism(profileShape, prismVec, true);
        if (prism.Shape().IsNull()) {
            err = "Extrude prism produced null shape";
        }
        return prism.Shape();
    };

    TopoDS_Shape result;
    if (params.twoDirections) {
        if (params.extrudeMode == ExtrudeMode::Symmetric ||
            params.extrudeMode2 == ExtrudeMode::Symmetric) {
            errorOut = "Symmetric is not valid with two directions";
            return {};
        }
        const gp_Dir dir2 = direction.Reversed();
        auto d1 = effectiveDistance(params.extrudeMode, params.distance, params.targetFaceId,
                                    direction, errorOut);
        if (!d1) {
            return {};
        }
        auto d2 = effectiveDistance(params.extrudeMode2, params.distance2, params.targetFaceId2,
                                    dir2, errorOut);
        if (!d2) {
            return {};
        }
        TopoDS_Shape p1 = makePrism(direction, *d1, errorOut);
        if (p1.IsNull()) {
            return {};
        }
        TopoDS_Shape p2 = makePrism(dir2, *d2, errorOut);
        if (p2.IsNull()) {
            return {};
        }
        BRepAlgoAPI_Fuse fuse(p1, p2);
        fuse.Build();
        if (!fuse.IsDone()) {
            errorOut = "Two-direction extrude fuse failed";
            return {};
        }
        result = fuse.Shape();
    } else if (params.extrudeMode == ExtrudeMode::Symmetric) {
        const double halfDist = params.distance * 0.5;
        gp_Vec fwdVec(direction.X() * halfDist, direction.Y() * halfDist, direction.Z() * halfDist);
        gp_Vec bwdVec = fwdVec.Reversed();
        BRepPrimAPI_MakePrism fwdPrism(profileShape, fwdVec, true);
        BRepPrimAPI_MakePrism bwdPrism(profileShape, bwdVec, true);
        if (fwdPrism.Shape().IsNull() || bwdPrism.Shape().IsNull()) {
            errorOut = "Symmetric extrude prism produced null shape";
            return {};
        }
        BRepAlgoAPI_Fuse fuse(fwdPrism.Shape(), bwdPrism.Shape());
        if (!fuse.IsDone()) {
            errorOut = "Symmetric extrude fuse failed";
            return {};
        }
        result = fuse.Shape();
    } else {
        auto d1 = effectiveDistance(params.extrudeMode, params.distance, params.targetFaceId,
                                    direction, errorOut);
        if (!d1) {
            return {};
        }
        result = makePrism(direction, *d1, errorOut);
        if (result.IsNull()) {
            return {};
        }
    }

    if (result.IsNull()) {
        errorOut = "Extrude prism produced null shape";
        return {};
    }

    // Apply draft angle if specified
    if (std::abs(params.draftAngleDeg) > kDraftAngleEpsilon) {
        const double angleRad = params.draftAngleDeg * M_PI / 180.0;
        gp_Dir draftDir = direction;
        if (params.distance < 0.0) {
            draftDir.Reverse();
        }

        BRepOffsetAPI_DraftAngle draft(result);
        gp_Pln neutralPlane = plane;

        for (TopExp_Explorer exp(result, TopAbs_FACE); exp.More(); exp.Next()) {
            TopoDS_Face face = TopoDS::Face(exp.Current());
            BRepAdaptor_Surface faceSurface(face, true);
            if (faceSurface.GetType() != GeomAbs_Plane) {
                continue;
            }
            gp_Pln facePlane = faceSurface.Plane();
            gp_Dir faceNormal = facePlane.Axis().Direction();
            if (face.Orientation() == TopAbs_REVERSED) {
                faceNormal.Reverse();
            }
            const double dot = std::abs(faceNormal.Dot(draftDir));
            if (dot > kSideFaceDotThreshold) {
                continue;
            }

            draft.Add(face, draftDir, angleRad, neutralPlane, true);
            if (!draft.AddDone()) {
                draft.Remove(face);
            }
        }

        draft.Build();
        if (draft.IsDone()) {
            result = draft.Shape();
        }
    }

    // Handle boolean mode
    if (params.booleanMode != BooleanMode::NewBody) {
        const std::string targetBodyId = resolveBooleanTargetBodyId(op, params.targetBodyId);
        if (targetBodyId.empty()) {
            errorOut = "Boolean target body is required for mode " +
                       std::string(booleanModeName(params.booleanMode));
            qCWarning(logRegen) << "buildExtrude:missing-boolean-target"
                                << "opId=" << QString::fromStdString(op.opId)
                                << "mode=" << static_cast<int>(params.booleanMode);
            return {};
        }

        // Temporal-order guard: forbid targeting a body produced by a later operation.
        if (!graph_.producesBefore(targetBodyId, op.opId)) {
            errorOut = "Boolean target references a body created by a later operation: " + targetBodyId;
            qCWarning(logRegen) << "buildExtrude:boolean-target-time-travel"
                                << "opId=" << QString::fromStdString(op.opId)
                                << "targetBodyId=" << QString::fromStdString(targetBodyId);
            return {};
        }

        auto targetOpt = resolveBody(targetBodyId);
        if (!targetOpt) {
            errorOut = "Target body not found: " + targetBodyId;
            qCWarning(logRegen) << "buildExtrude:boolean-target-not-found"
                                << "opId=" << QString::fromStdString(op.opId)
                                << "targetBodyId=" << QString::fromStdString(targetBodyId);
            return {};
        }

        result = checkedBooleanResult(*targetOpt, result, params.booleanMode, "Extrude", errorOut);
        if (result.IsNull()) {
            return {};
        }
    }

    return result;
    } catch (const Standard_Failure& failure) {
        errorOut = "Extrude operation failed: " + std::string(failure.GetMessageString());
        return {};
    } catch (...) {
        errorOut = "Extrude operation failed";
        return {};
    }
}

TopoDS_Shape RegenerationEngine::buildRevolve(const OperationRecord& op, std::string& errorOut) {
    if (!std::holds_alternative<RevolveParams>(op.params)) {
        errorOut = "Invalid params for revolve";
        return {};
    }

    const auto& params = std::get<RevolveParams>(op.params);
    if (std::abs(params.angleDeg) < kMinAngleDeg) {
        errorOut = "Revolve angle too small";
        return {};
    }

    TopoDS_Face baseFace;
    std::vector<TopoDS_Face> profileFaces;
    bool usedLegacyPatchFallback = false;

    if (std::holds_alternative<SketchRegionRef>(op.input)) {
        auto faceOpt = resolveFaceInput(op.input, errorOut);
        if (!faceOpt) {
            return {};
        }
        baseFace = *faceOpt;
        profileFaces = {baseFace};
    } else if (std::holds_alternative<FaceRef>(op.input)) {
        const auto& faceRef = std::get<FaceRef>(op.input);
        if (faceRef.bodyId.empty()) {
            errorOut = "Face owner body is required";
            return {};
        }

        auto bodyOpt = resolveBody(faceRef.bodyId);
        if (!bodyOpt) {
            errorOut = "Face owner body not found: " + faceRef.bodyId;
            return {};
        }

        auto patch = resolveFacePatchSelection(faceRef, *bodyOpt, doc_->elementMap(), errorOut,
                                               usedLegacyPatchFallback);
        if (!patch) {
            return {};
        }

        profileFaces = patch->memberFaces;
        if (profileFaces.empty()) {
            errorOut = "Face patch is empty";
            return {};
        }

        if (!faceRef.faceId.empty()) {
            auto seedFaceOpt = resolveFace(faceRef.faceId);
            if (seedFaceOpt) {
                baseFace = TopoDS::Face(*seedFaceOpt);
            }
        }
        if (baseFace.IsNull()) {
            baseFace = profileFaces.front();
        }

        qCDebug(logRegen) << "buildRevolve:coplanar-patch"
                          << "opId=" << QString::fromStdString(op.opId)
                          << "seedFaceId=" << QString::fromStdString(faceRef.faceId)
                          << "patchFaces=" << profileFaces.size()
                          << "patchFaceIds=" << patch->memberFaceIds.size()
                          << "legacyFallback=" << usedLegacyPatchFallback;
    } else {
        errorOut = "Invalid input type for revolve";
        return {};
    }

    // Resolve axis
    gp_Ax1 axis;
    bool axisResolved = false;

    if (std::holds_alternative<SketchLineRef>(params.axis)) {
        const auto& lineRef = std::get<SketchLineRef>(params.axis);
        core::sketch::Sketch* sketch = doc_->getSketch(lineRef.sketchId);
        if (!sketch) {
            errorOut = "Sketch not found: " + lineRef.sketchId;
            return {};
        }

        // Get line entity from sketch
        const auto* line = sketch->getEntityAs<core::sketch::SketchLine>(lineRef.lineId);
        if (!line) {
            errorOut = "Axis line not found: " + lineRef.lineId;
            return {};
        }

        // Get endpoints
        const auto* startPt = sketch->getEntityAs<core::sketch::SketchPoint>(line->startPointId());
        const auto* endPt = sketch->getEntityAs<core::sketch::SketchPoint>(line->endPointId());
        if (!startPt || !endPt) {
            errorOut = "Could not resolve axis line endpoints";
            return {};
        }

        const auto& sketchPlane = sketch->getPlane();
        const gp_Pnt2d& p1 = startPt->position();
        const gp_Pnt2d& p2 = endPt->position();

        gp_Pnt origin(sketchPlane.origin.x + p1.X() * sketchPlane.xAxis.x + p1.Y() * sketchPlane.yAxis.x,
                      sketchPlane.origin.y + p1.X() * sketchPlane.xAxis.y + p1.Y() * sketchPlane.yAxis.y,
                      sketchPlane.origin.z + p1.X() * sketchPlane.xAxis.z + p1.Y() * sketchPlane.yAxis.z);

        gp_Vec dir((p2.X() - p1.X()) * sketchPlane.xAxis.x + (p2.Y() - p1.Y()) * sketchPlane.yAxis.x,
                   (p2.X() - p1.X()) * sketchPlane.xAxis.y + (p2.Y() - p1.Y()) * sketchPlane.yAxis.y,
                   (p2.X() - p1.X()) * sketchPlane.xAxis.z + (p2.Y() - p1.Y()) * sketchPlane.yAxis.z);

        if (dir.Magnitude() > 1e-6) {
            axis = gp_Ax1(origin, gp_Dir(dir));
            axisResolved = true;
        }
    } else if (std::holds_alternative<EdgeRef>(params.axis)) {
        const auto& edgeRef = std::get<EdgeRef>(params.axis);
        auto edgeOpt = resolveEdge(edgeRef.edgeId);
        if (!edgeOpt) {
            errorOut = "Axis edge not found: " + edgeRef.edgeId;
            return {};
        }

        TopoDS_Edge edge = TopoDS::Edge(*edgeOpt);
        BRepAdaptor_Curve curve(edge);

        if (curve.GetType() == GeomAbs_Line) {
            gp_Lin line = curve.Line();
            axis = gp_Ax1(line.Location(), line.Direction());
            axisResolved = true;
        } else {
            errorOut = "Axis edge must be a straight line";
            return {};
        }
    }

    if (!axisResolved) {
        errorOut = "Could not resolve revolution axis";
        return {};
    }

    const double angleRad = params.angleDeg * M_PI / 180.0;
    TopoDS_Shape result;
    // BRepPrimAPI_MakeRevol is constructed with Copy=true and raises
    // Standard_ConstructionError from the constructor when the profile touches or
    // crosses the axis — a common user error that must fail the op, not the app.
    try {
        for (const TopoDS_Face& profileFace : profileFaces) {
            if (profileFace.IsNull()) {
                continue;
            }
            BRepPrimAPI_MakeRevol revol(profileFace, axis, angleRad, true);
            if (!revol.IsDone()) {
                errorOut = "Revolve operation failed";
                return {};
            }
            TopoDS_Shape revolShape = revol.Shape();
            if (revolShape.IsNull()) {
                continue;
            }
            if (result.IsNull()) {
                result = revolShape;
                continue;
            }
            BRepAlgoAPI_Fuse fuse(result, revolShape);
            if (!fuse.IsDone()) {
                errorOut = "Revolve patch fuse failed";
                return {};
            }
            result = fuse.Shape();
        }
    } catch (const Standard_Failure& failure) {
        errorOut = "Revolve failed: " +
                   std::string(failure.GetMessageString() ? failure.GetMessageString()
                                                          : "OCCT construction failure");
        return {};
    } catch (const std::exception& ex) {
        errorOut = "Revolve failed: " + std::string(ex.what());
        return {};
    } catch (...) {
        errorOut = "Revolve failed: unknown exception";
        return {};
    }

    if (result.IsNull()) {
        errorOut = "Revolve produced null shape";
        return {};
    }

    if (params.booleanMode != BooleanMode::NewBody) {
        const std::string targetBodyId = resolveBooleanTargetBodyId(op, params.targetBodyId);
        if (targetBodyId.empty()) {
            errorOut = "Boolean target body is required for mode " +
                       std::string(booleanModeName(params.booleanMode));
            qCWarning(logRegen) << "buildRevolve:missing-boolean-target"
                                << "opId=" << QString::fromStdString(op.opId)
                                << "mode=" << static_cast<int>(params.booleanMode);
            return {};
        }

        // Temporal-order guard: forbid targeting a body produced by a later operation.
        if (!graph_.producesBefore(targetBodyId, op.opId)) {
            errorOut = "Boolean target references a body created by a later operation: " + targetBodyId;
            qCWarning(logRegen) << "buildRevolve:boolean-target-time-travel"
                                << "opId=" << QString::fromStdString(op.opId)
                                << "targetBodyId=" << QString::fromStdString(targetBodyId);
            return {};
        }

        auto targetOpt = resolveBody(targetBodyId);
        if (!targetOpt) {
            errorOut = "Target body not found: " + targetBodyId;
            qCWarning(logRegen) << "buildRevolve:boolean-target-not-found"
                                << "opId=" << QString::fromStdString(op.opId)
                                << "targetBodyId=" << QString::fromStdString(targetBodyId);
            return {};
        }

        result = checkedBooleanResult(*targetOpt, result, params.booleanMode, "Revolve", errorOut);
        if (result.IsNull()) {
            return {};
        }
    }

    return result;
}

TopoDS_Shape RegenerationEngine::buildFillet(const OperationRecord& op, std::string& errorOut) {
    if (!std::holds_alternative<FilletChamferParams>(op.params)) {
        errorOut = "Invalid params for fillet";
        return {};
    }

    const auto& params = std::get<FilletChamferParams>(op.params);
    if (params.mode != FilletChamferParams::Mode::Fillet) {
        errorOut = "Expected Fillet mode";
        return {};
    }

    // Get target body
    std::string targetBodyId;
    if (std::holds_alternative<BodyRef>(op.input)) {
        targetBodyId = std::get<BodyRef>(op.input).bodyId;
    } else {
        errorOut = "Fillet requires body input";
        return {};
    }

    auto bodyOpt = resolveBody(targetBodyId);
    if (!bodyOpt) {
        errorOut = "Target body not found: " + targetBodyId;
        return {};
    }

    TopoDS_Shape targetShape = *bodyOpt;

    if (params.radius < kMinValue) {
        errorOut = "Fillet radius too small";
        return {};
    }

    try {
        BRepFilletAPI_MakeFillet fillet(targetShape);
        std::size_t addedEdges = 0;

        for (const auto& edgeId : params.edgeIds) {
            auto edgeOpt = resolveEdge(edgeId);
            if (!edgeOpt) {
                continue;
            }
            TopoDS_Edge edge = TopoDS::Edge(*edgeOpt);
            fillet.Add(params.radius, edge);
            ++addedEdges;
        }

        if (addedEdges == 0) {
            errorOut = "No valid edges for fillet";
            return {};
        }

        fillet.Build();
        if (fillet.IsDone()) {
            return fillet.Shape();
        }

        errorOut = "Fillet operation failed";
    } catch (...) {
        errorOut = "Fillet operation failed (radius too large?)";
    }

    return {};
}

TopoDS_Shape RegenerationEngine::buildChamfer(const OperationRecord& op, std::string& errorOut) {
    if (!std::holds_alternative<FilletChamferParams>(op.params)) {
        errorOut = "Invalid params for chamfer";
        return {};
    }

    const auto& params = std::get<FilletChamferParams>(op.params);
    if (params.mode != FilletChamferParams::Mode::Chamfer) {
        errorOut = "Expected Chamfer mode";
        return {};
    }

    std::string targetBodyId;
    if (std::holds_alternative<BodyRef>(op.input)) {
        targetBodyId = std::get<BodyRef>(op.input).bodyId;
    } else {
        errorOut = "Chamfer requires body input";
        return {};
    }

    auto bodyOpt = resolveBody(targetBodyId);
    if (!bodyOpt) {
        errorOut = "Target body not found: " + targetBodyId;
        return {};
    }

    TopoDS_Shape targetShape = *bodyOpt;

    if (params.radius < kMinValue) {
        errorOut = "Chamfer distance too small";
        return {};
    }

    try {
        BRepFilletAPI_MakeChamfer chamfer(targetShape);
        std::size_t addedEdges = 0;

        TopTools_IndexedDataMapOfShapeListOfShape edgeFaceMap;
        TopExp::MapShapesAndAncestors(targetShape, TopAbs_EDGE, TopAbs_FACE, edgeFaceMap);

        for (const auto& edgeId : params.edgeIds) {
            auto edgeOpt = resolveEdge(edgeId);
            if (!edgeOpt) {
                continue;
            }
            TopoDS_Edge edge = TopoDS::Edge(*edgeOpt);

            int idx = edgeFaceMap.FindIndex(edge);
            if (idx == 0) {
                continue;
            }

            const TopTools_ListOfShape& faces = edgeFaceMap(idx);
            if (faces.IsEmpty()) {
                continue;
            }

            TopoDS_Face refFace = TopoDS::Face(faces.First());
            chamfer.Add(params.radius, params.radius, edge, refFace);
            ++addedEdges;
        }

        if (addedEdges == 0) {
            errorOut = "No valid edges for chamfer";
            return {};
        }

        chamfer.Build();
        if (chamfer.IsDone()) {
            return chamfer.Shape();
        }

        errorOut = "Chamfer operation failed";
    } catch (...) {
        errorOut = "Chamfer operation failed";
    }

    return {};
}

TopoDS_Shape RegenerationEngine::buildShell(const OperationRecord& op, std::string& errorOut) {
    if (!std::holds_alternative<ShellParams>(op.params)) {
        errorOut = "Invalid params for shell";
        return {};
    }

    const auto& params = std::get<ShellParams>(op.params);

    std::string targetBodyId;
    if (std::holds_alternative<BodyRef>(op.input)) {
        targetBodyId = std::get<BodyRef>(op.input).bodyId;
    } else {
        errorOut = "Shell requires body input";
        return {};
    }

    auto bodyOpt = resolveBody(targetBodyId);
    if (!bodyOpt) {
        errorOut = "Target body not found: " + targetBodyId;
        return {};
    }

    TopoDS_Shape targetShape = *bodyOpt;

    if (params.thickness < kMinValue) {
        errorOut = "Shell thickness too small";
        return {};
    }

    try {
        TopTools_ListOfShape facesToRemove;
        const auto openFaceIds = sortedUniqueStrings(params.openFaceIds);
        if (openFaceIds.empty()) {
            errorOut = "No valid faces for shell";
            return {};
        }

        for (const auto& faceId : openFaceIds) {
            auto faceOpt = resolveFace(faceId);
            if (!faceOpt) {
                errorOut = "Open face not found: " + faceId;
                return {};
            }
            TopoDS_Face face = TopoDS::Face(*faceOpt);
            if (!faceBelongsToBody(targetShape, face)) {
                errorOut = "Open face not on target body: " + faceId;
                return {};
            }
            facesToRemove.Append(face);
        }

        BRepOffsetAPI_MakeThickSolid thickSolid;
        thickSolid.MakeThickSolidByJoin(targetShape, facesToRemove, -params.thickness,
                                         1e-3, BRepOffset_Skin, false, false,
                                         GeomAbs_Arc, false);

        if (thickSolid.IsDone()) {
            return thickSolid.Shape();
        }

        errorOut = "Shell operation failed";
    } catch (...) {
        errorOut = "Shell operation failed";
    }

    return {};
}

TopoDS_Shape RegenerationEngine::buildBoolean(const OperationRecord& op, std::string& errorOut) {
    if (!std::holds_alternative<BooleanParams>(op.params)) {
        errorOut = "Invalid params for boolean";
        return {};
    }

    const auto& params = std::get<BooleanParams>(op.params);

    // Temporal-order guard: both operands must come from earlier operations.
    if (!graph_.producesBefore(params.targetBodyId, op.opId)) {
        errorOut = "Boolean target references a body created by a later operation: " +
                   params.targetBodyId;
        return {};
    }
    if (!graph_.producesBefore(params.toolBodyId, op.opId)) {
        errorOut = "Boolean tool references a body created by a later operation: " +
                   params.toolBodyId;
        return {};
    }

    auto targetOpt = resolveBody(params.targetBodyId);
    if (!targetOpt) {
        errorOut = "Target body not found: " + params.targetBodyId;
        return {};
    }

    auto toolOpt = resolveBody(params.toolBodyId);
    if (!toolOpt) {
        errorOut = "Tool body not found: " + params.toolBodyId;
        return {};
    }

    try {
        switch (params.operation) {
        case BooleanParams::Op::Union: {
            BRepAlgoAPI_Fuse fuse(*targetOpt, *toolOpt);
            if (fuse.IsDone()) {
                return fuse.Shape();
            }
            break;
        }
        case BooleanParams::Op::Cut: {
            BRepAlgoAPI_Cut cut(*targetOpt, *toolOpt);
            if (cut.IsDone()) {
                return cut.Shape();
            }
            break;
        }
        case BooleanParams::Op::Intersect: {
            BRepAlgoAPI_Common common(*targetOpt, *toolOpt);
            if (common.IsDone()) {
                return common.Shape();
            }
            break;
        }
        }

        errorOut = "Boolean operation failed";
    } catch (...) {
        errorOut = "Boolean operation failed";
    }

    return {};
}

std::string RegenerationEngine::resolveLegacySketchHostBodyId(const OperationInput& input) const {
    if (!doc_ || !std::holds_alternative<SketchRegionRef>(input)) {
        return {};
    }

    const auto& ref = std::get<SketchRegionRef>(input);
    const core::sketch::Sketch* sketch = doc_->getSketch(ref.sketchId);
    if (!sketch) {
        qCWarning(logRegen) << "resolveLegacySketchHostBodyId:sketch-not-found"
                            << QString::fromStdString(ref.sketchId);
        return {};
    }

    const auto& hostFace = sketch->hostFaceAttachment();
    if (!hostFace || !hostFace->isValid()) {
        qCDebug(logRegen) << "resolveLegacySketchHostBodyId:no-host-face"
                          << QString::fromStdString(ref.sketchId);
        return {};
    }

    qCDebug(logRegen) << "resolveLegacySketchHostBodyId:resolved"
                      << "sketchId=" << QString::fromStdString(ref.sketchId)
                      << "bodyId=" << QString::fromStdString(hostFace->bodyId);
    return hostFace->bodyId;
}

std::string RegenerationEngine::resolveBooleanTargetBodyId(const OperationRecord& op,
                                                           const std::string& explicitTargetBodyId) const {
    if (!explicitTargetBodyId.empty()) {
        qCDebug(logRegen) << "resolveBooleanTargetBodyId:explicit"
                          << "opId=" << QString::fromStdString(op.opId)
                          << "targetBodyId=" << QString::fromStdString(explicitTargetBodyId);
        return explicitTargetBodyId;
    }

    if (std::holds_alternative<FaceRef>(op.input)) {
        const auto& face = std::get<FaceRef>(op.input);
        if (!face.bodyId.empty()) {
            qCDebug(logRegen) << "resolveBooleanTargetBodyId:from-face-input"
                              << "opId=" << QString::fromStdString(op.opId)
                              << "targetBodyId=" << QString::fromStdString(face.bodyId);
            return face.bodyId;
        }
    }

    const std::string legacyBody = resolveLegacySketchHostBodyId(op.input);
    if (!legacyBody.empty()) {
        qCDebug(logRegen) << "resolveBooleanTargetBodyId:from-legacy-host-face"
                          << "opId=" << QString::fromStdString(op.opId)
                          << "targetBodyId=" << QString::fromStdString(legacyBody);
    } else {
        qCDebug(logRegen) << "resolveBooleanTargetBodyId:unresolved"
                          << "opId=" << QString::fromStdString(op.opId);
    }
    return legacyBody;
}

std::optional<TopoDS_Face> RegenerationEngine::resolveFaceInput(const OperationInput& input, std::string& errorOut) {
    if (std::holds_alternative<SketchRegionRef>(input)) {
        const auto& ref = std::get<SketchRegionRef>(input);
        return buildFaceFromSketchRegion(ref.sketchId, ref.regionId, errorOut);
    } else if (std::holds_alternative<FaceRef>(input)) {
        const auto& ref = std::get<FaceRef>(input);
        auto faceOpt = resolveFace(ref.faceId);
        if (!faceOpt) {
            errorOut = "Face not found: " + ref.faceId;
            return std::nullopt;
        }
        return TopoDS::Face(*faceOpt);
    }

    errorOut = "Invalid input type for face resolution";
    return std::nullopt;
}

std::optional<TopoDS_Face> RegenerationEngine::buildFaceFromSketchRegion(const std::string& sketchId,
                                                                          const std::string& regionId,
                                                                          std::string& errorOut) {
    if (!doc_) {
        errorOut = "No document";
        return std::nullopt;
    }

    core::sketch::Sketch* sketch = doc_->getSketch(sketchId);
    if (!sketch) {
        errorOut = "Sketch not found: " + sketchId;
        return std::nullopt;
    }

    auto faceOpt = core::loop::resolveRegionFace(*sketch, regionId);
    if (!faceOpt) {
        errorOut = "Region not found: " + regionId;
        return std::nullopt;
    }

    core::loop::FaceBuilder builder;
    auto faceResult = builder.buildFace(*faceOpt, *sketch);
    if (!faceResult.success) {
        errorOut = faceResult.errorMessage.empty() ? "Face build failed" : faceResult.errorMessage;
        return std::nullopt;
    }

    return faceResult.face;
}

bool RegenerationEngine::applyBodyResult(const std::string& bodyId, const TopoDS_Shape& shape,
                                          const std::string& opId, std::string& errorOut) {
    if (!doc_) {
        errorOut = "Document is not available";
        return false;
    }

    if (doc_->getBodyShape(bodyId)) {
        return doc_->updateBodyShape(bodyId, shape, true, opId, &errorOut);
    }

    if (!doc_->addBodyWithId(bodyId, shape, {}, &errorOut)) {
        return false;
    }
    doc_->elementMap().rebindBody(bodyId, shape, opId);
    return true;
}

TopoDS_Shape RegenerationEngine::buildLinearPattern(const OperationRecord& op, std::string& errorOut) {
    try {
    if (!std::holds_alternative<LinearPatternParams>(op.params)) {
        errorOut = "Invalid params for linear pattern";
        return {};
    }
    const auto& p = std::get<LinearPatternParams>(op.params);

    if (p.count < 2) {
        errorOut = "Pattern count must be >= 2";
        return {};
    }
    if (std::abs(p.spacing) < 1e-9) {
        errorOut = "Pattern spacing must be non-zero";
        return {};
    }

    if (!graph_.producesBefore(p.sourceBodyId, op.opId)) {
        errorOut = "Pattern source references a body created by a later operation: " + p.sourceBodyId;
        return {};
    }
    const auto* sourceShape = doc_->getBodyShape(p.sourceBodyId);
    if (!sourceShape || sourceShape->IsNull()) {
        errorOut = "Source body not found for linear pattern";
        return {};
    }

    // Normalize direction
    double len = std::sqrt(p.dirX * p.dirX + p.dirY * p.dirY + p.dirZ * p.dirZ);
    if (len < 1e-10) {
        errorOut = "Direction vector is zero";
        return {};
    }
    double nx = p.dirX / len;
    double ny = p.dirY / len;
    double nz = p.dirZ / len;

    TopoDS_Shape result = *sourceShape;
    TopoDS_Compound compound;
    BRep_Builder builder;
    if (!p.fuseResult) {
        builder.MakeCompound(compound);
        builder.Add(compound, *sourceShape);
    }
    for (int i = 1; i < p.count; ++i) {
        gp_Trsf trsf;
        trsf.SetTranslation(gp_Vec(nx * p.spacing * i, ny * p.spacing * i, nz * p.spacing * i));
        BRepBuilderAPI_Transform xform(*sourceShape, trsf, true);
        if (!xform.IsDone()) {
            errorOut = "Linear pattern transform failed at instance " + std::to_string(i);
            return {};
        }

        if (p.fuseResult) {
            BRepAlgoAPI_Fuse fuse(result, xform.Shape());
            if (!fuse.IsDone()) {
                errorOut = "Linear pattern fuse failed at instance " + std::to_string(i);
                return {};
            }
            result = fuse.Shape();
        } else {
            builder.Add(compound, xform.Shape());
        }
    }

    return p.fuseResult ? result : compound;
    } catch (const Standard_Failure& e) {
        errorOut = std::string("OCCT error in linear pattern: ") + e.GetMessageString();
        return {};
    }
}

TopoDS_Shape RegenerationEngine::buildLoft(const OperationRecord& op, std::string& errorOut) {
    try {
    if (!std::holds_alternative<LoftParams>(op.params)) {
        errorOut = "Invalid params for loft";
        return {};
    }
    const auto& p = std::get<LoftParams>(op.params);

    if (p.profileSketchIds.size() < 2 ||
        p.profileSketchIds.size() != p.profileRegionIds.size()) {
        errorOut = "Loft requires at least 2 profiles with matching region IDs";
        return {};
    }

    BRepOffsetAPI_ThruSections loft(p.isSolid, p.isRuled);

    for (std::size_t i = 0; i < p.profileSketchIds.size(); ++i) {
        std::string faceErr;
        auto face = buildFaceFromSketchRegion(p.profileSketchIds[i], p.profileRegionIds[i], faceErr);
        if (!face.has_value()) {
            errorOut = "Loft: failed to build profile " + std::to_string(i) + ": " + faceErr;
            return {};
        }

        // Extract outer wire from face
        TopExp_Explorer wireExp(face.value(), TopAbs_WIRE);
        if (wireExp.More()) {
            loft.AddWire(TopoDS::Wire(wireExp.Current()));
        } else {
            errorOut = "Loft: profile " + std::to_string(i) + " has no wire";
            return {};
        }
    }

    loft.Build();
    if (!loft.IsDone()) {
        errorOut = "Loft build failed";
        return {};
    }

    return loft.Shape();
    } catch (const Standard_Failure& e) {
        errorOut = std::string("OCCT error in loft: ") + e.GetMessageString();
        return {};
    }
}

TopoDS_Shape RegenerationEngine::buildSweep(const OperationRecord& op, std::string& errorOut) {
    try {
    if (!std::holds_alternative<SweepParams>(op.params)) {
        errorOut = "Invalid params for sweep";
        return {};
    }
    const auto& p = std::get<SweepParams>(op.params);

    // Build profile face
    std::string faceErr;
    auto profileFace = buildFaceFromSketchRegion(p.profileSketchId, p.profileRegionId, faceErr);
    if (!profileFace.has_value()) {
        errorOut = "Sweep: failed to build profile: " + faceErr;
        return {};
    }

    // Extract profile wire
    TopExp_Explorer wireExp(profileFace.value(), TopAbs_WIRE);
    if (!wireExp.More()) {
        errorOut = "Sweep: profile has no wire";
        return {};
    }
    TopoDS_Wire profileWire = TopoDS::Wire(wireExp.Current());

    // Build path wire from path sketch
    TopoDS_Wire pathWire;
    if (!p.pathEdgeId.empty()) {
        auto edgeOpt = resolveEdge(p.pathEdgeId);
        if (!edgeOpt) {
            errorOut = "Sweep: path edge not found: " + p.pathEdgeId;
            return {};
        }
        TopoDS_Edge edge = TopoDS::Edge(*edgeOpt);
        BRepBuilderAPI_MakeWire wireMaker;
        wireMaker.Add(edge);
        if (!wireMaker.IsDone()) {
            errorOut = "Sweep: failed to build path wire from edge";
            return {};
        }
        pathWire = wireMaker.Wire();
    } else if (!p.pathSketchId.empty()) {
        auto* pathSketch = doc_->getSketch(p.pathSketchId);
        if (!pathSketch) {
            errorOut = "Sweep: path sketch not found";
            return {};
        }
        BRepBuilderAPI_MakeWire wireMaker;
        int edgeCount = 0;
        const auto& pathPlane = pathSketch->getPlane();
        for (const auto& entity : pathSketch->getAllEntities()) {
            if (!entity || entity->isConstruction() || entity->type() == core::sketch::EntityType::Point) {
                continue;
            }
            if (entity->type() != core::sketch::EntityType::Line) {
                errorOut = "Sweep: path sketch supports line entities only";
                return {};
            }
            const auto* line = dynamic_cast<const core::sketch::SketchLine*>(entity.get());
            if (!line) {
                continue;
            }
            const auto* start = pathSketch->getEntityAs<core::sketch::SketchPoint>(line->startPointId());
            const auto* end = pathSketch->getEntityAs<core::sketch::SketchPoint>(line->endPointId());
            if (!start || !end) {
                errorOut = "Sweep: path line has unresolved endpoints";
                return {};
            }
            const auto p1 = pathPlane.toWorld({start->position().X(), start->position().Y()});
            const auto p2 = pathPlane.toWorld({end->position().X(), end->position().Y()});
            BRepBuilderAPI_MakeEdge edgeMaker(gp_Pnt(p1.x, p1.y, p1.z), gp_Pnt(p2.x, p2.y, p2.z));
            if (!edgeMaker.IsDone() || edgeMaker.Edge().IsNull()) {
                errorOut = "Sweep: failed to build path edge";
                return {};
            }
            wireMaker.Add(edgeMaker.Edge());
            if (!wireMaker.IsDone()) {
                errorOut = "Sweep: path sketch edges do not form a wire";
                return {};
            }
            ++edgeCount;
        }
        if (edgeCount == 0) {
            errorOut = "Sweep: path sketch has no usable edges";
            return {};
        }
        pathWire = wireMaker.Wire();
    } else {
        errorOut = "Sweep: path edge or sketch is required";
        return {};
    }

    if (pathWire.IsNull()) {
        errorOut = "Sweep: could not build path wire";
        return {};
    }

    BRepOffsetAPI_MakePipe pipe(pathWire, profileWire);
    if (!pipe.IsDone()) {
        errorOut = "Sweep pipe build failed";
        return {};
    }

    TopoDS_Shape result = pipe.Shape();
    if (result.IsNull()) {
        errorOut = "Sweep pipe produced null shape";
        return {};
    }
    return result;
    } catch (const Standard_Failure& e) {
        errorOut = std::string("OCCT error in sweep: ") + e.GetMessageString();
        return {};
    }
}

TopoDS_Shape RegenerationEngine::buildCircularPattern(const OperationRecord& op, std::string& errorOut) {
    try {
    if (!std::holds_alternative<CircularPatternParams>(op.params)) {
        errorOut = "Invalid params for circular pattern";
        return {};
    }
    const auto& p = std::get<CircularPatternParams>(op.params);

    if (p.count < 2) {
        errorOut = "Pattern count must be >= 2";
        return {};
    }

    if (!graph_.producesBefore(p.sourceBodyId, op.opId)) {
        errorOut = "Pattern source references a body created by a later operation: " + p.sourceBodyId;
        return {};
    }
    const auto* sourceShape = doc_->getBodyShape(p.sourceBodyId);
    if (!sourceShape || sourceShape->IsNull()) {
        errorOut = "Source body not found for circular pattern";
        return {};
    }

    gp_Pnt axisPoint(p.axisX, p.axisY, p.axisZ);
    gp_Dir axisDir(p.axisDirX, p.axisDirY, p.axisDirZ);
    gp_Ax1 axis(axisPoint, axisDir);

    double stepAngle = (p.angleDeg / p.count) * M_PI / 180.0;

    TopoDS_Shape result = *sourceShape;
    TopoDS_Compound compound;
    BRep_Builder builder;
    if (!p.fuseResult) {
        builder.MakeCompound(compound);
        builder.Add(compound, *sourceShape);
    }
    for (int i = 1; i < p.count; ++i) {
        gp_Trsf trsf;
        trsf.SetRotation(axis, stepAngle * i);
        BRepBuilderAPI_Transform xform(*sourceShape, trsf, true);
        if (!xform.IsDone()) {
            errorOut = "Circular pattern transform failed at instance " + std::to_string(i);
            return {};
        }

        TopoDS_Shape instance = xform.Shape();
        if (instance.IsNull()) {
            errorOut = "Circular pattern transform produced null shape at instance " + std::to_string(i);
            return {};
        }

        if (p.fuseResult) {
            BRepAlgoAPI_Fuse fuse(result, instance);
            if (!fuse.IsDone()) {
                errorOut = "Circular pattern fuse failed at instance " + std::to_string(i);
                return {};
            }
            result = fuse.Shape();
            if (result.IsNull()) {
                errorOut = "Circular pattern fuse produced null shape at instance " + std::to_string(i);
                return {};
            }
        } else {
            builder.Add(compound, instance);
        }
    }

    return p.fuseResult ? result : compound;
    } catch (const Standard_Failure& e) {
        errorOut = std::string("OCCT error in circular pattern: ") + e.GetMessageString();
        return {};
    }
}

TopoDS_Shape RegenerationEngine::buildMirrorBody(const OperationRecord& op, std::string& errorOut) {
    try {
    if (!std::holds_alternative<MirrorBodyParams>(op.params)) {
        errorOut = "Invalid params for mirror body";
        return {};
    }
    const auto& p = std::get<MirrorBodyParams>(op.params);

    if (!graph_.producesBefore(p.sourceBodyId, op.opId)) {
        errorOut = "Mirror source references a body created by a later operation: " + p.sourceBodyId;
        return {};
    }
    const auto* sourceShape = doc_->getBodyShape(p.sourceBodyId);
    if (!sourceShape || sourceShape->IsNull()) {
        errorOut = "Source body not found for mirror";
        return {};
    }

    // Build mirror transformation
    gp_Pnt planePoint(p.planePointX, p.planePointY, p.planePointZ);
    gp_Dir planeNormal(p.planeNormalX, p.planeNormalY, p.planeNormalZ);
    gp_Ax2 mirrorPlane(planePoint, planeNormal);

    gp_Trsf mirrorTrsf;
    mirrorTrsf.SetMirror(mirrorPlane);

    BRepBuilderAPI_Transform mirror(*sourceShape, mirrorTrsf, true);
    if (!mirror.IsDone()) {
        errorOut = "Mirror transform failed";
        return {};
    }

    TopoDS_Shape mirrored = mirror.Shape();

    if (p.fuseWithOriginal) {
        BRepAlgoAPI_Fuse fuse(*sourceShape, mirrored);
        if (!fuse.IsDone()) {
            errorOut = "Mirror fuse failed";
            return {};
        }
        return fuse.Shape();
    }

    return mirrored;
    } catch (const Standard_Failure& e) {
        errorOut = std::string("OCCT error in mirror body: ") + e.GetMessageString();
        return {};
    }
}

} // namespace onecad::app::history
