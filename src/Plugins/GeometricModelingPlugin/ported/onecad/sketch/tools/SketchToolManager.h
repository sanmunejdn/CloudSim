/**
 * @file SketchToolManager.h
 * @brief Manages active sketch tool and routes events
 */

#ifndef ONECAD_CORE_SKETCH_TOOLS_SKETCHTOOLMANAGER_H
#define ONECAD_CORE_SKETCH_TOOLS_SKETCHTOOLMANAGER_H

#include "SketchTool.h"
#include "../SnapManager.h"
#include "../AutoConstrainer.h"

#include <QObject>
#include <memory>
#include <unordered_set>

namespace onecad::core::sketch {

class Sketch;
class SketchRenderer;

namespace tools {

/**
 * @brief Tool type enumeration for tool switching
 */
enum class ToolType {
    None,
    Line,
    Circle,
    Rectangle,
    Arc,
    Ellipse,
    Trim,
    Mirror
};

/**
 * @brief Manages sketch tools and routes input events
 *
 * The tool manager owns the active tool instance and forwards
 * mouse/keyboard events to it. It emits signals when geometry
 * is created or tool state changes.
 */
class SketchToolManager : public QObject {
    Q_OBJECT

public:
    explicit SketchToolManager(QObject* parent = nullptr);
    ~SketchToolManager() override;

    /**
     * @brief Set the sketch for tools to operate on
     */
    void setSketch(Sketch* sketch);

    /**
     * @brief Set the renderer for preview drawing
     */
    void setRenderer(SketchRenderer* renderer);

    /**
     * @brief Activate a specific tool type
     */
    void activateTool(ToolType type);

    /**
     * @brief Deactivate current tool
     */
    void deactivateTool();

    /**
     * @brief Get currently active tool type
     */
    ToolType currentToolType() const { return currentType_; }

    /**
     * @brief Check if any tool is active
     */
    bool hasActiveTool() const { return activeTool_ != nullptr; }

    /**
     * @brief Get active tool (may be nullptr)
     */
    SketchTool* activeTool() const { return activeTool_.get(); }

    // Event forwarding methods (called by Viewport)
    void handleMousePress(const Vec2d& pos, Qt::MouseButton button);
    void handleMouseMove(const Vec2d& pos);
    void handleMouseRelease(const Vec2d& pos, Qt::MouseButton button);
    void handleKeyPress(Qt::Key key);

    /**
     * @brief Render tool preview
     */
    void renderPreview();

    // ========== Snap & Auto-Constraint Access ==========

    /**
     * @brief Get current snap result (updated on mouse move)
     */
    const SnapResult& currentSnapResult() const { return currentSnapResult_; }

    /**
     * @brief Get current snapped position (snap point if snapped, raw pos otherwise)
     */
    Vec2d snappedPosition() const {
        return currentSnapResult_.snapped ? currentSnapResult_.position : rawCursorPos_;
    }

    /**
     * @brief Get current inferred constraints (updated on mouse move)
     */
    const std::vector<InferredConstraint>& currentInferredConstraints() const {
        return currentInferredConstraints_;
    }

    /**
     * @brief Get snap manager for configuration
     */
    SnapManager& snapManager() { return snapManager_; }
    const SnapManager& snapManager() const { return snapManager_; }

    /**
     * @brief Get auto-constrainer for configuration
     */
    AutoConstrainer& autoConstrainer() { return autoConstrainer_; }
    const AutoConstrainer& autoConstrainer() const { return autoConstrainer_; }

    /**
     * @brief Set entities to exclude from snapping (e.g., entity being drawn)
     */
    void setExcludeFromSnap(const std::unordered_set<EntityID>& entities) {
        excludeFromSnap_ = entities;
    }

    /**
     * @brief Clear snap exclusion set
     */
    void clearExcludeFromSnap() { excludeFromSnap_.clear(); }

signals:
    /**
     * @brief Emitted when active tool changes
     */
    void toolChanged(ToolType type);

    /**
     * @brief Emitted when geometry is created by a tool
     */
    void geometryCreated();

    /**
     * @brief Emitted when tool needs viewport update
     */
    void updateRequested();

private:
    std::unique_ptr<SketchTool> createTool(ToolType type);

    std::unique_ptr<SketchTool> activeTool_;
    ToolType currentType_ = ToolType::None;
    Sketch* sketch_ = nullptr;
    SketchRenderer* renderer_ = nullptr;

    // Snap & Auto-Constraint
    SnapManager snapManager_;
    AutoConstrainer autoConstrainer_;
    SnapResult currentSnapResult_;
    std::vector<InferredConstraint> currentInferredConstraints_;
    Vec2d rawCursorPos_{0, 0};
    bool hasRawCursorSample_ = false;
    Vec2d previewCursorPos_{0, 0};
    bool hasPreviewCursorSample_ = false;
    bool previewHadGridConflict_ = false;
    std::unordered_set<EntityID> excludeFromSnap_;
};

} // namespace tools
} // namespace onecad::core::sketch

#endif // ONECAD_CORE_SKETCH_TOOLS_SKETCHTOOLMANAGER_H
