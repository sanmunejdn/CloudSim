#ifndef ONECAD_APP_COMMANDS_SKETCHDRAGGESTURECOMMAND_H
#define ONECAD_APP_COMMANDS_SKETCHDRAGGESTURECOMMAND_H

#include "Command.h"

#include <string>

namespace onecad::app {
class Document;
}

namespace onecad::app::commands {

class SketchDragGestureCommand : public Command {
public:
    SketchDragGestureCommand(Document* document, std::string sketchId,
                             std::string label = "Sketch Drag Gesture");

    bool beginGesture();
    bool finalizeGesture();
    void cancelGesture();

    /// Restore the sketch to the begin-snapshot. Used when a tool gesture is
    /// cancelled after it already mutated the sketch (e.g. a line tool's first
    /// clicked point) — the mutation must not survive outside the undo stack.
    bool restoreBeginState();

    bool execute() override;
    bool undo() override;
    std::string label() const override { return label_; }

    [[nodiscard]] bool isReadyForExecution() const { return finalized_; }
    [[nodiscard]] bool hasCapturedChange() const { return changed_; }

private:
    bool snapshotSketchState(std::string* outJson,
                             std::string* outName,
                             bool* outVisible) const;
    bool restoreSketchState(const std::string& json,
                            const std::string& name,
                            bool visible);

    Document* document_ = nullptr;
    std::string sketchId_;
    std::string label_;

    std::string beforeJson_;
    std::string beforeName_;
    bool beforeVisible_ = true;

    std::string afterJson_;
    std::string afterName_;
    bool afterVisible_ = true;

    bool began_ = false;
    bool finalized_ = false;
    bool changed_ = false;
};

}

#endif
