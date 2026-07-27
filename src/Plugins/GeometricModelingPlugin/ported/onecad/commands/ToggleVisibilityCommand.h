/**
 * @file ToggleVisibilityCommand.h
 * @brief Command that toggles visibility of a body or sketch with undo support.
 */
#ifndef ONECAD_APP_COMMANDS_TOGGLEVISIBILITYCOMMAND_H
#define ONECAD_APP_COMMANDS_TOGGLEVISIBILITYCOMMAND_H

#include "Command.h"

#include <string>

namespace onecad::app {
class Document;
}

namespace onecad::app::commands {

class ToggleVisibilityCommand : public Command {
public:
    enum class ItemType { Body, Sketch };

    ToggleVisibilityCommand(Document* document,
                            const std::string& itemId,
                            ItemType type,
                            bool visible);

    bool execute() override;
    bool undo() override;
    std::string label() const override { return "Toggle Visibility"; }

private:
    Document* document_ = nullptr;
    std::string itemId_;
    ItemType type_;
    bool newVisible_;
    bool oldVisible_;
};

} // namespace onecad::app::commands

#endif // ONECAD_APP_COMMANDS_TOGGLEVISIBILITYCOMMAND_H
