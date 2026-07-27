/**
 * @file DeleteBodyCommand.h
 * @brief Command that deletes a body from the document with undo support.
 */
#ifndef ONECAD_APP_COMMANDS_DELETEBODYCOMMAND_H
#define ONECAD_APP_COMMANDS_DELETEBODYCOMMAND_H

#include "Command.h"

#include <TopoDS_Shape.hxx>
#include <string>

namespace onecad::app {
class Document;
}

namespace onecad::app::commands {

class DeleteBodyCommand : public Command {
public:
    DeleteBodyCommand(Document* document, const std::string& bodyId);

    bool execute() override;
    bool undo() override;
    std::string label() const override { return "Delete Body"; }

private:
    Document* document_ = nullptr;
    std::string bodyId_;
    std::string savedName_;
    TopoDS_Shape savedShape_;
    bool savedVisible_ = true;
};

} // namespace onecad::app::commands

#endif // ONECAD_APP_COMMANDS_DELETEBODYCOMMAND_H
