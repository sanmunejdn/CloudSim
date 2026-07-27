/**
 * @file AddBodyCommand.h
 * @brief Command that adds a body to the document.
 */
#ifndef ONECAD_APP_COMMANDS_ADDBODYCOMMAND_H
#define ONECAD_APP_COMMANDS_ADDBODYCOMMAND_H

#include "Command.h"

#include <TopoDS_Shape.hxx>
#include <string>

namespace onecad::app {
class Document;
}

namespace onecad::app::commands {

class AddBodyCommand : public Command {
public:
    AddBodyCommand(Document* document,
                   const TopoDS_Shape& shape,
                   const std::string& bodyId = {},
                   const std::string& bodyName = {});

    bool execute() override;
    bool undo() override;
    std::string label() const override { return "Add Body"; }

    const std::string& bodyId() const { return bodyId_; }
    const std::string& bodyName() const { return bodyName_; }

private:
    Document* document_ = nullptr;
    TopoDS_Shape shape_;
    std::string bodyId_;
    std::string bodyName_;
};

} // namespace onecad::app::commands

#endif // ONECAD_APP_COMMANDS_ADDBODYCOMMAND_H
