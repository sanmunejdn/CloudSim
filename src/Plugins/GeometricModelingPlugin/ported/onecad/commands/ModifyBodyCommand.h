#ifndef ONECAD_APP_COMMANDS_MODIFYBODYCOMMAND_H
#define ONECAD_APP_COMMANDS_MODIFYBODYCOMMAND_H

#include "Command.h"
#include <TopoDS_Shape.hxx>
#include <string>

namespace onecad::app {
class Document;
}

namespace onecad::app::commands {

class ModifyBodyCommand : public Command {
public:
    ModifyBodyCommand(Document* document,
                      const std::string& bodyId,
                      const TopoDS_Shape& newShape);

    bool execute() override;
    bool undo() override;
    std::string label() const override { return "Modify Body"; }

private:
    Document* document_ = nullptr;
    std::string bodyId_;
    TopoDS_Shape newShape_;
    TopoDS_Shape oldShape_;
};

} // namespace onecad::app::commands

#endif // ONECAD_APP_COMMANDS_MODIFYBODYCOMMAND_H
