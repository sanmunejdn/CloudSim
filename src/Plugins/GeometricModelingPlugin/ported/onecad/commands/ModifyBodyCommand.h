#ifndef GEOMETRICMODELINGPLUGIN_MODIFYBODYCOMMAND_H
#define GEOMETRICMODELINGPLUGIN_MODIFYBODYCOMMAND_H

/// @file ModifyBodyCommand.h
/// @brief ModifyBodyCommand 接口

#include "Command.h"

#include <string>

#include <TopoDS_Shape.hxx>

namespace onecad::app
{
class Document;
}

namespace onecad::app::commands
{
class ModifyBodyCommand : public Command
{
public:
	ModifyBodyCommand(Document* document, const std::string& bodyId, const TopoDS_Shape& newShape);

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

#endif // GEOMETRICMODELINGPLUGIN_MODIFYBODYCOMMAND_H
