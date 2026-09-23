/**
 * @file RenameSketchCommand.h
 * @brief Command that renames a sketch with undo support.
 */
#ifndef GEOMETRICMODELINGPLUGIN_RENAMESKETCHCOMMAND_H
#define GEOMETRICMODELINGPLUGIN_RENAMESKETCHCOMMAND_H

#include "Command.h"

#include <string>

namespace onecad::app
{
class Document;
}

namespace onecad::app::commands
{
class RenameSketchCommand : public Command
{
public:
	RenameSketchCommand(Document* document, const std::string& sketchId, const std::string& newName);

	bool execute() override;
	bool undo() override;
	std::string label() const override { return "Rename Sketch"; }

private:
	Document* document_ = nullptr;
	std::string sketchId_;
	std::string newName_;
	std::string oldName_;
};

} // namespace onecad::app::commands

#endif // GEOMETRICMODELINGPLUGIN_RENAMESKETCHCOMMAND_H
