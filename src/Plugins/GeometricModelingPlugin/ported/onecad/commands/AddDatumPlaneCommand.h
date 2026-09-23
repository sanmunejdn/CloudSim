/**
 * @file AddDatumPlaneCommand.h
 * @brief Undoable creation of a datum (reference) plane.
 */
#ifndef GEOMETRICMODELINGPLUGIN_ADDDATUMPLANECOMMAND_H
#define GEOMETRICMODELINGPLUGIN_ADDDATUMPLANECOMMAND_H

#include "../document/DatumPlane.h"
#include "Command.h"

namespace onecad::app
{
class Document;
}

namespace onecad::app::commands
{
class AddDatumPlaneCommand : public Command
{
public:
	AddDatumPlaneCommand(Document* document, DatumPlane datum);

	bool execute() override;
	bool undo() override;
	std::string label() const override { return "Create Datum Plane"; }

	const std::string& datumId() const { return datum_.id; }

private:
	Document* document_ = nullptr;
	DatumPlane datum_;
};

} // namespace onecad::app::commands

#endif // GEOMETRICMODELINGPLUGIN_ADDDATUMPLANECOMMAND_H
