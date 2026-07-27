/**
 * @file ImportStepCommand.cpp
 * @brief Undoable STEP import: adds bodies on execute, removes on undo
 */
#include "ImportStepCommand.h"
#include "../document/Document.h"

#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>

namespace onecad::app::commands {

ImportStepCommand::ImportStepCommand(Document* document, std::vector<io::ImportedBody> bodies)
    : document_(document)
    , bodies_(std::move(bodies))
{
}

bool ImportStepCommand::execute() {
    bool isRedo = !createdBodyIds_.empty();
    bodyFaceColors_.clear();

    if (!isRedo) {
        createdBodyIds_.reserve(bodies_.size());
    }

    for (size_t i = 0; i < bodies_.size(); ++i) {
        const auto& body = bodies_[i];
        std::string bodyId;

        if (isRedo) {
            bodyId = createdBodyIds_[i];
            if (!document_->addBodyWithId(bodyId, body.shape, body.name.toStdString())) {
                continue;
            }
        } else {
            bodyId = document_->addBody(body.shape);
            if (bodyId.empty()) {
                continue;
            }
            document_->setBodyName(bodyId, body.name.toStdString());
            createdBodyIds_.push_back(bodyId);
        }

        document_->addBaseBodyId(bodyId);

        // Build face-index → ElementMap-ID mapping and store colors
        if (!body.faceColors.empty()) {
            // Build face index to element ID map
            std::unordered_map<int, std::string> faceIndexToId;
            int faceIdx = 0;
            for (TopExp_Explorer exp(body.shape, TopAbs_FACE); exp.More(); exp.Next(), ++faceIdx) {
                auto ids = document_->elementMap().findIdsByShape(TopoDS::Face(exp.Current()));
                if (!ids.empty()) {
                    faceIndexToId[faceIdx] = ids.front().value;
                }
            }

            Document::FaceColorMap colorMap;
            for (const auto& fc : body.faceColors) {
                auto it = faceIndexToId.find(fc.faceIndex);
                if (it != faceIndexToId.end()) {
                    colorMap[it->second] = fc.rgba;
                }
            }

            if (!colorMap.empty()) {
                document_->setBodyFaceColors(bodyId, colorMap);
                bodyFaceColors_[bodyId] = colorMap;

                // Re-update the mesh to include face colors
                const auto* shape = document_->getBodyShape(bodyId);
                if (shape && !shape->IsNull()) {
                    document_->updateBodyShape(bodyId, *shape, true);
                }
            }
        }
    }

    return !createdBodyIds_.empty();
}

bool ImportStepCommand::undo() {
    for (auto it = createdBodyIds_.rbegin(); it != createdBodyIds_.rend(); ++it) {
        document_->removeBody(*it);
    }
    // Don't clear createdBodyIds_ — redo needs them to restore same IDs
    return true;
}

} // namespace onecad::app::commands
