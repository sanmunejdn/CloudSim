#include "SketchEntity.h"

#include <QUuid>

namespace onecad::core::sketch {

SketchEntity::SketchEntity()
    : m_id(generateId()) {
}

SketchEntity::SketchEntity(const EntityID& id)
    : m_id(id.empty() ? generateId() : id) {
}

EntityID SketchEntity::generateId() {
    return QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
}

} // namespace onecad::core::sketch
