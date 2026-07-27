#include "SketchPoint.h"

#include <QJsonObject>
#include <QLoggingCategory>
#include <QString>

#include <algorithm>
#include <cmath>
#include <utility>

namespace onecad::core::sketch {

Q_LOGGING_CATEGORY(logSketchPoint, "onecad.core.sketch.point")

SketchPoint::SketchPoint()
    : SketchEntity(),
      m_position(0.0, 0.0) {
}

SketchPoint::SketchPoint(const gp_Pnt2d& position)
    : SketchEntity(),
      m_position(position) {
}

SketchPoint::SketchPoint(double x, double y)
    : SketchEntity(),
      m_position(x, y) {
}

void SketchPoint::addConnectedEntity(const EntityID& entityId) {
    if (std::find(m_connectedEntities.begin(), m_connectedEntities.end(), entityId) != m_connectedEntities.end()) {
        return;
    }
    m_connectedEntities.push_back(entityId);
}

void SketchPoint::removeConnectedEntity(const EntityID& entityId) {
    auto it = std::remove(m_connectedEntities.begin(), m_connectedEntities.end(), entityId);
    if (it != m_connectedEntities.end()) {
        m_connectedEntities.erase(it, m_connectedEntities.end());
    }
}

BoundingBox2d SketchPoint::bounds() const {
    BoundingBox2d box;
    box.minX = m_position.X();
    box.maxX = m_position.X();
    box.minY = m_position.Y();
    box.maxY = m_position.Y();
    return box;
}

bool SketchPoint::isNear(const gp_Pnt2d& point, double tolerance) const {
    return m_position.Distance(point) <= tolerance;
}

void SketchPoint::serialize(QJsonObject& json) const {
    json["id"] = QString::fromStdString(m_id);
    json["type"] = QString::fromStdString(typeName());
    json["construction"] = m_isConstruction;
    json["referenceLocked"] = m_isReferenceLocked;
    json["x"] = m_position.X();
    json["y"] = m_position.Y();
}

bool SketchPoint::deserialize(const QJsonObject& json) {
    if (!json.contains("x") || !json.contains("y")) {
        return false;
    }

    if (!json["x"].isDouble() || !json["y"].isDouble()) {
        return false;
    }

    if (json.contains("id") && !json["id"].isString()) {
        return false;
    }

    if (json.contains("construction") && !json["construction"].isBool()) {
        return false;
    }
    if (json.contains("referenceLocked") && !json["referenceLocked"].isBool()) {
        qCWarning(logSketchPoint) << "deserialize:invalid-referenceLocked-type";
        return false;
    }

    EntityID newId = json.contains("id")
                         ? json["id"].toString().toStdString()
                         : generateId();
    bool newConstruction = json.contains("construction")
                               ? json["construction"].toBool()
                               : m_isConstruction;
    bool newReferenceLocked = json.contains("referenceLocked")
                                  ? json["referenceLocked"].toBool()
                                  : m_isReferenceLocked;
    gp_Pnt2d newPosition(json["x"].toDouble(), json["y"].toDouble());

    m_id = std::move(newId);
    m_isConstruction = newConstruction;
    m_isReferenceLocked = newReferenceLocked;
    m_position = newPosition;
    qCDebug(logSketchPoint) << "deserialize:done"
                            << "id=" << QString::fromStdString(m_id)
                            << "referenceLocked=" << m_isReferenceLocked;
    return true;
}

double SketchPoint::distanceTo(const SketchPoint& other) const {
    return distanceTo(other.m_position);
}

double SketchPoint::distanceTo(const gp_Pnt2d& point) const {
    return m_position.Distance(point);
}

bool SketchPoint::coincidentWith(const SketchPoint& other, double tolerance) const {
    return distanceTo(other) <= tolerance;
}

} // namespace onecad::core::sketch
