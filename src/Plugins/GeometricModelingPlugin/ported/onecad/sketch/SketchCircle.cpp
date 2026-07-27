#include "SketchCircle.h"

#include <QJsonObject>
#include <QLoggingCategory>
#include <QString>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

namespace onecad::core::sketch {

Q_LOGGING_CATEGORY(logSketchCircle, "onecad.core.sketch.circle")

SketchCircle::SketchCircle()
    : SketchEntity() {
}

SketchCircle::SketchCircle(const PointID& centerPointId, double radius)
    : SketchEntity(),
      m_centerPointId(centerPointId),
      m_radius(std::max(0.0, radius)) {
    if (radius < 0.0) {
        qCWarning(logSketchCircle) << "ctor:negative-radius-clamped";
    }
}

gp_Pnt2d SketchCircle::pointAtAngle(const gp_Pnt2d& centerPos, double angle) const {
    return gp_Pnt2d(centerPos.X() + m_radius * std::cos(angle),
                    centerPos.Y() + m_radius * std::sin(angle));
}

gp_Vec2d SketchCircle::tangentAtAngle(double angle) const {
    return gp_Vec2d(-std::sin(angle), std::cos(angle));
}

bool SketchCircle::containsPoint(const gp_Pnt2d& centerPos, const gp_Pnt2d& point) const {
    return centerPos.Distance(point) < m_radius;
}

double SketchCircle::distanceToEdge(const gp_Pnt2d& centerPos, const gp_Pnt2d& point) const {
    return centerPos.Distance(point) - m_radius;
}

BoundingBox2d SketchCircle::bounds() const {
    return {};
}

bool SketchCircle::isNear(const gp_Pnt2d&, double) const {
    return false;
}

void SketchCircle::serialize(QJsonObject& json) const {
    json["id"] = QString::fromStdString(m_id);
    json["type"] = QString::fromStdString(typeName());
    json["construction"] = m_isConstruction;
    json["referenceLocked"] = m_isReferenceLocked;
    json["center"] = QString::fromStdString(m_centerPointId);
    json["radius"] = m_radius;
}

bool SketchCircle::deserialize(const QJsonObject& json) {
    if (!json.contains("center") || !json.contains("radius")) {
        return false;
    }

    if (!json["center"].isString() || !json["radius"].isDouble()) {
        return false;
    }

    if (json.contains("id") && !json["id"].isString()) {
        return false;
    }

    if (json.contains("construction") && !json["construction"].isBool()) {
        return false;
    }
    if (json.contains("referenceLocked") && !json["referenceLocked"].isBool()) {
        qCWarning(logSketchCircle) << "deserialize:invalid-referenceLocked-type";
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
    PointID newCenter = json["center"].toString().toStdString();
    double radius = json["radius"].toDouble();
    if (radius < 0.0) {
        qCWarning(logSketchCircle) << "deserialize:negative-radius-clamped";
        radius = 0.0;
    }

    m_id = std::move(newId);
    m_isConstruction = newConstruction;
    m_isReferenceLocked = newReferenceLocked;
    m_centerPointId = std::move(newCenter);
    m_radius = radius;
    qCDebug(logSketchCircle) << "deserialize:done"
                             << "id=" << QString::fromStdString(m_id)
                             << "referenceLocked=" << m_isReferenceLocked
                             << "center=" << QString::fromStdString(m_centerPointId)
                             << "radius=" << m_radius;
    return true;
}

BoundingBox2d SketchCircle::boundsWithCenter(const gp_Pnt2d& centerPos) const {
    BoundingBox2d box;
    box.minX = centerPos.X() - m_radius;
    box.maxX = centerPos.X() + m_radius;
    box.minY = centerPos.Y() - m_radius;
    box.maxY = centerPos.Y() + m_radius;
    return box;
}

bool SketchCircle::isNearWithCenter(const gp_Pnt2d& testPoint, const gp_Pnt2d& centerPos,
                                    double tolerance) const {
    return std::abs(centerPos.Distance(testPoint) - m_radius) <= tolerance;
}

} // namespace onecad::core::sketch
