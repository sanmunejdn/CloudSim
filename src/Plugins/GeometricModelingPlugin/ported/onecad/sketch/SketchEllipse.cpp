#include "SketchEllipse.h"

#include <QJsonObject>
#include <QLoggingCategory>
#include <QString>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

namespace onecad::core::sketch {

Q_LOGGING_CATEGORY(logSketchEllipse, "onecad.core.sketch.ellipse")

namespace {
constexpr double TWO_PI = 2.0 * std::numbers::pi;
}

SketchEllipse::SketchEllipse()
    : SketchEntity() {
}

SketchEllipse::SketchEllipse(const PointID& centerPointId, double majorRadius,
                             double minorRadius, double rotation)
    : SketchEntity(),
      m_centerPointId(centerPointId),
      m_majorRadius(std::max(0.0, majorRadius)),
      m_minorRadius(std::max(0.0, minorRadius)),
      m_rotation(rotation) {
    // Enforce major >= minor
    if (m_minorRadius > m_majorRadius) {
        std::swap(m_majorRadius, m_minorRadius);
        m_rotation += std::numbers::pi / 2.0;  // Rotate 90° to maintain orientation
    }
    if (majorRadius < 0.0 || minorRadius < 0.0) {
        qCWarning(logSketchEllipse) << "ctor:negative-radius-clamped";
    }
}

void SketchEllipse::setMajorRadius(double r) {
    m_majorRadius = std::max(0.0, r);
    // Ensure major >= minor: if new major < current minor, raise minor to match
    if (m_majorRadius < m_minorRadius) {
        m_minorRadius = m_majorRadius;
    }
}

void SketchEllipse::setMinorRadius(double r) {
    // Clamp minor to not exceed current major radius
    m_minorRadius = std::clamp(r, 0.0, m_majorRadius);
}

double SketchEllipse::circumference() const {
    // Ramanujan's approximation
    double a = m_majorRadius;
    double b = m_minorRadius;
    double h = ((a - b) * (a - b)) / ((a + b) * (a + b));
    return std::numbers::pi * (a + b) * (1.0 + (3.0 * h) / (10.0 + std::sqrt(4.0 - 3.0 * h)));
}

gp_Pnt2d SketchEllipse::pointAtParameter(const gp_Pnt2d& centerPos, double t) const {
    // Point on unrotated ellipse
    double x = m_majorRadius * std::cos(t);
    double y = m_minorRadius * std::sin(t);

    // Rotate by m_rotation
    double cosR = std::cos(m_rotation);
    double sinR = std::sin(m_rotation);
    double rx = x * cosR - y * sinR;
    double ry = x * sinR + y * cosR;

    return gp_Pnt2d(centerPos.X() + rx, centerPos.Y() + ry);
}

gp_Vec2d SketchEllipse::tangentAtParameter(double t) const {
    // Derivative of parametric ellipse
    double dx = -m_majorRadius * std::sin(t);
    double dy = m_minorRadius * std::cos(t);

    // Rotate tangent
    double cosR = std::cos(m_rotation);
    double sinR = std::sin(m_rotation);
    double rx = dx * cosR - dy * sinR;
    double ry = dx * sinR + dy * cosR;

    // Normalize
    double len = std::sqrt(rx * rx + ry * ry);
    if (len > 1e-10) {
        return gp_Vec2d(rx / len, ry / len);
    }
    return gp_Vec2d(1.0, 0.0);
}

bool SketchEllipse::containsPoint(const gp_Pnt2d& centerPos, const gp_Pnt2d& point) const {
    // Transform point to ellipse-local coordinates
    double dx = point.X() - centerPos.X();
    double dy = point.Y() - centerPos.Y();

    // Rotate back by -m_rotation
    double cosR = std::cos(-m_rotation);
    double sinR = std::sin(-m_rotation);
    double lx = dx * cosR - dy * sinR;
    double ly = dx * sinR + dy * cosR;

    // Check if inside: (x/a)² + (y/b)² < 1
    if (m_majorRadius < 1e-10 || m_minorRadius < 1e-10) {
        return false;
    }
    double nx = lx / m_majorRadius;
    double ny = ly / m_minorRadius;
    return (nx * nx + ny * ny) < 1.0;
}

void SketchEllipse::getFoci(const gp_Pnt2d& centerPos, gp_Pnt2d& focus1, gp_Pnt2d& focus2) const {
    // Distance from center to foci: c = sqrt(a² - b²)
    double c = std::sqrt(std::max(0.0, m_majorRadius * m_majorRadius -
                                        m_minorRadius * m_minorRadius));

    // Foci along major axis
    double cosR = std::cos(m_rotation);
    double sinR = std::sin(m_rotation);

    focus1 = gp_Pnt2d(centerPos.X() + c * cosR, centerPos.Y() + c * sinR);
    focus2 = gp_Pnt2d(centerPos.X() - c * cosR, centerPos.Y() - c * sinR);
}

BoundingBox2d SketchEllipse::bounds() const {
    return {};  // Requires center position
}

bool SketchEllipse::isNear(const gp_Pnt2d&, double) const {
    return false;  // Requires center position
}

BoundingBox2d SketchEllipse::boundsWithCenter(const gp_Pnt2d& centerPos) const {
    // For rotated ellipse, compute bounds by sampling or using parametric extrema
    // Extrema occur where dx/dt = 0 and dy/dt = 0
    // t_x = atan2(-b*sin(θ), a*cos(θ)) and similar for y

    double cosR = std::cos(m_rotation);
    double sinR = std::sin(m_rotation);

    // X extrema: tan(t) = -(b/a)*tan(θ)
    // Simplified: sample at key angles
    double maxX = 0, maxY = 0;
    for (int i = 0; i < 36; ++i) {
        double t = i * TWO_PI / 36.0;
        gp_Pnt2d p = pointAtParameter(centerPos, t);
        maxX = std::max(maxX, std::abs(p.X() - centerPos.X()));
        maxY = std::max(maxY, std::abs(p.Y() - centerPos.Y()));
    }

    BoundingBox2d box;
    box.minX = centerPos.X() - maxX;
    box.maxX = centerPos.X() + maxX;
    box.minY = centerPos.Y() - maxY;
    box.maxY = centerPos.Y() + maxY;
    return box;
}

bool SketchEllipse::isNearWithCenter(const gp_Pnt2d& testPoint, const gp_Pnt2d& centerPos,
                                      double tolerance) const {
    // Transform to local coordinates
    double dx = testPoint.X() - centerPos.X();
    double dy = testPoint.Y() - centerPos.Y();

    double cosR = std::cos(-m_rotation);
    double sinR = std::sin(-m_rotation);
    double lx = dx * cosR - dy * sinR;
    double ly = dx * sinR + dy * cosR;

    // Compute normalized distance from ellipse
    if (m_majorRadius < 1e-10 || m_minorRadius < 1e-10) {
        return false;
    }

    // Approximate: find nearest point on ellipse by parametric search
    // For better accuracy, use Newton-Raphson, but this is sufficient for picking
    double minDist = 1e10;
    for (int i = 0; i < 72; ++i) {
        double t = i * TWO_PI / 72.0;
        double ex = m_majorRadius * std::cos(t);
        double ey = m_minorRadius * std::sin(t);
        double d = std::sqrt((lx - ex) * (lx - ex) + (ly - ey) * (ly - ey));
        minDist = std::min(minDist, d);
    }

    return minDist <= tolerance;
}

void SketchEllipse::serialize(QJsonObject& json) const {
    json["id"] = QString::fromStdString(m_id);
    json["type"] = QString::fromStdString(typeName());
    json["construction"] = m_isConstruction;
    json["referenceLocked"] = m_isReferenceLocked;
    json["center"] = QString::fromStdString(m_centerPointId);
    json["majorRadius"] = m_majorRadius;
    json["minorRadius"] = m_minorRadius;
    json["rotation"] = m_rotation;
}

bool SketchEllipse::deserialize(const QJsonObject& json) {
    if (!json.contains("center") || !json.contains("majorRadius") ||
        !json.contains("minorRadius")) {
        return false;
    }

    if (!json["center"].isString() || !json["majorRadius"].isDouble() ||
        !json["minorRadius"].isDouble()) {
        return false;
    }

    if (json.contains("id") && !json["id"].isString()) {
        return false;
    }

    if (json.contains("construction") && !json["construction"].isBool()) {
        return false;
    }
    if (json.contains("referenceLocked") && !json["referenceLocked"].isBool()) {
        qCWarning(logSketchEllipse) << "deserialize:invalid-referenceLocked-type";
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
    double majorR = json["majorRadius"].toDouble();
    double minorR = json["minorRadius"].toDouble();
    double rot = json.contains("rotation") ? json["rotation"].toDouble() : 0.0;

    if (majorR < 0.0 || minorR < 0.0) {
        qCWarning(logSketchEllipse) << "deserialize:negative-radius-clamped";
        majorR = std::max(0.0, majorR);
        minorR = std::max(0.0, minorR);
    }

    m_id = std::move(newId);
    m_isConstruction = newConstruction;
    m_isReferenceLocked = newReferenceLocked;
    m_centerPointId = std::move(newCenter);
    m_majorRadius = majorR;
    m_minorRadius = minorR;
    m_rotation = rot;

    // Enforce major >= minor
    if (m_minorRadius > m_majorRadius) {
        std::swap(m_majorRadius, m_minorRadius);
        m_rotation += std::numbers::pi / 2.0;
    }

    qCDebug(logSketchEllipse) << "deserialize:done"
                              << "id=" << QString::fromStdString(m_id)
                              << "referenceLocked=" << m_isReferenceLocked
                              << "center=" << QString::fromStdString(m_centerPointId)
                              << "majorRadius=" << m_majorRadius
                              << "minorRadius=" << m_minorRadius;
    return true;
}

} // namespace onecad::core::sketch
