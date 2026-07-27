/**
 * @file SketchRenderer.cpp
 * @brief OpenGL-based sketch rendering implementation
 *
 * Renders sketch geometry (lines, arcs, circles, points) and constraints.
 * Uses VBO batching for performance with adaptive arc tessellation.
 */
#include "SketchRenderer.h"

#include "Sketch.h"
#include "SketchArc.h"
#include "SketchCircle.h"
#include "SketchEllipse.h"
#include "SketchLine.h"
#include "SketchPoint.h"
#include "../loop/FaceBuilder.h"
#include "../loop/LoopDetector.h"
#include "../loop/RegionUtils.h"

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <TopLoc_Location.hxx>
#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QVector3D>
#include <QVector4D>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <unordered_map>
#include <unordered_set>

namespace onecad::core::sketch {

namespace loop = ::onecad::core::loop;

namespace {

// GLSL 410 core for macOS Metal compatibility
constexpr const char* kLineVertexShader = R"(
#version 410 core
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec4 aColor;

uniform mat4 uMVP;

out vec4 vColor;

void main() {
    gl_Position = uMVP * vec4(aPos, 0.0, 1.0);
    vColor = aColor;
}
)";

constexpr const char* kLineFragmentShader = R"(
#version 410 core
in vec4 vColor;
out vec4 FragColor;

void main() {
    FragColor = vColor;
}
)";

constexpr const char* kPointVertexShader = R"(
#version 410 core
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec4 aColor;
layout(location = 2) in float aSize;

uniform mat4 uMVP;

out vec4 vColor;

void main() {
    gl_Position = uMVP * vec4(aPos, 0.0, 1.0);
    gl_PointSize = aSize;
    vColor = aColor;
}
)";

constexpr const char* kPointFragmentShader = R"(
#version 410 core
in vec4 vColor;
out vec4 FragColor;

void main() {
    // Circular point with smooth edges
    vec2 coord = gl_PointCoord - vec2(0.5);
    float dist = length(coord);
    if (dist > 0.5) discard;

    float alpha = 1.0 - smoothstep(0.4, 0.5, dist);
    FragColor = vec4(vColor.rgb, vColor.a * alpha);
}
)";

Vec3d colorForState(SelectionState state, bool isConstruction, bool hasError,
                    const SketchColors& colors) {
    if (hasError) {
        return colors.errorGeometry;
    }
    switch (state) {
        case SelectionState::Selected:
        case SelectionState::Dragging:
            return colors.selectedGeometry;
        case SelectionState::Hover:
            return {colors.selectedGeometry.x * 0.7 + colors.normalGeometry.x * 0.3,
                    colors.selectedGeometry.y * 0.7 + colors.normalGeometry.y * 0.3,
                    colors.selectedGeometry.z * 0.7 + colors.normalGeometry.z * 0.3};
        default:
            return isConstruction ? colors.constructionGeometry : colors.normalGeometry;
    }
}

float lineWidthForState(SelectionState state, bool isConstruction,
                        const SketchRenderStyle& style) {
    switch (state) {
        case SelectionState::Selected:
        case SelectionState::Dragging:
            return style.selectedLineWidth;
        case SelectionState::Hover:
            return (style.normalLineWidth + style.selectedLineWidth) / 2.0f;
        default:
            return isConstruction ? style.constructionLineWidth : style.normalLineWidth;
    }
}

Vec3d snapColorForType(SnapType type) {
    switch (type) {
        case SnapType::Vertex:
        case SnapType::Endpoint:
        case SnapType::Midpoint:
        case SnapType::Center:
            return {0.2, 0.8, 0.3};
        case SnapType::Quadrant:
        case SnapType::Intersection:
        case SnapType::OnCurve:
            return {0.2, 0.7, 0.9};
        case SnapType::Perpendicular:
        case SnapType::Tangent:
        case SnapType::SketchGuide:
            return {0.9, 0.5, 0.1};
        case SnapType::Horizontal:
        case SnapType::Vertical:
            return {0.3, 0.5, 0.9};
        case SnapType::Grid:
            return {0.6, 0.6, 0.6};
        default:
            return {0.9, 0.5, 0.1};
    }
}

void appendSegment(std::vector<float>& data, const Vec2d& p1, const Vec2d& p2, const Vec3d& color) {
    data.push_back(static_cast<float>(p1.x));
    data.push_back(static_cast<float>(p1.y));
    data.push_back(static_cast<float>(color.x));
    data.push_back(static_cast<float>(color.y));
    data.push_back(static_cast<float>(color.z));
    data.push_back(1.0f);

    data.push_back(static_cast<float>(p2.x));
    data.push_back(static_cast<float>(p2.y));
    data.push_back(static_cast<float>(color.x));
    data.push_back(static_cast<float>(color.y));
    data.push_back(static_cast<float>(color.z));
    data.push_back(1.0f);
}

void appendSolidPolyline(std::vector<float>& data, const std::vector<Vec2d>& vertices,
                         const Vec3d& color) {
    if (vertices.size() < 2) {
        return;
    }
    for (size_t i = 0; i + 1 < vertices.size(); ++i) {
        appendSegment(data, vertices[i], vertices[i + 1], color);
    }
}

void appendDashedPolyline(std::vector<float>& data, const std::vector<Vec2d>& vertices,
                          const Vec3d& color, double dashLength, double gapLength) {
    if (vertices.size() < 2) {
        return;
    }
    if (dashLength <= 0.0 || gapLength < 0.0) {
        appendSolidPolyline(data, vertices, color);
        return;
    }

    double patternLength = dashLength + gapLength;
    if (patternLength <= 0.0) {
        appendSolidPolyline(data, vertices, color);
        return;
    }

    double patternPos = 0.0;
    for (size_t i = 0; i + 1 < vertices.size(); ++i) {
        const auto& p1 = vertices[i];
        const auto& p2 = vertices[i + 1];
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double segLen = std::hypot(dx, dy);
        if (segLen < 1e-12) {
            continue;
        }
        double invLen = 1.0 / segLen;
        double segPos = 0.0;

        while (segPos < segLen) {
            double remainingInPattern = patternLength - patternPos;
            double step = std::min(remainingInPattern, segLen - segPos);

            if (patternPos < dashLength && step > 0.0) {
                double startT = segPos * invLen;
                double endT = (segPos + step) * invLen;
                Vec2d start{p1.x + dx * startT, p1.y + dy * startT};
                Vec2d end{p1.x + dx * endT, p1.y + dy * endT};
                appendSegment(data, start, end, color);
            }

            segPos += step;
            patternPos += step;
            if (patternPos >= patternLength) {
                patternPos = std::fmod(patternPos, patternLength);
            }
        }
    }
}

constexpr double kGeometryEpsilon = 1e-9;

double cross2d(const Vec2d& a, const Vec2d& b, const Vec2d& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

double distanceSquared(const Vec2d& a, const Vec2d& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return dx * dx + dy * dy;
}

double polygonSignedArea(const std::vector<Vec2d>& polygon) {
    if (polygon.size() < 3) {
        return 0.0;
    }
    double area = 0.0;
    for (size_t i = 0; i < polygon.size(); ++i) {
        const auto& p1 = polygon[i];
        const auto& p2 = polygon[(i + 1) % polygon.size()];
        area += p1.x * p2.y - p2.x * p1.y;
    }
    return 0.5 * area;
}

std::vector<Vec2d> normalizePolygon(const std::vector<Vec2d>& polygon, double tolerance) {
    std::vector<Vec2d> cleaned;
    if (polygon.empty()) {
        return cleaned;
    }
    cleaned.reserve(polygon.size());
    double tol2 = tolerance * tolerance;
    for (const auto& p : polygon) {
        if (cleaned.empty() || distanceSquared(cleaned.back(), p) > tol2) {
            cleaned.push_back(p);
        }
    }
    if (cleaned.size() > 1 && distanceSquared(cleaned.front(), cleaned.back()) <= tol2) {
        cleaned.pop_back();
    }

    bool removed = true;
    while (removed && cleaned.size() >= 3) {
        removed = false;
        for (size_t i = 0; i < cleaned.size(); ++i) {
            size_t prev = (i + cleaned.size() - 1) % cleaned.size();
            size_t next = (i + 1) % cleaned.size();
            if (std::abs(cross2d(cleaned[prev], cleaned[i], cleaned[next])) <= tolerance) {
                cleaned.erase(cleaned.begin() + static_cast<long>(i));
                removed = true;
                break;
            }
        }
    }

    return cleaned;
}

bool isPointInPolygon(const Vec2d& point, const std::vector<Vec2d>& polygon) {
    if (polygon.size() < 3) {
        return false;
    }
    bool inside = false;
    for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        const auto& pi = polygon[i];
        const auto& pj = polygon[j];
        bool intersect = ((pi.y > point.y) != (pj.y > point.y)) &&
                         (point.x < (pj.x - pi.x) * (point.y - pi.y) / (pj.y - pi.y) + pi.x);
        if (intersect) {
            inside = !inside;
        }
    }
    return inside;
}

bool pointsCoincident(const Vec2d& a, const Vec2d& b, double tolerance) {
    return distanceSquared(a, b) <= tolerance * tolerance;
}

bool segmentsIntersect(const Vec2d& a1, const Vec2d& a2,
                       const Vec2d& b1, const Vec2d& b2) {
    auto cross = [](const Vec2d& a, const Vec2d& b, const Vec2d& c) {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    };

    auto onSegment = [](const Vec2d& a, const Vec2d& b, const Vec2d& c) {
        return std::min(a.x, b.x) <= c.x && c.x <= std::max(a.x, b.x) &&
               std::min(a.y, b.y) <= c.y && c.y <= std::max(a.y, b.y);
    };

    double d1 = cross(a1, a2, b1);
    double d2 = cross(a1, a2, b2);
    double d3 = cross(b1, b2, a1);
    double d4 = cross(b1, b2, a2);

    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
        ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
        return true;
    }

    if (d1 == 0.0 && onSegment(a1, a2, b1)) return true;
    if (d2 == 0.0 && onSegment(a1, a2, b2)) return true;
    if (d3 == 0.0 && onSegment(b1, b2, a1)) return true;
    if (d4 == 0.0 && onSegment(b1, b2, a2)) return true;

    return false;
}

bool segmentIntersectsPolygon(const Vec2d& a, const Vec2d& b,
                              const std::vector<Vec2d>& polygon,
                              double tolerance) {
    if (polygon.size() < 2) {
        return false;
    }
    for (size_t i = 0; i < polygon.size(); ++i) {
        size_t next = (i + 1) % polygon.size();
        const auto& p1 = polygon[i];
        const auto& p2 = polygon[next];
        if (pointsCoincident(p1, a, tolerance) || pointsCoincident(p2, a, tolerance) ||
            pointsCoincident(p1, b, tolerance) || pointsCoincident(p2, b, tolerance)) {
            continue;
        }
        if (segmentsIntersect(a, b, p1, p2)) {
            return true;
        }
    }
    return false;
}

bool polygonsIntersect(const std::vector<Vec2d>& poly1,
                       const std::vector<Vec2d>& poly2) {
    if (poly1.empty() || poly2.empty()) {
        return false;
    }
    for (size_t i = 0; i < poly1.size(); ++i) {
        size_t iNext = (i + 1) % poly1.size();
        for (size_t j = 0; j < poly2.size(); ++j) {
            size_t jNext = (j + 1) % poly2.size();
            if (segmentsIntersect(poly1[i], poly1[iNext], poly2[j], poly2[jNext])) {
                return true;
            }
        }
    }
    return false;
}

bool pointInTriangle(const Vec2d& p, const Vec2d& a, const Vec2d& b, const Vec2d& c,
                     double tolerance) {
    double c1 = cross2d(a, b, p);
    double c2 = cross2d(b, c, p);
    double c3 = cross2d(c, a, p);
    return c1 >= -tolerance && c2 >= -tolerance && c3 >= -tolerance;
}

bool triangulateSimplePolygon(const std::vector<Vec2d>& polygonInput,
                              std::vector<Vec2d>& outTriangles) {
    outTriangles.clear();
    std::vector<Vec2d> polygon = normalizePolygon(polygonInput, kGeometryEpsilon);
    if (polygon.size() < 3) {
        return false;
    }

    if (polygonSignedArea(polygon) < 0.0) {
        std::reverse(polygon.begin(), polygon.end());
    }

    std::vector<int> indices(polygon.size());
    for (size_t i = 0; i < polygon.size(); ++i) {
        indices[i] = static_cast<int>(i);
    }

    int guard = 0;
    int maxIterations = static_cast<int>(polygon.size() * polygon.size());
    while (indices.size() > 2 && guard < maxIterations) {
        bool earFound = false;
        for (size_t i = 0; i < indices.size(); ++i) {
            int iPrev = indices[(i + indices.size() - 1) % indices.size()];
            int iCurr = indices[i];
            int iNext = indices[(i + 1) % indices.size()];
            const auto& a = polygon[iPrev];
            const auto& b = polygon[iCurr];
            const auto& c = polygon[iNext];

            if (cross2d(a, b, c) <= kGeometryEpsilon) {
                continue;
            }

            bool hasPoint = false;
            for (int idx : indices) {
                if (idx == iPrev || idx == iCurr || idx == iNext) {
                    continue;
                }
                if (pointInTriangle(polygon[idx], a, b, c, kGeometryEpsilon)) {
                    hasPoint = true;
                    break;
                }
            }
            if (hasPoint) {
                continue;
            }

            outTriangles.push_back(a);
            outTriangles.push_back(b);
            outTriangles.push_back(c);
            indices.erase(indices.begin() + static_cast<long>(i));
            earFound = true;
            break;
        }
        if (!earFound) {
            break;
        }
        ++guard;
    }

    return outTriangles.size() >= 3;
}

int findBridgeVertex(const std::vector<Vec2d>& polygon, const Vec2d& holePoint) {
    double bestDist = std::numeric_limits<double>::infinity();
    int bestIndex = -1;
    for (size_t i = 0; i < polygon.size(); ++i) {
        const auto& candidate = polygon[i];
        if (segmentIntersectsPolygon(holePoint, candidate, polygon, kGeometryEpsilon)) {
            continue;
        }
        double dist = distanceSquared(holePoint, candidate);
        if (dist < bestDist) {
            bestDist = dist;
            bestIndex = static_cast<int>(i);
        }
    }
    return bestIndex;
}

std::vector<Vec2d> mergePolygons(const std::vector<Vec2d>& outer,
                                 int outerIndex,
                                 const std::vector<Vec2d>& hole,
                                 int holeIndex) {
    std::vector<Vec2d> merged;
    if (outer.empty() || hole.empty()) {
        return merged;
    }
    merged.reserve(outer.size() + hole.size() + 2);

    for (int i = 0; i <= outerIndex; ++i) {
        merged.push_back(outer[i]);
    }

    for (size_t i = 0; i < hole.size(); ++i) {
        merged.push_back(hole[(static_cast<size_t>(holeIndex) + i) % hole.size()]);
    }

    merged.push_back(outer[outerIndex]);

    for (size_t i = static_cast<size_t>(outerIndex + 1); i < outer.size(); ++i) {
        merged.push_back(outer[i]);
    }

    return merged;
}

bool triangulatePolygonWithHoles(const std::vector<Vec2d>& outerInput,
                                 const std::vector<std::vector<Vec2d>>& holesInput,
                                 std::vector<Vec2d>& outTriangles) {
    outTriangles.clear();
    std::vector<Vec2d> outer = normalizePolygon(outerInput, kGeometryEpsilon);
    if (outer.size() < 3) {
        return false;
    }

    if (polygonSignedArea(outer) < 0.0) {
        std::reverse(outer.begin(), outer.end());
    }

    std::vector<Vec2d> merged = outer;
    for (const auto& holeRaw : holesInput) {
        std::vector<Vec2d> hole = normalizePolygon(holeRaw, kGeometryEpsilon);
        if (hole.size() < 3) {
            continue;
        }

        if (polygonSignedArea(hole) > 0.0) {
            std::reverse(hole.begin(), hole.end());
        }

        int holeIndex = 0;
        for (size_t i = 1; i < hole.size(); ++i) {
            if (hole[i].x > hole[static_cast<size_t>(holeIndex)].x ||
                (hole[i].x == hole[static_cast<size_t>(holeIndex)].x &&
                 hole[i].y > hole[static_cast<size_t>(holeIndex)].y)) {
                holeIndex = static_cast<int>(i);
            }
        }
        Vec2d holePoint = hole[static_cast<size_t>(holeIndex)];

        std::vector<Vec2d> holeCCW = hole;
        std::reverse(holeCCW.begin(), holeCCW.end());
        int holeIndexCCW = static_cast<int>(holeCCW.size()) - 1 - holeIndex;

        int bridgeIndex = findBridgeVertex(merged, holePoint);
        if (bridgeIndex < 0) {
            return false;
        }

        merged = mergePolygons(merged, bridgeIndex, holeCCW, holeIndexCCW);
        merged = normalizePolygon(merged, kGeometryEpsilon);
        if (merged.size() < 3) {
            return false;
        }
    }

    return triangulateSimplePolygon(merged, outTriangles);
}

bool polygonContainsPolygon(const std::vector<Vec2d>& outer,
                            const std::vector<Vec2d>& inner,
                            double tolerance) {
    if (outer.size() < 3 || inner.size() < 3) {
        return false;
    }
    Vec2d outerMin = outer.front();
    Vec2d outerMax = outer.front();
    for (const auto& p : outer) {
        outerMin.x = std::min(outerMin.x, p.x);
        outerMin.y = std::min(outerMin.y, p.y);
        outerMax.x = std::max(outerMax.x, p.x);
        outerMax.y = std::max(outerMax.y, p.y);
    }
    for (const auto& p : inner) {
        if (p.x < outerMin.x - tolerance || p.y < outerMin.y - tolerance ||
            p.x > outerMax.x + tolerance || p.y > outerMax.y + tolerance) {
            return false;
        }
    }
    for (const auto& p : inner) {
        if (!isPointInPolygon(p, outer)) {
            return false;
        }
    }
    if (polygonsIntersect(outer, inner)) {
        return false;
    }
    return true;
}

bool buildTrianglesFromFace(const loop::Face& faceDef,
                            const Sketch& sketch,
                            std::vector<Vec2d>& outTriangles) {
    outTriangles.clear();

    loop::FaceBuilder builder;
    const auto faceResult = builder.buildFace(faceDef, sketch);
    if (!faceResult.success || faceResult.face.IsNull()) {
        return false;
    }

    BRepMesh_IncrementalMesh mesher(faceResult.face, 0.05, false, 0.5, true);
    mesher.Perform();
    if (!mesher.IsDone()) {
        return false;
    }

    TopLoc_Location location;
    Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(faceResult.face, location);
    if (triangulation.IsNull()) {
        return false;
    }

    const gp_Trsf& transform = location.Transformation();
    const auto& plane = sketch.getPlane();
    outTriangles.reserve(static_cast<size_t>(triangulation->NbTriangles()) * 3);

    for (int i = 1; i <= triangulation->NbTriangles(); ++i) {
        int n1 = 0;
        int n2 = 0;
        int n3 = 0;
        triangulation->Triangle(i).Get(n1, n2, n3);

        const gp_Pnt p1 = triangulation->Node(n1).Transformed(transform);
        const gp_Pnt p2 = triangulation->Node(n2).Transformed(transform);
        const gp_Pnt p3 = triangulation->Node(n3).Transformed(transform);

        outTriangles.push_back(plane.toSketch({p1.X(), p1.Y(), p1.Z()}));
        outTriangles.push_back(plane.toSketch({p2.X(), p2.Y(), p2.Z()}));
        outTriangles.push_back(plane.toSketch({p3.X(), p3.Y(), p3.Z()}));
    }

    return !outTriangles.empty();
}

QMatrix4x4 buildSketchModelMatrix(const SketchPlane& plane) {
    QVector3D origin(plane.origin.x, plane.origin.y, plane.origin.z);
    QVector3D normal(plane.normal.x, plane.normal.y, plane.normal.z);
    QVector3D xAxis(plane.xAxis.x, plane.xAxis.y, plane.xAxis.z);
    QVector3D yAxis(plane.yAxis.x, plane.yAxis.y, plane.yAxis.z);

    if (normal.lengthSquared() < 1e-12f) {
        normal = QVector3D::crossProduct(xAxis, yAxis);
    }
    if (normal.lengthSquared() < 1e-12f) {
        normal = QVector3D(0.0f, 0.0f, 1.0f);
    }
    normal.normalize();

    if (xAxis.lengthSquared() < 1e-12f) {
        xAxis = QVector3D::crossProduct(yAxis, normal);
    }
    if (xAxis.lengthSquared() < 1e-12f) {
        xAxis = (std::abs(normal.z()) < 0.9f) ? QVector3D::crossProduct(normal, QVector3D(0, 0, 1))
                                              : QVector3D::crossProduct(normal, QVector3D(0, 1, 0));
    }

    xAxis -= normal * QVector3D::dotProduct(normal, xAxis);
    if (xAxis.lengthSquared() < 1e-12f) {
        xAxis = (std::abs(normal.z()) < 0.9f) ? QVector3D::crossProduct(normal, QVector3D(0, 0, 1))
                                              : QVector3D::crossProduct(normal, QVector3D(0, 1, 0));
    }
    xAxis.normalize();

    QVector3D yOrtho = QVector3D::crossProduct(normal, xAxis).normalized();

    QMatrix4x4 model;
    model.setColumn(0, QVector4D(xAxis, 0.0f));
    model.setColumn(1, QVector4D(yOrtho, 0.0f));
    model.setColumn(2, QVector4D(normal, 0.0f));
    model.setColumn(3, QVector4D(origin, 1.0f));
    return model;
}

} // anonymous namespace

// Implementation class (PIMPL)
class SketchRendererImpl : protected QOpenGLFunctions {
public:
    SketchRendererImpl() = default;
    ~SketchRendererImpl() { cleanup(); }

    bool initialize();
    void cleanup();
    void buildVBOs(const std::vector<EntityRenderData>& entities,
                   const std::vector<SketchRenderer::RegionRenderData>& regions,
                   const SketchRenderStyle& style,
                   RegionDisplayMode regionDisplayMode,
                   bool suppressRegionFill,
                   const std::unordered_map<EntityID, SelectionState>& selections,
                   const std::unordered_set<std::string>& selectedRegions,
                   std::optional<std::string> hoverRegion,
                   EntityID hoverEntity,
                   const Viewport& viewport,
                   double pixelScale,
                   const std::vector<ConstraintRenderData>& constraints,
                   const std::vector<InferredConstraint>& ghosts,
                   bool snapActive,
                   SnapType snapType,
                   const Vec2d& snapPos,
                   float snapSize,
                   const Vec3d& snapColor,
                   const Vec2d& snapGuideOrigin,
                   bool snapHasGuide,
                   const std::vector<Vec2d>& anchorIndicators,
                   const std::vector<SketchRenderer::GuideLineInfo>& activeGuides,
                   const std::vector<SketchRenderer::GuideLineInfo>& dragGuides);
    void render(const QMatrix4x4& mvp, const SketchRenderStyle& style);
    void renderPoints(const QMatrix4x4& mvp);
    void renderPreview(const QMatrix4x4& mvp, const std::vector<Vec2d>& vertices,
                       const Vec3d& color, float lineWidth);

    bool initialized_ = false;

    // Line rendering
    std::unique_ptr<QOpenGLShaderProgram> lineShader_;
    std::unique_ptr<QOpenGLBuffer> lineVBO_;
    std::unique_ptr<QOpenGLVertexArrayObject> lineVAO_;
    int lineVertexCount_ = 0;
    std::unique_ptr<QOpenGLBuffer> constructionLineVBO_;
    std::unique_ptr<QOpenGLVertexArrayObject> constructionLineVAO_;
    int constructionLineVertexCount_ = 0;
    std::unique_ptr<QOpenGLBuffer> highlightLineVBO_;
    std::unique_ptr<QOpenGLVertexArrayObject> highlightLineVAO_;
    int highlightLineVertexCount_ = 0;
    std::unique_ptr<QOpenGLBuffer> guideLineVBO_;
    std::unique_ptr<QOpenGLVertexArrayObject> guideLineVAO_;
    int guideLineVertexCount_ = 0;

    // Region rendering
    std::unique_ptr<QOpenGLBuffer> regionVBO_;
    std::unique_ptr<QOpenGLVertexArrayObject> regionVAO_;
    int regionVertexCount_ = 0;

    // Point rendering
    std::unique_ptr<QOpenGLShaderProgram> pointShader_;
    std::unique_ptr<QOpenGLBuffer> pointVBO_;
    std::unique_ptr<QOpenGLVertexArrayObject> pointVAO_;
    int pointVertexCount_ = 0;

    // Preview line rendering
    std::unique_ptr<QOpenGLBuffer> previewVBO_;
    std::unique_ptr<QOpenGLVertexArrayObject> previewVAO_;
};

bool SketchRendererImpl::initialize() {
    if (initialized_) return true;

    initializeOpenGLFunctions();

    // Helper to cleanup on failure
    auto cleanupOnFailure = [this]() {
        lineShader_.reset();
        pointShader_.reset();
        if (lineVAO_ && lineVAO_->isCreated()) lineVAO_->destroy();
        if (lineVBO_ && lineVBO_->isCreated()) lineVBO_->destroy();
        if (constructionLineVAO_ && constructionLineVAO_->isCreated()) constructionLineVAO_->destroy();
        if (constructionLineVBO_ && constructionLineVBO_->isCreated()) constructionLineVBO_->destroy();
        if (highlightLineVAO_ && highlightLineVAO_->isCreated()) highlightLineVAO_->destroy();
        if (highlightLineVBO_ && highlightLineVBO_->isCreated()) highlightLineVBO_->destroy();
        if (guideLineVAO_ && guideLineVAO_->isCreated()) guideLineVAO_->destroy();
        if (guideLineVBO_ && guideLineVBO_->isCreated()) guideLineVBO_->destroy();
        if (regionVAO_ && regionVAO_->isCreated()) regionVAO_->destroy();
        if (regionVBO_ && regionVBO_->isCreated()) regionVBO_->destroy();
        if (pointVAO_ && pointVAO_->isCreated()) pointVAO_->destroy();
        if (pointVBO_ && pointVBO_->isCreated()) pointVBO_->destroy();
        if (previewVAO_ && previewVAO_->isCreated()) previewVAO_->destroy();
        if (previewVBO_ && previewVBO_->isCreated()) previewVBO_->destroy();
        lineVAO_.reset();
        lineVBO_.reset();
        constructionLineVAO_.reset();
        constructionLineVBO_.reset();
        highlightLineVAO_.reset();
        highlightLineVBO_.reset();
        guideLineVAO_.reset();
        guideLineVBO_.reset();
        regionVAO_.reset();
        regionVBO_.reset();
        pointVAO_.reset();
        pointVBO_.reset();
        previewVAO_.reset();
        previewVBO_.reset();
    };

    // Line shader
    lineShader_ = std::make_unique<QOpenGLShaderProgram>();
    if (!lineShader_->addShaderFromSourceCode(QOpenGLShader::Vertex, kLineVertexShader)) {
        cleanupOnFailure();
        return false;
    }
    if (!lineShader_->addShaderFromSourceCode(QOpenGLShader::Fragment, kLineFragmentShader)) {
        cleanupOnFailure();
        return false;
    }
    if (!lineShader_->link()) {
        cleanupOnFailure();
        return false;
    }

    // Point shader
    pointShader_ = std::make_unique<QOpenGLShaderProgram>();
    if (!pointShader_->addShaderFromSourceCode(QOpenGLShader::Vertex, kPointVertexShader)) {
        cleanupOnFailure();
        return false;
    }
    if (!pointShader_->addShaderFromSourceCode(QOpenGLShader::Fragment, kPointFragmentShader)) {
        cleanupOnFailure();
        return false;
    }
    if (!pointShader_->link()) {
        cleanupOnFailure();
        return false;
    }

    // Create buffers and VAOs
    lineVAO_ = std::make_unique<QOpenGLVertexArrayObject>();
    lineVBO_ = std::make_unique<QOpenGLBuffer>(QOpenGLBuffer::VertexBuffer);
    if (!lineVAO_->create() || !lineVBO_->create()) {
        cleanupOnFailure();
        return false;
    }

    constructionLineVAO_ = std::make_unique<QOpenGLVertexArrayObject>();
    constructionLineVBO_ = std::make_unique<QOpenGLBuffer>(QOpenGLBuffer::VertexBuffer);
    if (!constructionLineVAO_->create() || !constructionLineVBO_->create()) {
        cleanupOnFailure();
        return false;
    }

    highlightLineVAO_ = std::make_unique<QOpenGLVertexArrayObject>();
    highlightLineVBO_ = std::make_unique<QOpenGLBuffer>(QOpenGLBuffer::VertexBuffer);
    if (!highlightLineVAO_->create() || !highlightLineVBO_->create()) {
        cleanupOnFailure();
        return false;
    }

    guideLineVAO_ = std::make_unique<QOpenGLVertexArrayObject>();
    guideLineVBO_ = std::make_unique<QOpenGLBuffer>(QOpenGLBuffer::VertexBuffer);
    if (!guideLineVAO_->create() || !guideLineVBO_->create()) {
        cleanupOnFailure();
        return false;
    }

    regionVAO_ = std::make_unique<QOpenGLVertexArrayObject>();
    regionVBO_ = std::make_unique<QOpenGLBuffer>(QOpenGLBuffer::VertexBuffer);
    if (!regionVAO_->create() || !regionVBO_->create()) {
        cleanupOnFailure();
        return false;
    }

    pointVAO_ = std::make_unique<QOpenGLVertexArrayObject>();
    pointVBO_ = std::make_unique<QOpenGLBuffer>(QOpenGLBuffer::VertexBuffer);
    if (!pointVAO_->create() || !pointVBO_->create()) {
        cleanupOnFailure();
        return false;
    }

    previewVAO_ = std::make_unique<QOpenGLVertexArrayObject>();
    previewVBO_ = std::make_unique<QOpenGLBuffer>(QOpenGLBuffer::VertexBuffer);
    if (!previewVAO_->create() || !previewVBO_->create()) {
        cleanupOnFailure();
        return false;
    }

    lineVBO_->setUsagePattern(QOpenGLBuffer::DynamicDraw);
    constructionLineVBO_->setUsagePattern(QOpenGLBuffer::DynamicDraw);
    highlightLineVBO_->setUsagePattern(QOpenGLBuffer::DynamicDraw);
    guideLineVBO_->setUsagePattern(QOpenGLBuffer::DynamicDraw);
    regionVBO_->setUsagePattern(QOpenGLBuffer::DynamicDraw);
    pointVBO_->setUsagePattern(QOpenGLBuffer::DynamicDraw);
    previewVBO_->setUsagePattern(QOpenGLBuffer::DynamicDraw);

    initialized_ = true;
    return true;
}

void SketchRendererImpl::cleanup() {
    if (!initialized_) return;

    if (lineVAO_ && lineVAO_->isCreated()) lineVAO_->destroy();
    if (lineVBO_ && lineVBO_->isCreated()) lineVBO_->destroy();
    if (constructionLineVAO_ && constructionLineVAO_->isCreated()) constructionLineVAO_->destroy();
    if (constructionLineVBO_ && constructionLineVBO_->isCreated()) constructionLineVBO_->destroy();
    if (highlightLineVAO_ && highlightLineVAO_->isCreated()) highlightLineVAO_->destroy();
    if (highlightLineVBO_ && highlightLineVBO_->isCreated()) highlightLineVBO_->destroy();
    if (guideLineVAO_ && guideLineVAO_->isCreated()) guideLineVAO_->destroy();
    if (guideLineVBO_ && guideLineVBO_->isCreated()) guideLineVBO_->destroy();
    if (regionVAO_ && regionVAO_->isCreated()) regionVAO_->destroy();
    if (regionVBO_ && regionVBO_->isCreated()) regionVBO_->destroy();
    if (pointVAO_ && pointVAO_->isCreated()) pointVAO_->destroy();
    if (pointVBO_ && pointVBO_->isCreated()) pointVBO_->destroy();
    if (previewVAO_ && previewVAO_->isCreated()) previewVAO_->destroy();
    if (previewVBO_ && previewVBO_->isCreated()) previewVBO_->destroy();

    lineShader_.reset();
    pointShader_.reset();

    initialized_ = false;
}

void SketchRendererImpl::buildVBOs(
    const std::vector<EntityRenderData>& entities,
    const std::vector<SketchRenderer::RegionRenderData>& regions,
    const SketchRenderStyle& style,
    RegionDisplayMode regionDisplayMode,
    bool suppressRegionFill,
    const std::unordered_map<EntityID, SelectionState>& selections,
    const std::unordered_set<std::string>& selectedRegions,
    std::optional<std::string> hoverRegion,
    EntityID hoverEntity,
    const Viewport& viewport,
    double pixelScale,
    const std::vector<ConstraintRenderData>& constraints,
    const std::vector<InferredConstraint>& ghosts,
    bool snapActive,
    SnapType snapType,
    const Vec2d& snapPos,
    float snapSize,
    const Vec3d& snapColor,
    const Vec2d& snapGuideOrigin,
    bool snapHasGuide,
    const std::vector<Vec2d>& anchorIndicators,
    const std::vector<SketchRenderer::GuideLineInfo>& activeGuides,
    const std::vector<SketchRenderer::GuideLineInfo>& dragGuides) {

    // Region data: pos(2) + color(4) = 6 floats per vertex
    std::vector<float> regionData;
    // Line data: pos(2) + color(4) = 6 floats per vertex
    std::vector<float> lineData;
    std::vector<float> constructionLineData;
    std::vector<float> highlightLineData;
    std::vector<float> guideLineData;
    // Point data: pos(2) + color(4) + size(1) = 7 floats per vertex
    std::vector<float> pointData;

    double dashLength = style.dashLength * std::max(pixelScale, 1e-9);
    double gapLength = style.gapLength * std::max(pixelScale, 1e-9);

    if (!suppressRegionFill) {
        for (size_t i = 0; i < regions.size(); ++i) {
            const auto& region = regions[i];
            if (region.triangles.empty()) {
                continue;
            }
            if (viewport.size.x > 0.0 && viewport.size.y > 0.0) {
                if (!viewport.intersects(region.boundsMin, region.boundsMax)) {
                    continue;
                }
            }

            float alpha =
                (regionDisplayMode == RegionDisplayMode::AssistAll) ? style.regionOpacity : 0.0f;
            if (selectedRegions.find(region.id) != selectedRegions.end()) {
                alpha = style.regionSelectedOpacity;
            } else if (hoverRegion && *hoverRegion == region.id) {
                alpha = style.regionHoverOpacity;
            }

            if (alpha <= 0.0f) {
                continue;
            }

            Vec3d color = style.colors.regionFill;
            for (const auto& v : region.triangles) {
                regionData.push_back(static_cast<float>(v.x));
                regionData.push_back(static_cast<float>(v.y));
                regionData.push_back(static_cast<float>(color.x));
                regionData.push_back(static_cast<float>(color.y));
                regionData.push_back(static_cast<float>(color.z));
                regionData.push_back(alpha);
            }
        }
    }

    for (const auto& entity : entities) {
        if (viewport.size.x > 0.0 && viewport.size.y > 0.0) {
            if (!viewport.intersects(entity.bounds[0], entity.bounds[1])) {
                continue;
            }
        }

        SelectionState selState = SelectionState::None;
        if (auto it = selections.find(entity.id); it != selections.end()) {
            selState = it->second;
        }
        if (entity.id == hoverEntity && selState == SelectionState::None) {
            selState = SelectionState::Hover;
        }

        Vec3d color = colorForState(selState, entity.isConstruction, entity.hasError,
                                     style.colors);

        if (entity.type == EntityType::Point) {
            if (entity.vertices.empty()) continue;
            const auto& p = entity.vertices[0];
            float size = (selState == SelectionState::Selected || selState == SelectionState::Dragging)
                             ? style.selectedPointSize
                             : style.pointSize;
            pointData.push_back(static_cast<float>(p.x));
            pointData.push_back(static_cast<float>(p.y));
            pointData.push_back(static_cast<float>(color.x));
            pointData.push_back(static_cast<float>(color.y));
            pointData.push_back(static_cast<float>(color.z));
            pointData.push_back(1.0f);  // alpha
            pointData.push_back(size);
        } else {
            // Lines, arcs, circles - render as line segments
            bool isHighlight = (selState == SelectionState::Selected ||
                                selState == SelectionState::Dragging ||
                                selState == SelectionState::Hover);
            if (entity.isConstruction) {
                appendDashedPolyline(constructionLineData, entity.vertices, color,
                                     dashLength, gapLength);
            } else if (isHighlight) {
                appendSolidPolyline(highlightLineData, entity.vertices, color);
            } else {
                appendSolidPolyline(lineData, entity.vertices, color);
            }
            // Line midpoint indicator (straight lines only)
            if (entity.type == EntityType::Line && entity.vertices.size() == 2) {
                const Vec2d mid = {
                    (entity.vertices[0].x + entity.vertices[1].x) * 0.5,
                    (entity.vertices[0].y + entity.vertices[1].y) * 0.5
                };
                pointData.push_back(static_cast<float>(mid.x));
                pointData.push_back(static_cast<float>(mid.y));
                pointData.push_back(static_cast<float>(color.x));
                pointData.push_back(static_cast<float>(color.y));
                pointData.push_back(static_cast<float>(color.z));
                pointData.push_back(1.0f);
                pointData.push_back(style.midpointPointSize);
            }
        }
    }

    for (const auto& icon : constraints) {
        if (viewport.size.x > 0.0 && viewport.size.y > 0.0) {
            if (!viewport.contains(icon.position)) {
                continue;
            }
        }
        Vec3d color = icon.isConflicting ? style.colors.conflictHighlight : style.colors.constraintIcon;
        if (icon.isSelected) {
            color = style.colors.selectedGeometry;
        } else if (icon.isHovered) {
            color = {
                style.colors.selectedGeometry.x * 0.7 + style.colors.constraintIcon.x * 0.3,
                style.colors.selectedGeometry.y * 0.7 + style.colors.constraintIcon.y * 0.3,
                style.colors.selectedGeometry.z * 0.7 + style.colors.constraintIcon.z * 0.3
            };
        }
        pointData.push_back(static_cast<float>(icon.position.x));
        pointData.push_back(static_cast<float>(icon.position.y));
        pointData.push_back(static_cast<float>(color.x));
        pointData.push_back(static_cast<float>(color.y));
        pointData.push_back(static_cast<float>(color.z));
        pointData.push_back(1.0f);
        pointData.push_back(style.constraintIconSize);
    }

    // Render ghost constraints (inferred, semi-transparent)
    for (const auto& ghost : ghosts) {
        if (!ghost.position) continue;

        Vec2d pos = ghost.position.value();
        if (viewport.size.x > 0.0 && viewport.size.y > 0.0) {
            if (!viewport.contains(pos)) {
                continue;
            }
        }

        Vec3d color = style.colors.constraintIcon;
        float alpha = static_cast<float>(0.5 * ghost.confidence);  // Semi-transparent
        pointData.push_back(static_cast<float>(pos.x));
        pointData.push_back(static_cast<float>(pos.y));
        pointData.push_back(static_cast<float>(color.x));
        pointData.push_back(static_cast<float>(color.y));
        pointData.push_back(static_cast<float>(color.z));
        pointData.push_back(alpha);
        pointData.push_back(style.constraintIconSize);
    }

    const double guideDash = 8.0 * std::max(pixelScale, 1e-9);
    const double guideGap = 6.0 * std::max(pixelScale, 1e-9);
    const Vec2d vpMin = viewport.getMin();
    const Vec2d vpMax = viewport.getMax();
    const double viewportDiag = std::hypot(vpMax.x - vpMin.x, vpMax.y - vpMin.y);
    const double pointGuideMaxLength = 0.35 * viewportDiag;

    auto clipGuideRayToViewport = [&](const Vec2d& origin, const Vec2d& target,
                                      Vec2d& farStart, Vec2d& farEnd) {
        const double dx = target.x - origin.x;
        const double dy = target.y - origin.y;
        const double len = std::sqrt(dx * dx + dy * dy);
        if (len < 1e-6) {
            return false;
        }

        const double ndx = dx / len;
        const double ndy = dy / len;
        double tMin = -std::numeric_limits<double>::infinity();
        double tMax = std::numeric_limits<double>::infinity();

        if (std::abs(ndx) > 1e-12) {
            double t1 = (vpMin.x - origin.x) / ndx;
            double t2 = (vpMax.x - origin.x) / ndx;
            if (t1 > t2) std::swap(t1, t2);
            tMin = std::max(tMin, t1);
            tMax = std::min(tMax, t2);
        } else if (origin.x < vpMin.x || origin.x > vpMax.x) {
            return false;
        }

        if (std::abs(ndy) > 1e-12) {
            double t1 = (vpMin.y - origin.y) / ndy;
            double t2 = (vpMax.y - origin.y) / ndy;
            if (t1 > t2) std::swap(t1, t2);
            tMin = std::max(tMin, t1);
            tMax = std::min(tMax, t2);
        } else if (origin.y < vpMin.y || origin.y > vpMax.y) {
            return false;
        }

        const double rayStartT = std::max(0.0, tMin);
        if (rayStartT >= tMax) {
            return false;
        }

        farStart = {origin.x + rayStartT * ndx, origin.y + rayStartT * ndy};
        farEnd = {origin.x + tMax * ndx, origin.y + tMax * ndy};
        return true;
    };

    auto clampGuideLength = [&](Vec2d& start, Vec2d& end, bool clampForPointSnap) {
        if (!clampForPointSnap || pointGuideMaxLength <= 1e-9) {
            return;
        }
        const double dx = end.x - start.x;
        const double dy = end.y - start.y;
        const double len = std::sqrt((dx * dx) + (dy * dy));
        if (len <= pointGuideMaxLength || len <= 1e-9) {
            return;
        }
        const double scale = pointGuideMaxLength / len;
        end = {start.x + dx * scale, start.y + dy * scale};
    };

    if (!dragGuides.empty()) {
        constexpr Vec3d kDragGuideColor{0.24, 0.29, 0.36};
        for (const auto& guide : dragGuides) {
            Vec2d farStart;
            Vec2d farEnd;
            if (!clipGuideRayToViewport(guide.origin, guide.target, farStart, farEnd)) {
                continue;
            }
            appendDashedPolyline(guideLineData, {farStart, farEnd}, kDragGuideColor,
                                 guideDash, guideGap);
        }
    }

    if (snapActive) {

        constexpr Vec3d kGuideColor{0.58, 0.45, 0.78};
        constexpr Vec3d kGuideColorFaint{0.17, 0.14, 0.23};
        const bool pointSnap = (snapType == SnapType::Vertex || snapType == SnapType::Endpoint);
        const double pointGuideZoomFade = std::clamp(
            1.0 / std::sqrt(std::max(pixelScale, 1e-9) * 12.0),
            0.45,
            1.0);
        const Vec3d guideColor = pointSnap
            ? Vec3d{kGuideColorFaint.x * pointGuideZoomFade,
                    kGuideColorFaint.y * pointGuideZoomFade,
                    kGuideColorFaint.z * pointGuideZoomFade}
            : kGuideColor;

        // Guide cardinality (0/1/2) is resolved by SnapPreviewResolver policy.
        if (!activeGuides.empty()) {
            int count = 0;
            for (const auto& guide : activeGuides) {
                if (count >= 2) break;
                Vec2d farStart;
                Vec2d farEnd;
                if (!clipGuideRayToViewport(guide.origin, guide.target, farStart, farEnd)) continue;
                clampGuideLength(farStart, farEnd, pointSnap);
                appendDashedPolyline(guideLineData, {farStart, farEnd}, guideColor,
                                     guideDash, guideGap);
                count++;
            }
        } else if (snapHasGuide) {
            Vec2d farStart;
            Vec2d farEnd;
            if (clipGuideRayToViewport(snapGuideOrigin, snapPos, farStart, farEnd)) {
                clampGuideLength(farStart, farEnd, pointSnap);
                appendDashedPolyline(guideLineData, {farStart, farEnd}, guideColor, guideDash,
                                     guideGap);
            }
        }

        // Crosshair only for confirmed guide-cross snaps.
        const Vec3d crossColor = pointSnap
            ? guideColor
            : Vec3d{kGuideColor.x * 0.65, kGuideColor.y * 0.65, kGuideColor.z * 0.65};
        const double crossHalf = 5.0 * std::max(pixelScale, 1e-9);
        const bool guideCrossSnap =
            (snapType == SnapType::Intersection && snapHasGuide && activeGuides.size() >= 2);
        if (guideCrossSnap) {
            appendSegment(guideLineData,
                          {snapPos.x - crossHalf, snapPos.y},
                          {snapPos.x + crossHalf, snapPos.y}, crossColor);
            appendSegment(guideLineData,
                          {snapPos.x, snapPos.y - crossHalf},
                          {snapPos.x, snapPos.y + crossHalf}, crossColor);
        }

        pointData.push_back(static_cast<float>(snapPos.x));
        pointData.push_back(static_cast<float>(snapPos.y));
        pointData.push_back(static_cast<float>(snapColor.x));
        pointData.push_back(static_cast<float>(snapColor.y));
        pointData.push_back(static_cast<float>(snapColor.z));
        pointData.push_back(1.0f);
        pointData.push_back(snapSize);
    }

    const Vec3d anchorColor = snapColorForType(SnapType::None);
    for (const auto& anchor : anchorIndicators) {
        if (viewport.size.x > 0.0 && viewport.size.y > 0.0) {
            if (!viewport.contains(anchor)) {
                continue;
            }
        }
        pointData.push_back(static_cast<float>(anchor.x));
        pointData.push_back(static_cast<float>(anchor.y));
        pointData.push_back(static_cast<float>(anchorColor.x));
        pointData.push_back(static_cast<float>(anchorColor.y));
        pointData.push_back(static_cast<float>(anchorColor.z));
        pointData.push_back(1.0f);
        pointData.push_back(style.snapPointSize);
    }

    // Upload region data
    regionVertexCount_ = static_cast<int>(regionData.size() / 6);
    if (!regionData.empty()) {
        regionVAO_->bind();
        regionVBO_->bind();
        regionVBO_->allocate(regionData.data(), static_cast<int>(regionData.size() * sizeof(float)));

        // Position (2 floats)
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
        // Color (4 floats)
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                              reinterpret_cast<void*>(2 * sizeof(float)));

        regionVBO_->release();
        regionVAO_->release();
    }

    // Upload line data
    lineVertexCount_ = static_cast<int>(lineData.size() / 6);
    if (!lineData.empty()) {
        lineVAO_->bind();
        lineVBO_->bind();
        lineVBO_->allocate(lineData.data(), static_cast<int>(lineData.size() * sizeof(float)));

        // Position (2 floats)
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
        // Color (4 floats)
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                              reinterpret_cast<void*>(2 * sizeof(float)));

        lineVBO_->release();
        lineVAO_->release();
    }

    constructionLineVertexCount_ = static_cast<int>(constructionLineData.size() / 6);
    if (!constructionLineData.empty()) {
        constructionLineVAO_->bind();
        constructionLineVBO_->bind();
        constructionLineVBO_->allocate(constructionLineData.data(),
                                       static_cast<int>(constructionLineData.size() * sizeof(float)));

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                              reinterpret_cast<void*>(2 * sizeof(float)));

        constructionLineVBO_->release();
        constructionLineVAO_->release();
    }

    highlightLineVertexCount_ = static_cast<int>(highlightLineData.size() / 6);
    if (!highlightLineData.empty()) {
        highlightLineVAO_->bind();
        highlightLineVBO_->bind();
        highlightLineVBO_->allocate(highlightLineData.data(),
                                   static_cast<int>(highlightLineData.size() * sizeof(float)));

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                              reinterpret_cast<void*>(2 * sizeof(float)));

        highlightLineVBO_->release();
        highlightLineVAO_->release();
    }

    guideLineVertexCount_ = static_cast<int>(guideLineData.size() / 6);
    if (!guideLineData.empty()) {
        guideLineVAO_->bind();
        guideLineVBO_->bind();
        guideLineVBO_->allocate(guideLineData.data(),
                                static_cast<int>(guideLineData.size() * sizeof(float)));

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                              reinterpret_cast<void*>(2 * sizeof(float)));

        guideLineVBO_->release();
        guideLineVAO_->release();
    }

    // Upload point data
    pointVertexCount_ = static_cast<int>(pointData.size() / 7);
    if (!pointData.empty()) {
        pointVAO_->bind();
        pointVBO_->bind();
        pointVBO_->allocate(pointData.data(), static_cast<int>(pointData.size() * sizeof(float)));

        // Position (2 floats)
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 7 * sizeof(float), nullptr);
        // Color (4 floats)
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(float),
                              reinterpret_cast<void*>(2 * sizeof(float)));
        // Size (1 float)
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 7 * sizeof(float),
                              reinterpret_cast<void*>(6 * sizeof(float)));

        pointVBO_->release();
        pointVAO_->release();
    }
}

void SketchRendererImpl::render(const QMatrix4x4& mvp, const SketchRenderStyle& style) {
    if (!initialized_) return;

    lineShader_->bind();
    lineShader_->setUniformValue("uMVP", mvp);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    GLint prevDepthFunc = GL_LESS;
    glGetIntegerv(GL_DEPTH_FUNC, &prevDepthFunc);
    glDepthFunc(GL_LEQUAL);

    glDepthMask(GL_FALSE);

    if (regionVertexCount_ > 0) {
        regionVAO_->bind();
        glDrawArrays(GL_TRIANGLES, 0, regionVertexCount_);
        regionVAO_->release();
    }

    glEnable(GL_LINE_SMOOTH);

    if (constructionLineVertexCount_ > 0) {
        glLineWidth(style.constructionLineWidth);
        constructionLineVAO_->bind();
        glDrawArrays(GL_LINES, 0, constructionLineVertexCount_);
        constructionLineVAO_->release();
    }

    if (lineVertexCount_ > 0) {
        glLineWidth(style.normalLineWidth);
        lineVAO_->bind();
        glDrawArrays(GL_LINES, 0, lineVertexCount_);
        lineVAO_->release();
    }

    if (highlightLineVertexCount_ > 0) {
        glLineWidth(style.selectedLineWidth);
        highlightLineVAO_->bind();
        glDrawArrays(GL_LINES, 0, highlightLineVertexCount_);
        highlightLineVAO_->release();
    }

    if (guideLineVertexCount_ > 0) {
        glLineWidth(1.0f);
        guideLineVAO_->bind();
        glDrawArrays(GL_LINES, 0, guideLineVertexCount_);
        guideLineVAO_->release();
    }

    glDisable(GL_LINE_SMOOTH);
    glDisable(GL_BLEND);
    glLineWidth(1.0f);
    glDepthMask(GL_TRUE);
    glDepthFunc(prevDepthFunc);

    lineShader_->release();
}

void SketchRendererImpl::renderPoints(const QMatrix4x4& mvp) {
    if (!initialized_ || pointVertexCount_ == 0) return;

    pointShader_->bind();
    pointShader_->setUniformValue("uMVP", mvp);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_PROGRAM_POINT_SIZE);
    GLint prevDepthFunc = GL_LESS;
    glGetIntegerv(GL_DEPTH_FUNC, &prevDepthFunc);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_FALSE);

    pointVAO_->bind();
    glDrawArrays(GL_POINTS, 0, pointVertexCount_);
    pointVAO_->release();

    glDepthMask(GL_TRUE);
    glDepthFunc(prevDepthFunc);
    glDisable(GL_PROGRAM_POINT_SIZE);
    glDisable(GL_BLEND);

    pointShader_->release();
}

void SketchRendererImpl::renderPreview(const QMatrix4x4& mvp,
                                        const std::vector<Vec2d>& vertices,
                                        const Vec3d& color, float lineWidth) {
    if (!initialized_ || vertices.size() < 2) return;

    std::vector<float> data;
    for (size_t i = 0; i + 1 < vertices.size(); ++i) {
        const auto& p1 = vertices[i];
        const auto& p2 = vertices[i + 1];
        data.push_back(static_cast<float>(p1.x));
        data.push_back(static_cast<float>(p1.y));
        data.push_back(static_cast<float>(color.x));
        data.push_back(static_cast<float>(color.y));
        data.push_back(static_cast<float>(color.z));
        data.push_back(0.7f);  // semi-transparent preview
        data.push_back(static_cast<float>(p2.x));
        data.push_back(static_cast<float>(p2.y));
        data.push_back(static_cast<float>(color.x));
        data.push_back(static_cast<float>(color.y));
        data.push_back(static_cast<float>(color.z));
        data.push_back(0.7f);
    }

    previewVAO_->bind();
    previewVBO_->bind();
    previewVBO_->allocate(data.data(), static_cast<int>(data.size() * sizeof(float)));

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          reinterpret_cast<void*>(2 * sizeof(float)));

    lineShader_->bind();
    lineShader_->setUniformValue("uMVP", mvp);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glLineWidth(lineWidth);
    GLint prevDepthFunc = GL_LESS;
    glGetIntegerv(GL_DEPTH_FUNC, &prevDepthFunc);
    glDepthFunc(GL_LEQUAL);
    glDepthMask(GL_FALSE);

    glDrawArrays(GL_LINES, 0, static_cast<int>(data.size() / 6));

    glDepthMask(GL_TRUE);
    glDepthFunc(prevDepthFunc);
    glDisable(GL_BLEND);
    glLineWidth(1.0f);
    lineShader_->release();

    previewVBO_->release();
    previewVAO_->release();
}

// ============================================================================
// SketchRenderer public implementation
// ============================================================================

SketchRenderer::SketchRenderer() = default;

SketchRenderer::~SketchRenderer() = default;

bool SketchRenderer::initialize() {
    if (!impl_) {
        impl_ = std::make_unique<SketchRendererImpl>();
    }
    return impl_->initialize();
}

void SketchRenderer::cleanup() {
    if (impl_) {
        impl_->cleanup();
    }
}

void SketchRenderer::setSketch(Sketch* sketch) {
    sketch_ = sketch;
    regionRenderData_.clear();
    selectedRegions_.clear();
    hoverRegion_.reset();
    geometryDirty_ = true;
    constraintsDirty_ = true;
    vboDirty_ = true;
}

void SketchRenderer::updateGeometry() {
    if (!sketch_) return;

    entityRenderData_.clear();

    // Process all entities
    for (const auto& entityPtr : sketch_->getAllEntities()) {
        if (!entityPtr) continue;

        EntityRenderData data;
        data.id = entityPtr->id();
        data.type = entityPtr->type();
        data.isConstruction = entityPtr->isConstruction();
        data.hasError = false;

        switch (entityPtr->type()) {
            case EntityType::Point: {
                auto* point = dynamic_cast<const SketchPoint*>(entityPtr.get());
                if (point) {
                    data.vertices.push_back({point->x(), point->y()});
                }
                break;
            }
            case EntityType::Line: {
                auto* line = dynamic_cast<const SketchLine*>(entityPtr.get());
                if (line) {
                    auto* startPt = sketch_->getEntityAs<SketchPoint>(line->startPointId());
                    auto* endPt = sketch_->getEntityAs<SketchPoint>(line->endPointId());
                    if (startPt && endPt) {
                        data.vertices.push_back({startPt->x(), startPt->y()});
                        data.vertices.push_back({endPt->x(), endPt->y()});
                    }
                }
                break;
            }
            case EntityType::Arc: {
                auto* arc = dynamic_cast<const SketchArc*>(entityPtr.get());
                if (arc) {
                    auto* center = sketch_->getEntityAs<SketchPoint>(arc->centerPointId());
                    if (center) {
                        Vec2d c{center->x(), center->y()};
                        data.vertices = tessellateArc(c, arc->radius(),
                                                       arc->startAngle(), arc->endAngle());
                    }
                }
                break;
            }
            case EntityType::Circle: {
                auto* circle = dynamic_cast<const SketchCircle*>(entityPtr.get());
                if (circle) {
                    auto* center = sketch_->getEntityAs<SketchPoint>(circle->centerPointId());
                    if (center) {
                        Vec2d c{center->x(), center->y()};
                        // Full circle: 0 to 2π
                        data.vertices = tessellateArc(c, circle->radius(),
                                                       0.0, 2.0 * std::numbers::pi);
                    }
                }
                break;
            }
            case EntityType::Ellipse: {
                auto* ellipse = dynamic_cast<const SketchEllipse*>(entityPtr.get());
                if (ellipse) {
                    auto* center = sketch_->getEntityAs<SketchPoint>(ellipse->centerPointId());
                    if (center) {
                        Vec2d c{center->x(), center->y()};
                        data.vertices = tessellateEllipse(c, ellipse->majorRadius(),
                                                          ellipse->minorRadius(),
                                                          ellipse->rotation());
                    }
                }
                break;
            }
            default:
                break;
        }

        if (!data.vertices.empty()) {
            // Calculate bounds
            data.bounds[0] = data.vertices[0];
            data.bounds[1] = data.vertices[0];
            for (const auto& v : data.vertices) {
                data.bounds[0].x = std::min(data.bounds[0].x, v.x);
                data.bounds[0].y = std::min(data.bounds[0].y, v.y);
                data.bounds[1].x = std::max(data.bounds[1].x, v.x);
                data.bounds[1].y = std::max(data.bounds[1].y, v.y);
            }
            entityRenderData_.push_back(std::move(data));
        }
    }

    updateRegions();

    geometryDirty_ = false;
    vboDirty_ = true;
}

void SketchRenderer::updateConstraints() {
    constraintRenderData_.clear();
    if (!sketch_) {
        constraintsDirty_ = false;
        return;
    }

    std::unordered_set<ConstraintID> conflicting(conflictingConstraints_.begin(),
                                                 conflictingConstraints_.end());

    for (const auto& constraintPtr : sketch_->getAllConstraints()) {
        if (!constraintPtr) {
            continue;
        }

        ConstraintRenderData data;
        data.id = constraintPtr->id();
        data.type = constraintPtr->type();
        gp_Pnt2d iconPos = constraintPtr->getIconPosition(*sketch_);
        data.position = {iconPos.X(), iconPos.Y()};
        data.isConflicting = (conflicting.find(data.id) != conflicting.end());
        data.referencedEntities = constraintPtr->referencedEntities();

        if (auto* dim = dynamic_cast<const DimensionalConstraint*>(constraintPtr.get())) {
            data.value = dim->value();
        }

        constraintRenderData_.push_back(std::move(data));
    }

    constraintsDirty_ = false;
    vboDirty_ = true;
}

void SketchRenderer::updateRegions() {
    std::unordered_set<std::string> previousSelection = selectedRegions_;
    std::optional<std::string> previousHover = hoverRegion_;
    std::unordered_map<std::string, std::string> previousSignatureById;
    previousSignatureById.reserve(regionRenderData_.size());
    for (const auto& region : regionRenderData_) {
        if (!region.signature.empty()) {
            previousSignatureById.emplace(region.id, region.signature);
        }
    }

    regionRenderData_.clear();
    selectedRegions_.clear();
    hoverRegion_.reset();

    if (!sketch_) {
        return;
    }

    loop::LoopDetector detector;
    loop::LoopDetectorConfig config;
    config.findAllLoops = false;
    config.computeAreas = true;
    config.resolveHoles = true;
    config.validate = true;
    config.planarizeIntersections = true;
    detector.setConfig(config);

    auto result = detector.detect(*sketch_);
    if (!result.success) {
        return;
    }

    auto regions = loop::buildRegionDefinitions(result, constants::COINCIDENCE_TOLERANCE);
    if (regions.empty()) {
        return;
    }

    for (const auto& regionDef : regions) {
        RegionRenderData region;
        region.id = regionDef.id;
        region.signature = regionDef.signature;
        region.outerPolygon = normalizePolygon(regionDef.outerLoop.polygon, kGeometryEpsilon);
        if (region.outerPolygon.size() < 3) {
            continue;
        }
        if (polygonSignedArea(region.outerPolygon) < 0.0) {
            std::reverse(region.outerPolygon.begin(), region.outerPolygon.end());
        }

        for (const auto& holeLoop : regionDef.holes) {
            std::vector<Vec2d> hole = normalizePolygon(holeLoop.polygon, kGeometryEpsilon);
            if (hole.size() < 3) {
                continue;
            }
            if (polygonSignedArea(hole) > 0.0) {
                std::reverse(hole.begin(), hole.end());
            }
            region.holes.push_back(std::move(hole));
        }

        loop::Face face;
        face.outerLoop = regionDef.outerLoop;
        face.innerLoops = regionDef.holes;
        buildTrianglesFromFace(face, *sketch_, region.triangles);

        region.boundsMin = region.outerPolygon.front();
        region.boundsMax = region.outerPolygon.front();
        for (const auto& p : region.outerPolygon) {
            region.boundsMin.x = std::min(region.boundsMin.x, p.x);
            region.boundsMin.y = std::min(region.boundsMin.y, p.y);
            region.boundsMax.x = std::max(region.boundsMax.x, p.x);
            region.boundsMax.y = std::max(region.boundsMax.y, p.y);
        }

        region.area = std::abs(polygonSignedArea(region.outerPolygon));
        for (const auto& hole : region.holes) {
            region.area -= std::abs(polygonSignedArea(hole));
        }
        if (region.area <= kGeometryEpsilon) {
            continue;
        }

        regionRenderData_.push_back(std::move(region));
    }

    for (auto& region : regionRenderData_) {
        int containmentDepth = 0;
        for (const auto& candidate : regionRenderData_) {
            if (candidate.id == region.id) {
                continue;
            }
            if (candidate.area <= region.area) {
                continue;
            }
            if (!polygonContainsPolygon(candidate.outerPolygon, region.outerPolygon,
                                        kGeometryEpsilon)) {
                continue;
            }
            containmentDepth++;
        }
        region.containmentDepth = containmentDepth;
    }

    if (!regionRenderData_.empty()) {
        std::unordered_set<std::string> validIds;
        std::unordered_map<std::string, std::string> idBySignature;
        validIds.reserve(regionRenderData_.size());
        idBySignature.reserve(regionRenderData_.size());
        for (const auto& region : regionRenderData_) {
            validIds.insert(region.id);
            if (!region.signature.empty()) {
                idBySignature.emplace(region.signature, region.id);
            }
        }
        for (const auto& id : previousSelection) {
            if (validIds.find(id) != validIds.end()) {
                selectedRegions_.insert(id);
                continue;
            }
            auto oldSignature = previousSignatureById.find(id);
            if (oldSignature == previousSignatureById.end()) {
                continue;
            }
            auto remappedId = idBySignature.find(oldSignature->second);
            if (remappedId != idBySignature.end()) {
                selectedRegions_.insert(remappedId->second);
            }
        }
        if (previousHover && validIds.find(*previousHover) != validIds.end()) {
            hoverRegion_ = previousHover;
        } else if (previousHover) {
            auto oldSignature = previousSignatureById.find(*previousHover);
            if (oldSignature != previousSignatureById.end()) {
                auto remappedId = idBySignature.find(oldSignature->second);
                if (remappedId != idBySignature.end()) {
                    hoverRegion_ = remappedId->second;
                }
            }
        }
    }
}

std::optional<std::string> SketchRenderer::pickRegion(const Vec2d& sketchPos) const {
    const auto candidates = pickRegionCandidates(sketchPos);
    if (candidates.empty()) {
        return std::nullopt;
    }
    return candidates.front().id;
}

std::vector<RegionPickHit> SketchRenderer::pickRegionCandidates(const Vec2d& sketchPos) const {
    std::vector<RegionPickHit> hits;
    hits.reserve(regionRenderData_.size());

    for (const auto& region : regionRenderData_) {
        if (sketchPos.x < region.boundsMin.x || sketchPos.x > region.boundsMax.x ||
            sketchPos.y < region.boundsMin.y || sketchPos.y > region.boundsMax.y) {
            continue;
        }
        if (!isPointInPolygon(sketchPos, region.outerPolygon)) {
            continue;
        }
        bool inHole = false;
        for (const auto& hole : region.holes) {
            if (isPointInPolygon(sketchPos, hole)) {
                inHole = true;
                break;
            }
        }
        if (inHole) {
            continue;
        }

        hits.push_back({region.id, region.area, region.containmentDepth, !region.holes.empty()});
    }

    std::sort(hits.begin(), hits.end(), [](const RegionPickHit& a, const RegionPickHit& b) {
        if (a.area != b.area) {
            return a.area < b.area;
        }
        if (a.containmentDepth != b.containmentDepth) {
            return a.containmentDepth > b.containmentDepth;
        }
        return a.id < b.id;
    });
    return hits;
}

std::optional<RegionPickHit> SketchRenderer::getRegionPickHit(const std::string& regionId) const {
    if (regionId.empty()) {
        return std::nullopt;
    }
    for (const auto& region : regionRenderData_) {
        if (region.id == regionId) {
            return RegionPickHit{region.id, region.area, region.containmentDepth,
                                 !region.holes.empty()};
        }
    }
    return std::nullopt;
}

void SketchRenderer::setRegionDisplayMode(RegionDisplayMode mode) {
    if (regionDisplayMode_ == mode) {
        return;
    }
    regionDisplayMode_ = mode;
    vboDirty_ = true;
}

void SketchRenderer::setRegionFillSuppressed(bool suppressed) {
    if (suppressRegionFill_ == suppressed) {
        return;
    }
    suppressRegionFill_ = suppressed;
    vboDirty_ = true;
}

void SketchRenderer::setRegionHover(std::optional<std::string> regionId) {
    if (hoverRegion_ == regionId) {
        return;
    }
    hoverRegion_ = std::move(regionId);
    vboDirty_ = true;
}

void SketchRenderer::clearRegionHover() {
    setRegionHover(std::nullopt);
}

void SketchRenderer::toggleRegionSelection(const std::string& regionId) {
    if (selectedRegions_.find(regionId) != selectedRegions_.end()) {
        selectedRegions_.erase(regionId);
    } else {
        selectedRegions_.insert(regionId);
    }
    vboDirty_ = true;
}

void SketchRenderer::clearRegionSelection() {
    if (selectedRegions_.empty()) {
        return;
    }
    selectedRegions_.clear();
    vboDirty_ = true;
}

bool SketchRenderer::isRegionSelected(const std::string& regionId) const {
    return selectedRegions_.find(regionId) != selectedRegions_.end();
}

void SketchRenderer::render(const QMatrix4x4& viewMatrix, const QMatrix4x4& projMatrix) {
    if (!impl_->initialized_) return;

    if (geometryDirty_) {
        updateGeometry();
    }
    if (constraintsDirty_) {
        updateConstraints();
    }
    if (vboDirty_) {
        buildVBOs();
    }

    QMatrix4x4 model;
    if (sketch_) {
        model = buildSketchModelMatrix(sketch_->getPlane());
    }
    QMatrix4x4 mvp = projMatrix * viewMatrix * model;

    // Render lines first (construction, then normal)
    impl_->render(mvp, style_);

    // Render points on top
    impl_->renderPoints(mvp);

    // Render preview if active
    if (preview_.active && preview_.vertices.size() >= 2) {
        impl_->renderPreview(mvp, preview_.vertices, style_.colors.previewGeometry,
                             style_.previewLineWidth);
    }
}

void SketchRenderer::setEntitySelection(EntityID id, SelectionState state) {
    if (state == SelectionState::None) {
        entitySelections_.erase(id);
    } else {
        entitySelections_[id] = state;
    }
    vboDirty_ = true;
}

void SketchRenderer::clearSelection() {
    entitySelections_.clear();
    vboDirty_ = true;
}

std::vector<EntityID> SketchRenderer::getSelectedEntities() const {
    std::vector<EntityID> result;
    result.reserve(entitySelections_.size());
    for (const auto& [id, state] : entitySelections_) {
        if (state == SelectionState::Selected || state == SelectionState::Dragging) {
            result.push_back(id);
        }
    }
    return result;
}

void SketchRenderer::setHoverEntity(EntityID id) {
    if (hoverEntity_ != id) {
        hoverEntity_ = id;
        vboDirty_ = true;
    }
}

void SketchRenderer::toggleEntitySelection(EntityID id) {
    auto it = entitySelections_.find(id);
    if (it != entitySelections_.end() && it->second == SelectionState::Selected) {
        // Already selected - deselect
        entitySelections_.erase(it);
    } else {
        // Not selected or in other state - select
        entitySelections_[id] = SelectionState::Selected;
    }
    vboDirty_ = true;
}

void SketchRenderer::setConflictingConstraints(const std::vector<ConstraintID>& ids) {
    conflictingConstraints_ = ids;
    constraintsDirty_ = true;
    vboDirty_ = true;
}

void SketchRenderer::setSelectedConstraint(ConstraintID id) {
    if (selectedConstraint_ == id) {
        return;
    }
    selectedConstraint_ = std::move(id);
    vboDirty_ = true;
}

void SketchRenderer::setHoverConstraint(ConstraintID id) {
    if (hoverConstraint_ == id) {
        return;
    }
    hoverConstraint_ = std::move(id);
    vboDirty_ = true;
}

void SketchRenderer::setSuppressedConstraints(const std::vector<ConstraintID>& ids) {
    std::unordered_set<ConstraintID> updated(ids.begin(), ids.end());
    if (updated == suppressedConstraints_) {
        return;
    }
    suppressedConstraints_ = std::move(updated);
    vboDirty_ = true;
}

void SketchRenderer::setPreviewLine(const Vec2d& start, const Vec2d& end) {
    preview_.active = true;
    preview_.type = EntityType::Line;
    preview_.vertices = {start, end};
}

void SketchRenderer::setPreviewArc(const Vec2d& center, double radius,
                                    double startAngle, double endAngle) {
    preview_.active = true;
    preview_.type = EntityType::Arc;
    preview_.vertices = tessellateArc(center, radius, startAngle, endAngle);
}

void SketchRenderer::setPreviewCircle(const Vec2d& center, double radius) {
    // Circle is arc from 0 to 2π
    setPreviewArc(center, radius, 0.0, 2.0 * M_PI);
    preview_.type = EntityType::Circle;
}

void SketchRenderer::setPreviewEllipse(const Vec2d& center, double majorRadius,
                                        double minorRadius, double rotation) {
    preview_.active = true;
    preview_.type = EntityType::Ellipse;
    preview_.vertices = tessellateEllipse(center, majorRadius, minorRadius, rotation);
}

void SketchRenderer::setPreviewRectangle(const Vec2d& corner1, const Vec2d& corner2) {
    preview_.active = true;
    preview_.type = EntityType::Line;  // Render as lines

    double minX = std::min(corner1.x, corner2.x);
    double maxX = std::max(corner1.x, corner2.x);
    double minY = std::min(corner1.y, corner2.y);
    double maxY = std::max(corner1.y, corner2.y);

    // 4 corners
    Vec2d bl{minX, minY};  // bottom-left
    Vec2d br{maxX, minY};  // bottom-right
    Vec2d tr{maxX, maxY};  // top-right
    Vec2d tl{minX, maxY};  // top-left

    // Store as line segments (pairs of vertices)
    preview_.vertices = {bl, br, br, tr, tr, tl, tl, bl};
}

void SketchRenderer::clearPreview() {
    preview_.active = false;
    preview_.vertices.clear();
    previewDimensions_.clear();
}

void SketchRenderer::setPreviewDimensions(const std::vector<PreviewDimension>& dimensions) {
    previewDimensions_ = dimensions;
}

void SketchRenderer::clearPreviewDimensions() {
    previewDimensions_.clear();
}

void SketchRenderer::setActiveGuides(const std::vector<GuideLineInfo>& guides) {
    activeGuides_ = guides;
    vboDirty_ = true;
}

void SketchRenderer::setDragGuides(const std::vector<GuideLineInfo>& guides) {
    dragGuides_ = guides;
    vboDirty_ = true;
}

void SketchRenderer::clearDragGuides() {
    if (dragGuides_.empty()) {
        return;
    }
    dragGuides_.clear();
    vboDirty_ = true;
}

void SketchRenderer::showSnapIndicator(const Vec2d& pos, SnapType type,
                                       const Vec2d& guideOrigin,
                                       bool hasGuide,
                                       const std::string& hintText) {
    snapIndicator_.active = true;
    snapIndicator_.position = pos;
    snapIndicator_.type = type;
    snapIndicator_.guideOrigin = guideOrigin;
    snapIndicator_.hasGuide = hasGuide;
    snapIndicator_.hintText = hintText;
    vboDirty_ = true;
}

void SketchRenderer::hideSnapIndicator() {
    snapIndicator_.active = false;
    snapIndicator_.hasGuide = false;
    snapIndicator_.hintText.clear();
    activeGuides_.clear();
    vboDirty_ = true;
}

void SketchRenderer::setAnchorIndicators(const std::vector<Vec2d>& positions) {
    anchorIndicators_ = positions;
    vboDirty_ = true;
}

void SketchRenderer::clearAnchorIndicators() {
    if (anchorIndicators_.empty()) {
        return;
    }
    anchorIndicators_.clear();
    vboDirty_ = true;
}

void SketchRenderer::setGhostConstraints(const std::vector<InferredConstraint>& ghosts) {
    ghostConstraints_ = ghosts;
    vboDirty_ = true;
}

void SketchRenderer::clearGhostConstraints() {
    ghostConstraints_.clear();
    vboDirty_ = true;
}

void SketchRenderer::setStyle(const SketchRenderStyle& style) {
    style_ = style;
    vboDirty_ = true;
}

void SketchRenderer::setViewport(const Viewport& viewport) {
    constexpr double kViewportEpsilon = 1e-9;
    bool changed = std::abs(viewport_.center.x - viewport.center.x) > kViewportEpsilon ||
                   std::abs(viewport_.center.y - viewport.center.y) > kViewportEpsilon ||
                   std::abs(viewport_.size.x - viewport.size.x) > kViewportEpsilon ||
                   std::abs(viewport_.size.y - viewport.size.y) > kViewportEpsilon;
    viewport_ = viewport;
    if (changed) {
        vboDirty_ = true;
    }
}

void SketchRenderer::setPixelScale(double scale) {
    constexpr double kScaleEpsilon = 1e-9;
    if (std::abs(scale - pixelScale_) < kScaleEpsilon) {
        return;
    }
    pixelScale_ = scale;
    geometryDirty_ = true;
    vboDirty_ = true;
}

void SketchRenderer::setDOF(int dof) {
    currentDOF_ = dof;
    vboDirty_ = true;
}

void SketchRenderer::setShowDOF(bool show) {
    showDOF_ = show;
    vboDirty_ = true;
}

EntityID SketchRenderer::pickEntity(const Vec2d& screenPos, double tolerance) const {
    auto hits = pickEntities(screenPos, tolerance);
    if (hits.empty()) {
        return {};
    }
    return hits.front().id;
}

std::vector<EntityPickHit> SketchRenderer::pickEntities(const Vec2d& screenPos, double tolerance) const {
    // Note: screenPos must be in world/sketch coordinates, not pixel screen coordinates.
    // Caller should transform screen pixels to sketch space before calling.
    std::vector<EntityPickHit> hits;

    for (const auto& data : entityRenderData_) {
        if (!isEntityVisible(data)) continue;

        double minDist = tolerance;

        if (data.type == EntityType::Point) {
            if (!data.vertices.empty()) {
                double dx = screenPos.x - data.vertices[0].x;
                double dy = screenPos.y - data.vertices[0].y;
                double dist = std::sqrt(dx * dx + dy * dy);
                if (dist <= minDist) {
                    minDist = dist;
                } else {
                    continue;
                }
            } else {
                continue;
            }
        } else {
            bool hit = false;
            for (size_t i = 0; i + 1 < data.vertices.size(); ++i) {
                const auto& p1 = data.vertices[i];
                const auto& p2 = data.vertices[i + 1];

                // Distance from point to line segment
                double dx = p2.x - p1.x;
                double dy = p2.y - p1.y;
                double lenSq = dx * dx + dy * dy;
                if (lenSq < 1e-10) continue;

                double t = std::clamp(
                    ((screenPos.x - p1.x) * dx + (screenPos.y - p1.y) * dy) / lenSq,
                    0.0, 1.0);

                double projX = p1.x + t * dx;
                double projY = p1.y + t * dy;
                double dist = std::sqrt((screenPos.x - projX) * (screenPos.x - projX) +
                                         (screenPos.y - projY) * (screenPos.y - projY));

                if (dist <= minDist) {
                    minDist = dist;
                    hit = true;
                }
            }
            if (!hit) {
                continue;
            }
        }

        hits.push_back(EntityPickHit{data.id, data.type, data.isConstruction, minDist});
    }

    std::sort(hits.begin(), hits.end(), [](const EntityPickHit& a, const EntityPickHit& b) {
        return a.distance < b.distance;
    });

    return hits;
}

ConstraintID SketchRenderer::pickConstraint(const Vec2d& screenPos,
                                            double tolerance) const {
    // Note: screenPos must be in sketch coordinates, not pixel screen coordinates.
    // Caller should transform screen pixels to sketch space before calling.
    ConstraintID closest;
    double minDist = tolerance;

    for (const auto& data : constraintRenderData_) {
        if (!shouldRenderConstraintMarker(data)) {
            continue;
        }
        // Calculate distance from click to constraint icon position
        double dx = screenPos.x - data.position.x;
        double dy = screenPos.y - data.position.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < minDist) {
            minDist = dist;
            closest = data.id;
        }
    }

    return closest;
}

std::vector<Vec2d> SketchRenderer::tessellateArc(const Vec2d& center, double radius,
                                                  double startAngle, double endAngle) const {
    std::vector<Vec2d> result;

    // Calculate sweep angle
    double sweep = endAngle - startAngle;
    if (sweep < 0) {
        sweep += 2.0 * std::numbers::pi;
    }

    double arcAngleDeg = style_.arcTessellationAngle > 0 ? style_.arcTessellationAngle : 5.0;
    double arcAngleRad = arcAngleDeg * std::numbers::pi / 180.0;
    double pixelsPerUnit = pixelScale_ > 0.0 ? (1.0 / pixelScale_) : 1.0;
    double arcLengthPixels = radius * pixelsPerUnit * std::abs(sweep);
    double segmentsByPixels = arcLengthPixels > 0.0 ? (arcLengthPixels / 5.0) : 1.0;
    double segmentsByAngle = std::abs(sweep) / arcAngleRad;
    int segments = static_cast<int>(std::ceil(std::max(segmentsByPixels, segmentsByAngle)));
    segments = std::clamp(segments, style_.minArcSegments, style_.maxArcSegments);

    double step = sweep / static_cast<double>(segments);

    for (int i = 0; i <= segments; ++i) {
        double angle = startAngle + step * static_cast<double>(i);
        result.push_back({center.x + radius * std::cos(angle),
                          center.y + radius * std::sin(angle)});
    }

    return result;
}

std::vector<Vec2d> SketchRenderer::tessellateEllipse(const Vec2d& center, double majorRadius,
                                                     double minorRadius, double rotation) const {
    std::vector<Vec2d> result;

    double a = std::abs(majorRadius);
    double b = std::abs(minorRadius);
    if (a <= 0.0 || b <= 0.0) {
        return result;
    }

    double h = ((a - b) * (a - b)) / ((a + b) * (a + b));
    double circumference = std::numbers::pi * (a + b) *
        (1.0 + (3.0 * h) / (10.0 + std::sqrt(4.0 - 3.0 * h)));

    double arcAngleDeg = style_.arcTessellationAngle > 0 ? style_.arcTessellationAngle : 5.0;
    double arcAngleRad = arcAngleDeg * std::numbers::pi / 180.0;
    double pixelsPerUnit = pixelScale_ > 0.0 ? (1.0 / pixelScale_) : 1.0;
    double arcLengthPixels = circumference * pixelsPerUnit;
    double segmentsByPixels = arcLengthPixels > 0.0 ? (arcLengthPixels / 5.0) : 1.0;
    double segmentsByAngle = (2.0 * std::numbers::pi) / arcAngleRad;

    int segments = static_cast<int>(std::ceil(std::max(segmentsByPixels, segmentsByAngle)));
    segments = std::clamp(segments, style_.minArcSegments, style_.maxArcSegments);

    result.reserve(static_cast<size_t>(segments) + 1);

    double cosR = std::cos(rotation);
    double sinR = std::sin(rotation);
    double step = (2.0 * std::numbers::pi) / static_cast<double>(segments);

    for (int i = 0; i <= segments; ++i) {
        double t = step * static_cast<double>(i);
        double ex = a * std::cos(t);
        double ey = b * std::sin(t);

        double rx = ex * cosR - ey * sinR;
        double ry = ex * sinR + ey * cosR;

        result.push_back({center.x + rx, center.y + ry});
    }

    return result;
}

void SketchRenderer::buildVBOs() {
    if (!impl_->initialized_) return;
    if (geometryDirty_) {
        updateGeometry();
    }
    SketchRenderStyle renderStyle = style_;
    if (showDOF_) {
        if (currentDOF_ == 0) {
            renderStyle.colors.normalGeometry = style_.colors.fullyConstrained;
        } else if (currentDOF_ > 0) {
            renderStyle.colors.normalGeometry = style_.colors.underConstrained;
        } else {
            renderStyle.colors.normalGeometry = style_.colors.overConstrained;
        }
    }

    bool snapActive = snapIndicator_.active;
    SnapType snapType = snapIndicator_.type;
    Vec2d snapPos = snapIndicator_.position;
    float snapSize = renderStyle.snapPointSize;
    Vec3d snapColor = snapActive ? snapColorForType(snapIndicator_.type)
                                 : renderStyle.colors.constraintIcon;
    Vec2d snapGuideOrigin = snapIndicator_.guideOrigin;
    bool snapHasGuide = snapIndicator_.hasGuide;

    std::vector<ConstraintRenderData> visibleConstraints;
    visibleConstraints.reserve(constraintRenderData_.size());
    for (const auto& constraint : constraintRenderData_) {
        if (!shouldRenderConstraintMarker(constraint)) {
            continue;
        }
        ConstraintRenderData renderData = constraint;
        renderData.isSelected = (constraint.id == selectedConstraint_);
        renderData.isHovered = (constraint.id == hoverConstraint_);
        visibleConstraints.push_back(std::move(renderData));
    }

    impl_->buildVBOs(entityRenderData_, regionRenderData_, renderStyle,
                     regionDisplayMode_, suppressRegionFill_, entitySelections_,
                     selectedRegions_, hoverRegion_, hoverEntity_,
                     viewport_, pixelScale_, visibleConstraints,
                     ghostConstraints_, snapActive, snapType, snapPos, snapSize, snapColor,
                     snapGuideOrigin, snapHasGuide, anchorIndicators_, activeGuides_,
                     dragGuides_);
    vboDirty_ = false;
}

bool SketchRenderer::isEntityVisible(const EntityRenderData& data) const {
    return viewport_.intersects(data.bounds[0], data.bounds[1]);
}

bool SketchRenderer::shouldRenderConstraintMarker(const ConstraintRenderData& data) const {
    if (suppressedConstraints_.find(data.id) != suppressedConstraints_.end()) {
        return false;
    }

    const bool isSelected = (data.id == selectedConstraint_);
    const bool isHovered = (data.id == hoverConstraint_);
    const bool hasEntitySelection = !entitySelections_.empty() || !hoverEntity_.empty();

    bool referencesContext = false;
    if (hasEntitySelection) {
        if (!hoverEntity_.empty()) {
            referencesContext = std::find(data.referencedEntities.begin(),
                                          data.referencedEntities.end(),
                                          hoverEntity_) != data.referencedEntities.end();
        }
        if (!referencesContext) {
            for (const auto& referenced : data.referencedEntities) {
                auto it = entitySelections_.find(referenced);
                if (it == entitySelections_.end()) {
                    continue;
                }
                if (it->second == SelectionState::Selected ||
                    it->second == SelectionState::Dragging ||
                    it->second == SelectionState::Hover) {
                    referencesContext = true;
                    break;
                }
            }
        }
    }

    // Selection-first visualization: if nothing in context references this constraint,
    // do not render marker unless it is explicitly selected/hovered.
    if (!isSelected && !isHovered && !referencesContext) {
        return false;
    }

    // Keep historical midpoint-clutter reduction, except when actively focused.
    const bool midpointType = data.type == ConstraintType::Midpoint ||
                              data.type == ConstraintType::Horizontal ||
                              data.type == ConstraintType::Vertical ||
                              data.type == ConstraintType::Angle ||
                              data.type == ConstraintType::Parallel ||
                              data.type == ConstraintType::Perpendicular ||
                              data.type == ConstraintType::Equal;
    if (midpointType && !isSelected && !isHovered) {
        return false;
    }

    return true;
}

Vec2d SketchRenderer::calculateConstraintIconPosition(
    const SketchConstraint* /*constraint*/) const {
    // Not yet implemented
    return {0, 0};
}

// Note: SnapManager is now implemented in SnapManager.cpp

} // namespace onecad::core::sketch
