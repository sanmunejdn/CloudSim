#include "MeshSurfaceReconstructionInstantMeshes.h"

#include "InstantMeshesCore.h"

namespace geoalgo
{
namespace meshrecon
{

bool writeIndexedMeshObj(const IndexedMeshLite& mesh, const std::string& objPath, std::string* errMsg)
{
	instant_meshes::TriMesh tri;
	tri.vertices = mesh.vertices;
	tri.faces = mesh.faces;
	return instant_meshes::writeTriMeshObj(tri, objPath, errMsg);
}

bool writeQuadMeshObj(const QuadMeshLite& quad, const std::string& objPath, std::string* errMsg)
{
	instant_meshes::QuadMesh q;
	q.vertices = quad.vertices;
	q.quadFaces = quad.quadFaces;
	q.vertexUv = quad.vertexUv;
	return instant_meshes::writeQuadMeshObj(q, objPath, errMsg);
}

bool remeshToQuadMesh(
	const IndexedMeshLite& triIn,
	QuadMeshLite& quadOut,
	const InstantMeshesParams& params,
	std::string* errMsg)
{
	instant_meshes::TriMesh tri;
	tri.vertices = triIn.vertices;
	tri.faces = triIn.faces;
	instant_meshes::Params imParams;
	imParams.targetVertexCount = params.targetVertexCount;
	imParams.creaseAngleDeg = params.creaseAngleDeg;
	imParams.deterministic = params.deterministic;
	imParams.pureQuad = params.pureQuad;
	imParams.exePath = params.exePath;
	instant_meshes::QuadMesh quad;
	if (!instant_meshes::remeshToQuadMesh(tri, quad, imParams, errMsg))
	{
		return false;
	}
	quadOut.vertices = std::move(quad.vertices);
	quadOut.quadFaces = std::move(quad.quadFaces);
	quadOut.vertexUv = std::move(quad.vertexUv);
	return true;
}

} // namespace meshrecon
} // namespace geoalgo
