# -*- coding: utf-8 -*-
"""Enrich full_api_catalog.json with args_schema + risk."""
import json
from pathlib import Path

def backend(name="backend_id", filter="PointCloud", label="对象", required=True):
    return {"name": name, "type": "backend", "filter": filter, "label": label, "required": required}

def num(name, default, label, mn=None, mx=None, required=True):
    d = {"name": name, "type": "number", "default": default, "label": label, "required": required}
    if mn is not None:
        d["min"] = mn
    if mx is not None:
        d["max"] = mx
    return d

def pair(filter="PointCloudOrMesh", label_src="源", label_tgt="目标"):
    return {
        "name": "backend_pair",
        "type": "backend_pair",
        "filter": filter,
        "label_source": label_src,
        "label_target": label_tgt,
        "required": True,
    }

def file_f(name="path", label="文件", required=True):
    return {"name": name, "type": "file", "label": label, "required": required}

def directory(name="path", label="目录", required=True):
    return {"name": name, "type": "directory", "label": label, "required": required}

def enum_f(name, values, default, label):
    return {"name": name, "type": "enum", "values": values, "default": default, "label": label, "required": True}

def bool_f(name, default, label):
    return {"name": name, "type": "bool", "default": default, "label": label, "required": False}

schemas = {
    "createPrimitiveMesh": (
        "medium",
        [
            enum_f("primitive", ["box", "cylinder", "cone", "sphere"], "box", "基本体"),
            num("length_mm", 100, "长(mm)", 0.1, 1e6, False),
            num("width_mm", 100, "宽(mm)", 0.1, 1e6, False),
            num("height_mm", 100, "高(mm)", 0.1, 1e6, False),
            num("radius_mm", 50, "半径(mm)", 0.1, 1e6, False),
        ],
    ),
    "buildPrimitiveMeshSoup": ("low", []),
    "booleanMeshSoups": ("medium", []),
    "booleanPrimitiveMeshes": ("medium", []),
    "booleanMesh": (
        "medium",
        [
            backend("target", "Mesh", "目标网格"),
            backend("tool", "Mesh", "工具网格"),
            enum_f("op", ["difference", "union", "intersection"], "difference", "布尔运算"),
        ],
    ),
    "registerTriangleMesh": ("medium", []),
    "importFileIntoActiveDocument": (
        "medium",
        [file_f("path", "文件路径"), bool_f("is_point_cloud", True, "作为点云导入")],
    ),
    "downsamplePointCloudVoxel": (
        "medium",
        [backend(label="点云对象"), num("voxel_mm", 2.0, "体素大小(mm)", 0.1, 200)],
    ),
    "downsamplePointCloudRandom": (
        "medium",
        [backend(label="点云对象"), num("ratio", 0.5, "保留比", 0.01, 1.0)],
    ),
    "cropPointCloudByBox": ("medium", [backend(label="点云对象")]),
    "cropPointCloudBySphere": (
        "medium",
        [backend(label="点云对象"), num("radius_mm", 50.0, "球半径(mm)", 0.1, 1e6)],
    ),
    "cropPointCloudByPolyline": ("medium", [backend(label="点云对象")]),
    "removePointCloudOutliers": (
        "medium",
        [backend(label="点云对象"), num("removal_percent", 5.0, "移除百分比", 0.1, 50)],
    ),
    "smoothPointCloudBilateral": ("medium", [backend(label="点云对象")]),
    "estimatePointCloudNormalsPca": ("medium", [backend(label="点云对象")]),
    "orientPointCloudNormalsMst": ("medium", [backend(label="点云对象")]),
    "rigidRegisterPointCloudsIcp": ("medium", [pair()]),
    "nonRigidRegisterSpare": ("medium", [pair()]),
    "reconstructMeshPoissonAuto": ("medium", [backend(label="点云对象")]),
    "reconstructMeshScaleSpace": ("medium", [backend(label="点云对象")]),
    "exportMeshToPly": (
        "high",
        [backend("backend_id", "Mesh", "网格对象"), file_f("path", "导出路径")],
    ),
    "registerScanToCadTemplateCoarse": (
        "medium",
        [
            backend("backend_id", "PointCloudOrMesh", "扫描对象"),
            backend("template_backend_id", "Brep", "CAD 模板"),
        ],
    ),
    "registerScanToCadTemplateFine": (
        "medium",
        [
            backend("backend_id", "PointCloudOrMesh", "扫描对象"),
            backend("template_backend_id", "Brep", "CAD 模板"),
        ],
    ),
    "updateTemplateBrepFromAlignedScan": (
        "high",
        [
            backend("backend_id", "PointCloudOrMesh", "扫描对象"),
            backend("template_backend_id", "Brep", "CAD 模板"),
        ],
    ),
    "simplifyMesh": (
        "medium",
        [
            backend("backend_id", "Mesh", "网格"),
            num("target_face_count", 0, "目标面数(0=一半)", 0, 1e7, False),
        ],
    ),
    "smoothMeshLaplacian": (
        "medium",
        [backend("backend_id", "Mesh", "网格"), num("iterations", 3, "迭代次数", 1, 100)],
    ),
    "smoothMeshTaubin": (
        "medium",
        [
            backend("backend_id", "Mesh", "网格"),
            num("iterations", 3, "迭代次数", 1, 100),
            num("lambda", 0.2, "λ", 0.01, 1.0),
        ],
    ),
    "repairMesh": (
        "medium",
        [backend("backend_id", "Mesh", "网格"), bool_f("fill_holes", False, "填孔")],
    ),
    "remeshMeshIsotropic": (
        "medium",
        [
            backend("backend_id", "Mesh", "网格"),
            num("target_edge_mm", 2.0, "目标边长(mm)", 0.1, 100),
        ],
    ),
    "runMeshSurfaceReconstructPreprocess": ("medium", [backend("backend_id", "Mesh", "网格")]),
    "runMeshSurfaceReconstructPartition": ("medium", [backend("backend_id", "Mesh", "网格")]),
    "runMeshSurfaceReconstructSample": ("medium", [backend("backend_id", "Mesh", "网格")]),
    "runMeshSurfaceReconstructFit": ("medium", [backend("backend_id", "Mesh", "网格")]),
    "runMeshSurfaceReconstructBoundaryBlend": ("medium", [backend("backend_id", "Mesh", "网格")]),
    "runMeshSurfaceReconstructJunctionBlend": ("medium", [backend("backend_id", "Mesh", "网格")]),
    "runMeshSurfaceReconstructFair": ("medium", [backend("backend_id", "Mesh", "网格")]),
    "runMeshSurfaceReconstructAssemble": ("medium", [backend("backend_id", "Mesh", "网格")]),
    "reconstructSurfaceFromMesh": ("medium", [backend("backend_id", "Mesh", "网格")]),
    "clearMeshSurfaceReconstructSession": ("low", []),
    "discretizeBackendToMesh": (
        "medium",
        [
            backend("backend_id", "BrepOrMesh", "STEP/后端", False),
            file_f("step_path", "STEP 路径(可选)", False),
        ],
    ),
    "pickStepElementEdge": ("low", [backend("backend_id", "Brep", "后端(可选)", False)]),
    "pickStepElementFace": ("low", [backend("backend_id", "Brep", "后端(可选)", False)]),
    "intersectEdgeFace": ("medium", []),
    "intersectFaces": ("medium", []),
    "discretizeWireToTubeMesh": ("medium", [num("tube_radius_mm", 1.0, "管半径(mm)", 0.01, 100)]),
    "discretizeWireToRibbonMesh": ("medium", [num("ribbon_width_mm", 2.0, "带宽(mm)", 0.01, 100)]),
    "runTubularGrindingCenterline": ("medium", [backend("backend_id", "Mesh", "网格")]),
    "runTubularGrindingTemplatePoints": ("medium", [backend("backend_id", "Mesh", "网格")]),
    "runTubularGrindingProject": ("medium", [backend("backend_id", "Mesh", "网格")]),
    "runTubularGrindingFpfhRegionPartition": ("medium", [backend("backend_id", "Mesh", "网格")]),
    "clearTubularGrindingSession": ("low", []),
    "labelingPickClick": ("medium", [backend("backend_id", "PointCloudOrMesh", "标注对象")]),
    "labelingPickBrush": (
        "medium",
        [
            backend("backend_id", "PointCloudOrMesh", "标注对象"),
            num("brush_radius_px", 16, "笔刷半径(px)", 1, 200),
        ],
    ),
    "labelingPickLasso": ("medium", [backend("backend_id", "PointCloudOrMesh", "标注对象")]),
    "labelingErase": ("medium", [backend("backend_id", "PointCloudOrMesh", "标注对象")]),
    "cancelActiveLabelingPick": ("low", []),
    "labelingUndo": ("low", []),
    "labelingRedo": ("low", []),
    "pointNetPrelabel": ("medium", [backend("backend_id", "PointCloudOrMesh", "标注对象")]),
    "exportPointNetDataset": ("high", [directory("path", "导出目录")]),
}

here = Path(__file__).resolve().parent
path = here / "full_api_catalog.json"
root = json.loads(path.read_text(encoding="utf-8"))
for api in root["apis"]:
    aid = api["id"]
    if aid not in schemas:
        raise SystemExit(f"missing schema for {aid}")
    risk, schema = schemas[aid]
    api["risk"] = risk
    api["args_schema"] = schema
root["version"] = 2
path.write_text(json.dumps(root, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print("ok apis", len(root["apis"]))
