#!/usr/bin/env python3
"""Generate / sync CloudSim *.vcxproj.filters by functional rules.

Modes:
  --sync          (default) Keep existing Filter assignments; classify only new items;
                  drop orphans; preserve Filter GUIDs. Use after adding files to .vcxproj.
  --full          Reclassify every item (rewrites buckets; preserves GUIDs when path matches).
  --only-missing  Create .filters only when the file does not exist.

Examples:
  python scripts/generate_vcxproj_filters.py --sync
  python scripts/generate_vcxproj_filters.py --sync --project RobotWidget
  python scripts/generate_vcxproj_filters.py --full
"""

from __future__ import annotations

import argparse
import re
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NS = "{http://schemas.microsoft.com/developer/msbuild/2003}"
ITEM_TAGS = (
	"ClInclude",
	"ClCompile",
	"QtMoc",
	"QtUic",
	"QtRcc",
	"None",
	"ResourceCompile",
	"CustomBuild",
	"Image",
	"Text",
	"Xml",
	"Natvis",
)

# Stable GUIDs for top-level inc/src when creating fresh trees
FILTER_INC = "{93995380-89BD-4b04-88EB-625FBE52EBFB}"
FILTER_SRC = "{4FC737F1-C7A5-4376-A066-2A32D752A2FF}"


def new_guid() -> str:
	return "{" + str(uuid.uuid4()).upper() + "}"


def norm_path(p: str) -> str:
	return p.replace("/", "\\")


def leaf_stem(include: str) -> str:
	leaf = norm_path(include).split("\\")[-1]
	if "." in leaf:
		return leaf.rsplit(".", 1)[0]
	return leaf


def match_any(stem: str, patterns: list[str]) -> bool:
	s = stem
	for pat in patterns:
		# simple glob: * prefix/suffix/contains
		if pat.startswith("*") and pat.endswith("*"):
			if pat[1:-1].lower() in s.lower():
				return True
		elif pat.startswith("*"):
			if s.lower().endswith(pat[1:].lower()):
				return True
		elif pat.endswith("*"):
			if s.lower().startswith(pat[:-1].lower()):
				return True
		elif s.lower() == pat.lower():
			return True
	return False


def is_external_path(include: str) -> bool:
	n = norm_path(include)
	if n.startswith("..\\") or "\\..\\" in n:
		return True
	if re.search(r"(?i)(^|\\)bin\\SDK", n):
		return True
	if n.startswith("$("):
		return True
	return False


def external_filter(include: str) -> str:
	n = norm_path(include)
	m = re.match(r"^\$\(([^)]+)\)\\(.*)$", n)
	if m:
		rest = str(Path(m.group(2)).parent)
		if rest in (".", ""):
			return f"External\\{m.group(1)}"
		return f"External\\{m.group(1)}\\{rest.replace('/', '\\')}"
	parts = n.split("\\")
	i = 0
	while i < len(parts) and parts[i] == "..":
		i += 1
	if i >= len(parts):
		return "External"
	remain = "\\".join(parts[i:])
	parent = str(Path(remain).parent)
	if parent in (".", ""):
		return "External"
	return f"External\\{parent.replace('/', '\\')}"


def disk_mirror_filter(include: str) -> str | None:
	"""Prefer on-disk folder layout under inc/source/ops/resource when present."""
	n = norm_path(include)
	m = re.match(r"(?i)^ops\\([^\\]+)\\", n)
	if m:
		return f"ops\\{m.group(1)}"
	if re.match(r"(?i)^resource(\\.*)?$", n):
		parent = str(Path(n).parent)
		return "resource" if parent in (".", "") else parent.replace("/", "\\")
	if re.match(r"(?i)^discretizers\\", n):
		return "discretizers"
	m = re.match(r"(?i)^(inc|source|src)\\adapters\\", n)
	if m:
		root = "src" if m.group(1).lower() == "source" else m.group(1)
		return f"{root}\\adapters"
	m = re.match(r"(?i)^(inc|source|src)\\(sdf|spare)\\", n)
	if m:
		root = "src" if m.group(1).lower() == "source" else m.group(1)
		return f"{root}\\{m.group(2)}"
	m = re.match(r"(?i)^(inc|source|src)\\sim\\", n)
	if m:
		root = "src" if m.group(1).lower() == "source" else m.group(1)
		return f"{root}\\sim"
	m = re.match(r"(?i)^(inc|source|src)\\(calib|hik|mech|pose)\\", n)
	if m:
		root = "src" if m.group(1).lower() == "source" else m.group(1)
		return f"{root}\\{m.group(2)}"
	# PluginHost Ai tree → functional Ai buckets
	m = re.match(r"(?i)^(inc|source|src)\\Ai\\([^\\]+)$", n)
	if m:
		root = "src" if m.group(1).lower() == "source" else m.group(1)
		stem = Path(m.group(2)).stem
		ai = "Core"
		if re.match(r"(?i)^(AiAgent|AiAssistant|AiActionPlan|CatalogActionPlan)", stem):
			ai = "Agent"
		elif re.match(r"(?i)^(AiLlm|AiHttps|AiConfig|AiIntent|AiArgs|AiCommand|AiProgress)", stem):
			ai = "Llm"
		elif re.search(r"(?i)(DomainHandler|AiDomain)", stem):
			ai = "Domains"
		elif re.search(r"(?i)(Catalog|AiApiCatalog|AiTrajectoryFeatureCatalog)", stem):
			ai = "Catalog"
		elif re.search(
			r"(?i)(AiScene|AiMesh|AiProcessFlow|GeometryRecognize|MeshCompose|MeshCreate|TrajectoryFeature)",
			stem,
		):
			ai = "Rules"
		elif re.search(r"(?i)_global$", stem):
			ai = "Global"
		return f"{root}\\Ai\\{ai}"
	if re.match(r"(?i)^source\\detail\\", n):
		return "src\\detail"
	if re.match(r"(?i)^source\\MeshSurfaceReconstruction\\", n):
		return "src\\MeshSurfaceReconstruction"
	if re.match(r"(?i)^source\\TubularGrinding\\", n):
		return "src\\TubularGrinding"
	# GeometricModeling / vendor trees
	if re.match(r"(?i)^third_party\\planegcs", n):
		return "third_party\\planegcs"
	if re.match(r"(?i)^third_party\\", n):
		parent = str(Path(n).parent).replace("/", "\\")
		return parent if parent not in (".", "") else "third_party"
	if re.match(r"(?i)^ported\\", n):
		parent = str(Path(n).parent).replace("/", "\\")
		return parent if parent not in (".", "") else "ported"
	m = re.match(r"(?i)^(inc|source|src)\\Plugin\\", n)
	if m:
		root = "src" if m.group(1).lower() == "source" else m.group(1)
		return f"{root}\\Plugin"
	m = re.match(r"(?i)^(inc|source|src)\\Global\\", n)
	if m:
		root = "src" if m.group(1).lower() == "source" else m.group(1)
		return f"{root}\\Global"
	# Nested under inc\Foo\ or source\Foo\ (one-level functional folder already on disk)
	m = re.match(r"(?i)^(inc|source|src)\\([^\\]+)\\[^\\]+$", n)
	if m:
		root = "src" if m.group(1).lower() == "source" else m.group(1)
		sub = m.group(2)
		if sub.lower() not in ("inc", "src", "source"):
			return f"{root}\\{sub}"
	return None


def kind_root(include: str, item_type: str) -> str:
	n = norm_path(include)
	if item_type in ("ClInclude", "QtMoc"):
		return "inc"
	if item_type in ("None", "ResourceCompile", "QtRcc"):
		if re.match(r"(?i)^resource", n):
			return "resource"
		return "src"
	if re.match(r"(?i)^source\\", n) or re.match(r"(?i)^src\\", n):
		return "src"
	if re.match(r"(?i)^inc\\", n):
		return "inc"
	return "src"


def functional_bucket(project: str, stem: str) -> str:
	if re.match(r"(?i)^(pch|CloudSimCoreExport)$", stem) or re.search(r"(?i)_global$", stem):
		return "Global"

	table: dict[str, list[tuple[list[str], str]]] = {
		"GeometryAlgorithm": [
			(["SelfTest*"], "SelfTest"),
			(
				[
					"Feature*",
					"IFeature*",
					"Discretize*",
					"*Discretize*",
					"*Discretizer*",
					"FaceSection*",
					"ParamSurface*",
				],
				"FeatureDiscretize",
			),
			(["Mesh*", "GeoMesh*"], "Mesh"),
			(["Trajectory*", "Tubular*"], "Trajectory"),
			(["Sketch*", "SketchExtrude*", "SketchPlane*", "SketchSweep*"], "Sketch"),
			(
				[
					"Shape*",
					"Brep*",
					"Primitive*",
					"Shell*",
					"Wire*",
					"Intersection*",
					"Types",
					"ViewTessellate*",
					"TemplateBrep*",
				],
				"ShapeBrep",
			),
		],
		"CloudSimCore": [
			(["EventHub*", "CoreEvents*", "CoreTypes*"], "Events"),
			(
				[
					"ICloudSim*",
					"IDataService*",
					"IDocumentScope*",
					"IRenderView*",
					"IRobotService*",
					"NullCore*",
				],
				"Interfaces",
			),
			(["BackendTypeIds*", "CloudSimCoreFactories*", "CloudSimCoreVersion*"], "Types"),
		],
		"GeometryEngine": [
			(["SelfTest*"], "SelfTest"),
			(["RigidTransform*", "BackendWorldPose*"], "Transform"),
			(["ToolKinematics*"], "Kinematics"),
			(["Adapters*"], "Adapters"),
		],
		"VcgAlgorithms": [
			(["SelfTest*"], "SelfTest"),
			(["VcgMeshAdapter*"], "Adapter"),
			(["MeshSimplify*", "MeshRemesh*", "MeshSmooth*", "MeshNormalSmooth*"], "RemeshSmooth"),
			(["MeshRepair*", "MeshDefect*"], "Repair"),
			(["MeshReconstruct*"], "Reconstruct"),
		],
		"CloudSimAiSDK": [
			(["IAi*", "ICloudSimAi*"], "Interfaces"),
			(["AiConfig*", "AiAgent*", "AiDomain*", "AiParse*", "AiInference*", "AiTrajectory*"], "Types"),
			(["CloudSimAiVersion*"], "Version"),
		],
		"CloudSimPluginSDK": [
			(["ICloudSimPlugin*", "IPlugin*", "IProcessFlow*"], "Interfaces"),
			(["Plugin*Types*", "PluginBackend*", "PluginGeometry*", "PluginLabeling*", "PluginPointCloud*", "PluginPrimitive*"], "Types"),
			(["CloudSimPluginVersion*"], "Version"),
		],
		"GeometricModelingPlugin": [
			(["GeometricModelingPlugin*", "plugin*"], "Plugin"),
			(["GeometricModelingPage*", "GeometricModelingRibbon*", "CommandStack*"], "UI"),
			(["FeatureDocument*"], "Feature"),
			(["Sketch*"], "Sketch"),
		],
		"PlcCommSDK": [
			(["IPlc*"], "Interfaces"),
			(["PlcCommTypes*", "PlcTag*", "PlcCommClient*", "PlcCommSdk*"], "Client"),
		],
		"PlcCommUI": [
			(["PlcCommWidget*", "PlcCommController*", "PlcCommWorker*"], "UI"),
		],
		"PlcCommPlugin": [
			(["*Plugin*", "plugin*"], "Plugin"),
		],
		"LabelingPlugin": [
			(["*Plugin*", "plugin*"], "Plugin"),
			(["LabelingAnnot*", "LabelingTrain*", "LabelingConfig*", "Labeling*"], "UI"),
			(["PointNetTraining*", "PointNet*"], "Training"),
		],
		"PointCloudPlugin": [
			(["*Plugin*", "plugin*"], "Plugin"),
			(["PointCloudDock*", "TubularGrindingDock*"], "UI"),
		],
		"PointNetPlugin": [
			(["*Plugin*", "plugin*"], "Plugin"),
			(["PointNetDomain*", "PointNetInference*", "PointNetTypes*", "PointNet*"], "Inference"),
		],
		"CloudSimUiAssets": [
			(["AppIcon*", "UiIcon*", "UiIcons*"], "Icons"),
		],
		"CloudSimLabelingSDK": [
			(["*"], "Interfaces"),
		],
		"CloudSimMeshTrajectorySDK": [
			(["*"], "Interfaces"),
		],
		"RunLogger": [
			(["RunLogger*"], "Core"),
		],
		"CollisionAlgorithm": [
			(["*"], "Core"),
		],
		"InstantMeshesCore": [
			(["*"], "Core"),
		],
		"CloudSimBootstrap": [
			(["*"], "App"),
		],
		"CloudSim": [
			(["main*", "pch*"], "App"),
			(["*"], "App"),
		],
		"GeometryPlugin": [
			(["*Plugin*", "plugin*"], "Plugin"),
			(["*"], "UI"),
		],
		"BackendVisual": [
			(
				[
					"IBackendVisual*",
					"BackendVisualRegistry*",
					"BackendVisualMath*",
					"BackendGeometry*",
					"BackendId*",
					"BackendPose*",
					"BackendPick*",
				],
				"Core",
			),
			(["Mesh*", "Brep*", "PointCloud*", "Frame*"], "Backends"),
		],
		"Data": [
			(["BackendHierarchy*", "BackendFollow*", "FollowAttachment*"], "HierarchyFollow"),
			(
				[
					"Mesh*",
					"Brep*",
					"PointCloud*",
					"Frame*",
					"Geometry*",
					"PlyIo*",
					"geometry_base64*",
					"Parametric*",
				],
				"GeometryBackends",
			),
			(["BackendSpatial*", "BackendPrimitive*"], "SpatialPrimitive"),
			(
				[
					"BackendData*",
					"BackendRegistry*",
					"BackendComponent*",
					"BackendProperty*",
					"BackendRelations*",
					"BackendObject*",
					"BackendType*",
					"Property*",
				],
				"Core",
			),
		],
		"Widget": [
			(
				[
					"MainWindow*",
					"DocumentPage*",
					"RunInfoPage*",
					"ApplicationStyle*",
					"StyledDock*",
					"Viewport*",
					"ViewPreset*",
				],
				"MainWindow",
			),
			(
				[
					"OsgWidget*",
					"QWidgetViewer*",
					"WidgetOsg*",
					"WidgetRender*",
					"WidgetDocument*",
					"WidgetScene*",
				],
				"OsgWidget",
			),
			(["Backend*", "IBackend*"], "BackendTree"),
			(
				[
					"*Pick*",
					"*Operation*",
					"ObjectTransform*",
					"Selection*",
					"Labeling*",
					"MeshSection*",
					"RobotTcp*",
				],
				"PickOperations",
			),
			(
				[
					"JobSystem*",
					"ProgressManager*",
					"GraphicsWindow*",
					"LitMesh*",
					"QtKeyboard*",
					"ProjectPackage*",
					"pch",
					"widget_global",
				],
				"Infrastructure",
			),
		],
		"RobotWidget": [
			(["IRobot*"], "Interfaces"),
			(
				[
					"RobotInstruction*",
					"Instruction*",
					"ProgramEdit*",
					"PlanResult*",
					"BrandProgram*",
				],
				"Instructions",
			),
			(
				[
					"RobotAxis*",
					"RobotCollision*",
					"RobotFrame*",
					"RobotExternal*",
					"RobotComm*",
					"DevicePage*",
					"RobotUrdf*",
					"RobotProject*",
					"RobotOsgUiTypes*",
				],
				"RobotSettings",
			),
			(["Trajectory*"], "TrajectoryUi"),
			(["Feature*", "MeshTrajectory*", "MeshTriangle*"], "FeatureTrajectory"),
			(
				["Simulation*", "RobotSimulation*", "BackendCollision*", "PythonScript*"],
				"Simulation",
			),
		],
		"RobotScene": [
			(["IRobot*"], "Interfaces"),
			(["RobotInstruction*", "InstructionProgram*", "ProgramEdit*"], "Instructions"),
			(
				[
					"RobotProgram*",
					"RobotCanonical*",
					"Recipe*",
					"ProcessFlowPreset*",
				],
				"ProgramExport",
			),
			(
				[
					"RobotSceneKinematics*",
					"RobotCoordinate*",
					"RobotExternal*",
					"RobotTeach*",
					"ExternalAxis*",
					"RobotPerLink*",
					"RobotMatrix*",
				],
				"KinematicsFrames",
			),
			(
				[
					"Trajectory*",
					"*Trajectory*",
					"RawTrajectory*",
					"Unified*",
					"*Ingress*",
					"PathPlan*",
				],
				"Trajectory",
			),
			(["RobotCollision*", "RobotSceneGeometry*"], "CollisionGeometry"),
		],
		"CloudSimHost": [
			(["DocumentHost*", "CloudSimHost*", "CloudSimApplication*"], "DocumentHost"),
			(
				[
					"*Import*",
					"ProjectPackage*",
					"Annotation*",
					"HierarchyMesh*",
					"BackendProject*",
					"BackendFile*",
					"BackendFollow*",
					"BackendHierarchy*",
				],
				"ImportIo",
			),
			(["Robot*", "Urdf*", "PerLink*", "IPerLink*", "IRobot*"], "Robot"),
			(["BackendVisual*", "Selection*", "HostRender*"], "Visual"),
			(["*Adapter*"], "adapters"),
		],
		"PointCloudAlgorithm": [
			(["Registration*"], "Registration"),
			(["Reconstruction*"], "Reconstruction"),
			(["Crop*", "Downsample*", "Preprocess*", "Measure*", "Transform*"], "Preprocess"),
			(
				[
					"PointCloud*",
					"PointFeatures*",
					"KdTree*",
					"Parallel*",
					"Performance*",
					"SelfTest*",
				],
				"Core",
			),
		],
		"ProcessFlowPlugin": [
			(["ProcessFlowPlugin*", "plugin*", "plugin"], "Plugin"),
			(["ProcessFlowSim*", "*SimController*"], "SimController"),
			(
				[
					"DesEngine*",
					"Dispatch*",
					"IDispatch*",
					"IScheduler*",
					"IStation*",
					"JobSet*",
					"OperationTrace*",
					"PlantGraph*",
					"SimModel*",
					"SimRun*",
					"SimStatistics*",
				],
				"sim",
			),
			(["ProcessFlow*"], "UI"),
		],
		"IndustrialCameraSDK": [
			(["BoardDetector*", "HandEye*", "MechOfficial*"], "calib"),
			(["Hik*"], "hik"),
			(["MechEye*"], "mech"),
			(["*PoseSource*", "ManualPose*", "RealRobotPose*", "IRobotPose*"], "pose"),
			(["SimulatedCamera*", "CameraFactory*", "CameraTypes*", "ICamera*"], "Core"),
		],
		"IndustrialCameraPlugin": [
			(["*Plugin*"], "Plugin"),
			(["HandEye*"], "HandEye"),
			(["Camera*", "IndustrialCamera*"], "UI"),
		],
		"CloudSimPluginHost": [
			(["IPlugin*"], "Interfaces"),
			(["Document*"], "Document"),
			(["Plugin*"], "PluginHost"),
		],
		"OsgWidgetCore": [
			(["OsgScene*", "OsgCompass*", "OsgSection*"], "Scene"),
			(
				[
					"*Pick*",
					"Pick*",
					"BrepPick*",
					"MeshTopology*",
					"BackendPick*",
					"BackendVisualBinding*",
				],
				"Pick",
			),
			(["*Gizmo*", "ObjectGizmo*"], "Gizmo"),
			(["RobotOsg*", "PickTypes*"], "Types"),
		],
		"AiWidget": [
			(["*Settings*"], "Settings"),
			(["Ai*"], "Assistant"),
		],
		"TrajectoryAlgorithm": [
			(["ITrajectory*", "IOp*", "IExternal*", "INonRigid*"], "Interfaces"),
			(["TrajectoryOp*", "TrajectoryParam*", "TrajectoryTransform*"], "Registry"),
		],
		"RobotCommSDK": [
			(["IRobot*", "RobotComm*", "RobotMotion*"], "Client"),
		],
		"RobotKinematics": [
			(["Circular*"], "Geometry"),
			(["Serial*"], "Kinematics"),
		],
		"RobotUrdf": [
			(["Urdf*", "RobotSimulation*"], "Loader"),
		],
		"HelloAiPlugin": [
			(["HelloAi*", "*Plugin*", "plugin*"], "Plugin"),
		],
		"TrajectoryAlgorithmBuiltins": [
			(["TrajectoryOpBuiltinsRegister*", "TrajectoryOpConfig*", "TrajectoryOpFormat*"], "Registry"),
			(["TrajectoryUnified*", "UnifiedTrajectory*"], "Unified"),
		],
	}

	rules = table.get(project)
	if rules is not None:
		for patterns, bucket in rules:
			if match_any(stem, patterns):
				return bucket
		return "Other" if project != "TrajectoryAlgorithmBuiltins" else "Common"

	if match_any(stem, ["*Plugin*"]):
		return "Plugin"
	if match_any(stem, ["*global*", "*_global", "pch", "CloudSimCoreExport"]):
		return "Global"
	return "Core"


def resolve_filter(project: str, include: str, item_type: str) -> str:
	if is_external_path(include):
		return external_filter(include)
	mirror = disk_mirror_filter(include)
	if mirror is not None:
		return mirror
	kind = kind_root(include, item_type)
	if kind == "resource":
		return "resource"
	stem = leaf_stem(include)
	bucket = functional_bucket(project, stem) or "Other"
	if bucket == "adapters":
		return f"{kind}\\adapters"
	if bucket == "sim" and project == "ProcessFlowPlugin":
		return f"{kind}\\sim"
	if bucket in ("calib", "hik", "mech", "pose") and project == "IndustrialCameraSDK":
		return f"{kind}\\{bucket}"
	return f"{kind}\\{bucket}"


def collect_project_items(vcxproj: Path) -> list[tuple[str, str]]:
	tree = ET.parse(vcxproj)
	root = tree.getroot()
	out: list[tuple[str, str]] = []
	for tag in ITEM_TAGS:
		for elem in root.findall(f".//{NS}{tag}"):
			inc = elem.get("Include")
			if not inc or not inc.strip():
				continue
			if re.search(r"\.(props|targets)$", inc, re.I):
				continue
			if tag == "CustomBuild" and not re.search(r"\.(h|hpp|cpp|cxx|c|ui|qrc|ts|rc|qml)$", inc, re.I):
				continue
			out.append((tag, inc))
	return out


def parse_existing_filters(filters_path: Path) -> tuple[dict[str, str], dict[str, str]]:
	"""Returns (include -> filter, filter_path -> guid)."""
	include_to_filter: dict[str, str] = {}
	filter_guids: dict[str, str] = {}
	if not filters_path.exists():
		return include_to_filter, filter_guids
	raw = filters_path.read_bytes()
	# strip BOM
	if raw.startswith(b"\xef\xbb\xbf"):
		raw = raw[3:]
	text = raw.decode("utf-8", errors="replace")
	for m in re.finditer(
		r'<Filter Include="([^"]+)">\s*<UniqueIdentifier>(\{[^}]+\})</UniqueIdentifier>',
		text,
		re.I,
	):
		filter_guids[m.group(1)] = m.group(2)
	# item assignments
	for tag in ITEM_TAGS:
		for m in re.finditer(
			rf'<{tag} Include="([^"]+)">\s*<Filter>([^<]+)</Filter>\s*</{tag}>',
			text,
			re.I,
		):
			include_to_filter[m.group(1)] = m.group(2)
	return include_to_filter, filter_guids


def ensure_filter_tree(filter_guids: dict[str, str], filter_path: str) -> None:
	parts = filter_path.split("\\")
	for i in range(1, len(parts) + 1):
		sub = "\\".join(parts[:i])
		if sub not in filter_guids:
			if sub == "inc":
				filter_guids[sub] = FILTER_INC
			elif sub == "src":
				filter_guids[sub] = FILTER_SRC
			else:
				filter_guids[sub] = new_guid()


def write_filters(
	filters_path: Path,
	classified: list[tuple[str, str, str]],
	filter_guids: dict[str, str],
) -> None:
	used: set[str] = set()
	for _, _, f in classified:
		ensure_filter_tree(filter_guids, f)
		used.add(f)
		parts = f.split("\\")
		for i in range(1, len(parts)):
			used.add("\\".join(parts[:i]))

	# keep only used filter GUIDs (+ parents already in used)
	guids = {k: v for k, v in filter_guids.items() if k in used}
	for f in used:
		if f not in guids:
			ensure_filter_tree(guids, f)

	lines = [
		'<?xml version="1.0" encoding="utf-8"?>',
		'<Project xmlns="http://schemas.microsoft.com/developer/msbuild/2003" ToolsVersion="4.0">',
		"  <ItemGroup>",
	]
	for f in sorted(guids.keys(), key=lambda x: tuple(x.split("\\"))):
		lines.append(f'    <Filter Include="{f}">')
		lines.append(f"      <UniqueIdentifier>{guids[f]}</UniqueIdentifier>")
		# VS 对顶层 inc/src 带 Extensions 时嵌套筛选器更稳定
		if f == "inc":
			lines.append(
				"      <Extensions>h;hh;hpp;hxx;h++;hm;inl;inc;ipp;xsd</Extensions>"
			)
		elif f == "src":
			lines.append(
				"      <Extensions>cpp;c;cc;cxx;c++;cppm;ixx;def;odl;idl;hpj;bat;asm;asmx</Extensions>"
			)
		lines.append("    </Filter>")
	lines.append("  </ItemGroup>")

	by_type: dict[str, list[tuple[str, str]]] = {}
	for tag, inc, filt in classified:
		by_type.setdefault(tag, []).append((inc, filt))
	for tag in ITEM_TAGS:
		if tag not in by_type:
			continue
		lines.append("  <ItemGroup>")
		for inc, filt in sorted(by_type[tag], key=lambda x: x[0].lower()):
			lines.append(f'    <{tag} Include="{inc}">')
			lines.append(f"      <Filter>{filt}</Filter>")
			lines.append(f"    </{tag}>")
		lines.append("  </ItemGroup>")
	lines.append("</Project>")
	payload = ("\r\n".join(lines) + "\r\n").encode("utf-8")
	filters_path.write_bytes(b"\xef\xbb\xbf" + payload)


def process_project(vcxproj: Path, mode: str) -> None:
	project = vcxproj.stem
	filters_path = Path(str(vcxproj) + ".filters")
	items = collect_project_items(vcxproj)
	if not items:
		print(f"SKIP (no items): {project}")
		return

	existing_map, filter_guids = parse_existing_filters(filters_path)
	if mode == "only-missing" and filters_path.exists():
		print(f"SKIP (exists): {project}")
		return

	classified: list[tuple[str, str, str]] = []
	new_count = 0
	for tag, inc in items:
		if mode == "sync" and inc in existing_map:
			filt = existing_map[inc]
		else:
			filt = resolve_filter(project, inc, tag)
			if mode == "sync" and inc not in existing_map:
				new_count += 1
		classified.append((tag, inc, filt))

	write_filters(filters_path, classified, filter_guids)
	uniq = len({c[2] for c in classified})
	extra = f", +{new_count} new" if mode == "sync" else ""
	print(f"OK {project}: {len(classified)} items -> {uniq} filters ({mode}{extra})")


def discover_projects(only: list[str] | None) -> list[Path]:
	sln = ROOT / "CloudSim.sln"
	paths: list[Path] = []
	if sln.exists():
		for line in sln.read_text(encoding="utf-8", errors="replace").splitlines():
			m = re.search(
				r'Project\("\{8BC9CEB8-8B4A-11D0-8D11-00A0C91BC942\}"\)\s*=\s*"[^"]+",\s*"([^"]+\.vcxproj)"',
				line,
			)
			if m:
				paths.append((ROOT / m.group(1).replace("/", "\\")).resolve())
	# extras not always in sln
	for extra in (
		ROOT / "src/UI/CloudSimPluginHost/CloudSimPluginHost.vcxproj",
		ROOT / "src/Plugins/HelloAiPlugin/HelloAiPlugin.vcxproj",
	):
		if extra.exists() and extra.resolve() not in paths:
			paths.append(extra.resolve())

	# fallback: rglob
	if not paths:
		paths = [
			p.resolve()
			for p in (ROOT / "src").rglob("*.vcxproj")
			if ".vs" not in p.parts
			and "ThirdParty" not in p.parts
			and "vcglib" not in {x.lower() for x in p.parts}
			and "bin" not in {x.lower() for x in p.parts}
		]

	if only:
		want = {n.strip().lower() for n in only if n.strip()}
		paths = [p for p in paths if p.stem.lower() in want]
	return sorted(set(paths), key=lambda p: str(p).lower())


def main() -> None:
	ap = argparse.ArgumentParser(description="Generate / sync .vcxproj.filters by function")
	mode = ap.add_mutually_exclusive_group()
	mode.add_argument(
		"--sync",
		action="store_true",
		help="Keep existing assignments; classify only new vcxproj items (default)",
	)
	mode.add_argument(
		"--full",
		action="store_true",
		help="Reclassify all items by functional rules (preserve Filter GUIDs)",
	)
	mode.add_argument(
		"--only-missing",
		action="store_true",
		help="Only create .filters when missing",
	)
	ap.add_argument(
		"--project",
		action="append",
		default=[],
		help="Limit to project name(s); repeatable",
	)
	args = ap.parse_args()
	if args.full:
		run_mode = "full"
	elif args.only_missing:
		run_mode = "only-missing"
	else:
		run_mode = "sync"

	projects = discover_projects(args.project)
	print(f"Processing {len(projects)} projects (mode={run_mode})...")
	for vcx in projects:
		if not vcx.exists():
			print(f"MISSING: {vcx}")
			continue
		process_project(vcx, run_mode)
	print("Done.")


if __name__ == "__main__":
	main()
