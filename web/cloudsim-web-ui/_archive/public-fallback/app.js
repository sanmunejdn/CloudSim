import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { TransformControls } from "three/addons/controls/TransformControls.js";

const $ = (id) => document.getElementById(id);
const healthEl = $("health");
const statusEl = $("status");
const sseHint = $("sseHint");
const treeUnitsEl = $("treeUnits");
const pathEl = $("path");
const propsEl = $("props");
const sceneMount = $("scene");
const modeSelect = $("modeSelect");
const logEl = $("log");
const docTabTitle = $("docTabTitle");

const zUpToYUp = new THREE.Matrix4().makeRotationX(-Math.PI / 2);
const idToMesh = new Map();
let selectedId = null;
// 默认视图移动；对象选择需菜单开启，避免点连杆误挂平移罗盘
let sceneInteractMode = "view"; // "view" | "select"
let objects = [];
let detail = null;
let suppressPosePush = false;
let selectedRobot = null; // { sceneRootBackendId, anchorBackendId, flangeBackendId } | null
let posePushInFlight = false;
let posePushPending = false;
let tcpIkIncomplete = false;
let robotDragMode = false;
let dragFlangeId = null; // 拖拽罗盘锚定的法兰 backendId（IK 实例解析用）

// 指令程序：与 Host RobotProgramCatalog JSON 对齐
let programCatalogs = []; // [{sceneBackendId, activeProgramId, programs:[{id,name,isMain,instructions,groups}]}]
let selectedInstrId = null;
let arcViaPending = null; // {pose, eulerDeg, jointRadCsv} | null
let instrIdSeq = 1;
let programPlaying = false;
let programAbort = false;
let trajPickMode = null; // "edge" | "face" | null
let trajFeatures = []; // FeatureEntry-like
let trajFeatSel = -1;
let trajAppendMode = true;
let trajPipelineOps = [];
let trajOpSel = -1;
let rawPreviewActive = false;
let lastRawPreview = null; // { pointsMm, eulersDeg, pointCount }
let featStrategyCatalog = []; // [{strategyId,displayNameZh,affinity}]
let featSchemaCache = {}; // strategyId -> schema response
let featParamSuppress = false;
let autoDiscTimer = null;
/** 对齐桌面 m_featureEditActive：未开始修改时路径只读 */
let trajFeatureEditActive = false;

const CMD_LABEL = {
  ptp: "点到点",
  line: "直线",
  arc: "圆弧",
  wait: "等待",
  if: "条件",
  while: "循环",
  set_do: "数字输出",
  set_ao: "模拟输出",
  path_plan: "路径规划",
};

const VIEW_BG = 0xe8eaed;
const scene = new THREE.Scene();
// CAD 浅灰底：比中灰更亮，材质才抬得起来
scene.background = new THREE.Color(VIEW_BG);
const camera = new THREE.PerspectiveCamera(50, 1, 1, 1e7);
camera.position.set(800, 600, 1000);
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
renderer.setClearColor(VIEW_BG, 1);
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.NoToneMapping;
sceneMount.appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
// 对齐常见 CAD：中键平移，左键旋转，滚轮缩放
controls.mouseButtons.LEFT = THREE.MOUSE.ROTATE;
controls.mouseButtons.MIDDLE = THREE.MOUSE.PAN;
controls.mouseButtons.RIGHT = THREE.MOUSE.DOLLY;
// r160+：TransformControls 不再是 Object3D，必须挂 getHelper() 才看得见罗盘
const transform = new TransformControls(camera, renderer.domElement);
transform.setSize(1.25);
transform.setMode("translate");
transform.setSpace("local");
transform.enabled = false;
transform.addEventListener("dragging-changed", (e) => {
  controls.enabled = !e.value;
  if (e.value) {
    window.__tcpTranslateOriLock = null;
    window.__tcpTranslateOriArming = false;
    if (robotDragMode && transform.getMode() === "translate") {
      window.__tcpTranslateOriArming = true;
      void (async () => {
        if (selectedRobot?.sceneRootBackendId) {
          try {
            const pose = await (
              await fetch(
                `/api/robot/tcp-pose?sceneRootBackendId=${encodeURIComponent(selectedRobot.sceneRootBackendId)}`
              )
            ).json();
            if (pose.ok) applyDragProxyFromWorldMatrix16(pose.worldMatrix);
          } catch (_) {}
        }
        window.__tcpTranslateOriLock = dragProxy.quaternion.clone();
        window.__tcpTranslateOriArming = false;
        followActiveToolOverlayToDragProxy();
        posePushPending = true;
        void flushPosePush();
      })();
    }
    return;
  }
  void (async () => {
    posePushPending = true;
    await flushPosePush();
    for (let i = 0; i < 60 && tcpIkIncomplete; ++i) {
      posePushPending = true;
      await flushPosePush();
    }
    window.__tcpTranslateOriLock = null;
    window.__tcpTranslateOriArming = false;
    if (robotDragMode && dragFlangeId) {
      await refreshFrameOverlays();
      transform.enabled = true;
      transform.attach(dragProxy);
      transform.getHelper().visible = true;
    } else if (sceneInteractMode === "select" && isRobotSceneSelection(selectedId)) {
      snapPlaceProxyToAnchor();
      transform.enabled = true;
      transform.attach(dragProxy);
      transform.getHelper().visible = true;
    }
  })();
});
transform.addEventListener("objectChange", () => {
  if (suppressPosePush) return;
  if (robotDragMode && dragFlangeId) {
    if (transform.getMode() === "translate" && window.__tcpTranslateOriLock) {
      dragProxy.quaternion.copy(window.__tcpTranslateOriLock);
      dragProxy.updateMatrix();
    }
    followActiveToolOverlayToDragProxy();
    posePushPending = true;
    void flushPosePush();
    return;
  }
  // 对象选择：整机放置 / 普通物体位姿
  if (sceneInteractMode === "select" && transform.object) {
    posePushPending = true;
    void flushPosePush();
  }
});
scene.add(transform.getHelper());
// 半球填光 + 主/辅方向光：贴近桌面 HEADLIGHT 的可读性，避免 Standard 无 envMap 发闷
scene.add(new THREE.HemisphereLight(0xffffff, 0xb8bcc2, 0.9));
const keyLight = new THREE.DirectionalLight(0xffffff, 1.2);
keyLight.position.set(500, 900, 400);
scene.add(keyLight);
const fillLight = new THREE.DirectionalLight(0xf2f5fa, 0.55);
fillLight.position.set(-700, 350, -500);
scene.add(fillLight);
const rimLight = new THREE.DirectionalLight(0xffffff, 0.28);
rimLight.position.set(100, 200, -900);
scene.add(rimLight);
scene.add(new THREE.GridHelper(2000, 20, 0xa8adb4, 0xcdd1d6));
const root = new THREE.Group();
root.applyMatrix4(zUpToYUp);
scene.add(root);
// 末端罗盘代理：挂在 root 下，与连杆同坐标系，避免直接拧 mesh 矩阵
const dragProxy = new THREE.Object3D();
dragProxy.name = "robotDragProxy";
root.add(dragProxy);

const trajGeo = new THREE.BufferGeometry();
const trajLine = new THREE.Line(trajGeo, new THREE.LineBasicMaterial({ color: 0x33d9ff }));
trajLine.visible = false;
trajLine.frustumCulled = false;
// 与桌面 RawTrajectoryOverlay 对齐：按段折线 + 全量采样点
const rawPreviewGroup = new THREE.Group();
rawPreviewGroup.name = "rawPreviewOverlay";
rawPreviewGroup.visible = false;
root.add(rawPreviewGroup);
root.add(trajLine);
const pickHighlight = new THREE.Group();
pickHighlight.name = "pickHighlight";
root.add(pickHighlight);
// 示教点坐标轴：与连杆同 Z-up 局部系，rebuild 时保留
const instrMarkers = new THREE.Group();
instrMarkers.name = "instrMarkers";
root.add(instrMarkers);
const frameOverlays = new THREE.Group();
frameOverlays.name = "frameOverlays";
root.add(frameOverlays);

function resize() {
  const w = Math.max(sceneMount.clientWidth, 1);
  const h = Math.max(sceneMount.clientHeight, 1);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h, false);
}
window.addEventListener("resize", resize);
if (typeof ResizeObserver !== "undefined") {
  new ResizeObserver(() => resize()).observe(sceneMount);
}
resize();

function setStatus(text, level = "info") {
  statusEl.textContent = text;
  appendLog(text, level);
}

function appendLog(text, level = "info") {
  if (!logEl) return;
  const t = new Date();
  const hh = String(t.getHours()).padStart(2, "0");
  const mm = String(t.getMinutes()).padStart(2, "0");
  const ss = String(t.getSeconds()).padStart(2, "0");
  const line = document.createElement("div");
  line.className = `line ${level}`;
  line.textContent = `[${hh}:${mm}:${ss}] [${level.toUpperCase()}] ${text}`;
  logEl.appendChild(line);
  logEl.scrollTop = logEl.scrollHeight;
}

function eulerZyxDegToQuat(ex, ey, ez) {
  const qx = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), THREE.MathUtils.degToRad(ex));
  const qy = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), THREE.MathUtils.degToRad(ey));
  const qz = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), THREE.MathUtils.degToRad(ez));
  return qz.multiply(qy).multiply(qx);
}

function colorFromObject(o, selected) {
  const c = o?.color;
  if (c && Number.isFinite(c.r) && Number.isFinite(c.g) && Number.isFinite(c.b)) {
    let r = c.r;
    let g = c.g;
    let b = c.b;
    // 兼容 0–255 误传
    if (r > 1 || g > 1 || b > 1) {
      r /= 255;
      g /= 255;
      b /= 255;
    }
    const col = new THREE.Color(r, g, b);
    // 极暗固有色略抬亮度，避免 FANUC 黄被压成泥色
    const hsl = { h: 0, s: 0, l: 0 };
    col.getHSL(hsl);
    if (hsl.l < 0.22) col.offsetHSL(0, 0, 0.22 - hsl.l);
    if (selected) col.offsetHSL(0, 0.04, 0.08);
    return col;
  }
  return new THREE.Color(selected ? 0x2b79c2 : 0xb0b4b8);
}

function createMeshMaterial(color, selected) {
  return new THREE.MeshPhongMaterial({
    color,
    specular: new THREE.Color(0x2e2e2e),
    shininess: 42,
    side: THREE.DoubleSide,
    emissive: new THREE.Color(selected ? 0x1a4060 : 0x000000),
    emissiveIntensity: selected ? 0.28 : 0,
  });
}

function applyMeshSelectionStyle(mesh, o, selected) {
  if (!mesh?.material) return;
  mesh.material.color.copy(colorFromObject(o, selected));
  if (mesh.material.emissive) {
    mesh.material.emissive.setHex(selected ? 0x1a4060 : 0x000000);
    mesh.material.emissiveIntensity = selected ? 0.28 : 0;
  }
}

function applyObjectTransform(mesh, o) {
  const wm = o.worldMatrix;
  // Gateway 已转为 Three/OpenGL 列主序（平移在 12..14）
  if (Array.isArray(wm) && wm.length === 16) {
    mesh.matrixAutoUpdate = false;
    mesh.matrix.fromArray(wm.map(Number));
    // 与 position 同步，避免后续 prepareMeshForGizmo / gizmo 读到旧分解
    mesh.matrix.decompose(mesh.position, mesh.quaternion, mesh.scale);
    mesh.updateMatrixWorld(true);
    return;
  }
  mesh.matrixAutoUpdate = true;
  const p = o.pose?.positionMm || [0, 0, 0];
  const e = o.pose?.eulerDeg || [0, 0, 0];
  mesh.position.set(p[0], p[1], p[2]);
  mesh.quaternion.copy(eulerZyxDegToQuat(e[0], e[1], e[2]));
  mesh.scale.set(1, 1, 1);
}

/// 场景 FrameBackendData：客户端合成 RGB 短轴（无 mesh soup）
function makeCoordinateFrameAxes(axisLengthMm = 100) {
  const len = Math.max(1, Number(axisLengthMm) || 100);
  const g = new THREE.Group();
  const mk = (dir, color) => {
    const geo = new THREE.BufferGeometry().setFromPoints([
      new THREE.Vector3(0, 0, 0),
      new THREE.Vector3(dir[0] * len, dir[1] * len, dir[2] * len),
    ]);
    return new THREE.Line(geo, new THREE.LineBasicMaterial({ color, depthTest: true }));
  };
  g.add(mk([1, 0, 0], 0xe53935));
  g.add(mk([0, 1, 0], 0x43a047));
  g.add(mk([0, 0, 1], 0x1e88e5));
  g.userData.isCoordinateFrame = true;
  return g;
}

function isSceneCoordinateFrame(o) {
  if (!o) return false;
  const c = String(o.className || "");
  return c === "FrameBackendData" || c === "CoordinateFrame" || c === "Frame";
}

function prepareMeshForGizmo(mesh) {
  if (!mesh.matrixAutoUpdate) {
    mesh.matrix.decompose(mesh.position, mesh.quaternion, mesh.scale);
    mesh.matrixAutoUpdate = true;
  }
}

async function pickPath(purpose, extra = {}) {
  setStatus("请在弹出的系统对话框中选择…（若被挡住请看任务栏 CloudSimWeb）");
  let r;
  try {
    const resp = await fetch("/api/dialog/open", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ purpose, startDir: pathEl.value.trim() || undefined, ...extra }),
    });
    r = await resp.json();
  } catch (e) {
    setStatus(`对话框请求失败: ${e.message || e}`, "err");
    return null;
  }
  if (r.cancelled) {
    setStatus("已取消选择", "warn");
    return null;
  }
  if (!r.ok) {
    setStatus(r.error || "对话框失败", "err");
    return null;
  }
  return r.path || null;
}

async function openProjectAt(path) {
  if (!path) return;
  pathEl.value = path;
  setStatus(`正在打开 ${path} …`);
  let r;
  try {
    const resp = await fetch("/api/project/open", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ path }),
    });
    r = await resp.json();
  } catch (e) {
    setStatus(`打开请求失败: ${e.message || e}`, "err");
    return;
  }
  if (!r.ok) {
    setStatus(`打开失败: ${r.error || "未知错误"}`, "err");
    return;
  }
  const name = path.split(/[/\\]/).pop() || "工程";
  if (docTabTitle) docTabTitle.textContent = name;
  setStatus(`已加载 ${r.objectCount} 对象`);
  await refreshObjects(true);
  await loadModes();
}

let robotSceneRootIds = new Set(); // 机器人场景根：单元树只露这一层
let treeExpandedIds = new Set(); // 用户展开的节点

async function refreshRobotSceneRoots() {
  try {
    const inst = await (await fetch("/api/robot/instances")).json();
    const objectIds = new Set(objects.map((o) => o.id));
    // 场景里已无对应根时不计实例，避免删机后轴控制/坐标系仍挂幽灵
    robotSceneRootIds = new Set(
      (inst.instances || [])
        .map((i) => i.sceneRootBackendId)
        .filter((id) => id && (!objectIds.size || objectIds.has(id)))
    );
  } catch {
    robotSceneRootIds = new Set();
  }
}

function buildObjectChildMap() {
  const childMap = new Map();
  const ensure = (id) => {
    if (!childMap.has(id)) childMap.set(id, []);
    return childMap.get(id);
  };
  for (const o of objects) {
    for (const c of o.childIds || []) {
      const list = ensure(o.id);
      if (!list.includes(c)) list.push(c);
    }
    for (const p of o.parentIds || []) {
      const list = ensure(p);
      if (!list.includes(o.id)) list.push(o.id);
    }
  }
  return childMap;
}

function collectSubtreeIds(rootId, childMap) {
  const out = new Set();
  const stack = [...(childMap.get(rootId) || [])];
  while (stack.length) {
    const id = stack.pop();
    if (!id || out.has(id) || id === rootId) continue;
    out.add(id);
    for (const c of childMap.get(id) || []) stack.push(c);
  }
  return out;
}

/// 机器人连杆不进顶层列表；无父子数据时按常见 URDF 名兜底
function robotLinkIdsHiddenByDefault() {
  const hide = new Set();
  if (!robotSceneRootIds.size) return hide;
  const childMap = buildObjectChildMap();
  for (const rootId of robotSceneRootIds) {
    for (const id of collectSubtreeIds(rootId, childMap)) hide.add(id);
  }
  if (hide.size) return hide;
  const byId = new Map(objects.map((o) => [o.id, o]));
  for (const o of objects) {
    if (robotSceneRootIds.has(o.id)) continue;
    let cur = o;
    const seen = new Set();
    while (cur) {
      const parents = cur.parentIds || [];
      if (parents.some((p) => robotSceneRootIds.has(p))) {
        hide.add(o.id);
        break;
      }
      const p0 = parents[0];
      if (!p0 || seen.has(p0)) break;
      seen.add(p0);
      cur = byId.get(p0);
    }
  }
  if (hide.size) return hide;
  for (const o of objects) {
    if (robotSceneRootIds.has(o.id)) continue;
    const n = String(o.name || "").toLowerCase();
    if (/^(base_link|link_\d+|tool0|flange)$/.test(n)) hide.add(o.id);
  }
  return hide;
}

function robotRootOwning(backendId) {
  if (!backendId) return null;
  if (robotSceneRootIds.has(backendId)) return backendId;
  const childMap = buildObjectChildMap();
  for (const rootId of robotSceneRootIds) {
    if (collectSubtreeIds(rootId, childMap).has(backendId)) return rootId;
  }
  const byId = new Map(objects.map((o) => [o.id, o]));
  let cur = byId.get(backendId);
  const seen = new Set();
  while (cur) {
    for (const p of cur.parentIds || []) {
      if (robotSceneRootIds.has(p)) return p;
    }
    const p0 = (cur.parentIds || [])[0];
    if (!p0 || seen.has(p0)) break;
    seen.add(p0);
    cur = byId.get(p0);
  }
  // 无父子边时：单实例 + URDF 连杆名 → 挂到该根
  if (robotSceneRootIds.size === 1) {
    const n = String(byId.get(backendId)?.name || "").toLowerCase();
    if (/^(base_link|link_\d+|tool0|flange)$/.test(n)) return [...robotSceneRootIds][0];
  }
  return null;
}

/// 点任何连杆/法兰都归到机器人场景根
function resolveScenePickId(backendId) {
  if (!backendId) return backendId;
  return robotRootOwning(backendId) || backendId;
}

function selectionHighlightsId(mid) {
  if (!selectedId || !mid) return false;
  if (mid === selectedId) return true;
  if (robotSceneRootIds.has(selectedId) && robotRootOwning(mid) === selectedId) return true;
  return false;
}

function isRobotSceneSelection(id) {
  if (!id) return false;
  return robotSceneRootIds.has(id) || !!robotRootOwning(id);
}

function clearObjectTransformGizmo() {
  if (robotDragMode && dragFlangeId) return;
  transform.detach();
  transform.enabled = false;
  transform.getHelper().visible = false;
}

function snapPlaceProxyToAnchor() {
  const anchorId = selectedRobot?.anchorBackendId;
  const mesh = anchorId ? idToMesh.get(anchorId) : null;
  if (!mesh) return false;
  mesh.updateMatrixWorld(true);
  root.updateMatrixWorld(true);
  const local = new THREE.Matrix4().copy(root.matrixWorld).invert().multiply(mesh.matrixWorld);
  local.decompose(dragProxy.position, dragProxy.quaternion, dragProxy.scale);
  dragProxy.matrixAutoUpdate = true;
  dragProxy.updateMatrix();
  dragProxy.updateMatrixWorld(true);
  return true;
}

function attachObjectGizmoIfNeeded() {
  if (robotDragMode) return;
  if (sceneInteractMode !== "select" || !selectedId) {
    clearObjectTransformGizmo();
    return;
  }
  // 机器人：罗盘挂锚点代理，整机平移/旋转走 /api/robot/place
  if (isRobotSceneSelection(selectedId)) {
    if (!selectedRobot?.anchorBackendId || !snapPlaceProxyToAnchor()) {
      clearObjectTransformGizmo();
      return;
    }
    transform.enabled = true;
    transform.attach(dragProxy);
    transform.getHelper().visible = true;
    return;
  }
  const mesh = idToMesh.get(selectedId);
  if (!mesh) {
    clearObjectTransformGizmo();
    return;
  }
  prepareMeshForGizmo(mesh);
  transform.enabled = true;
  transform.attach(mesh);
  transform.setMode("translate");
  transform.getHelper().visible = true;
}

function syncInteractModeMenu() {
  $("btnInteractView")?.classList.toggle("on", sceneInteractMode === "view");
  $("btnInteractSelect")?.classList.toggle("on", sceneInteractMode === "select");
}

function setSceneInteractMode(mode) {
  sceneInteractMode = mode === "select" ? "select" : "view";
  syncInteractModeMenu();
  if (sceneInteractMode === "view") {
    if (!robotDragMode) clearObjectTransformGizmo();
    setStatus("视图移动：拖拽旋转/平移视图");
  } else {
    attachObjectGizmoIfNeeded();
    setStatus("对象选择：点击场景选中对象（机器人点任意连杆=整机）");
  }
}

function fillTree(el) {
  if (!el) return;
  el.innerHTML = "";
  const childMap = buildObjectChildMap();
  const hiddenLinks = robotLinkIdsHiddenByDefault();

  const appendRow = (o, isChild) => {
    const isRobotRoot = robotSceneRootIds.has(o.id);
    const li = document.createElement("li");
    const sel =
      o.id === selectedId ||
      (isRobotRoot && selectedId === o.id) ||
      (isRobotRoot && robotRootOwning(selectedId) === o.id);
    if (sel) li.classList.add("sel");
    if (isChild) li.classList.add("tree-child");

    const subIds = isRobotRoot ? collectSubtreeIds(o.id, childMap) : new Set();
    const linkCount = isRobotRoot ? subIds.size || [...hiddenLinks].filter((id) => robotRootOwning(id) === o.id).length : 0;
    const expanded = isRobotRoot && treeExpandedIds.has(o.id);
    const cls = isRobotRoot ? (linkCount ? `机器人 · ${linkCount} 连杆` : "机器人") : o.className || "";

    if (isRobotRoot && linkCount > 0) {
      const twist = document.createElement("button");
      twist.type = "button";
      twist.className = "tree-twist";
      twist.textContent = expanded ? "▼" : "▶";
      twist.title = expanded ? "折叠连杆" : "展开连杆";
      twist.onclick = (ev) => {
        ev.stopPropagation();
        if (treeExpandedIds.has(o.id)) treeExpandedIds.delete(o.id);
        else treeExpandedIds.add(o.id);
        fillTree(el);
      };
      li.appendChild(twist);
    } else {
      const pad = document.createElement("span");
      pad.className = "tree-twist-spacer";
      li.appendChild(pad);
    }

    const body = document.createElement("span");
    body.className = "tree-body";
    body.innerHTML = `<span class="tree-name"></span><span class="cls"></span>`;
    body.querySelector(".tree-name").textContent = o.name || o.id;
    body.querySelector(".cls").textContent = cls;
    li.appendChild(body);
    li.onclick = () => selectObject(o.id);
    li.oncontextmenu = (ev) => {
      ev.preventDefault();
      void selectObject(o.id);
      showUnitsContextMenu(ev.clientX, ev.clientY, o.id);
    };
    el.appendChild(li);
  };

  for (const o of objects) {
    if (hiddenLinks.has(o.id)) continue;
    appendRow(o, false);
    if (!robotSceneRootIds.has(o.id) || !treeExpandedIds.has(o.id)) continue;
    const kids = collectSubtreeIds(o.id, childMap);
    const kidSet = kids.size ? kids : new Set([...hiddenLinks].filter((id) => robotRootOwning(id) === o.id));
    for (const child of objects) {
      if (kidSet.has(child.id)) appendRow(child, true);
    }
  }
}

function hideUnitsContextMenu() {
  const menu = $("unitsCtxMenu");
  if (menu) menu.classList.add("hidden");
}

function showUnitsContextMenu(x, y, backendId) {
  let menu = $("unitsCtxMenu");
  if (!menu) {
    menu = document.createElement("div");
    menu.id = "unitsCtxMenu";
    menu.className = "ctx-menu hidden";
    document.body.appendChild(menu);
    document.addEventListener("click", hideUnitsContextMenu);
    document.addEventListener("contextmenu", (e) => {
      if (!menu.contains(e.target) && e.target.closest("#treeUnits") == null) hideUnitsContextMenu();
    });
  }
  menu.innerHTML = "";
  const addItem = (label, fn, danger) => {
    const btn = document.createElement("button");
    btn.type = "button";
    btn.textContent = label;
    if (danger) btn.className = "danger";
    btn.onclick = () => {
      hideUnitsContextMenu();
      void fn();
    };
    menu.appendChild(btn);
  };
  addItem("聚焦", () => focusObject(backendId));
  addItem("复制 ID", async () => {
    try {
      await navigator.clipboard.writeText(backendId);
      setStatus("已复制 ID");
    } catch {
      setStatus(backendId);
    }
  });
  addItem("删除", () => deleteObject(backendId), true);
  menu.classList.remove("hidden");
  const pad = 4;
  const mw = menu.offsetWidth || 160;
  const mh = menu.offsetHeight || 100;
  menu.style.left = `${Math.min(x, window.innerWidth - mw - pad)}px`;
  menu.style.top = `${Math.min(y, window.innerHeight - mh - pad)}px`;
}

async function focusObject(backendId) {
  const box = new THREE.Box3();
  let any = false;
  const add = (id) => {
    const mesh = idToMesh.get(id);
    if (!mesh) return;
    box.expandByObject(mesh);
    any = true;
  };
  add(backendId);
  if (robotSceneRootIds.has(backendId)) {
    for (const id of collectSubtreeIds(backendId, buildObjectChildMap())) add(id);
    if (!any) {
      for (const id of robotLinkIdsHiddenByDefault()) {
        if (robotRootOwning(id) === backendId) add(id);
      }
    }
  }
  if (!any) {
    setStatus("该对象无可见网格", "warn");
    return;
  }
  focusBox(box);
}

async function deleteObject(backendId) {
  if (!backendId) return;
  if (!window.confirm(`删除对象「${backendId}」及其子节点？`)) return;
  const deletedRobotRoot = resolveScenePickId(backendId);
  const wasRobot = !!deletedRobotRoot && (robotSceneRootIds.has(deletedRobotRoot) || robotSceneRootIds.has(backendId));
  const r = await (await fetch(`/api/objects/${encodeURIComponent(backendId)}`, { method: "DELETE" })).json();
  if (!r.ok) {
    setStatus(r.error || "删除失败", "err");
    return;
  }
  if (selectedId === backendId || selectedId === deletedRobotRoot || robotRootOwning(selectedId) === deletedRobotRoot) {
    selectedId = null;
    selectedRobot = null;
    clearObjectTransformGizmo();
  }
  if (dragFlangeId === backendId || (wasRobot && robotDragMode)) {
    robotDragMode = false;
    detachDragCompass();
    syncDragButtonUi();
  }
  if (wasRobot) {
    // 先清叠加与面板，避免 refresh 期间用残留 robotRoot 重画 TCP/用户系
    if ($("robotRoot") && ($("robotRoot").value.trim() === deletedRobotRoot || $("robotRoot").value.trim() === backendId)) {
      $("robotRoot").value = "";
    }
    clearCoordinateFramesUi();
  }
  setStatus("已删除");
  await refreshObjects(false);
  scrubStaleRobotRootRefs();
  void loadAxisControl();
  await loadCoordinateFrames();
}

function renderTree() {
  fillTree(treeUnitsEl);
}

function meshLocalMatrixArray(obj) {
  obj.updateMatrix();
  return Array.from(obj.matrix.elements);
}

function applyDragProxyFromWorldMatrix16(wm16) {
  if (!Array.isArray(wm16) || wm16.length < 16) return false;
  const m = new THREE.Matrix4().fromArray(wm16.map(Number));
  m.decompose(dragProxy.position, dragProxy.quaternion, dragProxy.scale);
  dragProxy.matrixAutoUpdate = true;
  dragProxy.updateMatrix();
  dragProxy.updateMatrixWorld(true);
  return true;
}

/// 与可见工具系叠加同一数据源（overlays），避免 tcp-pose 与叠加分叉导致罗盘漂移
async function fetchActiveToolWorldMatrix() {
  const rootId = activeSceneRootId();
  if (!rootId) return null;
  const q = encodeURIComponent(rootId);
  try {
    const r = await (await fetch(`/api/robot/frames/overlays?sceneRootBackendId=${q}`)).json();
    if (r.ok) {
      const tools = r.tools || [];
      const active = tools.find((t) => t.active && Array.isArray(t.worldMatrix) && t.worldMatrix.length >= 16);
      if (active) return active.worldMatrix;
      const any = tools.find((t) => Array.isArray(t.worldMatrix) && t.worldMatrix.length >= 16);
      if (any) return any.worldMatrix;
    }
  } catch {
    /* fall through */
  }
  try {
    const pose = await (await fetch(`/api/robot/tcp-pose?sceneRootBackendId=${q}`)).json();
    if (pose.ok && Array.isArray(pose.worldMatrix) && pose.worldMatrix.length >= 16) return pose.worldMatrix;
  } catch {
    /* ignore */
  }
  return null;
}

/// 罗盘跟可见末端 TCP（overlays 激活工具系）
async function snapDragProxyToActiveTcp() {
  if (transform.dragging) return false;
  const wm = await fetchActiveToolWorldMatrix();
  if (transform.dragging) return false;
  const wantAttach = robotDragMode && dragFlangeId;
  const wasOnProxy = transform.object === dragProxy;
  if (wasOnProxy) transform.detach();
  let ok = applyDragProxyFromWorldMatrix16(wm);
  if (!ok) ok = snapDragProxyToFlange(dragFlangeId);
  if (ok && (wasOnProxy || wantAttach) && !transform.dragging) {
    transform.enabled = true;
    transform.attach(dragProxy);
    transform.getHelper().visible = true;
  }
  return ok;
}

function snapDragProxyToFlange(flangeId) {
  const mesh = idToMesh.get(flangeId);
  if (!mesh) return false;
  mesh.updateMatrixWorld(true);
  root.updateMatrixWorld(true);
  const local = new THREE.Matrix4().copy(root.matrixWorld).invert().multiply(mesh.matrixWorld);
  local.decompose(dragProxy.position, dragProxy.quaternion, dragProxy.scale);
  dragProxy.matrixAutoUpdate = true;
  dragProxy.updateMatrix();
  dragProxy.updateMatrixWorld(true);
  return true;
}

async function attachDragCompass(flangeId) {
  dragFlangeId = flangeId;
  const ok = (await snapDragProxyToActiveTcp()) || snapDragProxyToFlange(flangeId);
  if (!ok) return false;
  transform.enabled = true;
  transform.attach(dragProxy);
  transform.setMode("translate");
  // 末端拖动固定 TCP 局部轴，对齐桌面示教
  transform.setSpace("local");
  transform.getHelper().visible = true;
  return true;
}

function detachDragCompass() {
  transform.detach();
  transform.enabled = false;
  transform.getHelper().visible = false;
  dragFlangeId = null;
}

async function flushPosePush() {
  if (posePushInFlight || !posePushPending) return;
  posePushInFlight = true;
  try {
    while (posePushPending) {
      posePushPending = false;
      await pushSelectedPoseOnce();
    }
  } finally {
    posePushInFlight = false;
    if (posePushPending) void flushPosePush();
  }
}

function applyAxisAnglesFromServer(jointAnglesRad) {
  if (!Array.isArray(jointAnglesRad) || !axisJointsState.length) return;
  const n = Math.min(jointAnglesRad.length, axisJointsState.length);
  for (let i = 0; i < n; ++i) {
    axisJointsState[i].angleRad = Number(jointAnglesRad[i]);
    clampAxis(i);
    syncAxisRowInputs(i);
  }
  if ($("jointsCsv2")) {
    $("jointsCsv2").value = axisJointsState.map((j) => j.angleRad.toFixed(6)).join(",");
  }
}

async function pushSelectedPoseOnce() {
  // 拖拽模式：末端世界位姿 → IK 更新关节
  if (robotDragMode && dragFlangeId) {
    dragProxy.updateMatrix();
    // 平移锁姿态，松手不拧朝向
    if (transform.getMode() === "translate") {
      if (window.__tcpTranslateOriArming) return;
      if (!window.__tcpTranslateOriLock) {
        window.__tcpTranslateOriLock = dragProxy.quaternion.clone();
      } else {
        dragProxy.quaternion.copy(window.__tcpTranslateOriLock);
        dragProxy.updateMatrix();
      }
    } else {
      window.__tcpTranslateOriLock = null;
    }
    const worldMatrix = meshLocalMatrixArray(dragProxy);
    const r = await (
      await fetch("/api/robot/tcp-ik", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          flangeBackendId: dragFlangeId,
          worldMatrix,
          translateOnly: transform.getMode() === "translate",
        }),
      })
    ).json();
    if (!r.ok) {
      setStatus(r.error || "末端 IK 失败", "err");
      return;
    }
    axisSyncQuietUntil = performance.now() + 120;
    applyAxisAnglesFromServer(r.jointAnglesRad);
    tcpIkIncomplete = !!r.incomplete;
    // incomplete 时继续追赶（对齐 React / 桌面）
    if (tcpIkIncomplete && transform.dragging) {
      posePushPending = true;
      setTimeout(() => void flushPosePush(), 8);
    }
    await syncObjectTransforms();
    if (transform.dragging) {
      followActiveToolOverlayToDragProxy();
    } else {
      await refreshFrameOverlays();
    }
    return;
  }

  // 对象选择 + 机器人：锚点世界阵 → 整机 basePlacement
  if (
    sceneInteractMode === "select" &&
    isRobotSceneSelection(selectedId) &&
    selectedRobot?.anchorBackendId &&
    transform.object === dragProxy
  ) {
    dragProxy.updateMatrix();
    const worldMatrix = meshLocalMatrixArray(dragProxy);
    const r = await (
      await fetch("/api/robot/place", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          anchorBackendId: selectedRobot.anchorBackendId,
          worldMatrix,
        }),
      })
    ).json();
    if (!r.ok) {
      setStatus(r.error || "整机放置失败", "err");
      return;
    }
    axisSyncQuietUntil = performance.now() + 120;
    await syncObjectTransforms();
    if (!transform.dragging) {
      snapPlaceProxyToAnchor();
      transform.enabled = true;
      transform.attach(dragProxy);
      transform.getHelper().visible = true;
    }
    return;
  }

  const obj = transform.object;
  if (!obj || !selectedId || obj === dragProxy) return;
  const e = new THREE.Euler().setFromQuaternion(obj.quaternion, "ZYX");
  const r = await (
    await fetch(`/api/objects/${encodeURIComponent(selectedId)}`, {
      method: "PATCH",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        pose: {
          positionMm: [obj.position.x, obj.position.y, obj.position.z],
          eulerDeg: [THREE.MathUtils.radToDeg(e.x), THREE.MathUtils.radToDeg(e.y), THREE.MathUtils.radToDeg(e.z)],
        },
      }),
    })
  ).json();
  if (r.ok) {
    axisSyncQuietUntil = performance.now() + 120;
    await syncObjectTransforms();
  }
}

async function resolveSelectedRobot(backendId) {
  selectedRobot = null;
  if (!backendId) return null;
  try {
    const r = await (await fetch(`/api/robot/resolve?backendId=${encodeURIComponent(backendId)}`)).json();
    if (r.ok && r.isRobot) {
      selectedRobot = {
        sceneRootBackendId: r.sceneRootBackendId,
        anchorBackendId: r.anchorBackendId || backendId,
        flangeBackendId: r.flangeBackendId || r.anchorBackendId || backendId,
      };
      if ($("robotRoot")) $("robotRoot").value = r.sceneRootBackendId || "";
      if (r.sceneRootBackendId) {
        for (const id of ["axisInstance", "frameInstance"]) {
          const sel = $(id);
          if (sel && [...sel.options].some((o) => o.value === r.sceneRootBackendId)) sel.value = r.sceneRootBackendId;
        }
      }
      return r;
    }
  } catch {
    selectedRobot = null;
  }
  return null;
}

function syncDragButtonUi() {
  const btn = $("btnDrag");
  if (btn) btn.classList.toggle("active", robotDragMode);
}

async function pickFlangeMeshId(sceneRootBackendId, preferredFlangeId) {
  if (preferredFlangeId && idToMesh.has(preferredFlangeId)) return preferredFlangeId;
  for (const [mid] of idToMesh) {
    const rr = await (await fetch(`/api/robot/resolve?backendId=${encodeURIComponent(mid)}`)).json();
    if (rr.ok && rr.isRobot && rr.sceneRootBackendId === sceneRootBackendId) {
      if (rr.flangeBackendId && idToMesh.has(rr.flangeBackendId)) return rr.flangeBackendId;
    }
  }
  for (const [mid] of idToMesh) {
    const rr = await (await fetch(`/api/robot/resolve?backendId=${encodeURIComponent(mid)}`)).json();
    if (rr.ok && rr.isRobot && rr.sceneRootBackendId === sceneRootBackendId) return mid;
  }
  return null;
}

async function setRobotDragMode(on) {
  if (!on) {
    robotDragMode = false;
    detachDragCompass();
    syncDragButtonUi();
    attachObjectGizmoIfNeeded();
    setStatus("已退出拖拽模式");
    return;
  }

  let rootId =
    ($("robotRoot") && $("robotRoot").value.trim()) ||
    ($("axisInstance") && $("axisInstance").value) ||
    (selectedRobot && selectedRobot.sceneRootBackendId) ||
    "";
  if (!rootId) {
    try {
      const inst = await (await fetch("/api/robot/instances")).json();
      if (inst.instances && inst.instances.length) rootId = inst.instances[0].sceneRootBackendId;
    } catch {
      /* ignore */
    }
  }
  if (!rootId) {
    robotDragMode = false;
    syncDragButtonUi();
    setStatus("请先从设备库导入机器人，或打开含机器人的工程", "warn");
    return;
  }

  const r = await resolveSelectedRobot(rootId);
  if (!r || !r.isRobot) {
    robotDragMode = false;
    syncDragButtonUi();
    setStatus("未找到可拖拽的机器人", "warn");
    return;
  }

  const flangeId = await pickFlangeMeshId(r.sceneRootBackendId, r.flangeBackendId || r.anchorBackendId);
  if (!flangeId) {
    robotDragMode = false;
    syncDragButtonUi();
    setStatus("末端连杆网格未就绪，请等场景加载完再开拖拽", "warn");
    return;
  }

  selectedId = flangeId;
  if (!frameState) await loadCoordinateFrames();
  if (!(await attachDragCompass(flangeId))) {
    robotDragMode = false;
    syncDragButtonUi();
    setStatus("无法在末端挂载罗盘", "err");
    return;
  }

  robotDragMode = true;
  syncDragButtonUi();
  setStatus("拖拽模式：罗盘在末端，拖动可移动整机（G 平移 / R 旋转；再点「拖拽」关闭）");
}

function showPane(id, visible) {
  const el = $(id);
  if (el) el.classList.toggle("hidden", !visible);
}

function bindUiChrome() {
  document.querySelectorAll(".menu-btn").forEach((btn) => {
    btn.onclick = (ev) => {
      ev.stopPropagation();
      const menu = btn.parentElement;
      const open = menu.classList.contains("open");
      document.querySelectorAll(".menu").forEach((m) => m.classList.remove("open"));
      if (!open) menu.classList.add("open");
    };
  });
  document.addEventListener("click", () => {
    document.querySelectorAll(".menu").forEach((m) => m.classList.remove("open"));
  });

  document.querySelectorAll("[data-left-tab]").forEach((btn) => {
    btn.onclick = () => {
      document.querySelectorAll("[data-left-tab]").forEach((b) => b.classList.toggle("active", b === btn));
      const tab = btn.getAttribute("data-left-tab");
      showPane("leftProps", tab === "props");
      showPane("leftDevices", tab === "devices");
      if (tab === "devices") void loadDeviceCatalog();
    };
  });

  document.querySelectorAll("[data-right-primary]").forEach((btn) => {
    btn.onclick = () => {
      document.querySelectorAll("[data-right-primary]").forEach((b) => b.classList.toggle("active", b === btn));
      const tab = btn.getAttribute("data-right-primary");
      showPane("rightWorkspace", tab === "workspace");
      showPane("rightAi", tab === "ai");
      showPane("rightCloud", tab === "cloud");
    };
  });

  document.querySelectorAll("[data-ws-tab]").forEach((btn) => {
    btn.onclick = () => {
      document.querySelectorAll("[data-ws-tab]").forEach((b) => b.classList.toggle("active", b === btn));
      const tab = btn.getAttribute("data-ws-tab");
      showPane("wsUnits", tab === "units");
      showPane("wsRobot", tab === "robot");
    };
  });

  document.querySelectorAll("[data-robot-tab]").forEach((btn) => {
    btn.onclick = () => {
      document.querySelectorAll("[data-robot-tab]").forEach((b) => b.classList.toggle("active", b === btn));
      const tab = btn.getAttribute("data-robot-tab");
      showPane("robotCmd", tab === "cmd");
      showPane("robotJoint", tab === "joint");
      showPane("robotTrajGen", tab === "trajGen");
      showPane("robotTrajEdit", tab === "trajEdit");
      showPane("robotFrame", tab === "frame");
      if (tab === "joint") void loadAxisControl();
      if (tab === "frame") void loadCoordinateFrames();
      if (tab === "trajGen" || tab === "trajEdit") void refreshTrajectoryUi();
    };
  });
  document.querySelectorAll("[data-traj-sub]").forEach((btn) => {
    btn.onclick = () => {
      document.querySelectorAll("[data-traj-sub]").forEach((b) => b.classList.toggle("active", b === btn));
      const sub = btn.getAttribute("data-traj-sub");
      showPane("trajCadPane", sub === "cad");
      const mesh = $("trajMeshPane");
      if (mesh) mesh.classList.toggle("hidden", sub !== "mesh");
    };
  });

  document.querySelectorAll("[data-cmd]").forEach((btn) => {
    btn.onclick = () => void teachAddInstruction(btn.getAttribute("data-cmd"));
  });
}

function activeSceneRootId() {
  // 拖拽/IK 优先 robotRoot（与轴控制写入源一致），frameInstance 仅作坐标系页选择
  return (
    ($("robotRoot") && $("robotRoot").value.trim()) ||
    ($("axisInstance") && $("axisInstance").value) ||
    ($("frameInstance") && $("frameInstance").value) ||
    (selectedRobot && selectedRobot.sceneRootBackendId) ||
    ""
  );
}

/// PathPlan/轨迹会话绑定的是机器人场景根，不是工件；空时回落到首个实例
async function resolveActiveSceneRootId() {
  let rootId = activeSceneRootId();
  if (!rootId) {
    try {
      const inst = await (await fetch("/api/robot/instances")).json();
      const list = (inst.instances || []).filter((i) => i.sceneRootBackendId);
      if (list.length) rootId = list[0].sceneRootBackendId;
    } catch {
      /* ignore */
    }
  }
  if (!rootId) return "";
  // resolve 成功时同步进本地根集合，避免尚未 refreshObjects 时 scrub 误清 robotRoot
  robotSceneRootIds.add(rootId);
  if ($("robotRoot")) $("robotRoot").value = rootId;
  if ($("axisInstance") && [...($("axisInstance").options || [])].some((o) => o.value === rootId)) {
    $("axisInstance").value = rootId;
  }
  if ($("frameInstance") && [...($("frameInstance").options || [])].some((o) => o.value === rootId)) {
    $("frameInstance").value = rootId;
  }
  return rootId;
}

/// robotRoot 被误清、或指到空 catalog 时，改用指令最多的场景根
function instructionCountOfEntry(entry) {
  if (!entry) return 0;
  const programs = entry.programs || [];
  const prog =
    programs.find((p) => p.id === entry.activeProgramId) || programs.find((p) => p.isMain) || programs[0];
  return (prog && Array.isArray(prog.instructions) && prog.instructions.length) || 0;
}

function programEntryFor(sceneRootId) {
  if (!sceneRootId) return null;
  return programCatalogs.find((c) => c.sceneBackendId === sceneRootId) || null;
}

function preferSceneRootForPrograms() {
  const countFor = (sid) => instructionCountOfEntry(programEntryFor(sid));
  let rootId = activeSceneRootId();
  if (rootId && countFor(rootId) > 0) return rootId;

  let best = "";
  let bestN = 0;
  for (const c of programCatalogs) {
    const sid = (c && c.sceneBackendId) || "";
    if (!sid) continue;
    const n = instructionCountOfEntry(c);
    if (n > bestN) {
      bestN = n;
      best = sid;
    }
  }
  if (best) {
    robotSceneRootIds.add(best);
    if ($("robotRoot")) $("robotRoot").value = best;
    if ($("axisInstance") && [...($("axisInstance").options || [])].some((o) => o.value === best)) {
      $("axisInstance").value = best;
    }
    if ($("frameInstance") && [...($("frameInstance").options || [])].some((o) => o.value === best)) {
      $("frameInstance").value = best;
    }
    return best;
  }
  return rootId || "";
}

function ensureCatalogEntry(sceneRootId) {
  // 空 id 只返回临时壳，禁止写入 programCatalogs，避免 PUT 整表时污染
  if (!sceneRootId) {
    return {
      sceneBackendId: "",
      activeProgramId: "main",
      programs: [{ id: "main", name: "Main", isMain: true, instructions: [], groups: [] }],
    };
  }
  let entry = programCatalogs.find((c) => c.sceneBackendId === sceneRootId);
  if (!entry) {
    entry = {
      sceneBackendId: sceneRootId,
      activeProgramId: "main",
      programs: [{ id: "main", name: "Main", isMain: true, instructions: [], groups: [] }],
    };
    programCatalogs.push(entry);
  }
  if (!entry.programs || !entry.programs.length) {
    entry.programs = [{ id: "main", name: "Main", isMain: true, instructions: [], groups: [] }];
  }
  if (!entry.activeProgramId) entry.activeProgramId = entry.programs[0].id;
  return entry;
}

function activeProgram(sceneRootId) {
  const entry = ensureCatalogEntry(sceneRootId);
  let prog = entry.programs.find((p) => p.id === entry.activeProgramId);
  if (!prog) {
    prog = entry.programs.find((p) => p.isMain) || entry.programs[0];
    entry.activeProgramId = prog.id;
  }
  if (!Array.isArray(prog.instructions)) prog.instructions = [];
  if (!Array.isArray(prog.groups)) prog.groups = [];
  return { entry, prog };
}

function newInstrId() {
  return `INS_web_${Date.now()}_${instrIdSeq++}`;
}

function vec3Obj(arr) {
  const a = arr || [0, 0, 0];
  return { x: Number(a[0]) || 0, y: Number(a[1]) || 0, z: Number(a[2]) || 0 };
}

let programsLoadEpoch = 0;
let programsReloadAfterPlay = false;
/** 运行中指令树/路点只读这份快照，避免并发 GET 把 catalog 刷空 */
let playbackRootId = "";
let playbackStepsSnapshot = null;

async function loadProgramsFromServer() {
  if (programPlaying) {
    programsReloadAfterPlay = true;
    return;
  }
  const epoch = ++programsLoadEpoch;
  try {
    const r = await (await fetch("/api/robot/programs")).json();
    if (epoch !== programsLoadEpoch) return;
    // await 期间可能已点了运行：禁止再覆盖本地 catalog
    if (programPlaying) {
      programsReloadAfterPlay = true;
      return;
    }
    programCatalogs = Array.isArray(r) ? r : r.programs || [];
  } catch {
    if (epoch !== programsLoadEpoch || programPlaying) return;
    programCatalogs = [];
  }
  renderProgramList();
  refreshInstructionMarkers();
}

async function saveProgramsToServer() {
  const r = await (
    await fetch("/api/robot/programs", {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(programCatalogs),
    })
  ).json();
  if (!r.ok) {
    setStatus(r.error || "保存程序失败", "err");
    return false;
  }
  return true;
}

function focusLeftPropsTab() {
  const btn = document.querySelector('[data-left-tab="props"]');
  if (btn && !btn.classList.contains("active")) btn.click();
}

function clearInstrMarkerChildren() {
  for (const c of [...instrMarkers.children]) {
    instrMarkers.remove(c);
    c.traverse((o) => {
      o.geometry?.dispose?.();
      const mat = o.material;
      if (Array.isArray(mat)) mat.forEach((m) => m.dispose?.());
      else mat?.dispose?.();
    });
  }
}

function disposeObject3DTree(obj) {
  obj.traverse((o) => {
    o.geometry?.dispose?.();
    const mat = o.material;
    if (Array.isArray(mat)) mat.forEach((m) => m.dispose?.());
    else mat?.dispose?.();
  });
}

function clearRawPreviewOverlay() {
  for (const c of [...rawPreviewGroup.children]) {
    rawPreviewGroup.remove(c);
    disposeObject3DTree(c);
  }
  rawPreviewGroup.visible = false;
  trajLine.visible = false;
}

/// 与桌面 setRawTrajectoryOverlay 一致：按 segmentEndExclusive 拆 LINE_STRIP
function rawPreviewSegmentRanges(pointCount, segmentEndExclusive) {
  if (pointCount < 2) return [];
  const ends = Array.isArray(segmentEndExclusive)
    ? segmentEndExclusive.map((v) => Math.trunc(Number(v))).filter((v) => v > 0 && v <= pointCount)
    : [];
  if (!ends.length) return [[0, pointCount]];
  const ranges = [];
  let start = 0;
  for (const end of ends) {
    if (end > start && end <= pointCount) {
      ranges.push([start, end]);
      start = end;
    }
  }
  if (start + 1 < pointCount) ranges.push([start, pointCount]);
  return ranges;
}

function addPoseMarker(pose, euler, selected, tag) {
  if (!pose) return null;
  const g = new THREE.Group();
  g.userData.instrTag = tag || "";
  g.position.set(Number(pose.x) || 0, Number(pose.y) || 0, Number(pose.z) || 0);
  if (euler) g.quaternion.copy(eulerZyxDegToQuat(Number(euler.x) || 0, Number(euler.y) || 0, Number(euler.z) || 0));
  // 与桌面 OsgWidget::setInstructionPoseAxes 一致：轴 40mm + 固定像素点，不用世界尺寸球体
  const len = selected ? 50 : 40;
  const addAxis = (dir, color) => {
    const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), dir.clone().multiplyScalar(len)]);
    g.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color, depthTest: true })));
  };
  addAxis(new THREE.Vector3(1, 0, 0), 0xff6666);
  addAxis(new THREE.Vector3(0, 1, 0), 0x66ff66);
  addAxis(new THREE.Vector3(0, 0, 1), 0x6699ff);
  const pt = new THREE.BufferGeometry();
  pt.setAttribute("position", new THREE.Float32BufferAttribute([0, 0, 0], 3));
  g.add(
    new THREE.Points(
      pt,
      new THREE.PointsMaterial({
        color: selected ? 0xff6600 : 0x00ff00,
        size: selected ? 6 : 5,
        sizeAttenuation: false,
        depthTest: true,
      })
    )
  );
  instrMarkers.add(g);
  return g.position.clone();
}

function readPreviewAxisOpts() {
  return {
    x: !!$("previewAxisX")?.checked,
    y: !!$("previewAxisY")?.checked,
    z: !!$("previewAxisZ")?.checked,
    interval: Math.max(0, parseInt($("previewAxisInterval")?.value ?? "0", 10) || 0),
  };
}

/// 与桌面 collectPreviewAxisPointIndices 一致：0=约 n/20
function collectPreviewAxisIndices(pointCount, axisInterval) {
  if (pointCount <= 0) return [];
  if (pointCount === 1) return [0];
  const autoStride = Math.max(1, Math.floor(pointCount / 20));
  const stride = axisInterval > 0 ? axisInterval : autoStride;
  const set = new Set([0, pointCount - 1]);
  for (let i = stride; i < pointCount; i += stride) set.add(i);
  return [...set].sort((a, b) => a - b);
}

function addRawAxisMarker(pose, euler, axisOpts, axisDirs) {
  if (!pose) return;
  const g = new THREE.Group();
  g.userData.instrTag = "raw";
  g.position.set(Number(pose.x) || 0, Number(pose.y) || 0, Number(pose.z) || 0);
  // 优先用服务端按桌面 OSG local*R 算出的世界轴；勿再解欧拉（易与 OSG quat 约定不一致）
  const hasDirs = axisDirs && (axisDirs.x || axisDirs.y || axisDirs.z);
  if (!hasDirs && euler) {
    g.quaternion.copy(eulerZyxDegToQuat(Number(euler.x) || 0, Number(euler.y) || 0, Number(euler.z) || 0));
  }
  const len = 40;
  const addAxis = (dir, color) => {
    const d = dir.clone().normalize().multiplyScalar(len);
    const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), d]);
    g.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color, depthTest: true })));
  };
  const localOrWorld = (worldArr, local) => {
    if (Array.isArray(worldArr) && worldArr.length >= 3) {
      return new THREE.Vector3(Number(worldArr[0]) || 0, Number(worldArr[1]) || 0, Number(worldArr[2]) || 0);
    }
    return local;
  };
  if (axisOpts.x) addAxis(localOrWorld(axisDirs?.x, new THREE.Vector3(1, 0, 0)), 0xe53935);
  if (axisOpts.y) addAxis(localOrWorld(axisDirs?.y, new THREE.Vector3(0, 1, 0)), 0x43a047);
  if (axisOpts.z) addAxis(localOrWorld(axisDirs?.z, new THREE.Vector3(0, 0, 1)), 0x1e88e5);
  const pt = new THREE.BufferGeometry();
  pt.setAttribute("position", new THREE.Float32BufferAttribute([0, 0, 0], 3));
  g.add(
    new THREE.Points(
      pt,
      new THREE.PointsMaterial({ color: 0x33d9ff, size: 2, sizeAttenuation: false, depthTest: true })
    )
  );
  instrMarkers.add(g);
}

function refreshBrepInfoLabel() {
  const el = $("trajBrepInfo");
  if (!el) return;
  const f = trajFeatSel >= 0 ? trajFeatures[trajFeatSel] : null;
  if (!f) {
    el.textContent = "未选择特征";
    return;
  }
  const faces = f.geometry?.faceIndices || [];
  const edges = f.geometry?.edgeIndices || [];
  let typeText = "未知";
  if (faces.length && edges.length) typeText = "面+边";
  else if (faces.length) typeText = "面";
  else if (edges.length) typeText = "边";
  const fmt = (arr) => (arr.length ? arr.join(", ") : "(无)");
  el.textContent = `特征: ${f.featureId || "(无)"}\n类型: ${typeText}\nfaceIndices: ${fmt(faces)}\nedgeIndices: ${fmt(edges)}`;
}

function redrawRawPreviewOverlay() {
  const preview = lastRawPreview;
  const ptsArr = preview?.pointsMm || [];
  clearRawPreviewOverlay();
  if (ptsArr.length < 1) {
    rawPreviewActive = false;
    refreshInstructionMarkers();
    return;
  }
  rawPreviewActive = true;
  clearInstrMarkerChildren();

  const toVec = (p) => new THREE.Vector3(Number(p[0]) || 0, Number(p[1]) || 0, Number(p[2]) || 0);
  const lineMat = new THREE.LineBasicMaterial({ color: 0x33d9ff, depthTest: true });
  const ranges = rawPreviewSegmentRanges(ptsArr.length, preview?.segmentEndExclusive);
  for (const [begin, endEx] of ranges) {
    if (endEx <= begin + 1) continue;
    const pts = [];
    for (let i = begin; i < endEx; i++) pts.push(toVec(ptsArr[i]));
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    geo.computeBoundingSphere();
    const line = new THREE.Line(geo, lineMat);
    line.frustumCulled = false;
    rawPreviewGroup.add(line);
  }

  // 桌面 OSG 对每个采样点画 POINTS，避免只见稀疏折线像“丢段”
  const allPts = ptsArr.map(toVec);
  const ptGeo = new THREE.BufferGeometry().setFromPoints(allPts);
  ptGeo.computeBoundingSphere();
  const ptsObj = new THREE.Points(
    ptGeo,
    new THREE.PointsMaterial({ color: 0x00e676, size: 3, sizeAttenuation: false, depthTest: true })
  );
  ptsObj.frustumCulled = false;
  rawPreviewGroup.add(ptsObj);
  rawPreviewGroup.visible = true;

  const axisOpts = readPreviewAxisOpts();
  if (!(axisOpts.x || axisOpts.y || axisOpts.z)) return;
  const eulers = preview.eulersDeg || [];
  const axesX = preview.axesX || [];
  const axesY = preview.axesY || [];
  const axesZ = preview.axesZ || [];
  const indices = collectPreviewAxisIndices(ptsArr.length, axisOpts.interval);
  for (const i of indices) {
    const p = ptsArr[i] || [];
    const ev = eulers[i];
    const euler = Array.isArray(ev) ? { x: ev[0], y: ev[1], z: ev[2] } : { x: 0, y: 0, z: 0 };
    addRawAxisMarker(
      { x: p[0], y: p[1], z: p[2] },
      euler,
      axisOpts,
      { x: axesX[i], y: axesY[i], z: axesZ[i] }
    );
  }
}

function applyRawOverlayFromPreview(preview) {
  lastRawPreview = preview;
  redrawRawPreviewOverlay();
}

function fmtPoseXyz(pose) {
  if (!pose || typeof pose !== "object") return "0.0, 0.0, 0.0";
  return `${Number(pose.x || 0).toFixed(1)}, ${Number(pose.y || 0).toFixed(1)}, ${Number(pose.z || 0).toFixed(1)}`;
}

// 与桌面 renumberMotionPointIndices 一致，供 P1…Pn 标签
function renumberMotionPointIndices(steps) {
  let next = 1;
  const walk = (arr) => {
    for (const ins of arr || []) {
      if (!ins) continue;
      const t = String(ins.type || "").toLowerCase();
      if (t === "ptp" || t === "line" || t === "arc") {
        ins.pointIndex = next++;
      } else if (t === "if") {
        walk(ins.then);
        walk(ins.else);
      } else if (t === "while") {
        walk(ins.body);
      }
    }
  };
  walk(steps);
}

function pathPlanFeatureId(ins) {
  const sf = ins && ins.sourceFeature;
  if (!sf) return "";
  if (typeof sf === "object" && sf.id) return String(sf.id);
  if (typeof sf === "string") {
    try {
      const o = JSON.parse(sf);
      return o && o.id ? String(o.id) : "";
    } catch {
      return "";
    }
  }
  return "";
}

function formatInstructionLabel(ins) {
  const type = String(ins.type || "").toLowerCase();
  const typeLabel = CMD_LABEL[type] || type;
  if (type === "path_plan") {
    let title = (ins.name && String(ins.name)) || "路径规划";
    const featureId = pathPlanFeatureId(ins);
    if (featureId) title += ` · ${featureId}`;
    const phase = String(ins.phase || "").toLowerCase();
    const phaseZh =
      phase === "applied" ? "已应用" : phase === "raw_ready" || phase === "rawready" ? "已离散" : "草稿";
    return `${title} · ${phaseZh}`;
  }
  if (type === "wait") {
    return `[${typeLabel}] 时长 ${Number(ins.durationSec || 0).toFixed(2)} s`;
  }
  if (type === "if" || type === "while") {
    const c = ins.condition;
    let summary = "始终";
    if (c && typeof c === "object") {
      const kind = String(c.kind || "").toLowerCase();
      if (kind === "never") summary = "永不";
      else if (kind === "io") summary = `IO${c.ioPort}==${c.ioEquals ? 1 : 0}`;
      else if (kind === "compare") summary = `${c.compareLeft || ""} ${c.compareOp || ""} ${c.compareRight ?? ""}`.trim();
    }
    return `[${typeLabel}] ${summary}`;
  }
  if (type === "set_do") {
    return `[${typeLabel}] port ${ins.ioPort ?? "?"} = ${ins.ioBoolValue ? 1 : 0}`;
  }
  if (type === "set_ao") {
    return `[${typeLabel}] port ${ins.ioPort ?? "?"} = ${Number(ins.ioAnalogValue || 0).toFixed(2)}`;
  }
  if (type === "arc") {
    const pi = Number(ins.pointIndex) || 0;
    return pi > 0 ? `P${pi} [${typeLabel}]` : `[${typeLabel}]`;
  }
  // ptp / line：对齐桌面 formatInstructionLabel
  const pi = Number(ins.pointIndex) || 0;
  const xyz = fmtPoseXyz(ins.pose);
  const summary = pi > 0 ? `P${pi} · 第${pi}点 · XYZ ${xyz}` : `XYZ ${xyz}`;
  return pi > 0 ? `P${pi} [${typeLabel}] ${summary}` : `[${typeLabel}] ${summary}`;
}

function appendProgTreeRow(parentEl, opts) {
  const kind = opts.kind || "instr";
  const wrap = document.createElement("div");
  wrap.className = `prog-tree-node kind-${kind}`;
  const row = document.createElement("div");
  const selectable = !!opts.instrId && opts.selectable !== false;
  row.className =
    "prog-tree-row" +
    (selectable && opts.instrId === selectedInstrId ? " sel" : "") +
    (selectable ? "" : " struct");
  const toggle = document.createElement("span");
  toggle.className = "prog-tree-toggle";
  toggle.textContent = opts.hasChildren ? "▾" : "";
  if (!opts.hasChildren) toggle.classList.add("empty");
  const text = document.createElement("span");
  text.className = "t";
  text.textContent = opts.label || "";
  text.title = opts.label || "";
  row.appendChild(toggle);
  row.appendChild(text);
  const kids = document.createElement("div");
  kids.className = "prog-tree-kids";
  if (selectable) {
    row.onclick = (e) => {
      e.stopPropagation();
      void selectInstruction(opts.instrId);
    };
  } else if (opts.hasChildren) {
    row.onclick = (e) => {
      e.stopPropagation();
      const collapsed = kids.classList.toggle("collapsed");
      toggle.textContent = collapsed ? "▸" : "▾";
    };
  }
  wrap.appendChild(row);
  wrap.appendChild(kids);
  parentEl.appendChild(wrap);
  return { wrap, row, kids, toggle };
}

function appendInstructionTreeNode(parentEl, ins) {
  if (!ins) return null;
  const type = String(ins.type || "").toLowerCase();
  const nested =
    type === "arc" ||
    type === "if" ||
    (type === "while" && Array.isArray(ins.body) && ins.body.length > 0);
  const node = appendProgTreeRow(parentEl, {
    kind: "instr",
    label: formatInstructionLabel(ins),
    instrId: ins.id,
    hasChildren: nested,
  });
  if (type === "arc") {
    appendProgTreeRow(node.kids, {
      kind: "waypoint",
      label: `  途经  ${fmtPoseXyz(ins.viaPose)}`,
      instrId: ins.id,
      hasChildren: false,
    });
    appendProgTreeRow(node.kids, {
      kind: "waypoint",
      label: `  终点  ${fmtPoseXyz(ins.pose)}`,
      instrId: ins.id,
      hasChildren: false,
    });
  } else if (type === "if") {
    const thenNode = appendProgTreeRow(node.kids, {
      kind: "branch",
      label: "Then（真）",
      hasChildren: true,
    });
    for (const s of ins.then || []) appendInstructionTreeNode(thenNode.kids, s);
    const elseNode = appendProgTreeRow(node.kids, {
      kind: "branch",
      label: "Else（假）",
      hasChildren: true,
    });
    for (const s of ins.else || []) appendInstructionTreeNode(elseNode.kids, s);
  } else if (type === "while") {
    for (const s of ins.body || []) appendInstructionTreeNode(node.kids, s);
  }
  return node;
}

// 对齐桌面 InstructionProgramTreeWidget::rebuildFromProgram
function renderProgramTree(listEl, steps, groups) {
  const arr = Array.isArray(steps) ? steps : [];
  renumberMotionPointIndices(arr);
  const groupList = Array.isArray(groups) ? groups : [];
  const instrToGroupId = new Map();
  for (const g of groupList) {
    for (const mid of g.memberIds || []) {
      if (mid) instrToGroupId.set(String(mid), g.id);
    }
  }
  const rootPathPlans = [];
  const motionRoots = [];
  for (const ins of arr) {
    if (!ins) continue;
    if (String(ins.type || "").toLowerCase() === "path_plan") rootPathPlans.push(ins);
    else motionRoots.push(ins);
  }
  if (rootPathPlans.length) {
    const section = appendProgTreeRow(listEl, {
      kind: "planning",
      label: "路径规划",
      hasChildren: true,
    });
    for (const pp of rootPathPlans) {
      const ppNode = appendInstructionTreeNode(section.kids, pp);
      if (!ppNode) continue;
      for (const g of groupList) {
        if (
          g.role === "path_plan_output" &&
          g.pathPlanInstructionId === pp.id &&
          Array.isArray(g.memberIds) &&
          g.memberIds.length
        ) {
          appendProgTreeRow(ppNode.kids, {
            kind: "output-ref",
            label: `↳ 输出: ${g.name || g.id}（${g.memberIds.length} 点）`,
            instrId: pp.id,
            hasChildren: false,
          });
          ppNode.toggle.textContent = "▾";
          ppNode.toggle.classList.remove("empty");
          break;
        }
      }
    }
  }
  const groupsRendered = new Set();
  for (const ins of motionRoots) {
    if (!ins) continue;
    const groupId = instrToGroupId.get(ins.id);
    if (groupId) {
      if (groupsRendered.has(groupId)) continue;
      const groupDef = groupList.find((g) => g.id === groupId);
      if (!groupDef) {
        appendInstructionTreeNode(listEl, ins);
        continue;
      }
      const groupNode = appendProgTreeRow(listEl, {
        kind: "group",
        label: `分组: ${groupDef.name || groupDef.id}`,
        hasChildren: true,
      });
      for (const step of motionRoots) {
        if (!step) continue;
        if (instrToGroupId.get(step.id) === groupId) {
          appendInstructionTreeNode(groupNode.kids, step);
        }
      }
      groupsRendered.add(groupId);
    } else {
      appendInstructionTreeNode(listEl, ins);
    }
  }
}

function renderProgramList(forcedRootId) {
  const list = $("progList");
  const title = $("progTitle");
  if (!list) return;
  list.innerHTML = "";

  // 运行中只用快照，catalog 被并发 reload 也不会把树刷空
  if (programPlaying && Array.isArray(playbackStepsSnapshot) && playbackStepsSnapshot.length) {
    const rootId = forcedRootId || playbackRootId || "";
    if (title) title.textContent = rootId ? `程序 ${rootId}` : "程序";
    list.classList.remove("muted");
    const entry = programEntryFor(rootId);
    const programs = (entry && entry.programs) || [];
    const prog =
      programs.find((p) => p.id === (entry && entry.activeProgramId)) ||
      programs.find((p) => p.isMain) ||
      programs[0];
    renderProgramTree(list, playbackStepsSnapshot, (prog && prog.groups) || []);
    return;
  }

  const rootId = forcedRootId || preferSceneRootForPrograms();
  if (!rootId) {
    list.classList.add("muted");
    list.textContent = "请先导入或选择机器人";
    if (title) title.textContent = "程序";
    return;
  }
  // 只读已有 catalog，避免 render 时 ensure 出一个空壳盖住真程序
  const entry = programEntryFor(rootId);
  const programs = (entry && entry.programs) || [];
  const prog =
    programs.find((p) => p.id === (entry && entry.activeProgramId)) ||
    programs.find((p) => p.isMain) ||
    programs[0];
  if (title) title.textContent = `程序 ${(prog && (prog.name || prog.id)) || "—"}`;
  const steps = (prog && prog.instructions) || [];
  if (!steps.length) {
    list.classList.add("muted");
    list.textContent = "暂无指令";
    return;
  }
  list.classList.remove("muted");
  renderProgramTree(list, steps, (prog && prog.groups) || []);
}

async function loadInstrProps(instructionId) {
  focusLeftPropsTab();
  if (!instructionId) {
    if (!selectedId) renderPropsTable([]);
    else await loadDetail();
    return;
  }
  const r = await (await fetch(`/api/robot/instructions/${encodeURIComponent(instructionId)}/properties`)).json();
  if (!r.ok) {
    propsEl.classList.remove("empty");
    propsEl.innerHTML = "";
    const tip = document.createElement("div");
    tip.className = "prop-table empty";
    tip.textContent = r.error || "无属性";
    propsEl.appendChild(tip);
    return;
  }
  const props = r.properties || [];
  propsEl.classList.remove("empty");
  propsEl.innerHTML = "";
  const cap = document.createElement("div");
  cap.className = "prop-caption";
  cap.textContent = `指令属性 · ${instructionId}`;
  propsEl.appendChild(cap);
  if (!props.length) {
    const empty = document.createElement("div");
    empty.className = "prop-row";
    empty.innerHTML = `<span class="prop-key">—</span><span class="prop-val">无可编辑属性</span>`;
    propsEl.appendChild(empty);
    return;
  }
  for (const p of props) {
    const div = document.createElement("div");
    div.className = "prop-row" + (p.editable ? " editable" : "");
    const key = document.createElement("span");
    key.className = "prop-key";
    key.textContent = p.label || p.key;
    key.title = p.key;
    const val = document.createElement("span");
    val.className = "prop-val";
    if (p.editable) {
      const input = document.createElement("input");
      input.className = "prop-input";
      input.value = p.value ?? "";
      input.onchange = async () => {
        const pr = await (
          await fetch(`/api/robot/instructions/${encodeURIComponent(instructionId)}`, {
            method: "PATCH",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ key: p.key, value: input.value }),
          })
        ).json();
        if (!pr.ok) {
          setStatus(pr.error || "属性更新失败", "err");
          return;
        }
        await loadProgramsFromServer();
        selectedInstrId = instructionId;
        renderProgramList();
        refreshInstructionMarkers();
        await loadInstrProps(instructionId);
        setStatus("属性已更新");
      };
      val.appendChild(input);
    } else {
      val.textContent = p.value ?? "";
      val.title = p.value ?? "";
    }
    div.appendChild(key);
    div.appendChild(val);
    propsEl.appendChild(div);
  }
}

function sleepMs(ms) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function reshapeJointFrames(flat, dof) {
  const arr = (flat || []).map(Number).filter((n) => !Number.isNaN(n));
  if (!dof || dof < 1 || !arr.length) return [];
  if (arr.length === dof) return [arr];
  if (arr.length % dof === 0) {
    const frames = [];
    for (let i = 0; i < arr.length; i += dof) frames.push(arr.slice(i, i + dof));
    return frames;
  }
  return [arr.slice(0, dof)];
}

async function applyJointAngles(rootId, joints) {
  const jr = await (
    await fetch("/api/robot/joints", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ sceneRootBackendId: rootId, jointAnglesRad: joints }),
    })
  ).json();
  if (!jr.ok) return false;
  axisSyncQuietUntil = performance.now() + 200;
  applyAxisAnglesFromServer(joints);
  await syncObjectTransforms();
  void refreshFrameOverlays();
  return true;
}

function buildPlanBody(rootId, step) {
  const type = (step.type || "").toLowerCase();
  const jointRadCsv =
    (step.extensions && step.extensions["context.currentJointRadCsv"]) ||
    (axisJointsState.length ? axisJointsState.map((j) => j.angleRad.toFixed(6)).join(",") : "");
  const body = {
    sceneRootBackendId: rootId,
    instructionType: type,
    jointRadCsv,
    targetPose: {
      positionMm: [step.pose?.x || 0, step.pose?.y || 0, step.pose?.z || 0],
      eulerDeg: [step.eulerDeg?.x || 0, step.eulerDeg?.y || 0, step.eulerDeg?.z || 0],
    },
  };
  if (type === "arc") {
    body.extensions = {
      viaPose: step.viaPose || { x: 0, y: 0, z: 0 },
      viaEulerDeg: step.viaEulerDeg || { x: 0, y: 0, z: 0 },
    };
  }
  return body;
}

async function planStepFrames(rootId, step) {
  const type = (step.type || "").toLowerCase();
  if (!["ptp", "line", "arc"].includes(type)) {
    return { ok: false, error: "非运动指令" };
  }
  const r = await (
    await fetch("/api/robot/plan", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(buildPlanBody(rootId, step)),
    })
  ).json();
  if (!r.ok) return { ok: false, error: r.error || "规划失败" };
  const dof = axisJointsState.length > 0 ? axisJointsState.length : 6;
  const frames = reshapeJointFrames(r.jointTargetsRad, dof);
  if (!frames.length) return { ok: false, error: "规划无关节结果" };
  return { ok: true, frames };
}

async function playJointFrames(rootId, frames, durationSec) {
  const rate = Math.max(0.1, Number($("simRate")?.value) || 1);
  const dur = Math.max(0.15, durationSec / rate);
  if (frames.length === 1) {
    const start = axisJointsState.length
      ? axisJointsState.map((j) => j.angleRad)
      : frames[0].map(() => 0);
    const end = frames[0];
    const n = Math.max(6, Math.ceil(dur * 24));
    for (let i = 1; i <= n; ++i) {
      if (programAbort) return false;
      const u = i / n;
      const joints = end.map((t, k) => (start[k] ?? 0) + (t - (start[k] ?? 0)) * u);
      if (!(await applyJointAngles(rootId, joints))) return false;
      await sleepMs((dur * 1000) / n);
    }
    return true;
  }
  const dt = dur / Math.max(1, frames.length - 1);
  for (const f of frames) {
    if (programAbort) return false;
    if (!(await applyJointAngles(rootId, f))) return false;
    await sleepMs(dt * 1000);
  }
  return true;
}

function estimateStepDurationSec(step, frameCount) {
  const speed = Number(step.speed) || 100;
  if (frameCount > 1) return Math.max(0.35, frameCount * 0.04 * (100 / speed));
  return Math.max(0.4, 1.2 * (100 / speed));
}

async function selectInstruction(instructionId) {
  selectedInstrId = instructionId;
  renderProgramList();
  refreshInstructionMarkers();
  await loadInstrProps(instructionId);
  const rootId = activeSceneRootId();
  const step = findSelectedStep();
  if (!rootId || !step) return;
  const type = (step.type || "").toLowerCase();
  if (type === "path_plan") {
    const sel = $("pathPlanSelect");
    if (sel) {
      const exists = [...sel.options].some((o) => o.value === step.id);
      if (!exists) await reloadPathPlanSelect();
      sel.value = step.id;
    }
    await bindSelectedPathPlan();
    document.querySelectorAll("[data-robot-tab]").forEach((b) => {
      const on = b.getAttribute("data-robot-tab") === "trajGen";
      b.classList.toggle("active", on);
    });
    showPane("robotCmd", false);
    showPane("robotJoint", false);
    showPane("robotTrajGen", true);
    showPane("robotTrajEdit", false);
    showPane("robotFrame", false);
    setStatus(`已绑定 PathPlan ${step.id}`);
    return;
  }
  if (!["ptp", "line", "arc"].includes(type)) return;
  const planned = await planStepFrames(rootId, step);
  if (!planned.ok) {
    setStatus(planned.error || "跳转失败", "err");
    return;
  }
  const target = planned.frames[planned.frames.length - 1];
  if (!(await applyJointAngles(rootId, target))) {
    setStatus("应用关节失败", "err");
    return;
  }
  setStatus(`已跳转到 ${CMD_LABEL[type] || type}`);
}

async function planSelectedInstruction() {
  const rootId = activeSceneRootId();
  const step = findSelectedStep();
  if (!rootId || !step) {
    setStatus("请先选中运动指令", "warn");
    return;
  }
  const planned = await planStepFrames(rootId, step);
  if (!planned.ok) {
    setStatus(planned.error || "规划失败", "err");
    return;
  }
  const target = planned.frames[planned.frames.length - 1];
  if (!(await applyJointAngles(rootId, target))) {
    setStatus("应用关节失败", "err");
    return;
  }
  setStatus(`规划预览 ok · ${target.length} 轴`);
}

async function stopProgramPlayback() {
  programAbort = true;
  try {
    await fetch("/api/robot/stop", { method: "POST" });
  } catch {
    /* ignore */
  }
  setStatus("已停止运行");
}

async function runProgramPlayback() {
  if (programPlaying) {
    await stopProgramPlayback();
    return;
  }
  // 优先用「有指令」的 catalog，避免 resolve 到空 root
  let rootId = preferSceneRootForPrograms();
  if (!rootId) rootId = await resolveActiveSceneRootId();
  if (!rootId) {
    setStatus("请先导入机器人", "warn");
    return;
  }
  const entry = programEntryFor(rootId);
  const programs = (entry && entry.programs) || [];
  const prog =
    programs.find((p) => p.id === (entry && entry.activeProgramId)) ||
    programs.find((p) => p.isMain) ||
    programs[0];
  const steps = [...((prog && prog.instructions) || [])];
  if (!steps.length) {
    setStatus("程序无指令", "warn");
    return;
  }
  rawPreviewActive = false;
  clearRawPreviewOverlay();
  playbackRootId = rootId;
  playbackStepsSnapshot = steps;
  programPlaying = true;
  programAbort = false;
  programsReloadAfterPlay = false;
  renderProgramList(rootId);
  refreshInstructionMarkers();
  try {
    try {
      await fetch("/api/robot/run", { method: "POST" });
    } catch {
      /* ignore */
    }
    setStatus("运行中…");
    for (const step of steps) {
      if (programAbort) break;
      selectedInstrId = step.id;
      renderProgramList(rootId);
      refreshInstructionMarkers();
      await loadInstrProps(step.id);
      const type = (step.type || "").toLowerCase();
      if (type === "wait") {
        const sec = Number(step.durationSec) || 1;
        const rate = Math.max(0.1, Number($("simRate")?.value) || 1);
        await sleepMs((sec * 1000) / rate);
        continue;
      }
      if (!["ptp", "line", "arc"].includes(type)) continue;
      const planned = await planStepFrames(rootId, step);
      if (!planned.ok) {
        setStatus(planned.error || `规划失败: ${step.id}`, "err");
        break;
      }
      const ok = await playJointFrames(rootId, planned.frames, estimateStepDurationSec(step, planned.frames.length));
      if (!ok && programAbort) break;
      if (!ok) {
        setStatus(`执行失败: ${step.id}`, "err");
        break;
      }
    }
    if (!programAbort) setStatus("运行完成");
  } finally {
    programPlaying = false;
    playbackRootId = "";
    playbackStepsSnapshot = null;
    if (programsReloadAfterPlay) {
      programsReloadAfterPlay = false;
      await loadProgramsFromServer();
    } else {
      renderProgramList(rootId);
      refreshInstructionMarkers();
    }
  }
}

async function fetchTcpPose(sceneRootId) {
  const r = await (await fetch(`/api/robot/tcp-pose?sceneRootBackendId=${encodeURIComponent(sceneRootId)}`)).json();
  if (!r.ok) throw new Error(r.error || "tcp-pose failed");
  return r;
}

function motionStepFromPose(type, pose) {
  const step = {
    type,
    id: newInstrId(),
    name: (CMD_LABEL[type] || type).toUpperCase(),
    pose: vec3Obj(pose.positionMm),
    eulerDeg: vec3Obj(pose.eulerDeg),
    speed: type === "ptp" ? 100 : 200,
    accel: type === "ptp" ? 100 : 200,
    extensions: {
      "context.currentJointRadCsv": pose.jointRadCsv || "",
    },
  };
  if (type === "line" || type === "arc") step.blendRadius = 0;
  return step;
}

async function teachAddInstruction(cmdType) {
  const type = (cmdType || "").toLowerCase();
  if (type === "path_plan") {
    void createPathPlanFromUi();
    return;
  }
  const rootId = activeSceneRootId();
  if (!rootId) {
    setStatus("请先导入机器人", "warn");
    return;
  }
  const { prog } = activeProgram(rootId);

  if (type === "ptp" || type === "line") {
    try {
      let pose = await fetchTcpPose(rootId);
      // 拖动示教中用罗盘代理位姿，避免 FK 欧拉漂移
      if (robotDragMode && dragFlangeId && dragProxy) {
        dragProxy.updateMatrix();
        const els = meshLocalMatrixArray(dragProxy);
        const m = new THREE.Matrix4().fromArray(els.map(Number));
        const p = new THREE.Vector3();
        const q = new THREE.Quaternion();
        const s = new THREE.Vector3();
        m.decompose(p, q, s);
        const e = new THREE.Euler().setFromQuaternion(q, "ZYX");
        pose = {
          ...pose,
          positionMm: [p.x, p.y, p.z],
          eulerDeg: [
            THREE.MathUtils.radToDeg(e.x),
            THREE.MathUtils.radToDeg(e.y),
            THREE.MathUtils.radToDeg(e.z),
          ],
        };
      }
      prog.instructions.push(motionStepFromPose(type, pose));
      selectedInstrId = prog.instructions[prog.instructions.length - 1].id;
      if (!(await saveProgramsToServer())) return;
      renderProgramList();
      refreshInstructionMarkers();
      void loadInstrProps(selectedInstrId);
      setStatus(`已添加 ${CMD_LABEL[type]}`);
    } catch (e) {
      setStatus(String(e.message || e), "err");
    }
    return;
  }

  if (type === "arc") {
    try {
      const pose = await fetchTcpPose(rootId);
      if (!arcViaPending) {
        arcViaPending = {
          pose: vec3Obj(pose.positionMm),
          eulerDeg: vec3Obj(pose.eulerDeg),
          jointRadCsv: pose.jointRadCsv || "",
        };
        setStatus("圆弧：已记录途径点，请再点一次「圆弧」记录终点", "warn");
        return;
      }
      const step = motionStepFromPose("arc", pose);
      step.viaPose = arcViaPending.pose;
      step.viaEulerDeg = arcViaPending.eulerDeg;
      if (arcViaPending.jointRadCsv) {
        step.extensions["context.currentJointRadCsv"] = arcViaPending.jointRadCsv;
      }
      arcViaPending = null;
      prog.instructions.push(step);
      selectedInstrId = step.id;
      if (!(await saveProgramsToServer())) return;
      renderProgramList();
      refreshInstructionMarkers();
      void loadInstrProps(selectedInstrId);
      setStatus("已添加圆弧");
    } catch (e) {
      setStatus(String(e.message || e), "err");
    }
    return;
  }

  const step = { type, id: newInstrId(), name: CMD_LABEL[type] || type };
  if (type === "wait") step.durationSec = 1;
  if (type === "if") {
    step.condition = {};
    step.then = [];
    step.else = [];
  }
  if (type === "while") {
    step.condition = {};
    step.body = [];
  }
  if (type === "set_do") {
    step.port = 0;
    step.value = false;
  }
  if (type === "set_ao") {
    step.port = 0;
    step.value = 0;
  }
  prog.instructions.push(step);
  selectedInstrId = step.id;
  if (!(await saveProgramsToServer())) return;
  renderProgramList();
  refreshInstructionMarkers();
  void loadInstrProps(selectedInstrId);
  setStatus(`已添加 ${CMD_LABEL[type] || type}`);
}

async function deleteSelectedInstruction() {
  const rootId = activeSceneRootId();
  if (!rootId || !selectedInstrId) {
    setStatus("请先选中指令", "warn");
    return;
  }
  const { prog } = activeProgram(rootId);
  const before = prog.instructions.length;
  prog.instructions = prog.instructions.filter((s) => s.id !== selectedInstrId);
  if (prog.instructions.length === before) {
    setStatus("未找到选中指令", "warn");
    return;
  }
  selectedInstrId = null;
  if (!(await saveProgramsToServer())) return;
  renderProgramList();
  refreshInstructionMarkers();
  void loadInstrProps(null);
  setStatus("已删除指令");
}

async function clearActiveProgram() {
  const rootId = activeSceneRootId();
  if (!rootId) return;
  if (!window.confirm("清空当前程序全部指令？")) return;
  const { prog } = activeProgram(rootId);
  prog.instructions = [];
  selectedInstrId = null;
  arcViaPending = null;
  if (!(await saveProgramsToServer())) return;
  renderProgramList();
  refreshInstructionMarkers();
  void loadInstrProps(null);
  setStatus("程序已清空");
}

function findSelectedStep() {
  const rootId = activeSceneRootId();
  if (!rootId || !selectedInstrId) return null;
  const { prog } = activeProgram(rootId);
  return prog.instructions.find((s) => s.id === selectedInstrId) || null;
}

async function refreshHealth() {
  try {
    const h = await (await fetch("/api/health")).json();
    if (h.role !== "web") throw new Error("role");
    healthEl.textContent = `web :${h.port}`;
    healthEl.className = "pill ok";
  } catch {
    healthEl.textContent = "offline";
    healthEl.className = "pill bad";
  }
}

async function selectObject(id) {
  const resolved = resolveScenePickId(id);
  selectedId = resolved;
  await resolveSelectedRobot(resolved);
  await fetch("/api/selection", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ backendId: resolved }),
  });
  renderTree();
  await loadDetail();
  if (!robotDragMode) attachObjectGizmoIfNeeded();
  for (const [mid, m] of idToMesh) {
    const o = objects.find((x) => x.id === mid);
    applyMeshSelectionStyle(m, o, selectionHighlightsId(mid));
  }
}

function renderPropsTable(rows) {
  propsEl.classList.remove("empty");
  propsEl.innerHTML = "";
  if (!rows.length) {
    propsEl.classList.add("empty");
    propsEl.textContent = "未选中";
    return;
  }
  for (const row of rows) {
    const div = document.createElement("div");
    div.className = "prop-row" + (row.editable ? " editable" : "");
    div.innerHTML = `<span class="prop-key" title="${row.key}">${row.key}</span><span class="prop-val" title="${row.value}">${row.value}</span>`;
    if (row.editable) {
      div.ondblclick = async () => {
        const value = prompt(`修改 ${row.key}`, String(row.value ?? ""));
        if (value == null || !selectedId) return;
        await fetch(`/api/objects/${encodeURIComponent(selectedId)}`, {
          method: "PATCH",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ propertyKey: row.key, propertyValue: value }),
        });
        await refreshObjects(false);
        await loadDetail();
      };
    }
    propsEl.appendChild(div);
  }
}

async function loadDetail() {
  if (!selectedId) {
    renderPropsTable([]);
    return;
  }
  detail = await (await fetch(`/api/objects/${encodeURIComponent(selectedId)}`)).json();
  const rows = [
    { key: "id", value: detail.id, editable: false },
    { key: "name", value: detail.name, editable: false },
    { key: "className", value: detail.className, editable: false },
    { key: "visible", value: String(detail.visible), editable: false },
  ];
  if (detail.pose) {
    rows.push({ key: "positionMm", value: (detail.pose.positionMm || []).join(", "), editable: false });
    rows.push({ key: "eulerDeg", value: (detail.pose.eulerDeg || []).join(", "), editable: false });
  }
  for (const r of detail.properties || []) {
    rows.push({ key: r.key, value: r.value, editable: !!r.editable });
  }
  renderPropsTable(rows);
}

function disposeSceneMesh(mesh) {
  if (!mesh) return;
  if (mesh.children?.length) {
    for (const c of [...mesh.children]) {
      mesh.remove(c);
      disposeSceneMesh(c);
    }
  }
  mesh.geometry?.dispose?.();
  const mat = mesh.material;
  if (Array.isArray(mat)) mat.forEach((m) => m.dispose?.());
  else mat?.dispose?.();
}

/// 只保留 idToMesh 与辅助节点；并发 rebuild/选中重拉留下的旧网格会变成鬼影
function pruneOrphanMeshes() {
  const keep = new Set([trajLine, rawPreviewGroup, dragProxy, instrMarkers, frameOverlays, pickHighlight]);
  for (const m of idToMesh.values()) keep.add(m);
  for (const c of [...root.children]) {
    if (keep.has(c)) continue;
    root.remove(c);
    disposeSceneMesh(c);
  }
}

// mesh 拉取是 await 点：导入时 refresh + SSE 会叠跑，必须串行；重建中禁止 sync/prune 拆台
let rebuildSceneChain = Promise.resolve();
let sceneRebuildDepth = 0;
let rebuildGen = 0;

async function rebuildScene() {
  const job = async () => {
    const gen = ++rebuildGen;
    sceneRebuildDepth++;
    suppressPosePush = true;
    try {
      const keepDrag = robotDragMode;
      transform.detach();
      transform.enabled = false;
      for (const c of [...root.children]) {
        if (c === trajLine || c === rawPreviewGroup || c === dragProxy || c === instrMarkers || c === frameOverlays || c === pickHighlight)
          continue;
        root.remove(c);
        disposeSceneMesh(c);
      }
      if (!root.children.includes(rawPreviewGroup)) root.add(rawPreviewGroup);
      if (!root.children.includes(trajLine)) root.add(trajLine);
      if (!root.children.includes(dragProxy)) root.add(dragProxy);
      if (!root.children.includes(instrMarkers)) root.add(instrMarkers);
      if (!root.children.includes(frameOverlays)) root.add(frameOverlays);
      if (!root.children.includes(pickHighlight)) root.add(pickHighlight);
      idToMesh.clear();
      const box = new THREE.Box3();
      let meshOk = 0;
      let meshFail = 0;
      resize();
      const snapshot = objects.slice();
      for (const o of snapshot) {
        if (gen !== rebuildGen) return box;
        if (!o.visible) continue;
        let mesh;
        if (isSceneCoordinateFrame(o)) {
          mesh = makeCoordinateFrameAxes(100);
          mesh.userData.backendId = o.id;
          applyObjectTransform(mesh, o);
        } else {
          if (!o.hasGeometry || o.geometryKind === 1) continue;
          const r = await fetch(`/api/mesh/${encodeURIComponent(o.id)}`);
          if (gen !== rebuildGen) return box;
          if (!r.ok) {
            meshFail++;
            continue;
          }
          const soup = new Float32Array(await r.arrayBuffer());
          if (gen !== rebuildGen) return box;
          if (soup.length < 9) {
            meshFail++;
            continue;
          }
          const geo = new THREE.BufferGeometry();
          geo.setAttribute("position", new THREE.BufferAttribute(soup, 3));
          geo.computeVertexNormals();
          mesh = new THREE.Mesh(
            geo,
            createMeshMaterial(colorFromObject(o, selectionHighlightsId(o.id)), selectionHighlightsId(o.id))
          );
          mesh.userData.backendId = o.id;
          applyObjectTransform(mesh, o);
        }
        const prev = idToMesh.get(o.id);
        if (prev && prev !== mesh) {
          root.remove(prev);
          disposeSceneMesh(prev);
        }
        root.add(mesh);
        idToMesh.set(o.id, mesh);
        mesh.updateMatrixWorld(true);
        box.expandByObject(mesh);
        meshOk++;
      }
      if (gen !== rebuildGen) return box;
      pruneOrphanMeshes();
      if (keepDrag && dragFlangeId && idToMesh.has(dragFlangeId)) {
        void attachDragCompass(dragFlangeId);
      } else if (!keepDrag) {
        attachObjectGizmoIfNeeded();
      }
      // 重建后刷新选中高亮（含整机）
      for (const [mid, m] of idToMesh) {
        const o = objects.find((x) => x.id === mid);
        applyMeshSelectionStyle(m, o, selectionHighlightsId(mid));
      }
      refreshInstructionMarkers();
      statusEl.textContent = `对象 ${objects.length} · 网格 ${meshOk}${meshFail ? ` · 失败 ${meshFail}` : ""}`;
      return box;
    } finally {
      sceneRebuildDepth = Math.max(0, sceneRebuildDepth - 1);
      suppressPosePush = false;
    }
  };
  const p = rebuildSceneChain.then(job, job);
  rebuildSceneChain = p.then(
    () => undefined,
    () => undefined
  );
  return p;
}

function focusBox(box) {
  if (!box || box.isEmpty()) return;
  const size = box.getSize(new THREE.Vector3()).length();
  const center = box.getCenter(new THREE.Vector3());
  controls.target.copy(center);
  camera.position.copy(center.clone().add(new THREE.Vector3(size, size * 0.7, size)));
}

async function focusAll() {
  const box = new THREE.Box3();
  for (const m of idToMesh.values()) box.expandByObject(m);
  focusBox(box);
}

let refreshObjectsChain = Promise.resolve();
let refreshObjectsEpoch = 0;

async function refreshObjects(focus) {
  const epoch = ++refreshObjectsEpoch;
  const wantFocus = !!focus;
  const job = async () => {
    // 合并同一时刻多次 SSE（注册对象连发）
    await Promise.resolve();
    if (epoch !== refreshObjectsEpoch) return null;
    const list = await (await fetch("/api/objects")).json();
    if (epoch !== refreshObjectsEpoch) return null;
    objects = list.objects || [];
    if (list.projectPath) {
      pathEl.value = list.projectPath;
      const name = list.projectPath.split(/[/\\]/).pop();
      if (name && docTabTitle) docTabTitle.textContent = name;
    }
    await refreshRobotSceneRoots();
    if (!robotSceneRootIds.size) clearActiveRobotSelection();
    else scrubStaleRobotRootRefs();
    renderTree();
    const box = await rebuildScene();
    if (epoch !== refreshObjectsEpoch) return null;
    if (wantFocus) focusBox(box);
    return box;
  };
  const p = refreshObjectsChain.then(job, job);
  refreshObjectsChain = p.then(
    () => undefined,
    () => undefined
  );
  return p;
}

/// 仅刷新位姿：FK/滑条拖动时禁止 rebuildScene（重拉 mesh 会卡成“拖完才跳”）
async function syncObjectTransforms() {
  if (sceneRebuildDepth > 0) return;
  const list = await (await fetch("/api/objects")).json();
  if (sceneRebuildDepth > 0) return;
  objects = list.objects || [];
  const draggingRobot = robotDragMode && transform.dragging;
  // 拖拽模式下保留罗盘挂接，避免 FK 同步时 detach 造成拖不动
  const placingRobot =
    sceneInteractMode === "select" && isRobotSceneSelection(selectedId) && transform.object === dragProxy;
  if (!draggingRobot && !(robotDragMode && dragFlangeId) && !(placingRobot && transform.dragging)) {
    transform.detach();
  }
  pruneOrphanMeshes();
  for (const o of objects) {
    const mesh = idToMesh.get(o.id);
    if (!mesh) continue;
    applyObjectTransform(mesh, o);
    mesh.visible = !!o.visible;
  }
  if (robotDragMode && dragFlangeId) {
    if (!draggingRobot) {
      await refreshFrameOverlays();
    }
    return;
  }
  if (placingRobot && transform.dragging) {
    // 拖整机中：代理跟手，连杆已由 place 刷新
    transform.enabled = true;
    transform.attach(dragProxy);
    transform.getHelper().visible = true;
    return;
  }
  attachObjectGizmoIfNeeded();
}

async function applyJointsFrom(inputId) {
  const jointAnglesRad = $(inputId)
    .value.split(",")
    .map((s) => Number(s.trim()))
    .filter((n) => !Number.isNaN(n));
  const r = await (
    await fetch("/api/robot/joints", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ sceneRootBackendId: $("robotRoot").value.trim(), jointAnglesRad }),
    })
  ).json();
  setStatus(r.ok ? "关节已应用" : r.error || "失败", r.ok ? "info" : "err");
  if (r.ok) await syncObjectTransforms();
}

async function importUrdf(pathOverride) {
  let urdfPath = pathOverride || $("urdfPath").value.trim();
  if (!urdfPath) {
    urdfPath = await pickPath("urdf");
    if (!urdfPath) return;
    $("urdfPath").value = urdfPath;
  }
  const r = await (
    await fetch("/api/robot/urdf/import", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ urdfPath }),
    })
  ).json();
  setStatus(r.ok ? `URDF ${r.sceneRootBackendId}` : r.error || "URDF 失败", r.ok ? "info" : "err");
  if (r.sceneRootBackendId) $("robotRoot").value = r.sceneRootBackendId;
  await refreshObjects(false);
  if (r.ok) {
    void loadAxisControl();
    void loadProgramsFromServer();
  }
}

let deviceCatalog = { types: [], brandsByType: {}, packages: [] };

async function loadDeviceCatalog() {
  const status = $("deviceStatus");
  const grid = $("deviceGrid");
  if (!grid) return;
  try {
    const r = await (await fetch("/api/devices/catalog")).json();
    if (!r.ok) throw new Error(r.error || "catalog failed");
    deviceCatalog = r;
    const typeSel = $("deviceType");
    typeSel.innerHTML = "";
    for (const t of r.types || []) {
      const opt = document.createElement("option");
      opt.value = t;
      opt.textContent = t;
      typeSel.appendChild(opt);
    }
    fillDeviceBrands();
    rebuildDeviceTiles();
    if (status) status.textContent = (r.packages || []).length ? "" : `未找到含 URDF 的设备包\n${r.modelsRoot || ""}`;
  } catch (e) {
    if (status) status.textContent = "设备库加载失败";
    setStatus(String(e.message || e), "err");
  }
}

function fillDeviceBrands() {
  const type = $("deviceType")?.value || "";
  const brandSel = $("deviceBrand");
  if (!brandSel) return;
  brandSel.innerHTML = "";
  const brands = (deviceCatalog.brandsByType && deviceCatalog.brandsByType[type]) || [];
  for (const b of brands) {
    const opt = document.createElement("option");
    opt.value = b;
    opt.textContent = b;
    brandSel.appendChild(opt);
  }
}

function rebuildDeviceTiles() {
  const grid = $("deviceGrid");
  const status = $("deviceStatus");
  if (!grid) return;
  grid.innerHTML = "";
  const type = $("deviceType")?.value || "";
  const brand = $("deviceBrand")?.value || "";
  const pkgs = (deviceCatalog.packages || []).filter((p) => p.type === type && p.brand === brand);
  for (const p of pkgs) {
    const tile = document.createElement("button");
    tile.type = "button";
    tile.className = "device-tile";
    tile.title = `${p.name}\n${p.urdfPath}`;
    if (p.thumbnailUrl) {
      const img = document.createElement("img");
      img.src = p.thumbnailUrl;
      img.alt = p.name;
      tile.appendChild(img);
    } else {
      const ph = document.createElement("div");
      ph.className = "ph";
      ph.textContent = "R";
      tile.appendChild(ph);
    }
    const name = document.createElement("div");
    name.className = "name";
    name.textContent = p.name;
    tile.appendChild(name);
    tile.onclick = () => void importUrdf(p.urdfPath);
    grid.appendChild(tile);
  }
  if (status) status.textContent = pkgs.length ? "" : "此品牌下暂无可用型号";
}

let axisJointsState = [];
let axisPostTimer = null;

async function loadAxisControl() {
  const instSel = $("axisInstance");
  const box = $("axisJoints");
  if (!instSel || !box) return;
  scrubStaleRobotRootRefs();
  const prev = instSel.value || $("robotRoot").value.trim();
  const inst = await (await fetch("/api/robot/instances")).json();
  const objectIds = new Set(objects.map((o) => o.id));
  const instances = (inst.instances || []).filter(
    (it) => it.sceneRootBackendId && (!objectIds.size || objectIds.has(it.sceneRootBackendId))
  );
  instSel.innerHTML = "";
  for (const it of instances) {
    const opt = document.createElement("option");
    opt.value = it.sceneRootBackendId;
    opt.textContent = `${it.label || it.sceneRootBackendId} (${it.jointCount})`;
    instSel.appendChild(opt);
  }
  if (prev && [...instSel.options].some((o) => o.value === prev)) instSel.value = prev;
  else if (instSel.options.length) {
    instSel.selectedIndex = 0;
    $("robotRoot").value = instSel.value;
  }
  const rootId = instSel.value;
  if (!rootId) {
    box.innerHTML = '<p class="hint">暂无机器人实例，请从设备库导入或打开含机器人的工程</p>';
    axisJointsState = [];
    if ($("frameInstance")) $("frameInstance").innerHTML = "";
    clearCoordinateFramesUi();
    clearActiveRobotSelection();
    return;
  }
  $("robotRoot").value = rootId;
  if ($("frameInstance") && [...$("frameInstance").options].some((o) => o.value === rootId)) {
    $("frameInstance").value = rootId;
  }
  const meta = await (await fetch(`/api/robot/joints?sceneRootBackendId=${encodeURIComponent(rootId)}`)).json();
  if (!meta.ok) {
    box.innerHTML = `<p class="hint">${meta.error || "无法读取关节"}</p>`;
    axisJointsState = [];
    return;
  }
  axisJointsState = (meta.joints || []).map((j) => ({ ...j }));
  renderAxisJoints();
}

function renderAxisJoints() {
  const box = $("axisJoints");
  if (!box) return;
  box.innerHTML = "";
  axisJointsState.forEach((j, idx) => {
    const row = document.createElement("div");
    row.className = "axis-row";
    const deg = THREE.MathUtils.radToDeg(j.angleRad);
    row.innerHTML = `
      <div class="axis-row-head"><span class="jn">${j.name}</span>
        <button type="button" class="btn-ghost" data-reset="${idx}">重置</button></div>
      <input type="range" min="${j.lowerRad}" max="${j.upperRad}" step="0.001" value="${j.angleRad}" data-idx="${idx}" />
      <div class="axis-row-vals">
        <label>° <input type="number" step="0.1" value="${deg.toFixed(2)}" data-deg="${idx}" /></label>
        <label>rad <input type="number" step="0.001" value="${j.angleRad.toFixed(4)}" data-rad="${idx}" /></label>
      </div>`;
    box.appendChild(row);
  });
  box.querySelectorAll("input[type=range]").forEach((el) => {
    el.oninput = () => {
      const i = Number(el.dataset.idx);
      axisJointsState[i].angleRad = Number(el.value);
      syncAxisRowInputs(i);
      scheduleAxisPost();
    };
  });
  box.querySelectorAll("[data-deg]").forEach((el) => {
    el.onchange = () => {
      const i = Number(el.dataset.deg);
      axisJointsState[i].angleRad = THREE.MathUtils.degToRad(Number(el.value));
      clampAxis(i);
      syncAxisRowInputs(i);
      scheduleAxisPost();
    };
  });
  box.querySelectorAll("[data-rad]").forEach((el) => {
    el.onchange = () => {
      const i = Number(el.dataset.rad);
      axisJointsState[i].angleRad = Number(el.value);
      clampAxis(i);
      syncAxisRowInputs(i);
      scheduleAxisPost();
    };
  });
  box.querySelectorAll("[data-reset]").forEach((el) => {
    el.onclick = () => {
      const i = Number(el.dataset.reset);
      axisJointsState[i].angleRad = 0;
      clampAxis(i);
      syncAxisRowInputs(i);
      scheduleAxisPost();
    };
  });
  if ($("jointsCsv2")) {
    $("jointsCsv2").value = axisJointsState.map((j) => j.angleRad.toFixed(6)).join(",");
  }
}

function clampAxis(i) {
  const j = axisJointsState[i];
  if (!j) return;
  j.angleRad = Math.min(j.upperRad, Math.max(j.lowerRad, j.angleRad));
}

function syncAxisRowInputs(i) {
  const box = $("axisJoints");
  if (!box) return;
  const j = axisJointsState[i];
  const range = box.querySelector(`input[type=range][data-idx="${i}"]`);
  const deg = box.querySelector(`[data-deg="${i}"]`);
  const rad = box.querySelector(`[data-rad="${i}"]`);
  if (range) range.value = String(j.angleRad);
  if (deg) deg.value = THREE.MathUtils.radToDeg(j.angleRad).toFixed(2);
  if (rad) rad.value = j.angleRad.toFixed(4);
}

let axisPostInFlight = false;
let axisPostPending = false;
// SSE 与本地 POST 可能叠两次 sync；短窗内吞掉重复
let axisSyncQuietUntil = 0;

function scheduleAxisPost() {
  axisPostPending = true;
  void flushAxisPosts();
}

async function flushAxisPosts() {
  if (axisPostInFlight) return;
  axisPostInFlight = true;
  try {
    while (axisPostPending) {
      axisPostPending = false;
      await postAxisJointsOnce();
    }
  } finally {
    axisPostInFlight = false;
    if (axisPostPending) void flushAxisPosts();
  }
}

async function postAxisJointsOnce() {
  const rootId = $("axisInstance")?.value || $("robotRoot").value.trim();
  if (!rootId || !axisJointsState.length) return;
  const jointAnglesRad = axisJointsState.map((j) => j.angleRad);
  if ($("jointsCsv")) $("jointsCsv").value = jointAnglesRad.join(",");
  if ($("jointsCsv2")) $("jointsCsv2").value = jointAnglesRad.join(",");
  const r = await (
    await fetch("/api/robot/joints", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ sceneRootBackendId: rootId, jointAnglesRad }),
    })
  ).json();
  if (!r.ok) {
    setStatus(r.error || "关节失败", "err");
    return;
  }
  axisSyncQuietUntil = performance.now() + 120;
  await syncObjectTransforms();
}

$("btnNew").onclick = async () => {
  setStatus("新建…");
  const r = await (await fetch("/api/project/new", { method: "POST" })).json();
  setStatus(r.ok ? "已新建" : r.error || "失败", r.ok ? "info" : "err");
  if (docTabTitle) docTabTitle.textContent = "未命名1";
  pathEl.value = "";
  await refreshObjects(true);
};

$("btnOpen").onclick = async () => {
  document.querySelectorAll(".menu").forEach((m) => m.classList.remove("open"));
  const path = await pickPath("project");
  if (path) await openProjectAt(path);
};

$("btnOpenFolder").onclick = async () => {
  document.querySelectorAll(".menu").forEach((m) => m.classList.remove("open"));
  const path = await pickPath("directory");
  if (path) await openProjectAt(path);
};

$("btnSave").onclick = async () => {
  document.querySelectorAll(".menu").forEach((m) => m.classList.remove("open"));
  let path = pathEl.value.trim();
  if (!path) {
    path = await pickPath("saveProject");
    if (!path) return;
    pathEl.value = path;
  }
  setStatus("保存…");
  const r = await (
    await fetch("/api/project/save", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ path }),
    })
  ).json();
  setStatus(r.ok ? `已保存 ${r.path || ""}` : r.error || "失败", r.ok ? "info" : "err");
};

$("btnImport").onclick = async () => {
  document.querySelectorAll(".menu").forEach((m) => m.classList.remove("open"));
  const path = await pickPath("import");
  if (!path) return;
  const isPointCloud = /\.(pcd|ply|las|laz)$/i.test(path);
  const r = await (
    await fetch("/api/objects/import", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ path, isPointCloud }),
    })
  ).json();
  setStatus(r.ok ? `导入 ${r.id}` : r.error || "失败", r.ok ? "info" : "err");
  await refreshObjects(false);
};

$("btnFocus").onclick = () => void focusAll();
$("btnFocusToolbar").onclick = () => void focusAll();
$("btnHomeView").onclick = () => {
  camera.position.set(800, 600, 1000);
  controls.target.set(0, 0, 0);
};
$("btnUrdf").onclick = () => void importUrdf();
$("btnUrdfMenu").onclick = () => {
  document.querySelectorAll(".menu").forEach((m) => m.classList.remove("open"));
  void importUrdf();
};

function openInsertFrameDialog() {
  document.querySelectorAll(".menu").forEach((m) => m.classList.remove("open"));
  const dlg = $("frameDialog");
  if (!dlg) return;
  if ($("frameDlgName") && !$("frameDlgName").value.trim()) $("frameDlgName").value = "坐标系";
  dlg.classList.remove("hidden");
}

function closeInsertFrameDialog() {
  $("frameDialog")?.classList.add("hidden");
}

async function createCoordinateFrameFromDialog() {
  const name = ($("frameDlgName")?.value || "坐标系").trim() || "坐标系";
  const body = {
    name,
    positionMm: [Number($("frameDlgX")?.value || 0), Number($("frameDlgY")?.value || 0), Number($("frameDlgZ")?.value || 0)],
    eulerDeg: [Number($("frameDlgRx")?.value || 0), Number($("frameDlgRy")?.value || 0), Number($("frameDlgRz")?.value || 0)],
  };
  const r = await (
    await fetch("/api/objects/coordinate-frame", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    })
  ).json();
  if (!r.ok) {
    setStatus(r.error || "创建坐标系失败", "err");
    return;
  }
  closeInsertFrameDialog();
  setStatus(`已创建坐标系 ${r.backendId || r.id || ""}`);
  await refreshObjects(false);
  // 轨迹编辑若正打开转换工件型，立刻刷新外部 TCP 下拉
  if (!$("robotTrajEdit")?.classList.contains("hidden")) void syncOpParamsEditor();
}

$("btnInsertFrame") && ($("btnInsertFrame").onclick = () => openInsertFrameDialog());
$("frameDlgCancel") && ($("frameDlgCancel").onclick = () => closeInsertFrameDialog());
$("frameDlgOk") && ($("frameDlgOk").onclick = () => void createCoordinateFrameFromDialog());
$("frameDialog")?.addEventListener("click", (e) => {
  if (e.target === $("frameDialog")) closeInsertFrameDialog();
});
$("btnJoints").onclick = () => void applyJointsFrom("jointsCsv");
$("btnJoints2").onclick = () => {
  $("jointsCsv").value = $("jointsCsv2").value;
  void applyJointsFrom("jointsCsv2");
};
$("btnDeviceRefresh") && ($("btnDeviceRefresh").onclick = () => void loadDeviceCatalog());
$("deviceType") &&
  ($("deviceType").onchange = () => {
    fillDeviceBrands();
    rebuildDeviceTiles();
  });
$("deviceBrand") && ($("deviceBrand").onchange = () => rebuildDeviceTiles());
$("btnAxisReload") && ($("btnAxisReload").onclick = () => void loadAxisControl());
$("btnAxisResetAll") &&
  ($("btnAxisResetAll").onclick = () => {
    axisJointsState.forEach((j, i) => {
      j.angleRad = 0;
      clampAxis(i);
    });
    renderAxisJoints();
    scheduleAxisPost();
  });
$("axisInstance") &&
  ($("axisInstance").onchange = () => {
    const v = $("axisInstance").value;
    $("robotRoot").value = v;
    if ($("frameInstance") && [...$("frameInstance").options].some((o) => o.value === v)) {
      $("frameInstance").value = v;
    }
    void loadAxisControl();
    void loadProgramsFromServer();
    if (!$("robotFrame")?.classList.contains("hidden")) void loadCoordinateFrames();
    else void refreshFrameOverlays();
  });

function refreshInstructionMarkers() {
  if (rawPreviewActive && !programPlaying) {
    // Raw 预览时隐藏指令点轴，避免双轨；运行中强制显示路点
    clearInstrMarkerChildren();
    return;
  }
  if (programPlaying) {
    rawPreviewActive = false;
    rawPreviewGroup.visible = false;
    trajLine.visible = false;
  } else {
    trajLine.visible = false;
    rawPreviewGroup.visible = false;
  }

  let steps = null;
  if (programPlaying && Array.isArray(playbackStepsSnapshot) && playbackStepsSnapshot.length) {
    steps = playbackStepsSnapshot;
  } else {
    const rootId = preferSceneRootForPrograms();
    if (!rootId) return;
    const entry = programEntryFor(rootId);
    const programs = (entry && entry.programs) || [];
    const prog =
      programs.find((p) => p.id === (entry && entry.activeProgramId)) ||
      programs.find((p) => p.isMain) ||
      programs[0];
    steps = (prog && prog.instructions) || [];
  }
  if (!steps.length) return;
  // 有可画数据后再清，避免 root 瞬时为空时把已有路点抹掉
  clearInstrMarkerChildren();
  for (const step of steps) {
    const type = (step.type || "").toLowerCase();
    if (!["ptp", "line", "arc"].includes(type) || !step.pose) continue;
    const sel = step.id === selectedInstrId;
    if (type === "arc" && step.viaPose) {
      addPoseMarker(step.viaPose, step.viaEulerDeg, false, "via");
    }
    addPoseMarker(step.pose, step.eulerDeg, sel, step.id);
  }
}

async function refreshTrajectoryUi() {
  fillWorkpieceSelects();
  await loadFeatureStrategyCatalog();
  await refreshDiscTemplateSelect();
  await reloadPathPlanSelect();
  const s = await syncTrajSessionStatus();
  applyFeaturesFromSession(s);
  renderFeatTable();
  await rebuildFeatParamForm();
  syncFeatWriteModeButtons();
  updatePickStatusLabel();
  await loadOpPalette();
  refreshTrajEditScopeCombos();
  await reloadPipelineFromServer();
  await refreshPipeTemplateSelect();
}

function fillWorkpieceSelects() {
  for (const id of ["trajWorkpiece", "meshWorkpiece"]) {
    const sel = $(id);
    if (!sel) continue;
    const cur = sel.value;
    sel.innerHTML = "";
    for (const o of objects) {
      if (!o.hasGeometry) continue;
      const opt = document.createElement("option");
      opt.value = o.id;
      opt.textContent = o.name || o.id;
      sel.appendChild(opt);
    }
    if (cur && [...sel.options].some((o) => o.value === cur)) sel.value = cur;
  }
}

async function reloadPathPlanSelect() {
  const sel = $("pathPlanSelect");
  if (!sel) return;
  const r = await (await fetch(`/api/trajectory/path-plans?sceneRootBackendId=${encodeURIComponent(activeSceneRootId())}`)).json();
  sel.innerHTML = "";
  for (const p of r.pathPlans || []) {
    const opt = document.createElement("option");
    opt.value = p.id;
    opt.textContent = `${p.name || p.id} (${p.phase || "?"})`;
    if (p.bound) opt.selected = true;
    sel.appendChild(opt);
  }
}

async function syncTrajSessionStatus() {
  const s = await (await fetch("/api/trajectory/session")).json();
  trajFeatureEditActive = !!s.featureEditActive;
  const bound = !!(s.pathPlanId || "").trim();
  const editing = trajFeatureEditActive;
  const gate = $("trajEditGate");
  if (gate) {
    gate.textContent = editing
      ? "修改中：可拾取 / 离散 / 编辑算子"
      : bound
        ? "未开始修改：已规划路径只读，点「开始修改」后才能改"
        : "未绑定 PathPlan：请先新建或选择路径规划";
  }
  const st = $("trajRawStatus");
  if (st) st.textContent = s.hasRaw ? `Raw: ${s.rawPointCount} 点 · ${s.phase || ""}` : "Raw: —";
  const nRaw = Math.floor(Number(s.rawPointCount) || 0);
  window.__trajRawPointCount = s.hasRaw && nRaw > 0 ? nRaw : 0;
  const editSt = $("trajEditRawStatus");
  if (editSt) {
    editSt.textContent =
      s.rawStatusText ||
      (window.__trajRawPointCount
        ? `Raw ${window.__trajRawPointCount} 点 · 新建算子默认 P 范围`
        : "请先在轨迹生成页离散");
  }
  const pickLocked = !editing;
  [
    "btnPickEdge",
    "btnPickFace",
    "btnFeatDiscretize",
    "btnMeshGenerate",
    "btnFeatAppend",
    "btnFeatNew",
    "btnFeatDelete",
    "featStrategy",
    "btnDiscTplSave",
    "btnDiscTplLoad",
    "btnDiscTplDelete",
    "btnDiscTplImport",
    "btnDiscTplExport",
  ].forEach((id) => {
    const el = $(id);
    if (el) el.disabled = pickLocked;
  });
  const editLocked = !editing;
  [
    "btnRecipeFill",
    "btnTrajApply",
    "btnTrajReset",
    "btnTrajUndo",
    "btnTrajRedo",
    "trajPreviewOn",
    "btnTrajPreviewRaw",
    "pipeTplSelect",
    "btnPipeTplSave",
    "btnPipeTplLoad",
    "btnPipeTplDelete",
    "btnPipeTplImport",
    "btnPipeTplExport",
  ].forEach((id) => {
    const el = $(id);
    if (el) el.disabled = editLocked;
  });
  const emitBtn = $("btnTrajEmit");
  if (emitBtn) emitBtn.disabled = !editing || !s.hasRaw || !!s.emitDisabled;
  const applyBtn = $("btnTrajApply");
  if (applyBtn) applyBtn.disabled = !editing || !s.hasRaw;
  const beginBtn = $("btnTrajBeginEdit");
  if (beginBtn) beginBtn.disabled = !bound || editing;
  const cancelBtn = $("btnTrajCancelEdit");
  if (cancelBtn) cancelBtn.disabled = !editing;
  const palette = $("opPalette");
  if (palette) palette.classList.toggle("disabled-pane", editLocked);
  const pipe = $("opPipeline");
  if (pipe) pipe.classList.toggle("disabled-pane", editLocked);
  const params = $("opParamsForm");
  if (params) params.classList.toggle("disabled-pane", editLocked);
  const u = $("btnTrajUndo");
  const r = $("btnTrajRedo");
  if (u) u.disabled = editLocked || !s.canUndo;
  if (r) r.disabled = editLocked || !s.canRedo;
  return s;
}

function clearTrajFeaturesUi() {
  if (autoDiscTimer) {
    clearTimeout(autoDiscTimer);
    autoDiscTimer = null;
  }
  trajFeatures = [];
  trajFeatSel = -1;
  trajPickMode = null;
  trajAppendMode = false;
  syncPickModeButtons();
  clearPickHighlight();
  renderFeatTable();
  void rebuildFeatParamForm();
  updatePickStatusLabel();
}

/// 生成/应用/取消修改后：退出编辑态 UI（Host 已关 featureEditActive）
function exitTrajEditUiAfterCommit() {
  trajFeatureEditActive = false;
  rawPreviewActive = false;
  clearRawPreviewOverlay();
  clearTrajFeaturesUi();
}

function applyFeaturesFromSession(s) {
  const raw = s && s.sourceFeatureJson;
  if (!raw) return;
  try {
    const doc = typeof raw === "string" ? JSON.parse(raw) : raw;
    if (Array.isArray(doc.features)) {
      trajFeatures = doc.features.map((f) => ({
        ...f,
        status: f.status || "就绪",
        params: f.params || {},
      }));
      trajFeatSel = trajFeatures.length ? 0 : -1;
      if (doc.workpiece?.backendIdUtf8 && $("trajWorkpiece")) {
        $("trajWorkpiece").value = doc.workpiece.backendIdUtf8;
      }
      if (doc.defaultStrategyId && $("featStrategy")) $("featStrategy").value = doc.defaultStrategyId;
      else if (trajFeatures[0]?.strategyId && $("featStrategy")) $("featStrategy").value = trajFeatures[0].strategyId;
      renderFeatTable();
      void rebuildFeatParamForm();
    }
  } catch {
    /* 忽略损坏的特征 JSON */
  }
}

async function createPathPlanFromUi() {
  const sceneRootBackendId = await resolveActiveSceneRootId();
  if (!sceneRootBackendId) {
    setStatus("创建 PathPlan 需要机器人：请先从设备库导入或打开含机器人的工程", "err");
    return;
  }
  const r = await (
    await fetch("/api/robot/path-plan", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ sceneRootBackendId }),
    })
  ).json();
  if (!r.ok) {
    setStatus(r.error || "创建 PathPlan 失败", "err");
    return;
  }
  setStatus(`已创建 PathPlan ${r.pathPlanId}`);
  await loadProgramsFromServer();
  await refreshTrajectoryUi();
  document.querySelectorAll("[data-robot-tab]").forEach((b) => {
    const on = b.getAttribute("data-robot-tab") === "trajGen";
    b.classList.toggle("active", on);
  });
  showPane("robotCmd", false);
  showPane("robotJoint", false);
  showPane("robotTrajGen", true);
  showPane("robotTrajEdit", false);
  showPane("robotFrame", false);
  // 对齐桌面：新建只绑定，不自动进入修改
  clearTrajFeaturesUi();
  rawPreviewActive = false;
  clearRawPreviewOverlay();
  await syncTrajSessionStatus();
  setStatus(`已创建 PathPlan ${r.pathPlanId}，请点「开始修改」再拾取/离散`);
}

async function bindSelectedPathPlan() {
  const id = $("pathPlanSelect")?.value;
  if (!id) return;
  const sceneRootBackendId = await resolveActiveSceneRootId();
  if (!sceneRootBackendId) {
    setStatus("绑定 PathPlan 需要机器人：请先导入或选择机器人", "err");
    return;
  }
  const r = await (
    await fetch("/api/trajectory/bind", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ pathPlanId: id, sceneRootBackendId }),
    })
  ).json();
  if (!r.ok) {
    setStatus(r.error || "绑定失败", "err");
    return;
  }
  // 对齐桌面：切换 PathPlan 只 bind，清特征表、不预览，等「开始修改」
  clearTrajFeaturesUi();
  rawPreviewActive = false;
  clearRawPreviewOverlay();
  await reloadPipelineFromServer();
  await syncTrajSessionStatus();
  setStatus("已绑定 PathPlan，点击「开始修改」加载特征与预览");
}

async function beginTrajEditFromUi() {
  const r = await (await fetch("/api/trajectory/begin-edit", { method: "POST" })).json();
  if (!r.ok) {
    setStatus(r.error || "开始修改失败", "err");
    return;
  }
  const s = await syncTrajSessionStatus();
  applyFeaturesFromSession(s);
  await reloadPipelineFromServer();
  if (s.hasRaw) void previewTrajectoryRawUi();
  setStatus(s.hasRaw ? "已开始修改（已加载 Raw 预览）" : "已开始修改，请拾取或导入特征");
}

async function cancelTrajEditFromUi() {
  await fetch("/api/trajectory/cancel-edit", { method: "POST" });
  exitTrajEditUiAfterCommit();
  await reloadPipelineFromServer();
  await syncTrajSessionStatus();
  setStatus("已取消修改；特征表已清空，预览已关闭。已落盘 PathPlan 保留");
}

/// 生成/应用后刷新指令树（对齐桌面 refreshInstructionList）
async function refreshInstructionTreeAfterTrajectory() {
  await loadProgramsFromServer();
  refreshTrajEditScopeCombos();
  const cmdTab = document.querySelector('[data-robot-tab="cmd"]');
  if (cmdTab && !cmdTab.classList.contains("active")) cmdTab.click();
  const rootId = (await resolveActiveSceneRootId()) || preferSceneRootForPrograms();
  const { prog } = activeProgram(rootId);
  const steps = prog.instructions || [];
  const firstMotion = steps.find((s) => {
    const t = (s.type || "").toLowerCase();
    return t === "line" || t === "ptp" || t === "arc";
  });
  if (firstMotion?.id) {
    selectedInstrId = firstMotion.id;
    renderProgramList();
    refreshInstructionMarkers();
    void loadInstrProps(firstMotion.id);
  } else {
    renderProgramList();
    refreshInstructionMarkers();
  }
  const nLine = steps.filter((s) => (s.type || "").toLowerCase() === "line").length;
  if (nLine) setStatus(`指令树已更新：${nLine} 条直线`);
}

function strategyDisplayName(id) {
  const hit = featStrategyCatalog.find((s) => s.strategyId === id);
  return hit?.displayNameZh || id || "—";
}

function featureStatusLabel(status) {
  if (status === "离散失败") return "失败";
  if (status === "离散完成") return "完成";
  return status || "草稿";
}

function featureStatusClass(status) {
  const s = featureStatusLabel(status);
  if (s === "失败") return "st-err";
  if (s === "就绪" || s === "完成") return "st-ok";
  return "st-draft";
}

function clearAllTrajFeatures() {
  if (!trajFeatures.length) {
    setStatus("特征表已空");
    return;
  }
  if (!confirm(`清空全部 ${trajFeatures.length} 条特征？`)) return;
  if (autoDiscTimer) {
    clearTimeout(autoDiscTimer);
    autoDiscTimer = null;
  }
  trajFeatures = [];
  trajFeatSel = -1;
  lastRawPreview = null;
  rawPreviewActive = false;
  clearRawPreviewOverlay();
  clearPickHighlight();
  refreshInstructionMarkers();
  renderFeatTable();
  void rebuildFeatParamForm();
  updatePickStatusLabel();
  setStatus("已清空特征表");
}

function hideFeatTableContextMenu() {
  const menu = $("featTableCtxMenu");
  if (menu) menu.classList.add("hidden");
}

let featCtxIgnoreClickUntil = 0;

function showFeatTableContextMenu(x, y, rowIndex) {
  let menu = $("featTableCtxMenu");
  if (!menu) {
    menu = document.createElement("div");
    menu.id = "featTableCtxMenu";
    menu.className = "ctx-menu hidden";
    document.body.appendChild(menu);
    document.addEventListener("click", (e) => {
      if (Date.now() < featCtxIgnoreClickUntil) return;
      if (menu.contains(e.target)) return;
      hideFeatTableContextMenu();
    });
  }
  menu.innerHTML = "";
  const addItem = (label, fn, danger) => {
    const btn = document.createElement("button");
    btn.type = "button";
    btn.textContent = label;
    if (danger) btn.className = "danger";
    btn.onclick = (e) => {
      e.stopPropagation();
      hideFeatTableContextMenu();
      fn();
    };
    menu.appendChild(btn);
  };

  if (rowIndex >= 0 && rowIndex < trajFeatures.length) {
    const f = trajFeatures[rowIndex];
    const faces = f.geometry?.faceIndices || [];
    const edges = f.geometry?.edgeIndices || [];
    if (faces.length) {
      addItem("删末面", () => {
        faces.pop();
        f.status = "草稿";
        renderFeatTable();
        void rebuildFeatParamForm();
        scheduleAutoDiscretize();
      });
    }
    if (edges.length) {
      addItem("删末边", () => {
        edges.pop();
        f.status = "草稿";
        renderFeatTable();
        void rebuildFeatParamForm();
        scheduleAutoDiscretize();
      });
    }
    addItem(
      "删除整行特征",
      () => {
        trajFeatures.splice(rowIndex, 1);
        trajFeatSel = trajFeatures.length ? Math.min(rowIndex, trajFeatures.length - 1) : -1;
        if (!trajFeatures.length) {
          lastRawPreview = null;
          rawPreviewActive = false;
          clearRawPreviewOverlay();
          clearPickHighlight();
          refreshInstructionMarkers();
        }
        renderFeatTable();
        void rebuildFeatParamForm();
        scheduleAutoDiscretize();
        setStatus("已删除特征");
      },
      true
    );
  }
  addItem("清空全部特征", () => clearAllTrajFeatures(), true);

  // 避免同一次右键后续 click 立刻关掉菜单
  featCtxIgnoreClickUntil = Date.now() + 400;
  menu.classList.remove("hidden");
  const pad = 4;
  menu.style.left = `${x}px`;
  menu.style.top = `${y}px`;
  const mw = menu.offsetWidth || 160;
  const mh = menu.offsetHeight || 100;
  menu.style.left = `${Math.min(x, window.innerWidth - mw - pad)}px`;
  menu.style.top = `${Math.min(y, window.innerHeight - mh - pad)}px`;
}

function geometrySummary(f) {
  const edges = f.geometry?.edgeIndices || [];
  const faces = f.geometry?.faceIndices || [];
  const parts = [];
  if (edges.length) parts.push(`边 ${edges.join(",")}`);
  if (faces.length) parts.push(`面 ${faces.join(",")}`);
  return parts.join(" · ") || "—";
}

function syncFeatWriteModeButtons() {
  $("btnFeatAppend")?.classList.toggle("active", trajAppendMode);
  $("btnFeatNew")?.classList.toggle("active", !trajAppendMode);
}

function updatePickStatusLabel() {
  const el = $("trajPickStatus");
  if (!el) return;
  if (!trajPickMode) {
    el.textContent = "3D 拾取未激活";
    return;
  }
  const mode = trajPickMode === "face" ? "面" : "线";
  const append = trajAppendMode ? " · 追加到选中" : " · 新建特征";
  el.textContent = `拾取${mode}中${append}`;
}

async function refreshDiscTemplateSelect() {
  const sel = $("discTplSelect");
  if (!sel) return;
  const prev = sel.value;
  const r = await (await fetch("/api/trajectory/templates/discretize")).json();
  const names = Array.isArray(r) ? r : r.templates || r.names || [];
  sel.innerHTML = "";
  const empty = document.createElement("option");
  empty.value = "";
  empty.textContent = names.length ? "（选择模板）" : "（无模板）";
  sel.appendChild(empty);
  for (const n of names) {
    const name = typeof n === "string" ? n : n.name || n.id || "";
    if (!name) continue;
    const o = document.createElement("option");
    o.value = name;
    o.textContent = name;
    sel.appendChild(o);
  }
  if (prev && [...sel.options].some((o) => o.value === prev)) sel.value = prev;
}

function currentDiscTplName() {
  return ($("discTplSelect")?.value || $("discTplName")?.value || "").trim();
}

async function loadFeatureStrategyCatalog() {
  const r = await (await fetch("/api/trajectory/feature-schema")).json();
  if (!r.ok || !Array.isArray(r.strategies)) return;
  featStrategyCatalog = r.strategies;
  const sel = $("featStrategy");
  if (!sel) return;
  const prev = sel.value;
  sel.innerHTML = "";
  for (const s of featStrategyCatalog) {
    const o = document.createElement("option");
    o.value = s.strategyId;
    o.textContent = s.displayNameZh || s.strategyId;
    sel.appendChild(o);
  }
  if (prev && [...sel.options].some((o) => o.value === prev)) sel.value = prev;
  else if (sel.options.length) sel.selectedIndex = 0;
}

async function fetchFeatureSchema(strategyId) {
  if (!strategyId) return null;
  if (featSchemaCache[strategyId]) return featSchemaCache[strategyId];
  const r = await (await fetch(`/api/trajectory/feature-schema?strategyId=${encodeURIComponent(strategyId)}`)).json();
  if (!r.ok) return null;
  featSchemaCache[strategyId] = r;
  return r;
}

function defaultsFromSchema(schema) {
  if (!schema) return {};
  const out = {};
  for (const f of schema.fields || []) {
    if (f.type === "Bool") out[f.key] = !!f.defaultBool;
    else if (f.type === "Int") out[f.key] = f.defaultInt ?? 0;
    else if (f.type === "Enum") out[f.key] = (f.enumValues || [])[f.defaultInt || 0] ?? "";
    else if (f.type === "Message" || f.type === "Vec3") continue;
    else out[f.key] = f.defaultDouble ?? 0;
  }
  const d = schema.defaults;
  if (d && typeof d === "object") {
    const flat = d.params && typeof d.params === "object" ? d.params : d;
    Object.assign(out, flat);
  }
  return out;
}

function scheduleAutoDiscretize() {
  if (!trajFeatureEditActive) return;
  if (autoDiscTimer) clearTimeout(autoDiscTimer);
  autoDiscTimer = setTimeout(() => {
    autoDiscTimer = null;
    if (!trajFeatureEditActive || !trajFeatures.length) return;
    void discretizeFeaturesUi({ silent: true });
  }, 400);
}

function renderFeatGeomChips() {
  const box = $("featGeomChips");
  if (!box) return;
  box.innerHTML = "";
  const f = trajFeatSel >= 0 ? trajFeatures[trajFeatSel] : null;
  if (!f) {
    box.classList.add("muted");
    box.textContent = "选中特征后可删除面/边索引";
    return;
  }
  box.classList.remove("muted");
  const addChip = (kind, idx, arr) => {
    const chip = document.createElement("span");
    chip.className = "chip";
    chip.textContent = `${kind}${idx}`;
    const btn = document.createElement("button");
    btn.type = "button";
    btn.title = "删除";
    btn.textContent = "×";
    btn.onclick = (e) => {
      e.stopPropagation();
      const i = arr.indexOf(idx);
      if (i >= 0) arr.splice(i, 1);
      f.status = "草稿";
      renderFeatTable();
      scheduleAutoDiscretize();
    };
    chip.appendChild(btn);
    box.appendChild(chip);
  };
  for (const i of f.geometry?.faceIndices || []) addChip("面", i, f.geometry.faceIndices);
  for (const i of f.geometry?.edgeIndices || []) addChip("边", i, f.geometry.edgeIndices);
  if (!box.children.length) {
    box.classList.add("muted");
    box.textContent = "无几何索引（请拾取）";
  }
}

function renderFeatTable() {
  const body = $("featTableBody");
  if (!body) return;
  body.innerHTML = "";
  if (!trajFeatures.length) {
    body.innerHTML = `<tr class="empty"><td colspan="5">暂无特征</td></tr>`;
    const emptyTr = body.querySelector("tr.empty");
    if (emptyTr) {
      emptyTr.oncontextmenu = (ev) => {
        ev.preventDefault();
        ev.stopPropagation();
        showFeatTableContextMenu(ev.clientX, ev.clientY, -1);
      };
    }
    renderFeatGeomChips();
    refreshBrepInfoLabel();
    return;
  }
  trajFeatures.forEach((f, i) => {
    if (!f.status) f.status = "草稿";
    const tr = document.createElement("tr");
    if (i === trajFeatSel) tr.classList.add("sel");
    const sid = f.featureId || "—";
    const strat = strategyDisplayName(f.strategyId);
    const geom = geometrySummary(f);
    const st = featureStatusLabel(f.status);
    tr.innerHTML = `<td>${i + 1}</td><td title="${sid}">${sid}</td><td title="${strat}">${strat}</td><td title="${geom}">${geom}</td><td class="${featureStatusClass(
      f.status
    )}" title="${f.status}">${st}</td>`;
    tr.onclick = () => {
      trajFeatSel = i;
      if ($("featStrategy") && f.strategyId) $("featStrategy").value = f.strategyId;
      syncFeatWriteModeButtons();
      renderFeatTable();
      void rebuildFeatParamForm();
      updatePickStatusLabel();
    };
    tr.oncontextmenu = (ev) => {
      ev.preventDefault();
      ev.stopPropagation();
      trajFeatSel = i;
      body.querySelectorAll("tr").forEach((row, idx) => row.classList.toggle("sel", idx === i));
      if ($("featStrategy") && f.strategyId) $("featStrategy").value = f.strategyId;
      showFeatTableContextMenu(ev.clientX, ev.clientY, i);
      void rebuildFeatParamForm();
      updatePickStatusLabel();
    };
    body.appendChild(tr);
  });
  renderFeatGeomChips();
  refreshBrepInfoLabel();
}

async function rebuildFeatParamForm() {
  const box = $("featParamForm");
  if (!box) return;
  const strategyId = $("featStrategy")?.value || trajFeatures[trajFeatSel]?.strategyId;
  if (!strategyId) {
    box.classList.add("muted");
    box.textContent = "选择策略后显示参数";
    return;
  }
  const schema = await fetchFeatureSchema(strategyId);
  if (!schema) {
    box.classList.add("muted");
    box.textContent = "无法加载策略参数";
    return;
  }
  const f = trajFeatSel >= 0 ? trajFeatures[trajFeatSel] : null;
  if (f && f.strategyId !== strategyId) {
    f.strategyId = strategyId;
    f.params = defaultsFromSchema(schema);
    f.status = "草稿";
  }
  const values = f?.params && Object.keys(f.params).length ? f.params : defaultsFromSchema(schema);
  if (f && (!f.params || !Object.keys(f.params).length)) f.params = { ...values };

  featParamSuppress = true;
  box.classList.remove("muted");
  box.innerHTML = "";
  const fields = [...(schema.fields || [])].sort((a, b) => (a.order || 0) - (b.order || 0));
  for (const field of fields) {
    if (field.type === "Message") {
      const row = document.createElement("div");
      row.className = "op-field msg";
      row.textContent = field.messageZh || field.key;
      box.appendChild(row);
      continue;
    }
    const row = document.createElement("div");
    row.className = "op-field";
    const label = document.createElement("label");
    label.textContent = (field.labelZh || field.labelEn || field.key) + (field.unit ? ` (${field.unit})` : "");
    row.appendChild(label);
    let ctrl;
    const cur = values[field.key];
    const onChange = () => {
      if (featParamSuppress) return;
      const target = trajFeatSel >= 0 ? trajFeatures[trajFeatSel] : null;
      if (!target) return;
      if (!target.params) target.params = {};
      if (field.type === "Bool") target.params[field.key] = !!ctrl.checked;
      else if (field.type === "Int") target.params[field.key] = parseInt(ctrl.value, 10);
      else if (field.type === "Enum") target.params[field.key] = ctrl.value;
      else target.params[field.key] = Number(ctrl.value);
      target.status = "草稿";
      renderFeatTable();
      scheduleAutoDiscretize();
    };
    if (field.type === "Bool") {
      ctrl = document.createElement("input");
      ctrl.type = "checkbox";
      ctrl.checked = cur !== undefined ? !!cur : !!field.defaultBool;
      ctrl.onchange = onChange;
    } else if (field.type === "Enum") {
      ctrl = document.createElement("select");
      (field.enumValues || []).forEach((v, i) => {
        const o = document.createElement("option");
        o.value = v;
        o.textContent = (field.enumLabelsZh || [])[i] || v;
        ctrl.appendChild(o);
      });
      ctrl.value = cur !== undefined && cur !== null ? String(cur) : (field.enumValues || [])[0] || "";
      ctrl.onchange = onChange;
    } else {
      ctrl = document.createElement("input");
      ctrl.type = "number";
      ctrl.step = field.type === "Int" ? 1 : field.step || 0.1;
      if (field.type === "Int") {
        if (field.minInt != null) ctrl.min = field.minInt;
        if (field.maxInt != null) ctrl.max = field.maxInt;
        ctrl.value = cur !== undefined ? cur : field.defaultInt;
      } else {
        if (field.min != null) ctrl.min = field.min;
        if (field.max != null) ctrl.max = field.max;
        ctrl.value = cur !== undefined ? cur : field.defaultDouble;
      }
      ctrl.onchange = onChange;
    }
    row.appendChild(ctrl);
    box.appendChild(row);
  }
  featParamSuppress = false;
  renderFeatTable();
}

function newFeatureId() {
  return `F${trajFeatures.length + 1}`;
}

async function makeFeatureParams(strategyId) {
  const schema = await fetchFeatureSchema(strategyId);
  return defaultsFromSchema(schema);
}

function buildFeatureListDoc() {
  for (const f of trajFeatures) {
    const hasFace = (f.geometry?.faceIndices || []).length > 0;
    const hasEdge = (f.geometry?.edgeIndices || []).length > 0;
    if (hasFace && !hasEdge && !isFaceStrategy(f.strategyId)) f.strategyId = "FaceBoundary";
    if (hasEdge && !hasFace && isFaceStrategy(f.strategyId)) f.strategyId = "EdgeChain";
  }
  const wp = $("trajWorkpiece")?.value || "";
  // 只提交契约字段，去掉 UI 用的 status 等
  const features = trajFeatures.map((f) => ({
    featureId: f.featureId || "",
    strategyId: f.strategyId || "EdgeChain",
    geometry: {
      faceIndices: [...(f.geometry?.faceIndices || [])],
      edgeIndices: [...(f.geometry?.edgeIndices || [])],
      polylineXyz: [...(f.geometry?.polylineXyz || [])],
    },
    params: f.params && typeof f.params === "object" ? { ...f.params } : {},
  }));
  return {
    schemaVersion: 2,
    workpiece: { backendIdUtf8: wp, stepPathUtf8: "", frameId: "workpiece" },
    defaultStrategyId: $("featStrategy")?.value || "EdgeChain",
    features,
  };
}

async function discretizeFeaturesUi(opts = {}) {
  const silent = !!opts.silent;
  if (!trajFeatureEditActive) {
    if (!silent) setStatus("请先「开始修改」再离散", "warn");
    return;
  }
  if (!trajFeatures.length) {
    if (!silent) setStatus("无特征可离散", "warn");
    return;
  }
  const body = buildFeatureListDoc();
  const r = await (
    await fetch("/api/trajectory/discretize", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    })
  ).json();
  if (r.ok) {
    for (const f of trajFeatures) f.status = "就绪";
    renderFeatTable();
    await syncTrajSessionStatus();
    // 对齐桌面特征页：离散后只预览 Raw，不跑轨迹编辑管线
    void previewTrajectoryRawUi();
    if (!silent) setStatus("离散完成");
  } else {
    for (const f of trajFeatures) f.status = "失败";
    renderFeatTable();
    if (!silent) setStatus(r.error || "离散失败", "err");
  }
}

async function applyTrajectoryPreviewResponse(r, label) {
  if (!r.ok) {
    setStatus(r.error || `${label}失败`, "err");
    lastRawPreview = null;
    rawPreviewActive = false;
    clearRawPreviewOverlay();
    refreshInstructionMarkers();
    return false;
  }
  applyRawOverlayFromPreview(r);
  const segs = Array.isArray(r.segmentEndExclusive) ? r.segmentEndExclusive.length : 0;
  setStatus(`${label} ${r.pointCount || 0} 点${segs ? ` · ${segs} 段` : ""}`);
  const rawEl = $("trajRawStatus");
  if (rawEl) rawEl.textContent = `Raw: ${r.pointCount || 0} 点${segs ? ` / ${segs} 段` : ""}`;
  return true;
}

/// 特征页 / Raw：file→world，不算子
async function previewTrajectoryRawUi() {
  const r = await (await fetch("/api/trajectory/preview-raw", { method: "POST" })).json();
  return applyTrajectoryPreviewResponse(r, "Raw 预览");
}

/// 轨迹编辑：file→world + 管线
async function previewTrajectoryUi() {
  const r = await (await fetch("/api/trajectory/preview", { method: "POST" })).json();
  return applyTrajectoryPreviewResponse(r, "管线预览");
}

function worldRayFromPointer(ev) {
  const rect = renderer.domElement.getBoundingClientRect();
  const ndc = new THREE.Vector2(
    ((ev.clientX - rect.left) / rect.width) * 2 - 1,
    -((ev.clientY - rect.top) / rect.height) * 2 + 1
  );
  const raycaster = new THREE.Raycaster();
  raycaster.setFromCamera(ndc, camera);
  root.updateMatrixWorld(true);
  const invRoot = new THREE.Matrix4().copy(root.matrixWorld).invert();
  const o = raycaster.ray.origin.clone().applyMatrix4(invRoot);
  const d = raycaster.ray.direction.clone().transformDirection(invRoot).normalize();
  const out = {
    originMm: [o.x, o.y, o.z],
    dir: [d.x, d.y, d.z],
    hitPointWorldMm: null,
    hitNormalWorld: null,
    hitBackendId: null,
    hitLocal: null,
  };
  const meshes = [];
  for (const m of idToMesh.values()) {
    if (m && m.visible) meshes.push(m);
  }
  const hits = raycaster.intersectObjects(meshes, false);
  if (!hits.length) return out;
  let prefer = hits[0];
  const wp = $("trajWorkpiece")?.value;
  if (wp) {
    const matched = hits.find((h) => h.object?.userData?.backendId === wp);
    if (matched) prefer = matched;
  }
  const hpRoot = prefer.point.clone().applyMatrix4(invRoot);
  out.hitPointWorldMm = [hpRoot.x, hpRoot.y, hpRoot.z];
  out.hitBackendId = prefer.object?.userData?.backendId || null;
  if (prefer.face && prefer.object) {
    const nRoot = prefer.face.normal
      .clone()
      .transformDirection(prefer.object.matrixWorld)
      .transformDirection(invRoot)
      .normalize();
    out.hitNormalWorld = [nRoot.x, nRoot.y, nRoot.z];
  }
  out.hitLocal = prefer;
  return out;
}

function isFaceStrategy(id) {
  return String(id || "").startsWith("Face");
}

/// 拾取模式与离散策略对齐，避免面拾取仍走 EdgeChain
function resolveFeatureStrategy() {
  const sel = $("featStrategy");
  const cur = sel?.value || "";
  if (trajPickMode === "face") {
    if (!isFaceStrategy(cur)) {
      if (sel) sel.value = "FaceBoundary";
      return "FaceBoundary";
    }
    return cur;
  }
  if (trajPickMode === "edge") {
    if (!cur || isFaceStrategy(cur)) {
      if (sel) sel.value = "EdgeChain";
      return "EdgeChain";
    }
    return cur;
  }
  return cur || "EdgeChain";
}

function syncPickModeButtons() {
  $("btnPickEdge")?.classList.toggle("active", trajPickMode === "edge");
  $("btnPickFace")?.classList.toggle("active", trajPickMode === "face");
}

async function commitTrajPick(ev) {
  if (!trajPickMode) return;
  let wp = $("trajWorkpiece")?.value;
  const ray = worldRayFromPointer(ev);
  // 以实际点中的 mesh 为准，避免下拉选错工件导致错面
  if (ray.hitBackendId) {
    wp = ray.hitBackendId;
    if ($("trajWorkpiece") && [...$("trajWorkpiece").options].some((o) => o.value === wp)) {
      $("trajWorkpiece").value = wp;
    }
  }
  if (!wp) {
    setStatus("请选择工件", "warn");
    return;
  }
  const body = {
    mode: trajPickMode,
    workpieceBackendId: wp,
    originMm: ray.originMm,
    dir: ray.dir,
  };
  if (ray.hitPointWorldMm) body.hitPointWorldMm = ray.hitPointWorldMm;
  if (ray.hitNormalWorld) body.hitNormalWorld = ray.hitNormalWorld;
  const r = await (
    await fetch("/api/pick/mesh-element", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    })
  ).json();
  if (!r.ok) {
    const msg = r.error || "拾取未命中";
    if (/no in-memory B-rep|no STEP/i.test(msg)) {
      setStatus("当前工件无 BREP/STEP，请改选 CAD 工件后再拾取", "warn");
    } else {
      setStatus(msg, "warn");
    }
    return;
  }
  const strategy = resolveFeatureStrategy();
  const append = trajAppendMode && trajFeatSel >= 0;
  if (append) {
    const f = trajFeatures[trajFeatSel];
    if (trajPickMode === "edge" && r.edgeIndex >= 0) f.geometry.edgeIndices.push(r.edgeIndex);
    if (trajPickMode === "face" && r.faceIndex >= 0) f.geometry.faceIndices.push(r.faceIndex);
    if (trajPickMode === "face" && !isFaceStrategy(f.strategyId)) f.strategyId = strategy;
    if (trajPickMode === "edge" && isFaceStrategy(f.strategyId)) f.strategyId = strategy;
    f.status = "草稿";
  } else {
    const params = await makeFeatureParams(strategy);
    trajFeatures.push({
      featureId: newFeatureId(),
      strategyId: strategy,
      geometry: {
        edgeIndices: trajPickMode === "edge" && r.edgeIndex >= 0 ? [r.edgeIndex] : [],
        faceIndices: trajPickMode === "face" && r.faceIndex >= 0 ? [r.faceIndex] : [],
        polylineXyz: [],
      },
      params,
      status: "草稿",
    });
    trajFeatSel = trajFeatures.length - 1;
    trajAppendMode = false;
    if ($("featStrategy")) $("featStrategy").value = strategy;
  }
  renderFeatTable();
  void rebuildFeatParamForm();
  if ((r.soupWorldMm && r.soupWorldMm.length >= 9) || (r.polylinesWorld && r.polylinesWorld.length)) {
    applyPickHighlight(r.polylinesWorld, r.soupWorldMm);
  }
  const pickedKind = trajPickMode === "face" ? "面" : "线";
  // 单次拾取后退出，避免连续误点追加几何
  trajPickMode = null;
  syncPickModeButtons();
  updatePickStatusLabel();
  setStatus(`拾取${pickedKind} ok · face=${r.faceIndex} edge=${r.edgeIndex} · ${strategyDisplayName(strategy)}`);
  scheduleAutoDiscretize();
}

const OP_PALETTE_FALLBACK = [
  ["Translate", "平移"],
  ["Rotate", "旋转"],
  ["Delete", "删除"],
  ["Duplicate", "复制"],
  ["Mirror", "轴反向"],
  ["Reorder", "固定姿态"],
  ["Resample", "重采样"],
  ["OffsetAlongNormal", "法向偏移"],
  ["OffsetLateral", "横向偏移"],
  ["SmoothPose", "姿态平滑"],
  ["AssignBlend", "过渡半径"],
  ["AssignSpeedZone", "速度区"],
  ["Weave", "摆动"],
  ["ReachabilityFilter", "可达性过滤"],
  ["ExternalAxisSearch", "外部轴搜索"],
  ["Approach", "进刀"],
  ["Retract", "退刀"],
  ["ProjectToGeometry", "轨迹投影"],
  ["NonRigidRegistration", "非刚性配准纠正"],
  ["ToWorkpieceInHand", "转换工件型"],
];
let opPaletteEntries = OP_PALETTE_FALLBACK.map(([kind, zh]) => ({ kind, displayNameZh: zh }));
const opDisplayNameMap = Object.fromEntries(OP_PALETTE_FALLBACK);

function opDisplayName(kind) {
  return opDisplayNameMap[kind] || kind || "?";
}

async function loadOpPalette() {
  try {
    const r = await (await fetch("/api/trajectory/op-palette")).json();
    const ops = r.ops || [];
    if (r.ok && ops.length) {
      opPaletteEntries = ops.map((o) => ({
        kind: o.kind,
        displayNameZh: o.displayNameZh || o.kind,
      }));
      for (const o of opPaletteEntries) opDisplayNameMap[o.kind] = o.displayNameZh;
    }
  } catch {
    /* 用内置中文表 */
  }
  renderOpPalette();
}

let pickMeshOverlays = [];

function disposePickObject(c) {
  if (c.geometry) c.geometry.dispose();
  if (c.material) {
    if (Array.isArray(c.material)) c.material.forEach((m) => m.dispose());
    else c.material.dispose();
  }
}

function clearPickHighlight() {
  while (pickHighlight.children.length) {
    const c = pickHighlight.children[0];
    pickHighlight.remove(c);
    disposePickObject(c);
  }
  for (const c of pickMeshOverlays) {
    c.parent?.remove(c);
    disposePickObject(c);
  }
  pickMeshOverlays = [];
}

function facePickMaterial() {
  return new THREE.MeshBasicMaterial({
    color: 0xffcc33,
    transparent: true,
    opacity: 0.55,
    side: THREE.DoubleSide,
    depthWrite: false,
    polygonOffset: true,
    polygonOffsetFactor: -4,
    polygonOffsetUnits: -4,
  });
}

/// BREP 面三角 soup / 边折线（服务端离散，非整块显示 mesh 冒充）
function applyPickHighlight(polylinesWorld, soupWorldMm) {
  clearPickHighlight();
  const hasSoup = Array.isArray(soupWorldMm) && soupWorldMm.length >= 9;
  if (hasSoup) {
    const arr = new Float32Array(soupWorldMm.map(Number));
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.BufferAttribute(arr, 3));
    geo.computeVertexNormals();
    const mesh = new THREE.Mesh(geo, facePickMaterial());
    mesh.renderOrder = 10;
    pickHighlight.add(mesh);
  }
  if (!hasSoup && Array.isArray(polylinesWorld) && polylinesWorld.length) {
    const mat = new THREE.LineBasicMaterial({ color: 0xff9900, depthTest: false });
    for (const poly of polylinesWorld) {
      if (!Array.isArray(poly) || poly.length < 2) continue;
      const pts = poly.map((p) => new THREE.Vector3(Number(p[0]) || 0, Number(p[1]) || 0, Number(p[2]) || 0));
      const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts), mat.clone());
      line.renderOrder = 11;
      pickHighlight.add(line);
    }
  }
}

let hoverPickTimer = null;
let hoverPickSeq = 0;
async function hoverTrajPick(ev) {
  if (!trajPickMode) {
    clearPickHighlight();
    return;
  }
  let wp = $("trajWorkpiece")?.value;
  const ray = worldRayFromPointer(ev);
  if (ray.hitBackendId) wp = ray.hitBackendId;
  if (!wp || !ray.hitPointWorldMm) {
    clearPickHighlight();
    return;
  }
  const seq = ++hoverPickSeq;
  const body = {
    mode: trajPickMode,
    workpieceBackendId: wp,
    originMm: ray.originMm,
    dir: ray.dir,
    hitPointWorldMm: ray.hitPointWorldMm,
  };
  if (ray.hitNormalWorld) body.hitNormalWorld = ray.hitNormalWorld;
  const r = await (
    await fetch("/api/pick/hover", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    })
  ).json();
  if (seq !== hoverPickSeq) return;
  if (!r.ok) {
    clearPickHighlight();
    return;
  }
  applyPickHighlight(r.polylinesWorld || [], r.soupWorldMm);
}

function scheduleHoverTrajPick(ev) {
  if (!trajPickMode) return;
  if (hoverPickTimer) clearTimeout(hoverPickTimer);
  const clone = { clientX: ev.clientX, clientY: ev.clientY };
  hoverPickTimer = setTimeout(() => void hoverTrajPick(clone), 80);
}

function applySchemaValueToOp(op, key, value) {
  if (!op.scope) op.scope = { kind: 0 };
  if (!op.params) op.params = {};
  if (key.startsWith("scope.")) {
    const sk = key.slice("scope.".length);
    if (sk === "kind") op.scope.kind = Number(value);
    else if (sk === "groupId") op.scope.groupId = String(value);
    else if (sk === "pointFrom") op.scope.pointFrom = Number(value);
    else if (sk === "pointTo") op.scope.pointTo = Number(value);
    else op.scope[sk] = value;
  } else {
    op.params[key] = value;
  }
}

function readSchemaValueFromOp(op, key) {
  if (key.startsWith("scope.")) {
    const sk = key.slice("scope.".length);
    return op.scope ? op.scope[sk] : undefined;
  }
  return op.params ? op.params[key] : undefined;
}

const TO_WORKPIECE_MANUAL_TCP_KEYS = new Set([
  "toWorkpiece.externalTcpXMm",
  "toWorkpiece.externalTcpYMm",
  "toWorkpiece.externalTcpZMm",
  "toWorkpiece.externalTcpRxDeg",
  "toWorkpiece.externalTcpRyDeg",
  "toWorkpiece.externalTcpRzDeg",
]);

function scopeKindToken(kind) {
  const n = Number(kind);
  if (n === 0) return "EntireProgram";
  if (n === 1) return "Group";
  if (n === 2) return "PointIndexRange";
  if (n === 3) return "InstructionIds";
  const s = String(kind ?? "").trim();
  return s || "EntireProgram";
}

function defaultScopeForNewOp(rawPointCount, groupId) {
  const n = Math.floor(Number(rawPointCount) || 0);
  if (n > 0) return { kind: 2, pointFrom: 1, pointTo: n };
  const gid = String(groupId || "").trim();
  if (gid) return { kind: 1, groupId: gid };
  return { kind: 0 };
}

function refreshOpFieldVisibility(box, fields, op) {
  const token = scopeKindToken(op.scope?.kind ?? 0);
  const kindNum = String(Number(op.scope?.kind ?? 0));
  const externalTcpId = String(readSchemaValueFromOp(op, "toWorkpiece.externalTcpBackendId") || "").trim();
  for (const f of fields) {
    const row = [...box.querySelectorAll("[data-field-key]")].find((el) => el.dataset.fieldKey === f.key);
    if (!row) continue;
    let vis = true;
    if (f.visibleWhenScopeKind) {
      const allowed = String(f.visibleWhenScopeKind)
        .split(",")
        .map((x) => x.trim())
        .filter(Boolean);
      if (allowed.length) vis = allowed.includes(token) || allowed.includes(kindNum);
    }
    if (f.visibleWhenFieldKey) {
      const other = readSchemaValueFromOp(op, f.visibleWhenFieldKey);
      vis = vis && Number(other) === Number(f.visibleWhenIntValue);
    }
    // 选中场景 Frame 后隐藏手动六自由度（对齐桌面 TrajectoryOpParamPanel）
    if (TO_WORKPIECE_MANUAL_TCP_KEYS.has(f.key) && externalTcpId) vis = false;
    row.classList.toggle("hidden", !vis);
  }
}

function listSceneCoordinateFrames() {
  return (objects || []).filter((o) => isSceneCoordinateFrame(o));
}

/// 对齐桌面 populateExternalTcpFrameCombo：以 Host 登记的 FrameBackendData 为准
async function fetchSceneCoordinateFrames() {
  try {
    const r = await (await fetch("/api/objects/coordinate-frames")).json();
    if (r && r.ok && Array.isArray(r.frames) && r.frames.length) {
      return r.frames
        .filter((f) => f && f.id)
        .map((f) => ({ id: String(f.id), name: String(f.name || f.id) }));
    }
  } catch {
    /* 回落到本地 objects */
  }
  // 本地缓存可能尚未 refresh，再扫一遍 objects
  const local = listSceneCoordinateFrames().map((o) => ({
    id: String(o.id),
    name: String(o.name || o.id),
  }));
  if (local.length) return local;
  try {
    const list = await (await fetch("/api/objects")).json();
    const objs = list.objects || [];
    return objs
      .filter((o) => isSceneCoordinateFrame(o))
      .map((o) => ({ id: String(o.id), name: String(o.name || o.id) }));
  } catch {
    return [];
  }
}

function isExternalTcpBackendField(f) {
  const k = (f && f.key) || "";
  return k === "toWorkpiece.externalTcpBackendId" || /externalTcpBackendId$/i.test(k);
}

/// 不用原生 select：Qt WebEngine 下父级 overflow:auto 会裁掉弹出列表
function buildExternalTcpFramePicker(frameOptions, curValue, onPick) {
  const wrap = document.createElement("div");
  wrap.className = "op-frame-picker";
  let cur = String(curValue ?? "").trim();
  // 误存成名称时回落到 id
  if (cur && !frameOptions.some((f) => f.id === cur)) {
    const byName = frameOptions.find((f) => f.name === cur);
    if (byName) cur = byName.id;
  }
  const addOpt = (value, text) => {
    const lab = document.createElement("label");
    lab.className = "op-frame-opt" + (cur === value || (!cur && value === "") ? " sel" : "");
    const inp = document.createElement("input");
    inp.type = "radio";
    inp.name = "toWorkpiece_externalTcpBackendId";
    inp.value = value;
    inp.checked = cur === value || (!cur && value === "");
    inp.onchange = () => {
      if (!inp.checked) return;
      wrap.querySelectorAll(".op-frame-opt").forEach((el) => el.classList.remove("sel"));
      lab.classList.add("sel");
      onPick(value);
    };
    lab.appendChild(inp);
    const span = document.createElement("span");
    span.textContent = text;
    lab.appendChild(span);
    wrap.appendChild(lab);
  };
  addOpt("", "手动（填写下方六自由度）");
  for (const fr of frameOptions) {
    const text = fr.name && fr.name !== fr.id ? `${fr.name}  ·  ${fr.id}` : fr.id;
    addOpt(fr.id, text);
  }
  if (!frameOptions.length) {
    const tip = document.createElement("div");
    tip.className = "op-frame-empty";
    tip.textContent = "暂无场景坐标系。请菜单「插入 → 坐标系…」创建后再选。";
    wrap.appendChild(tip);
  }
  return wrap;
}

async function syncOpParamsEditor() {
  const box = $("opParamsForm");
  const ta = $("opParamsJson");
  if (!box) return;
  if (trajOpSel < 0 || trajOpSel >= trajPipelineOps.length) {
    box.classList.add("muted");
    box.textContent = "选中算子后显示 schema 表单";
    if (ta) ta.value = "";
    return;
  }
  const op = trajPipelineOps[trajOpSel];
  if (ta) ta.value = JSON.stringify({ scope: op.scope || {}, params: op.params || {} }, null, 2);
  box.classList.remove("muted");
  box.innerHTML = "";
  const kind = op.kind || "Translate";
  let schema;
  try {
    schema = await (
      await fetch(`/api/trajectory/op-schema?kind=${encodeURIComponent(kind)}&opIndex=${trajOpSel}`)
    ).json();
  } catch {
    box.textContent = "schema 拉取失败";
    return;
  }
  if (!schema.ok) {
    box.textContent = schema.error || "无 schema";
    return;
  }
  const fields = [...(schema.fields || [])].sort((a, b) => {
    const ag = String(a.key || "").startsWith("scope.") ? 0 : 1;
    const bg = String(b.key || "").startsWith("scope.") ? 0 : 1;
    if (ag !== bg) return ag - bg;
    return (a.order || 0) - (b.order || 0);
  });
  const values = schema.values || {};
  const keepScope = op.scope && Object.keys(op.scope).length > 0;
  for (const f of fields) {
    if (values[f.key] === undefined) continue;
    if (String(f.key).startsWith("scope.") && keepScope) continue;
    applySchemaValueToOp(op, f.key, values[f.key]);
  }
  const nPts = currentRawPointCount();
  if (scopeKindToken(op.scope?.kind) === "PointIndexRange" && nPts > 0) {
    const from = Number(op.scope.pointFrom);
    const to = Number(op.scope.pointTo);
    if (!(to > 1) || from < 1 || to < from || to > nPts || from > nPts) {
      op.scope.pointFrom = 1;
      op.scope.pointTo = nPts;
    }
  }
  const needFrames = fields.some((f) => isExternalTcpBackendField(f));
  const frameOptions = needFrames ? await fetchSceneCoordinateFrames() : [];
  const onChange = () => {
    refreshOpFieldVisibility(box, fields, op);
    if (ta) ta.value = JSON.stringify({ scope: op.scope || {}, params: op.params || {} }, null, 2);
    void savePipelineToServer();
    if ($("trajPreviewOn")?.checked) void previewTrajectoryUi();
  };
  for (const f of fields) {
    const row = document.createElement("div");
    const isTcpFrame = isExternalTcpBackendField(f);
    row.className =
      "op-field" +
      (f.type === "Message" && !isTcpFrame ? " msg" : "") +
      (isTcpFrame ? " op-field-frame" : "");
    row.dataset.fieldKey = f.key;
    const label = document.createElement("label");
    label.textContent = isTcpFrame
      ? "外部 TCP 坐标系"
      : (f.labelZh || f.messageZh || f.labelEn || f.key) + (f.unit ? ` (${f.unit})` : "");
    row.appendChild(label);
    // schema 仍为 Message；展开列表供选择（避免原生 select 被 overflow 裁掉）
    if (isTcpFrame) {
      const curTcp = values[f.key] !== undefined ? values[f.key] : readSchemaValueFromOp(op, f.key);
      const picker = buildExternalTcpFramePicker(frameOptions, curTcp, (backendId) => {
        applySchemaValueToOp(op, f.key, backendId || "");
        onChange();
      });
      row.appendChild(picker);
      box.appendChild(row);
      continue;
    }
    if (f.type === "Message") {
      row.textContent = f.messageZh || f.messageEn || f.key;
      box.appendChild(row);
      continue;
    }
    let ctrl;
    const cur = values[f.key] !== undefined ? values[f.key] : readSchemaValueFromOp(op, f.key);
    if (f.type === "Bool") {
      ctrl = document.createElement("input");
      ctrl.type = "checkbox";
      ctrl.checked = !!cur;
      ctrl.onchange = () => {
        applySchemaValueToOp(op, f.key, ctrl.checked);
        onChange();
      };
    } else if (f.type === "Enum") {
      ctrl = document.createElement("select");
      const opts = f.enumValues || [];
      const labs = f.enumLabelsZh || [];
      opts.forEach((v, i) => {
        const o = document.createElement("option");
        o.value = v;
        o.textContent = labs[i] || v;
        ctrl.appendChild(o);
      });
      ctrl.value = cur !== undefined && cur !== null ? String(cur) : opts[0] || "0";
      ctrl.onchange = () => {
        const n = Number(ctrl.value);
        applySchemaValueToOp(op, f.key, Number.isNaN(n) ? ctrl.value : n);
        if (f.key === "scope.kind" && Number(ctrl.value) === 2 && currentRawPointCount() > 0) {
          op.scope.pointFrom = 1;
          op.scope.pointTo = currentRawPointCount();
        }
        onChange();
      };
    } else if (f.type === "Vec3") {
      ctrl = document.createElement("div");
      ctrl.className = "op-vec3";
      const sx = f.vec3SuffixX || ".x";
      const sy = f.vec3SuffixY || ".y";
      const sz = f.vec3SuffixZ || ".z";
      for (const [suf, axis] of [
        [sx, "x"],
        [sy, "y"],
        [sz, "z"],
      ]) {
        const k = f.key + suf;
        const inp = document.createElement("input");
        inp.type = "number";
        inp.step = f.step || 0.1;
        inp.placeholder = axis;
        const vv = values[k] !== undefined ? values[k] : readSchemaValueFromOp(op, k);
        inp.value = vv !== undefined ? vv : 0;
        inp.onchange = () => {
          applySchemaValueToOp(op, k, Number(inp.value));
          onChange();
        };
        ctrl.appendChild(inp);
      }
    } else {
      ctrl = document.createElement("input");
      ctrl.type = "number";
      ctrl.step = f.type === "Int" ? 1 : f.step || 0.1;
      if (f.type === "Int") {
        const isP = f.key === "scope.pointFrom" || f.key === "scope.pointTo";
        if (isP) {
          ctrl.min = 1;
          ctrl.max = Math.max(1, currentRawPointCount() || Number(f.maxInt) || 1);
        } else {
          if (f.minInt != null) ctrl.min = f.minInt;
          if (f.maxInt != null) ctrl.max = f.maxInt;
        }
      } else {
        if (f.min != null) ctrl.min = f.min;
        if (f.max != null) ctrl.max = f.max;
      }
      ctrl.value = cur !== undefined && cur !== null ? cur : f.type === "Int" ? f.defaultInt : f.defaultDouble;
      ctrl.onchange = () => {
        applySchemaValueToOp(op, f.key, f.type === "Int" ? parseInt(ctrl.value, 10) : Number(ctrl.value));
        onChange();
      };
    }
    row.appendChild(ctrl);
    box.appendChild(row);
  }
  const enRow = document.createElement("div");
  enRow.className = "op-field";
  enRow.innerHTML = `<label>启用</label>`;
  const en = document.createElement("input");
  en.type = "checkbox";
  en.checked = op.enabled !== false;
  en.onchange = () => {
    op.enabled = en.checked;
    onChange();
  };
  enRow.appendChild(en);
  box.appendChild(enRow);
  refreshOpFieldVisibility(box, fields, op);
}

function currentEditGroupId() {
  return $("trajEditGroup")?.value || "";
}

function currentRawPointCount() {
  const n = Math.floor(Number(window.__trajRawPointCount) || 0);
  return n > 0 ? n : 0;
}

function makeDefaultPipelineOp(kind) {
  const scope = defaultScopeForNewOp(currentRawPointCount(), currentEditGroupId());
  return { kind, enabled: false, scope, params: {} };
}

function refreshTrajEditScopeCombos() {
  const progSel = $("trajEditProgram");
  const groupSel = $("trajEditGroup");
  if (!progSel || !groupSel) return;
  const rootId = activeSceneRootId();
  const { entry, prog } = activeProgram(rootId);
  const prevProg = progSel.value;
  const prevGroup = groupSel.value;
  progSel.innerHTML = "";
  for (const p of entry?.programs || []) {
    const o = document.createElement("option");
    o.value = p.id;
    o.textContent = p.name || p.id;
    progSel.appendChild(o);
  }
  if ([...progSel.options].some((o) => o.value === prevProg)) progSel.value = prevProg;
  else if (entry?.activeProgramId) progSel.value = entry.activeProgramId;
  groupSel.innerHTML = "";
  const none = document.createElement("option");
  none.value = "";
  none.textContent = "（无）";
  groupSel.appendChild(none);
  for (const g of prog?.groups || []) {
    const o = document.createElement("option");
    o.value = g.id;
    o.textContent = g.name || g.id;
    groupSel.appendChild(o);
  }
  if ([...groupSel.options].some((o) => o.value === prevGroup)) groupSel.value = prevGroup;
}

function renderOpPalette() {
  const box = $("opPalette");
  if (!box) return;
  box.innerHTML = "";
  for (const entry of opPaletteEntries) {
    const b = document.createElement("button");
    b.type = "button";
    b.textContent = entry.displayNameZh || entry.kind;
    b.title = entry.kind;
    b.onclick = () => {
      if (!trajFeatureEditActive) {
        setStatus("请先「开始修改」再添加算子", "warn");
        return;
      }
      trajPipelineOps.push(makeDefaultPipelineOp(entry.kind));
      trajOpSel = trajPipelineOps.length - 1;
      renderOpPipeline();
      void syncOpParamsEditor();
      void savePipelineToServer();
      setStatus(`已添加「${entry.displayNameZh}」（未启用，请勾选启用）`);
    };
    box.appendChild(b);
  }
}

function renderOpPipeline() {
  const box = $("opPipeline");
  if (!box) return;
  box.innerHTML = "";
  if (!trajPipelineOps.length) {
    box.classList.add("muted");
    box.textContent = "空";
    trajOpSel = -1;
    void syncOpParamsEditor();
    return;
  }
  box.classList.remove("muted");
  trajPipelineOps.forEach((op, i) => {
    const row = document.createElement("div");
    const en = op.enabled !== false;
    row.className = "step" + (i === trajOpSel ? " sel" : "") + (en ? "" : " disabled-op");
    row.innerHTML = `<span class="t">${i + 1}. ${opDisplayName(op.kind)}</span>
      <label class="en"><input type="checkbox" ${en ? "checked" : ""}/>启用</label>
      <button type="button" class="rm" title="移除">×</button>`;
    row.onclick = () => {
      trajOpSel = i;
      renderOpPipeline();
      void syncOpParamsEditor();
    };
    row.querySelector("input").onchange = (ev) => {
      ev.stopPropagation();
      if (!trajFeatureEditActive) {
        ev.target.checked = op.enabled !== false;
        setStatus("请先「开始修改」再编辑算子", "warn");
        return;
      }
      op.enabled = !!ev.target.checked;
      renderOpPipeline();
      void savePipelineToServer();
    };
    row.querySelector(".rm").onclick = (ev) => {
      ev.stopPropagation();
      if (!trajFeatureEditActive) {
        setStatus("请先「开始修改」再编辑算子", "warn");
        return;
      }
      trajPipelineOps.splice(i, 1);
      if (trajOpSel >= trajPipelineOps.length) trajOpSel = trajPipelineOps.length - 1;
      renderOpPipeline();
      void syncOpParamsEditor();
      void savePipelineToServer();
    };
    row.oncontextmenu = (ev) => {
      ev.preventDefault();
      trajOpSel = i;
      const choices = [];
      if (i > 0) choices.push("上移");
      if (i + 1 < trajPipelineOps.length) choices.push("下移");
      choices.push("移除");
      const tip = `${opDisplayName(op.kind)}\n` + choices.map((c, n) => `${n + 1}. ${c}`).join("\n") + "\n0. 取消";
      const ans = prompt(tip, "1");
      if (ans == null || ans === "0") return;
      const n = parseInt(ans, 10);
      const label = choices[n - 1];
      if (label === "上移" && i > 0) {
        [trajPipelineOps[i - 1], trajPipelineOps[i]] = [trajPipelineOps[i], trajPipelineOps[i - 1]];
        trajOpSel = i - 1;
      } else if (label === "下移" && i + 1 < trajPipelineOps.length) {
        [trajPipelineOps[i], trajPipelineOps[i + 1]] = [trajPipelineOps[i + 1], trajPipelineOps[i]];
        trajOpSel = i + 1;
      } else if (label === "移除") {
        trajPipelineOps.splice(i, 1);
        trajOpSel = Math.min(i, trajPipelineOps.length - 1);
      } else return;
      renderOpPipeline();
      void syncOpParamsEditor();
      void savePipelineToServer();
    };
    box.appendChild(row);
  });
}

async function savePipelineToServer() {
  if (!trajFeatureEditActive) {
    setStatus("请先「开始修改」再编辑算子", "warn");
    return;
  }
  const r = await (
    await fetch("/api/trajectory/pipeline", {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(trajPipelineOps),
    })
  ).json();
  if (!r.ok) setStatus(r.error || "管线保存失败", "err");
  else if ($("trajPreviewOn")?.checked) void previewTrajectoryUi();
}

async function reloadPipelineFromServer() {
  try {
    const raw = await (await fetch("/api/trajectory/pipeline")).json();
    trajPipelineOps = Array.isArray(raw) ? raw : raw.pipeline || [];
  } catch {
    trajPipelineOps = [];
  }
  renderOpPipeline();
}

$("btnPlan").onclick = () => void planSelectedInstruction();
$("btnDelInstr") && ($("btnDelInstr").onclick = () => void deleteSelectedInstruction());
$("btnDelInstr2") && ($("btnDelInstr2").onclick = () => void deleteSelectedInstruction());
$("btnClearProg").onclick = () => void clearActiveProgram();

$("btnPathPlanNew") && ($("btnPathPlanNew").onclick = () => void createPathPlanFromUi());
$("pathPlanSelect") && ($("pathPlanSelect").onchange = () => void bindSelectedPathPlan());
$("btnTrajBeginEdit") && ($("btnTrajBeginEdit").onclick = () => void beginTrajEditFromUi());
$("btnTrajCancelEdit") && ($("btnTrajCancelEdit").onclick = () => void cancelTrajEditFromUi());
$("btnPickEdge") &&
  ($("btnPickEdge").onclick = () => {
    trajPickMode = "edge";
    resolveFeatureStrategy();
    syncPickModeButtons();
    updatePickStatusLabel();
    setStatus("拾取线：在视口点击 BREP 边");
  });
$("btnPickFace") &&
  ($("btnPickFace").onclick = () => {
    trajPickMode = "face";
    resolveFeatureStrategy();
    syncPickModeButtons();
    updatePickStatusLabel();
    setStatus("拾取面：在视口点击 BREP 面");
  });
$("btnPickCancel") &&
  ($("btnPickCancel").onclick = () => {
    trajPickMode = null;
    trajAppendMode = false;
    syncPickModeButtons();
    clearPickHighlight();
    updatePickStatusLabel();
    setStatus("已取消拾取");
  });
document.querySelector(".feat-table-wrap")?.addEventListener("contextmenu", (ev) => {
  if (ev.target.closest("tr") && !ev.target.closest("tr.empty")) return;
  ev.preventDefault();
  ev.stopPropagation();
  showFeatTableContextMenu(ev.clientX, ev.clientY, -1);
});
$("btnFeatNew") &&
  ($("btnFeatNew").onclick = () => {
    trajAppendMode = false;
    syncFeatWriteModeButtons();
    updatePickStatusLabel();
    setStatus("新建特征：下一次拾取写入新行");
  });
$("btnFeatAppend") &&
  ($("btnFeatAppend").onclick = () => {
    trajAppendMode = true;
    syncFeatWriteModeButtons();
    updatePickStatusLabel();
    setStatus(trajFeatSel < 0 ? "追加模式：请先选中特征行" : "追加模式：下一次拾取写入选中行", trajFeatSel < 0 ? "warn" : "info");
  });
$("btnFeatDelete") &&
  ($("btnFeatDelete").onclick = () => {
    if (trajFeatSel < 0) {
      setStatus("请先选中特征行", "warn");
      return;
    }
    trajFeatures.splice(trajFeatSel, 1);
    trajFeatSel = trajFeatures.length ? Math.min(trajFeatSel, trajFeatures.length - 1) : -1;
    renderFeatTable();
    void rebuildFeatParamForm();
    scheduleAutoDiscretize();
  });
$("featStrategy") &&
  ($("featStrategy").onchange = () => {
    const sid = $("featStrategy").value;
    if (trajFeatSel >= 0) {
      trajFeatures[trajFeatSel].strategyId = sid;
      void (async () => {
        trajFeatures[trajFeatSel].params = await makeFeatureParams(sid);
        trajFeatures[trajFeatSel].status = "草稿";
        renderFeatTable();
        await rebuildFeatParamForm();
        scheduleAutoDiscretize();
      })();
    } else {
      void rebuildFeatParamForm();
    }
  });
$("btnFeatCatalog") &&
  ($("btnFeatCatalog").onclick = async () => {
    const wp = $("trajWorkpiece")?.value;
    if (!wp) {
      setStatus("请选择工件", "warn");
      return;
    }
    const r = await (await fetch(`/api/trajectory/feature-catalog?workpiece=${encodeURIComponent(wp)}`)).json();
    if (r.ok === false) {
      setStatus(r.error || "特征目录失败", "err");
      return;
    }
    setStatus(`特征目录已拉取（详见控制台）`);
    console.info("[feature-catalog]", r);
  });
$("btnFeatDiscretize") && ($("btnFeatDiscretize").onclick = () => void discretizeFeaturesUi());
$("btnDiscTplSave") &&
  ($("btnDiscTplSave").onclick = async () => {
    let name = currentDiscTplName();
    if (!name) name = prompt("模板名称") || "";
    name = name.trim();
    if (!name) {
      setStatus("请输入离散模板名", "warn");
      return;
    }
    const strategyId = $("featStrategy")?.value || "EdgeChain";
    const f = trajFeatSel >= 0 ? trajFeatures[trajFeatSel] : null;
    const params = f?.params || (await makeFeatureParams(strategyId));
    const r = await (
      await fetch("/api/trajectory/templates/discretize", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name, payload: { strategyId, params } }),
      })
    ).json();
    if (r.ok) await refreshDiscTemplateSelect();
    if ($("discTplSelect") && r.ok) $("discTplSelect").value = name;
    setStatus(r.ok ? "离散模板已存" : r.error || "保存失败", r.ok ? "info" : "err");
  });
$("btnDiscTplLoad") &&
  ($("btnDiscTplLoad").onclick = async () => {
    const name = currentDiscTplName();
    if (!name) {
      setStatus("请先选择模板", "warn");
      return;
    }
    const raw = await (await fetch(`/api/trajectory/templates/discretize/${encodeURIComponent(name)}`)).json();
    if (raw && raw.ok === false) {
      setStatus(raw.error || "载入失败", "err");
      return;
    }
    const payload = raw.payload || raw;
    const strategyId = payload.strategyId || $("featStrategy")?.value;
    const params = payload.params || {};
    if ($("featStrategy") && strategyId) $("featStrategy").value = strategyId;
    if (trajFeatSel >= 0) {
      trajFeatures[trajFeatSel].strategyId = strategyId;
      trajFeatures[trajFeatSel].params = { ...params };
      trajFeatures[trajFeatSel].status = "草稿";
    }
    renderFeatTable();
    await rebuildFeatParamForm();
    scheduleAutoDiscretize();
    setStatus("离散模板已载入");
  });
$("btnDiscTplDelete") &&
  ($("btnDiscTplDelete").onclick = async () => {
    const name = currentDiscTplName();
    if (!name) return;
    const r = await (
      await fetch(`/api/trajectory/templates/discretize/${encodeURIComponent(name)}`, { method: "DELETE" })
    ).json();
    if (r.ok) await refreshDiscTemplateSelect();
    setStatus(r.ok ? "模板已删" : r.error || "删除失败", r.ok ? "info" : "err");
  });
$("btnDiscTplExport") &&
  ($("btnDiscTplExport").onclick = async () => {
    const name = currentDiscTplName();
    if (!name) {
      setStatus("请先选择模板", "warn");
      return;
    }
    const raw = await (await fetch(`/api/trajectory/templates/discretize/${encodeURIComponent(name)}`)).json();
    if (raw && raw.ok === false) {
      setStatus(raw.error || "导出失败", "err");
      return;
    }
    const blob = new Blob([JSON.stringify(raw.payload || raw, null, 2)], { type: "application/json" });
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = `${name}.json`;
    a.click();
    URL.revokeObjectURL(a.href);
    setStatus("模板已导出");
  });
$("btnDiscTplImport") &&
  ($("btnDiscTplImport").onclick = () => {
    const inp = document.createElement("input");
    inp.type = "file";
    inp.accept = "application/json,.json";
    inp.onchange = async () => {
      const file = inp.files?.[0];
      if (!file) return;
      try {
        const text = await file.text();
        const payload = JSON.parse(text);
        const name = (file.name || "import").replace(/\.json$/i, "");
        const r = await (
          await fetch("/api/trajectory/templates/discretize", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ name, payload }),
          })
        ).json();
        if (r.ok) {
          await refreshDiscTemplateSelect();
          if ($("discTplSelect")) $("discTplSelect").value = name;
        }
        setStatus(r.ok ? "模板已导入" : r.error || "导入失败", r.ok ? "info" : "err");
      } catch (e) {
        setStatus(`导入失败: ${e.message || e}`, "err");
      }
    };
    inp.click();
  });
$("btnTrajPreviewRaw") && ($("btnTrajPreviewRaw").onclick = () => void previewTrajectoryRawUi());
["previewAxisX", "previewAxisY", "previewAxisZ"].forEach((id) => {
  const el = $(id);
  if (el) el.onchange = () => redrawRawPreviewOverlay();
});
$("previewAxisInterval") &&
  ($("previewAxisInterval").onchange = () => redrawRawPreviewOverlay());
$("btnOpParamsApply") &&
  ($("btnOpParamsApply").onclick = () => {
    if (trajOpSel < 0 || trajOpSel >= trajPipelineOps.length) {
      setStatus("请先选中流水线算子", "warn");
      return;
    }
    try {
      const raw = JSON.parse($("opParamsJson")?.value || "{}");
      if (raw.params || raw.scope) {
        if (raw.scope) trajPipelineOps[trajOpSel].scope = raw.scope;
        if (raw.params) trajPipelineOps[trajOpSel].params = raw.params;
        if (raw.enabled !== undefined) trajPipelineOps[trajOpSel].enabled = !!raw.enabled;
      } else {
        trajPipelineOps[trajOpSel].params = raw;
      }
      void savePipelineToServer();
      void syncOpParamsEditor();
      setStatus("算子参数已更新");
    } catch (e) {
      setStatus("参数 JSON 无效", "err");
    }
  });
$("btnMeshGenerate") &&
  ($("btnMeshGenerate").onclick = async () => {
    const parseCsv3 = (s) =>
      String(s || "")
        .split(",")
        .map((x) => Number(x.trim()) || 0);
    const method = $("meshMethod")?.value || "CrossSection";
    const triCsv = String($("meshTriIndices")?.value || "")
      .split(",")
      .map((x) => x.trim())
      .filter(Boolean)
      .map((x) => Number(x));
    if (method === "BsplineRegion" && !triCsv.length) {
      setStatus("B样条需要填写三角索引 CSV", "warn");
      return;
    }
    const o = parseCsv3($("meshPlaneOrigin")?.value);
    const n = parseCsv3($("meshPlaneNormal")?.value);
    const spec = {
      schemaVersion: 1,
      workpiece: { backendIdUtf8: $("meshWorkpiece")?.value || "", frameId: "workpiece" },
      method,
      crossSection: { planeOriginMm: o, planeNormal: n },
      discretize: { stepMm: Number($("meshStepMm")?.value) || 2 },
      region: { triangleIndices: method === "BsplineRegion" ? triCsv : [] },
      bspline: method === "BsplineRegion" ? { uvCountU: 8, uvCountV: 8 } : {},
    };
    const r = await (
      await fetch("/api/trajectory/mesh-spec", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(spec),
      })
    ).json();
    setStatus(r.ok ? "Mesh Raw 已生成" : r.error || "失败", r.ok ? "info" : "err");
    if (r.ok) {
      await syncTrajSessionStatus();
      void previewTrajectoryRawUi();
    }
  });
$("btnRecipeFill") &&
  ($("btnRecipeFill").onclick = async () => {
    const r = await (
      await fetch("/api/trajectory/recipe", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ recipe: $("trajRecipe")?.value || "weld" }),
      })
    ).json();
    setStatus(r.ok ? "工艺模板已填充到流水线" : r.error || "失败", r.ok ? "info" : "err");
    if (r.ok) await reloadPipelineFromServer();
  });
$("btnTrajEmit") &&
  ($("btnTrajEmit").onclick = async () => {
    const r = await (await fetch("/api/trajectory/emit", { method: "POST" })).json();
    setStatus(r.ok ? "已生成 → LINE" : r.error || "生成失败", r.ok ? "info" : "err");
    if (r.ok) {
      exitTrajEditUiAfterCommit();
      await refreshInstructionTreeAfterTrajectory();
      await reloadPipelineFromServer();
      await syncTrajSessionStatus();
    }
  });
$("btnTrajApply") &&
  ($("btnTrajApply").onclick = async () => {
    const r = await (await fetch("/api/trajectory/apply", { method: "POST" })).json();
    setStatus(r.ok ? "已应用 → LINE" : r.error || "应用失败", r.ok ? "info" : "err");
    if (r.ok) {
      exitTrajEditUiAfterCommit();
      trajPipelineOps = [];
      trajOpSel = -1;
      renderOpPipeline();
      await refreshInstructionTreeAfterTrajectory();
      await reloadPipelineFromServer();
      await syncTrajSessionStatus();
    }
  });
$("btnTrajReset") &&
  ($("btnTrajReset").onclick = async () => {
    await fetch("/api/trajectory/reset", { method: "POST" });
    await reloadPipelineFromServer();
    clearRawPreviewOverlay();
    rawPreviewActive = false;
    await syncTrajSessionStatus();
  });
$("btnTrajUndo") &&
  ($("btnTrajUndo").onclick = async () => {
    await fetch("/api/trajectory/undo", { method: "POST" });
    await syncTrajSessionStatus();
    await reloadPipelineFromServer();
  });
$("btnTrajRedo") &&
  ($("btnTrajRedo").onclick = async () => {
    await fetch("/api/trajectory/redo", { method: "POST" });
    await syncTrajSessionStatus();
    await reloadPipelineFromServer();
  });

async function refreshPipeTemplateSelect() {
  const sel = $("pipeTplSelect");
  if (!sel) return;
  const prev = sel.value;
  const r = await (await fetch("/api/trajectory/templates/pipeline")).json();
  const names = Array.isArray(r) ? r : r.templates || r.names || [];
  sel.innerHTML = "";
  const empty = document.createElement("option");
  empty.value = "";
  empty.textContent = names.length ? "（选择模板）" : "（无模板）";
  sel.appendChild(empty);
  for (const n of names) {
    const name = typeof n === "string" ? n : n.name || n.id || "";
    if (!name) continue;
    const o = document.createElement("option");
    o.value = name;
    o.textContent = name;
    sel.appendChild(o);
  }
  if ([...sel.options].some((o) => o.value === prev)) sel.value = prev;
}

function currentPipeTplName() {
  return $("pipeTplSelect")?.value || $("tplName")?.value?.trim() || "";
}

$("btnTplSave") &&
  ($("btnTplSave").onclick = async () => {
    let name = currentPipeTplName();
    if (!name) name = prompt("流水线模板名称") || "";
    name = name.trim();
    if (!name) {
      setStatus("请输入模板名", "warn");
      return;
    }
    const r = await (
      await fetch("/api/trajectory/templates/pipeline", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ name, payload: trajPipelineOps }),
      })
    ).json();
    if (r.ok) await refreshPipeTemplateSelect();
    if ($("pipeTplSelect") && r.ok) $("pipeTplSelect").value = name;
    setStatus(r.ok ? "流水线模板已存" : r.error || "失败", r.ok ? "info" : "err");
  });
$("btnTplLoad") &&
  ($("btnTplLoad").onclick = async () => {
    const name = currentPipeTplName();
    if (!name) {
      setStatus("请先选择模板", "warn");
      return;
    }
    const raw = await (await fetch(`/api/trajectory/templates/pipeline/${encodeURIComponent(name)}`)).json();
    if (raw && raw.ok === false) {
      setStatus(raw.error || "载入失败", "err");
      return;
    }
    trajPipelineOps = Array.isArray(raw) ? raw : raw.payload || raw.pipeline || [];
    trajOpSel = trajPipelineOps.length ? 0 : -1;
    renderOpPipeline();
    syncOpParamsEditor();
    void savePipelineToServer();
    setStatus("流水线模板已加载");
  });
$("btnTplDelete") &&
  ($("btnTplDelete").onclick = async () => {
    const name = currentPipeTplName();
    if (!name) return;
    if (!confirm(`删除流水线模板「${name}」？`)) return;
    const r = await (await fetch(`/api/trajectory/templates/pipeline/${encodeURIComponent(name)}`, { method: "DELETE" })).json();
    if (r.ok) await refreshPipeTemplateSelect();
    setStatus(r.ok ? "模板已删" : r.error || "删除失败", r.ok ? "info" : "err");
  });
$("btnTplExport") &&
  ($("btnTplExport").onclick = () => {
    const name = currentPipeTplName() || "pipeline";
    const blob = new Blob([JSON.stringify({ name, payload: trajPipelineOps }, null, 2)], { type: "application/json" });
    const a = document.createElement("a");
    a.href = URL.createObjectURL(blob);
    a.download = `${name}.pipeline.json`;
    a.click();
    URL.revokeObjectURL(a.href);
    setStatus("已导出流水线模板");
  });
$("btnTplImport") &&
  ($("btnTplImport").onclick = () => {
    const input = document.createElement("input");
    input.type = "file";
    input.accept = "application/json,.json";
    input.onchange = async () => {
      const file = input.files?.[0];
      if (!file) return;
      try {
        const text = await file.text();
        const doc = JSON.parse(text);
        const name = (doc.name || file.name.replace(/\.json$/i, "") || "imported").trim();
        const payload = doc.payload || doc.pipeline || doc;
        const ops = Array.isArray(payload) ? payload : [];
        const r = await (
          await fetch("/api/trajectory/templates/pipeline", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ name, payload: ops }),
          })
        ).json();
        if (r.ok) {
          await refreshPipeTemplateSelect();
          if ($("pipeTplSelect")) $("pipeTplSelect").value = name;
          trajPipelineOps = ops;
          trajOpSel = ops.length ? 0 : -1;
          renderOpPipeline();
          void syncOpParamsEditor();
          void savePipelineToServer();
        }
        setStatus(r.ok ? `已导入「${name}」` : r.error || "导入失败", r.ok ? "info" : "err");
      } catch (e) {
        setStatus(e.message || "导入失败", "err");
      }
    };
    input.click();
  });
$("trajEditProgram") &&
  ($("trajEditProgram").onchange = () => {
    const rootId = activeSceneRootId();
    const { entry } = activeProgram(rootId);
    if (entry && $("trajEditProgram").value) entry.activeProgramId = $("trajEditProgram").value;
    refreshTrajEditScopeCombos();
  });

$("btnTplList") &&
  ($("btnTplList").onclick = async () => {
    const r = await (await fetch("/api/trajectory/templates/pipeline")).json();
    const out = $("tplListOut");
    if (out) out.textContent = JSON.stringify(r.templates || r || [], null, 2);
  });

renderer.domElement.addEventListener("pointerdown", (ev) => {
  if (!trajPickMode || ev.button !== 0) return;
  if (transform.dragging) return;
  ev.preventDefault();
  void commitTrajPick(ev);
});
renderer.domElement.addEventListener("pointermove", (ev) => {
  if (!trajPickMode || transform.dragging) return;
  scheduleHoverTrajPick(ev);
});
renderer.domElement.addEventListener("pointerleave", () => {
  if (hoverPickTimer) clearTimeout(hoverPickTimer);
  hoverPickSeq++;
  clearPickHighlight();
});

$("btnRun").onclick = () => void runProgramPlayback();
$("btnStop").onclick = () => void stopProgramPlayback();
$("btnExport").onclick = () => {
  setStatus("导出后续迭代（当前为 stub）", "warn");
  fetch("/api/robot/export", { method: "POST" });
};
$("btnDrag") &&
  ($("btnDrag").onclick = () => {
    void setRobotDragMode(!robotDragMode);
  });

$("btnGeo").onclick = async () => {
  const op = $("geoOp").value;
  const endpoint = op === "boolean" ? "/api/geometry/op" : "/api/pointcloud/op";
  const r = await (
    await fetch(endpoint, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ op, targetId: selectedId }),
    })
  ).json();
  setStatus(r.ok ? `作业 ${op} 已排队` : "失败", r.ok ? "info" : "err");
};

$("btnAi").onclick = async () => {
  const r = await (
    await fetch("/api/ai/chat", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ message: $("aiPrompt").value }),
    })
  ).json();
  $("aiOut").textContent = r.content || JSON.stringify(r);
};

$("btnAbout").onclick = () => {
  document.querySelectorAll(".menu").forEach((m) => m.classList.remove("open"));
  setStatus("CloudSim Web — 与桌面布局对齐的浏览器壳");
};

async function loadModes() {
  const cat = await (await fetch("/api/modes")).json();
  modeSelect.innerHTML = "";
  for (const m of cat.modes || []) {
    const opt = document.createElement("option");
    opt.value = m.id;
    opt.textContent = m.title;
    if (m.id === cat.active) opt.selected = true;
    modeSelect.appendChild(opt);
  }
}
modeSelect.onchange = async () => {
  await fetch("/api/sidecar/workspaceMode", {
    method: "PUT",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ mode: modeSelect.value }),
  });
  setStatus(`模式 ${modeSelect.value}`);
};

window.addEventListener("keydown", (ev) => {
  if (ev.target && /INPUT|TEXTAREA|SELECT/.test(ev.target.tagName)) return;
  if (sceneInteractMode !== "select" || robotDragMode) return;
  if (!transform.object) return;
  if (ev.key === "g" || ev.key === "G") transform.setMode("translate");
  if (ev.key === "r" || ev.key === "R") transform.setMode("rotate");
});

renderer.domElement.addEventListener("click", (ev) => {
  if (transform.dragging) return;
  if (trajPickMode) return;
  if (sceneInteractMode !== "select") return;
  const rect = renderer.domElement.getBoundingClientRect();
  const pointer = new THREE.Vector2(
    ((ev.clientX - rect.left) / rect.width) * 2 - 1,
    -((ev.clientY - rect.top) / rect.height) * 2 + 1
  );
  const raycaster = new THREE.Raycaster();
  raycaster.setFromCamera(pointer, camera);
  const hits = raycaster.intersectObjects([...idToMesh.values()], true);
  if (!hits.length) return;
  let o = hits[0].object;
  while (o && !o.userData.backendId) o = o.parent;
  if (o) void selectObject(resolveScenePickId(o.userData.backendId));
});

$("btnInteractView") &&
  ($("btnInteractView").onclick = () => {
    setSceneInteractMode("view");
    document.querySelectorAll(".menu").forEach((m) => m.classList.remove("open"));
  });
$("btnInteractSelect") &&
  ($("btnInteractSelect").onclick = () => {
    setSceneInteractMode("select");
    document.querySelectorAll(".menu").forEach((m) => m.classList.remove("open"));
  });
syncInteractModeMenu();

bindUiChrome();
appendLog("CloudSim Web 就绪");
void (async () => {
  await refreshObjects(false);
  await loadProgramsFromServer();
})();

// —— 坐标系面板（对齐 RobotFrameSettingsWidget）——
let frameState = null;
let frameLinkNames = [];
let selectedToolId = "";
let selectedUserId = "";
let frameUiBlock = false;
let framesPutTimer = 0;
let framesIgnoreSseReload = 0;

function emptyRigid() {
  return { positionMm: [0, 0, 0], eulerDeg: [0, 0, 0] };
}

function ensureFramesDefaults(f) {
  if (!f) f = {};
  if (!Array.isArray(f.toolFrames)) f.toolFrames = [];
  if (!Array.isArray(f.userFrames)) f.userFrames = [];
  if (typeof f.showToolFrame !== "boolean") f.showToolFrame = true;
  if (typeof f.showUserFrames !== "boolean") f.showUserFrames = true;
  if (!f.toolFrames.length) {
    f.toolFrames.push({
      id: "TFR_1",
      name: "Tool1",
      T_flange_tool: emptyRigid(),
      flangeLinkName: "",
      showInScene: true,
    });
    f.activeToolFrameId = "TFR_1";
  }
  if (!f.activeToolFrameId) f.activeToolFrameId = f.toolFrames[0].id;
  if (!f.userFrames.length) {
    f.userFrames.push({ id: "UFR_1", name: "User1", T_base_user: emptyRigid(), showInScene: true });
    f.activeUserFrameId = "UFR_1";
  }
  if (!f.activeUserFrameId) f.activeUserFrameId = f.userFrames[0].id;
  return f;
}

function clearFrameOverlayGroup() {
  for (const c of [...frameOverlays.children]) {
    frameOverlays.remove(c);
    c.traverse((o) => {
      if (o.geometry) o.geometry.dispose();
      if (o.material) {
        if (Array.isArray(o.material)) o.material.forEach((m) => m.dispose());
        else o.material.dispose();
      }
    });
  }
}

function addFrameAxisMarker(worldMatrixOrPose, euler, active, kind) {
  const g = new THREE.Group();
  g.userData.frameKind = kind;
  g.userData.active = !!active;
  if (Array.isArray(worldMatrixOrPose) && worldMatrixOrPose.length >= 16) {
    // 与连杆 mesh 同一 worldMatrix 编码，避免欧拉分解与 Z-up 叠加后轴向相反
    const m = new THREE.Matrix4().fromArray(worldMatrixOrPose.map(Number));
    m.decompose(g.position, g.quaternion, g.scale);
  } else if (worldMatrixOrPose) {
    const pose = worldMatrixOrPose;
    g.position.set(Number(pose[0]) || 0, Number(pose[1]) || 0, Number(pose[2]) || 0);
    if (euler) g.quaternion.copy(eulerZyxDegToQuat(Number(euler[0]) || 0, Number(euler[1]) || 0, Number(euler[2]) || 0));
  } else {
    return;
  }
  const len = active ? 55 : 40;
  const addAxis = (dir, color) => {
    const geo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), dir.clone().multiplyScalar(len)]);
    g.add(new THREE.Line(geo, new THREE.LineBasicMaterial({ color, depthTest: true })));
  };
  addAxis(new THREE.Vector3(1, 0, 0), kind === "user" ? 0xffaa66 : 0xff6666);
  addAxis(new THREE.Vector3(0, 1, 0), kind === "user" ? 0xaaff66 : 0x66ff66);
  addAxis(new THREE.Vector3(0, 0, 1), kind === "user" ? 0x66aaff : 0x6699ff);
  const pt = new THREE.BufferGeometry();
  pt.setAttribute("position", new THREE.Float32BufferAttribute([0, 0, 0], 3));
  g.add(
    new THREE.Points(
      pt,
      new THREE.PointsMaterial({
        color: active ? 0xffcc00 : kind === "user" ? 0x66ccff : 0x00ff88,
        size: active ? 7 : 5,
        sizeAttenuation: false,
        depthTest: true,
      })
    )
  );
  frameOverlays.add(g);
}

/// 拖拽中让当前工具系叠加立刻跟罗盘，不等 IK/SSE
function followActiveToolOverlayToDragProxy() {
  if (!robotDragMode || !dragFlangeId) return;
  for (const child of frameOverlays.children) {
    if (child.userData.frameKind === "tool" && child.userData.active) {
      child.position.copy(dragProxy.position);
      child.quaternion.copy(dragProxy.quaternion);
      child.scale.copy(dragProxy.scale);
    }
  }
}

async function refreshFrameOverlays() {
  scrubStaleRobotRootRefs();
  let rootId = activeSceneRootId();
  if (!robotSceneRootIds.size || (rootId && !robotSceneRootIds.has(rootId))) {
    clearFrameOverlayGroup();
    return;
  }
  if (!rootId) {
    clearFrameOverlayGroup();
    return;
  }
  try {
    const q = encodeURIComponent(rootId);
    const r = await (await fetch(`/api/robot/frames/overlays?sceneRootBackendId=${q}`)).json();
    clearFrameOverlayGroup();
    if (!r.ok) return;
    // 请求回来时机器人可能已被删
    if (!robotSceneRootIds.has(rootId)) return;
    for (const t of r.tools || []) {
      const wm = Array.isArray(t.worldMatrix) && t.worldMatrix.length >= 16 ? t.worldMatrix : null;
      addFrameAxisMarker(wm || t.positionMm, t.eulerDeg, !!t.active, "tool");
    }
    for (const u of r.users || []) {
      const wm = Array.isArray(u.worldMatrix) && u.worldMatrix.length >= 16 ? u.worldMatrix : null;
      addFrameAxisMarker(wm || u.positionMm, u.eulerDeg, !!u.active, "user");
    }
    // 拖拽空闲时罗盘直接拷贝刚画好的激活工具系，与末端 TCP 轴共点
    if (robotDragMode && dragFlangeId && !transform.dragging) {
      for (const child of frameOverlays.children) {
        if (child.userData.frameKind !== "tool" || !child.userData.active) continue;
        const wasOnProxy = transform.object === dragProxy;
        if (wasOnProxy) transform.detach();
        dragProxy.position.copy(child.position);
        dragProxy.quaternion.copy(child.quaternion);
        dragProxy.scale.set(1, 1, 1);
        dragProxy.matrixAutoUpdate = true;
        dragProxy.updateMatrix();
        dragProxy.updateMatrixWorld(true);
        transform.enabled = true;
        transform.attach(dragProxy);
        transform.getHelper().visible = true;
        break;
      }
    }
  } catch {
    /* ignore */
  }
}

function readSpinRigid(prefix) {
  const el = (k) => document.querySelector(`[data-${prefix}="${k}"]`);
  return {
    positionMm: [Number(el("px")?.value) || 0, Number(el("py")?.value) || 0, Number(el("pz")?.value) || 0],
    eulerDeg: [Number(el("ex")?.value) || 0, Number(el("ey")?.value) || 0, Number(el("ez")?.value) || 0],
  };
}

function writeSpinRigid(prefix, rigid) {
  const r = rigid || emptyRigid();
  const pos = r.positionMm || [0, 0, 0];
  const eu = r.eulerDeg || [0, 0, 0];
  const set = (k, v) => {
    const node = document.querySelector(`[data-${prefix}="${k}"]`);
    if (node) node.value = String(Number(v) || 0);
  };
  set("px", pos[0]);
  set("py", pos[1]);
  set("pz", pos[2]);
  set("ex", eu[0]);
  set("ey", eu[1]);
  set("ez", eu[2]);
}

function renderFrameLists() {
  const f = frameState;
  const toolUl = $("toolFrameList");
  const userUl = $("userFrameList");
  if (!toolUl || !userUl || !f) return;
  frameUiBlock = true;
  toolUl.innerHTML = "";
  for (const t of f.toolFrames) {
    const li = document.createElement("li");
    if (t.id === selectedToolId) li.classList.add("sel");
    const nm = document.createElement("span");
    nm.className = "nm";
    nm.textContent = `${t.name || t.id}${t.id === f.activeToolFrameId ? " *" : ""}`;
    const chk = document.createElement("input");
    chk.type = "checkbox";
    chk.checked = t.showInScene !== false;
    chk.onclick = (ev) => {
      ev.stopPropagation();
      t.showInScene = !!chk.checked;
      void commitFramesNow();
    };
    li.appendChild(nm);
    li.appendChild(chk);
    li.onclick = () => {
      selectedToolId = t.id;
      loadToolFields();
      renderFrameLists();
    };
    toolUl.appendChild(li);
  }
  userUl.innerHTML = "";
  for (const u of f.userFrames) {
    const li = document.createElement("li");
    if (u.id === selectedUserId) li.classList.add("sel");
    const nm = document.createElement("span");
    nm.className = "nm";
    nm.textContent = `${u.name || u.id}${u.id === f.activeUserFrameId ? " *" : ""}`;
    const chk = document.createElement("input");
    chk.type = "checkbox";
    chk.checked = u.showInScene !== false;
    chk.onclick = (ev) => {
      ev.stopPropagation();
      u.showInScene = !!chk.checked;
      void commitFramesNow();
    };
    li.appendChild(nm);
    li.appendChild(chk);
    li.onclick = () => {
      selectedUserId = u.id;
      loadUserFields();
      renderFrameLists();
    };
    userUl.appendChild(li);
  }
  const flangeSel = $("toolFlangeLink");
  if (flangeSel) {
    const curTool = f.toolFrames.find((t) => t.id === selectedToolId);
    const cur = (curTool && curTool.flangeLinkName) || f.flangeLinkName || "";
    flangeSel.innerHTML = "";
    const opt0 = document.createElement("option");
    opt0.value = "";
    opt0.textContent = "(默认)";
    flangeSel.appendChild(opt0);
    for (const name of frameLinkNames) {
      const o = document.createElement("option");
      o.value = name;
      o.textContent = name;
      flangeSel.appendChild(o);
    }
    flangeSel.value = cur;
  }
  if ($("chkShowToolFrames")) $("chkShowToolFrames").checked = !!f.showToolFrame;
  if ($("chkShowUserFrames")) $("chkShowUserFrames").checked = !!f.showUserFrames;
  frameUiBlock = false;
}

function loadToolFields() {
  const f = frameState;
  if (!f) return;
  const t = f.toolFrames.find((x) => x.id === selectedToolId) || f.toolFrames[0];
  if (!t) return;
  selectedToolId = t.id;
  frameUiBlock = true;
  writeSpinRigid("tool", t.T_flange_tool);
  frameUiBlock = false;
}

function loadUserFields() {
  const f = frameState;
  if (!f) return;
  const u = f.userFrames.find((x) => x.id === selectedUserId) || f.userFrames[0];
  if (!u) return;
  selectedUserId = u.id;
  frameUiBlock = true;
  writeSpinRigid("user", u.T_base_user);
  frameUiBlock = false;
}

function saveToolFieldsFromUi() {
  const f = frameState;
  if (!f || frameUiBlock) return;
  const t = f.toolFrames.find((x) => x.id === selectedToolId);
  if (!t) return;
  t.T_flange_tool = readSpinRigid("tool");
  t.flangeLinkName = $("toolFlangeLink")?.value || "";
}

function saveUserFieldsFromUi() {
  const f = frameState;
  if (!f || frameUiBlock) return;
  const u = f.userFrames.find((x) => x.id === selectedUserId);
  if (!u) return;
  u.T_base_user = readSpinRigid("user");
}

async function putFramesNow() {
  const rootId = activeSceneRootId();
  if (!rootId || !frameState) return false;
  saveToolFieldsFromUi();
  saveUserFieldsFromUi();
  if ($("chkShowToolFrames")) frameState.showToolFrame = !!$("chkShowToolFrames").checked;
  if ($("chkShowUserFrames")) frameState.showUserFrames = !!$("chkShowUserFrames").checked;
  framesIgnoreSseReload += 1;
  try {
    const r = await (
      await fetch("/api/robot/frames", {
        method: "PUT",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ sceneRootBackendId: rootId, frames: frameState }),
      })
    ).json();
    if (!r.ok) {
      setStatus(r.error || "坐标系保存失败", "err");
      return false;
    }
    await refreshFrameOverlays();
    return true;
  } finally {
    setTimeout(() => {
      framesIgnoreSseReload = Math.max(0, framesIgnoreSseReload - 1);
    }, 300);
  }
}

function scheduleFramesPut() {
  if (frameUiBlock) return;
  clearTimeout(framesPutTimer);
  framesPutTimer = setTimeout(() => void putFramesNow(), 180);
}

function commitFramesNow() {
  clearTimeout(framesPutTimer);
  return putFramesNow();
}

async function mutateFrames(action, id) {
  const rootId = activeSceneRootId();
  if (!rootId) {
    setStatus("无机器人实例", "err");
    return false;
  }
  await putFramesNow();
  framesIgnoreSseReload += 1;
  try {
    const body = { sceneRootBackendId: rootId, action };
    if (id) body.id = id;
    const r = await (
      await fetch("/api/robot/frames/mutate", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      })
    ).json();
    if (!r.ok) {
      setStatus(r.error || "坐标系操作失败", "err");
      return false;
    }
    frameState = ensureFramesDefaults(r.frames || {});
    if (Array.isArray(r.linkNames)) frameLinkNames = r.linkNames;
    if (r.kind === "tool" && r.selectedId) selectedToolId = r.selectedId;
    if (r.kind === "user" && r.selectedId) selectedUserId = r.selectedId;
    renderFrameLists();
    loadToolFields();
    loadUserFields();
    await refreshFrameOverlays();
    return true;
  } finally {
    setTimeout(() => {
      framesIgnoreSseReload = Math.max(0, framesIgnoreSseReload - 1);
    }, 300);
  }
}

function clearCoordinateFramesUi() {
  frameState = null;
  frameLinkNames = [];
  selectedToolId = "";
  selectedUserId = "";
  const toolUl = $("toolFrameList");
  const userUl = $("userFrameList");
  if (toolUl) toolUl.innerHTML = "";
  if (userUl) userUl.innerHTML = "";
  const flangeSel = $("toolFlangeLink");
  if (flangeSel) flangeSel.innerHTML = "";
  const instSel = $("frameInstance");
  if (instSel) instSel.innerHTML = "";
  frameUiBlock = true;
  writeSpinRigid("tool", emptyRigid());
  writeSpinRigid("user", emptyRigid());
  if ($("chkShowToolFrames")) $("chkShowToolFrames").checked = true;
  if ($("chkShowUserFrames")) $("chkShowUserFrames").checked = true;
  frameUiBlock = false;
  clearFrameOverlayGroup();
  const hint = $("frameEmptyHint");
  const body = $("frameBody");
  if (hint) hint.classList.remove("hidden");
  if (body) body.classList.add("hidden");
}

/// 清掉不在 robotSceneRootIds 中的残留选择；集合未同步前禁止整表清空（运行首帧会 refresh overlays）
function scrubStaleRobotRootRefs() {
  if (!robotSceneRootIds.size) return;
  const rootEl = $("robotRoot");
  const stale = rootEl?.value.trim() || "";
  if (stale && !robotSceneRootIds.has(stale)) {
    rootEl.value = "";
  }
  const axis = $("axisInstance");
  if (axis && axis.value && !robotSceneRootIds.has(axis.value)) axis.value = "";
  const frame = $("frameInstance");
  if (frame && frame.value && !robotSceneRootIds.has(frame.value)) frame.value = "";
}

function clearActiveRobotSelection() {
  if ($("robotRoot")) $("robotRoot").value = "";
  selectedRobot = null;
  const axis = $("axisInstance");
  if (axis) axis.innerHTML = "";
  const frame = $("frameInstance");
  if (frame) frame.innerHTML = "";
}

async function loadCoordinateFrames() {
  scrubStaleRobotRootRefs();
  const instSel = $("frameInstance");
  const hint = $("frameEmptyHint");
  const body = $("frameBody");
  const prev =
    (instSel && instSel.value) ||
    ($("axisInstance") && $("axisInstance").value) ||
    ($("robotRoot") && $("robotRoot").value.trim()) ||
    "";
  const inst = await (await fetch("/api/robot/instances")).json();
  const objectIds = new Set(objects.map((o) => o.id));
  const instances = (inst.instances || []).filter(
    (it) => it.sceneRootBackendId && (!objectIds.size || objectIds.has(it.sceneRootBackendId))
  );
  // 与本地根集合对齐，删机后若服务端短暂残留也以空场景为准
  if (!robotSceneRootIds.size && !instances.length) {
    if (instSel) instSel.innerHTML = "";
    clearCoordinateFramesUi();
    clearActiveRobotSelection();
    return;
  }
  if (instSel) {
    instSel.innerHTML = "";
    for (const it of instances) {
      if (robotSceneRootIds.size && !robotSceneRootIds.has(it.sceneRootBackendId)) continue;
      const opt = document.createElement("option");
      opt.value = it.sceneRootBackendId;
      opt.textContent = `${it.label || it.sceneRootBackendId}`;
      instSel.appendChild(opt);
    }
    if (prev && [...instSel.options].some((o) => o.value === prev)) instSel.value = prev;
    else if (instSel.options.length) instSel.selectedIndex = 0;
  }
  const rootId = (instSel && instSel.value) || "";
  if ($("robotRoot") && rootId) $("robotRoot").value = rootId;
  if ($("axisInstance") && rootId && [...($("axisInstance").options || [])].some((o) => o.value === rootId)) {
    $("axisInstance").value = rootId;
  }
  if (!rootId) {
    clearCoordinateFramesUi();
    return;
  }
  if (hint) hint.classList.add("hidden");
  if (body) body.classList.remove("hidden");
  const r = await (await fetch(`/api/robot/frames?sceneRootBackendId=${encodeURIComponent(rootId)}`)).json();
  if (!r.ok) {
    clearCoordinateFramesUi();
    scrubStaleRobotRootRefs();
    setStatus(r.error || "加载坐标系失败", "err");
    return;
  }
  frameState = ensureFramesDefaults(r.frames || {});
  frameLinkNames = Array.isArray(r.linkNames) ? r.linkNames : [];
  if (!frameState.flangeLinkName && frameLinkNames.length) {
    frameState.flangeLinkName = frameLinkNames[frameLinkNames.length - 1];
  }
  if (!selectedToolId || !frameState.toolFrames.some((t) => t.id === selectedToolId)) {
    selectedToolId = frameState.activeToolFrameId || frameState.toolFrames[0]?.id || "";
  }
  if (!selectedUserId || !frameState.userFrames.some((u) => u.id === selectedUserId)) {
    selectedUserId = frameState.activeUserFrameId || frameState.userFrames[0]?.id || "";
  }
  renderFrameLists();
  loadToolFields();
  loadUserFields();
  await refreshFrameOverlays();
}

function wireFramePanel() {
  $("btnFrameReload") && ($("btnFrameReload").onclick = () => void loadCoordinateFrames());
  $("frameInstance") &&
    ($("frameInstance").onchange = () => {
      const v = $("frameInstance").value;
      if ($("robotRoot")) $("robotRoot").value = v;
      if ($("axisInstance") && [...$("axisInstance").options].some((o) => o.value === v)) {
        $("axisInstance").value = v;
      }
      void loadCoordinateFrames();
    });
  const onToolSpin = () => {
    if (frameUiBlock) return;
    saveToolFieldsFromUi();
    scheduleFramesPut();
  };
  const onUserSpin = () => {
    if (frameUiBlock) return;
    saveUserFieldsFromUi();
    scheduleFramesPut();
  };
  document.querySelectorAll("[data-tool]").forEach((el) => {
    el.addEventListener("change", onToolSpin);
    el.addEventListener("input", onToolSpin);
  });
  document.querySelectorAll("[data-user]").forEach((el) => {
    el.addEventListener("change", onUserSpin);
    el.addEventListener("input", onUserSpin);
  });
  $("toolFlangeLink") &&
    ($("toolFlangeLink").onchange = () => {
      if (frameUiBlock) return;
      saveToolFieldsFromUi();
      scheduleFramesPut();
    });
  $("chkShowToolFrames") &&
    ($("chkShowToolFrames").onchange = () => {
      if (frameUiBlock || !frameState) return;
      frameState.showToolFrame = !!$("chkShowToolFrames").checked;
      void commitFramesNow();
    });
  $("chkShowUserFrames") &&
    ($("chkShowUserFrames").onchange = () => {
      if (frameUiBlock || !frameState) return;
      frameState.showUserFrames = !!$("chkShowUserFrames").checked;
      void commitFramesNow();
    });

  $("btnToolAdd") && ($("btnToolAdd").onclick = () => void mutateFrames("addTool"));
  $("btnToolDup") &&
    ($("btnToolDup").onclick = () => {
      if (!selectedToolId) return;
      void mutateFrames("duplicateTool", selectedToolId);
    });
  $("btnToolDel") &&
    ($("btnToolDel").onclick = () => {
      if (!frameState || frameState.toolFrames.length <= 1) return;
      frameState.toolFrames = frameState.toolFrames.filter((t) => t.id !== selectedToolId);
      if (frameState.activeToolFrameId === selectedToolId) {
        frameState.activeToolFrameId = frameState.toolFrames[0].id;
      }
      selectedToolId = frameState.activeToolFrameId;
      renderFrameLists();
      loadToolFields();
      void commitFramesNow();
    });
  $("btnToolActive") &&
    ($("btnToolActive").onclick = () => {
      if (!frameState || !selectedToolId) return;
      frameState.activeToolFrameId = selectedToolId;
      renderFrameLists();
      void commitFramesNow();
    });
  $("btnToolCapture") &&
    ($("btnToolCapture").onclick = async () => {
      const rootId = activeSceneRootId();
      if (!rootId) return;
      await putFramesNow();
      const r = await (
        await fetch("/api/robot/frames/capture-tool", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ sceneRootBackendId: rootId }),
        })
      ).json();
      if (!r.ok) {
        setStatus(r.error || "捕获工具系失败", "err");
        return;
      }
      await loadCoordinateFrames();
    });
  $("btnToolReset") &&
    ($("btnToolReset").onclick = async () => {
      const rootId = activeSceneRootId();
      if (!rootId) return;
      await putFramesNow();
      const r = await (
        await fetch("/api/robot/frames/reset-tool", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ sceneRootBackendId: rootId }),
        })
      ).json();
      if (!r.ok) {
        setStatus(r.error || "重置失败", "err");
        return;
      }
      await loadCoordinateFrames();
    });

  $("btnUserAdd") && ($("btnUserAdd").onclick = () => void mutateFrames("addUser"));
  $("btnUserDup") &&
    ($("btnUserDup").onclick = () => {
      if (!selectedUserId) return;
      void mutateFrames("duplicateUser", selectedUserId);
    });
  $("btnUserDel") &&
    ($("btnUserDel").onclick = () => {
      if (!frameState || frameState.userFrames.length <= 1) return;
      frameState.userFrames = frameState.userFrames.filter((u) => u.id !== selectedUserId);
      if (frameState.activeUserFrameId === selectedUserId) {
        frameState.activeUserFrameId = frameState.userFrames[0].id;
      }
      selectedUserId = frameState.activeUserFrameId;
      renderFrameLists();
      loadUserFields();
      void commitFramesNow();
    });
  $("btnUserActive") &&
    ($("btnUserActive").onclick = () => {
      if (!frameState || !selectedUserId) return;
      frameState.activeUserFrameId = selectedUserId;
      renderFrameLists();
      void commitFramesNow();
    });
  $("btnUserCapture") &&
    ($("btnUserCapture").onclick = async () => {
      const rootId = activeSceneRootId();
      if (!rootId) return;
      await putFramesNow();
      const r = await (
        await fetch("/api/robot/frames/capture-user", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ sceneRootBackendId: rootId }),
        })
      ).json();
      if (!r.ok) {
        setStatus(r.error || "捕获用户系失败", "err");
        return;
      }
      await loadCoordinateFrames();
    });
}
wireFramePanel();

(function tick() {
  controls.update();
  if (sceneRebuildDepth === 0) pruneOrphanMeshes();
  renderer.render(scene, camera);
  requestAnimationFrame(tick);
})();

refreshHealth();
setInterval(refreshHealth, 5000);
void loadModes();
const es = new EventSource("/api/events");
es.onmessage = async (ev) => {
  try {
    const msg = JSON.parse(ev.data);
    sseHint.textContent = msg.type || "event";
    if (msg.type === "RobotKinematicsApplied" || msg.type === "PoseCommitted" || msg.type === "ObjectPatched") {
      if (performance.now() < axisSyncQuietUntil) return;
      await syncObjectTransforms();
      // 拖拽中叠加由罗盘本地跟随，避免 SSE 重刷把 TCP 拉回旧位
      if (!(robotDragMode && transform.dragging)) void refreshFrameOverlays();
      return;
    }
    if (msg.type === "RobotCoordinateFramesChanged") {
      if (framesIgnoreSseReload > 0) {
        void refreshFrameOverlays();
        return;
      }
      if (!$("robotFrame")?.classList.contains("hidden")) void loadCoordinateFrames();
      else void refreshFrameOverlays();
      return;
    }
    if (msg.type === "SelectionChanged") {
      if (selectedId) await loadDetail();
      return;
    }
    if (
      ["ProjectLoaded", "ProjectSaved", "BackendObjectRegistered", "BackendObjectRemoved", "WorkspaceModeChanged"].includes(
        msg.type
      )
    ) {
      await refreshObjects(msg.type === "ProjectLoaded");
      if (selectedId) await loadDetail();
      if (
        msg.type === "ProjectLoaded" ||
        msg.type === "BackendObjectRegistered" ||
        msg.type === "BackendObjectRemoved"
      ) {
        scrubStaleRobotRootRefs();
        void loadAxisControl();
        void loadProgramsFromServer();
        void loadCoordinateFrames();
      }
    }
  } catch {
    /* ignore */
  }
};
