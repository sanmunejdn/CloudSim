import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { TransformControls } from "three/addons/controls/TransformControls.js";

const $ = (id) => document.getElementById(id);
const healthEl = $("health");
const statusEl = $("status");
const sseHint = $("sseHint");
const treeEl = $("tree");
const pathEl = $("path");
const propsEl = $("props");
const sceneMount = $("scene");
const modeSelect = $("modeSelect");

const zUpToYUp = new THREE.Matrix4().makeRotationX(-Math.PI / 2);
const idToMesh = new Map();
let selectedId = null;
let objects = [];
let detail = null;
let suppressPosePush = false;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x1a1d23);
const camera = new THREE.PerspectiveCamera(50, 1, 1, 1e7);
camera.position.set(800, 600, 1000);
const renderer = new THREE.WebGLRenderer({ antialias: true });
sceneMount.appendChild(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
const transform = new TransformControls(camera, renderer.domElement);
transform.setSize(0.9);
transform.addEventListener("dragging-changed", (e) => {
  controls.enabled = !e.value;
});
transform.addEventListener("objectChange", () => {
  if (suppressPosePush || !selectedId || !transform.object) return;
  const obj = transform.object;
  const e = new THREE.Euler().setFromQuaternion(obj.quaternion, "ZYX");
  const body = {
    pose: {
      positionMm: [obj.position.x, obj.position.y, obj.position.z],
      eulerDeg: [THREE.MathUtils.radToDeg(e.x), THREE.MathUtils.radToDeg(e.y), THREE.MathUtils.radToDeg(e.z)],
    },
  };
  fetch(`/api/objects/${encodeURIComponent(selectedId)}`, {
    method: "PATCH",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
});
scene.add(transform);
scene.add(new THREE.AmbientLight(0xffffff, 0.45));
const dir = new THREE.DirectionalLight(0xffffff, 1.05);
dir.position.set(1, 2, 3);
scene.add(dir);
scene.add(new THREE.GridHelper(2000, 20, 0x445, 0x333));
const root = new THREE.Group();
root.applyMatrix4(zUpToYUp);
scene.add(root);

const trajGeo = new THREE.BufferGeometry();
const trajLine = new THREE.Line(trajGeo, new THREE.LineBasicMaterial({ color: 0xffcc44 }));
root.add(trajLine);

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

function eulerZyxDegToQuat(ex, ey, ez) {
  const qx = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), THREE.MathUtils.degToRad(ex));
  const qy = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), THREE.MathUtils.degToRad(ey));
  const qz = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), THREE.MathUtils.degToRad(ez));
  return qz.multiply(qy).multiply(qx);
}

function colorFromObject(o, selected) {
  const c = o?.color;
  if (c && Number.isFinite(c.r) && Number.isFinite(c.g) && Number.isFinite(c.b)) {
    const col = new THREE.Color(c.r, c.g, c.b);
    if (selected) col.offsetHSL(0, 0.05, 0.12);
    return col;
  }
  return new THREE.Color(selected ? 0x4ea1ff : 0xb0b8c4);
}

function applyObjectTransform(mesh, o) {
  const wm = o.worldMatrix;
  // Gateway 已转为 Three/OpenGL 列主序（平移在 12..14）
  if (Array.isArray(wm) && wm.length === 16) {
    mesh.matrixAutoUpdate = false;
    mesh.matrix.fromArray(wm.map(Number));
    mesh.matrixWorldNeedsUpdate = true;
    return;
  }
  mesh.matrixAutoUpdate = true;
  const p = o.pose?.positionMm || [0, 0, 0];
  const e = o.pose?.eulerDeg || [0, 0, 0];
  mesh.position.set(p[0], p[1], p[2]);
  mesh.quaternion.copy(eulerZyxDegToQuat(e[0], e[1], e[2]));
  mesh.scale.set(1, 1, 1);
}

async function pickPath(purpose, extra = {}) {
  statusEl.textContent = "请在弹出的系统对话框中选择…（若被挡住请看任务栏 CloudSimWeb）";
  let r;
  try {
    const resp = await fetch("/api/dialog/open", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ purpose, startDir: pathEl.value.trim() || undefined, ...extra }),
    });
    r = await resp.json();
  } catch (e) {
    statusEl.textContent = `对话框请求失败: ${e.message || e}`;
    return null;
  }
  if (r.cancelled) {
    statusEl.textContent = "已取消选择";
    return null;
  }
  if (!r.ok) {
    statusEl.textContent = r.error || "对话框失败";
    return null;
  }
  return r.path || null;
}

async function openProjectAt(path) {
  if (!path) return;
  pathEl.value = path;
  statusEl.textContent = `正在打开 ${path} …`;
  let r;
  try {
    const resp = await fetch("/api/project/open", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ path }),
    });
    r = await resp.json();
  } catch (e) {
    statusEl.textContent = `打开请求失败: ${e.message || e}`;
    return;
  }
  if (!r.ok) {
    statusEl.textContent = `打开失败: ${r.error || "未知错误"}`;
    return;
  }
  statusEl.textContent = `已加载 ${r.objectCount} 对象`;
  await refreshObjects(true);
  await loadModes();
}

function prepareMeshForGizmo(mesh) {
  if (!mesh.matrixAutoUpdate) {
    mesh.matrix.decompose(mesh.position, mesh.quaternion, mesh.scale);
    mesh.matrixAutoUpdate = true;
  }
}

function bindDockTabs() {
  document.querySelectorAll("[data-left-tab]").forEach((btn) => {
    btn.onclick = () => {
      document.querySelectorAll("[data-left-tab]").forEach((b) => b.classList.toggle("active", b === btn));
      const tab = btn.getAttribute("data-left-tab");
      $("leftProps").classList.toggle("hidden", tab !== "props");
      $("leftAi").classList.toggle("hidden", tab !== "ai");
    };
  });
  document.querySelectorAll("[data-right-tab]").forEach((btn) => {
    btn.onclick = () => {
      document.querySelectorAll("[data-right-tab]").forEach((b) => b.classList.toggle("active", b === btn));
      const tab = btn.getAttribute("data-right-tab");
      $("rightTree").classList.toggle("hidden", tab !== "tree");
      $("rightSim").classList.toggle("hidden", tab !== "sim");
    };
  });
}

async function refreshHealth() {
  try {
    const h = await (await fetch("/api/health")).json();
    if (h.role !== "web") throw new Error("role");
    healthEl.textContent = `role=${h.role} pid=${h.pid} :${h.port}`;
    healthEl.className = "pill ok";
  } catch {
    healthEl.textContent = "offline";
    healthEl.className = "pill bad";
  }
}

function renderTree() {
  treeEl.innerHTML = "";
  for (const o of objects) {
    const li = document.createElement("li");
    if (o.id === selectedId) li.className = "sel";
    li.innerHTML = `<span>${o.name || o.id}</span><span class="cls">${o.className}</span>`;
    li.onclick = () => selectObject(o.id);
    treeEl.appendChild(li);
  }
}

async function selectObject(id) {
  selectedId = id;
  await fetch("/api/selection", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ backendId: id }),
  });
  renderTree();
  await loadDetail();
  const mesh = idToMesh.get(id);
  if (mesh) {
    prepareMeshForGizmo(mesh);
    transform.attach(mesh);
  } else transform.detach();
  for (const [mid, m] of idToMesh) {
    const o = objects.find((x) => x.id === mid);
    m.material.color.copy(colorFromObject(o, mid === id));
    m.material.emissive?.setHex(mid === id ? 0x1a3a66 : 0x000000);
  }
}

async function loadDetail() {
  if (!selectedId) {
    propsEl.textContent = "未选中";
    return;
  }
  detail = await (await fetch(`/api/objects/${encodeURIComponent(selectedId)}`)).json();
  const lines = [
    `id: ${detail.id}`,
    `name: ${detail.name}`,
    `class: ${detail.className}`,
    `visible: ${detail.visible}`,
  ];
  if (detail.pose) {
    lines.push(`pos: ${detail.pose.positionMm?.join(", ")}`);
    lines.push(`euler: ${detail.pose.eulerDeg?.join(", ")}`);
  }
  for (const r of detail.properties || []) {
    lines.push(`${r.key}=${r.value}${r.editable ? "" : " (ro)"}`);
  }
  propsEl.textContent = lines.join("\n");
  propsEl.onclick = async (ev) => {
    if (ev.detail < 2 || !detail?.properties?.length) return;
    const key = prompt("属性 key", detail.properties[0].key);
    if (!key) return;
    const value = prompt("新值", "");
    if (value == null) return;
    await fetch(`/api/objects/${encodeURIComponent(selectedId)}`, {
      method: "PATCH",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ propertyKey: key, propertyValue: value }),
    });
    await refreshObjects(false);
    await loadDetail();
  };
}

async function rebuildScene() {
  suppressPosePush = true;
  transform.detach();
  for (const c of [...root.children]) root.remove(c);
  root.add(trajLine);
  idToMesh.clear();
  const box = new THREE.Box3();
  let meshOk = 0;
  let meshFail = 0;
  resize();
  for (const o of objects) {
    if (!o.visible || !o.hasGeometry || o.geometryKind === 1) continue;
    const r = await fetch(`/api/mesh/${encodeURIComponent(o.id)}`);
    if (!r.ok) {
      meshFail++;
      continue;
    }
    const soup = new Float32Array(await r.arrayBuffer());
    if (soup.length < 9) {
      meshFail++;
      continue;
    }
    const geo = new THREE.BufferGeometry();
    geo.setAttribute("position", new THREE.BufferAttribute(soup, 3));
    geo.computeVertexNormals();
    const mesh = new THREE.Mesh(
      geo,
      new THREE.MeshStandardMaterial({
        color: colorFromObject(o, o.id === selectedId),
        metalness: 0.15,
        roughness: 0.55,
        side: THREE.DoubleSide,
        emissive: o.id === selectedId ? 0x1a3a66 : 0x000000,
      })
    );
    mesh.userData.backendId = o.id;
    applyObjectTransform(mesh, o);
    root.add(mesh);
    idToMesh.set(o.id, mesh);
    mesh.updateMatrixWorld(true);
    box.expandByObject(mesh);
    meshOk++;
  }
  if (selectedId && idToMesh.has(selectedId)) {
    prepareMeshForGizmo(idToMesh.get(selectedId));
    transform.attach(idToMesh.get(selectedId));
  }
  suppressPosePush = false;
  statusEl.textContent = `对象 ${objects.length} · 网格 ${meshOk}${meshFail ? ` · 失败 ${meshFail}` : ""}`;
  return box;
}

function focusBox(box) {
  if (!box || box.isEmpty()) return;
  const size = box.getSize(new THREE.Vector3()).length();
  const center = box.getCenter(new THREE.Vector3());
  controls.target.copy(center);
  camera.position.copy(center.clone().add(new THREE.Vector3(size, size * 0.7, size)));
}

async function refreshObjects(focus) {
  const list = await (await fetch("/api/objects")).json();
  objects = list.objects || [];
  if (list.projectPath) pathEl.value = list.projectPath;
  renderTree();
  const box = await rebuildScene();
  if (focus) focusBox(box);
}

$("btnNew").onclick = async () => {
  statusEl.textContent = "新建…";
  const r = await (await fetch("/api/project/new", { method: "POST" })).json();
  statusEl.textContent = r.ok ? "已新建" : r.error || "失败";
  await refreshObjects(true);
};

$("btnOpen").onclick = async () => {
  // 底栏已有路径时直接打开；否则弹系统对话框
  const typed = pathEl.value.trim();
  if (typed) {
    await openProjectAt(typed);
    return;
  }
  const path = await pickPath("project");
  if (path) await openProjectAt(path);
};

$("btnOpenFolder").onclick = async () => {
  const path = await pickPath("directory");
  if (path) await openProjectAt(path);
};

$("btnSave").onclick = async () => {
  let path = pathEl.value.trim();
  if (!path) {
    path = await pickPath("saveProject");
    if (!path) return;
    pathEl.value = path;
  }
  statusEl.textContent = "保存…";
  const r = await (
    await fetch("/api/project/save", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ path }),
    })
  ).json();
  statusEl.textContent = r.ok ? `已保存 ${r.path || ""}` : r.error || "失败";
};

$("btnImport").onclick = async () => {
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
  statusEl.textContent = r.ok ? `导入 ${r.id}` : r.error || "失败";
  await refreshObjects(false);
};

$("btnFocus").onclick = async () => {
  const box = new THREE.Box3();
  for (const m of idToMesh.values()) box.expandByObject(m);
  focusBox(box);
};

$("btnUrdf").onclick = async () => {
  let urdfPath = $("urdfPath").value.trim();
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
  statusEl.textContent = r.ok ? `URDF ${r.sceneRootBackendId}` : r.error || "URDF 失败";
  if (r.sceneRootBackendId) $("robotRoot").value = r.sceneRootBackendId;
  await refreshObjects(false);
};

$("btnJoints").onclick = async () => {
  const jointAnglesRad = $("jointsCsv")
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
  statusEl.textContent = r.ok ? "关节已应用" : r.error || "失败";
  await refreshObjects(false);
};

$("btnPlan").onclick = async () => {
  const r = await (
    await fetch("/api/robot/plan", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        sceneRootBackendId: $("robotRoot").value.trim(),
        instructionType: "PTP",
        jointRadCsv: $("jointsCsv").value.trim(),
      }),
    })
  ).json();
  statusEl.textContent = r.ok ? `规划 ok joints=${(r.jointTargetsRad || []).length}` : r.error || "规划失败";
  if (r.jointTargetsRad?.length) {
    const pts = r.jointTargetsRad.map((v, i) => new THREE.Vector3(i * 50, v * 100, 0));
    trajGeo.setFromPoints(pts);
  }
};

$("btnRun").onclick = () => fetch("/api/robot/run", { method: "POST" });
$("btnStop").onclick = () => fetch("/api/robot/stop", { method: "POST" });

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
  statusEl.textContent = r.ok ? `作业 ${op} 已排队` : "失败";
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
  statusEl.textContent = `模式 ${modeSelect.value}`;
};

renderer.domElement.addEventListener("click", (ev) => {
  if (transform.dragging) return;
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
  if (o) void selectObject(o.userData.backendId);
});

bindDockTabs();

(function tick() {
  controls.update();
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
    if (
      [
        "ProjectLoaded",
        "ProjectSaved",
        "SelectionChanged",
        "PoseCommitted",
        "ObjectPatched",
        "BackendObjectRegistered",
        "BackendObjectRemoved",
        "RobotKinematicsApplied",
        "WorkspaceModeChanged",
      ].includes(msg.type)
    ) {
      await refreshObjects(msg.type === "ProjectLoaded");
      if (selectedId) await loadDetail();
    }
  } catch {
    /* ignore */
  }
};
