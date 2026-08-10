import * as THREE from "three";

/** 场景内容经 Rx(-90) 后：世界 (x,y,z) → Three (x,z,-y) */
export function worldDirToThree(wx: number, wy: number, wz: number): THREE.Vector3 {
  return new THREE.Vector3(wx, wz, -wy);
}

export function threeDirToWorld(v: THREE.Vector3): THREE.Vector3 {
  // 逆：Three (x,y,z) → 世界 (x,-z,y)
  return new THREE.Vector3(v.x, -v.z, v.y);
}

type FaceDef = {
  label: string;
  /** 相机相对目标的世界系视线方向（从目标指向相机） */
  eyeWorld: [number, number, number];
  /** 立方体面中心（Three 系，半边长约 0.58） */
  center: THREE.Vector3;
  normal: THREE.Vector3;
};

/** 对齐桌面：顶/底/前/后/右/左 ↔ ±Z/±Y/±X（世界 Z-up） */
const FACES: FaceDef[] = [
  {
    label: "顶",
    eyeWorld: [0, 0, 1],
    center: new THREE.Vector3(0, 0.58, 0),
    normal: new THREE.Vector3(0, 1, 0),
  },
  {
    label: "底",
    eyeWorld: [0, 0, -1],
    center: new THREE.Vector3(0, -0.58, 0),
    normal: new THREE.Vector3(0, -1, 0),
  },
  {
    label: "前",
    eyeWorld: [0, 1, 0],
    center: new THREE.Vector3(0, 0, -0.58),
    normal: new THREE.Vector3(0, 0, -1),
  },
  {
    label: "后",
    eyeWorld: [0, -1, 0],
    center: new THREE.Vector3(0, 0, 0.58),
    normal: new THREE.Vector3(0, 0, 1),
  },
  {
    label: "右",
    eyeWorld: [1, 0, 0],
    center: new THREE.Vector3(0.58, 0, 0),
    normal: new THREE.Vector3(1, 0, 0),
  },
  {
    label: "左",
    eyeWorld: [-1, 0, 0],
    center: new THREE.Vector3(-0.58, 0, 0),
    normal: new THREE.Vector3(-1, 0, 0),
  },
];

function makeFaceTexture(label: string): THREE.CanvasTexture {
  const c = document.createElement("canvas");
  c.width = 128;
  c.height = 128;
  const ctx = c.getContext("2d")!;
  ctx.fillStyle = "rgba(236, 240, 245, 0.92)";
  ctx.fillRect(0, 0, 128, 128);
  ctx.strokeStyle = "rgba(0,0,0,0.22)";
  ctx.lineWidth = 4;
  ctx.strokeRect(3, 3, 122, 122);
  ctx.fillStyle = "#222";
  ctx.font = "bold 52px 'Segoe UI', 'Microsoft YaHei', sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(label, 64, 66);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  tex.needsUpdate = true;
  return tex;
}

function makeAxisLetterTexture(letter: string, color: string): THREE.CanvasTexture {
  const c = document.createElement("canvas");
  c.width = 64;
  c.height = 64;
  const ctx = c.getContext("2d")!;
  ctx.clearRect(0, 0, 64, 64);
  ctx.fillStyle = color;
  ctx.font = "bold 42px 'Segoe UI', sans-serif";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(letter, 32, 34);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  return tex;
}

function pickUpHintWorld(eye: THREE.Vector3): THREE.Vector3 {
  if (Math.abs(eye.z) > 0.92) return new THREE.Vector3(0, 1, 0);
  return new THREE.Vector3(0, 0, 1);
}

export type ViewCubeHud = {
  canvas: HTMLCanvasElement;
  sync: (mainCamera: THREE.Camera) => void;
  dispose: () => void;
  /** 点击立方体面时回调世界系 eyeDir / upHint */
  onFacePick?: (eyeWorld: THREE.Vector3, upWorld: THREE.Vector3) => void;
};

export function createViewCubeHud(onFacePick: (eyeWorld: THREE.Vector3, upWorld: THREE.Vector3) => void): ViewCubeHud {
  const canvas = document.createElement("canvas");
  canvas.className = "view-cube-canvas";
  canvas.width = 112;
  canvas.height = 112;
  canvas.title = "点击面切换视图";

  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(112, 112, false);
  renderer.setClearColor(0x000000, 0);

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(35, 1, 0.1, 20);
  camera.position.set(0, 0, 3.2);

  const root = new THREE.Group();
  scene.add(root);
  scene.add(new THREE.AmbientLight(0xffffff, 0.95));

  const faceMeshes: THREE.Mesh[] = [];
  const h = 0.58;
  for (const face of FACES) {
    const geo = new THREE.PlaneGeometry(h * 2 * 0.98, h * 2 * 0.98);
    const mat = new THREE.MeshBasicMaterial({
      map: makeFaceTexture(face.label),
      transparent: true,
      side: THREE.DoubleSide,
      depthWrite: true,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.position.copy(face.center);
    mesh.lookAt(face.center.clone().add(face.normal));
    mesh.userData.eyeWorld = face.eyeWorld;
    mesh.userData.label = face.label;
    root.add(mesh);
    faceMeshes.push(mesh);

    // 细边框盒子增强立体感（只加一次）
  }
  const box = new THREE.LineSegments(
    new THREE.EdgesGeometry(new THREE.BoxGeometry(h * 2, h * 2, h * 2)),
    new THREE.LineBasicMaterial({ color: 0x000000, transparent: true, opacity: 0.2 }),
  );
  root.add(box);

  const raycaster = new THREE.Raycaster();
  const pointer = new THREE.Vector2();
  let hover: THREE.Mesh | null = null;

  const setHover = (m: THREE.Mesh | null) => {
    if (hover === m) return;
    if (hover) {
      const mat = hover.material as THREE.MeshBasicMaterial;
      mat.color.setHex(0xffffff);
    }
    hover = m;
    if (hover) {
      const mat = hover.material as THREE.MeshBasicMaterial;
      mat.color.setHex(0xc8e0f8);
    }
  };

  const pickAt = (clientX: number, clientY: number) => {
    const rect = canvas.getBoundingClientRect();
    pointer.x = ((clientX - rect.left) / rect.width) * 2 - 1;
    pointer.y = -((clientY - rect.top) / rect.height) * 2 + 1;
    raycaster.setFromCamera(pointer, camera);
    return raycaster.intersectObjects(faceMeshes, false);
  };

  const onMove = (ev: PointerEvent) => {
    const hits = pickAt(ev.clientX, ev.clientY);
    setHover(hits.length ? (hits[0].object as THREE.Mesh) : null);
    canvas.style.cursor = hits.length ? "pointer" : "default";
  };

  const onLeave = () => {
    setHover(null);
    canvas.style.cursor = "default";
  };

  const onClick = (ev: MouseEvent) => {
    const hits = pickAt(ev.clientX, ev.clientY);
    if (!hits.length) return;
    const eyeArr = hits[0].object.userData.eyeWorld as [number, number, number];
    const eye = new THREE.Vector3(...eyeArr);
    onFacePick(eye, pickUpHintWorld(eye));
  };

  canvas.addEventListener("pointermove", onMove);
  canvas.addEventListener("pointerleave", onLeave);
  canvas.addEventListener("click", onClick);

  return {
    canvas,
    sync(mainCamera: THREE.Camera) {
      // 与主相机同旋转，立方体朝向反映当前视图
      const q = mainCamera.quaternion;
      const dist = 3.2;
      camera.position.set(0, 0, dist).applyQuaternion(q);
      camera.quaternion.copy(q);
      camera.up.set(0, 1, 0).applyQuaternion(q);
      renderer.render(scene, camera);
    },
    dispose() {
      canvas.removeEventListener("pointermove", onMove);
      canvas.removeEventListener("pointerleave", onLeave);
      canvas.removeEventListener("click", onClick);
      for (const m of faceMeshes) {
        m.geometry.dispose();
        const mat = m.material as THREE.MeshBasicMaterial;
        mat.map?.dispose();
        mat.dispose();
      }
      box.geometry.dispose();
      (box.material as THREE.Material).dispose();
      renderer.dispose();
    },
  };
}

export type WorldAxesHud = {
  canvas: HTMLCanvasElement;
  sync: (mainCamera: THREE.Camera) => void;
  dispose: () => void;
};

/** 左下角世界系三轴朝向（红X / 绿Y / 蓝Z，Z-up） */
export function createWorldAxesHud(): WorldAxesHud {
  const canvas = document.createElement("canvas");
  canvas.className = "axis-hud-canvas";
  canvas.width = 96;
  canvas.height = 96;
  canvas.title = "世界坐标系朝向";

  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.setSize(96, 96, false);
  renderer.setClearColor(0x000000, 0);

  const scene = new THREE.Scene();
  const camera = new THREE.OrthographicCamera(-1.35, 1.35, 1.35, -1.35, 0.1, 10);
  camera.position.set(0, 0, 4);
  camera.lookAt(0, 0, 0);

  const root = new THREE.Group();
  scene.add(root);

  // Three 系中的世界轴：X=(1,0,0), Y=(0,0,-1), Z=(0,1,0)
  const axes: { dir: THREE.Vector3; color: number; letter: string; letterColor: string }[] = [
    { dir: new THREE.Vector3(1, 0, 0), color: 0xe53935, letter: "X", letterColor: "#e53935" },
    { dir: new THREE.Vector3(0, 0, -1), color: 0x43a047, letter: "Y", letterColor: "#43a047" },
    { dir: new THREE.Vector3(0, 1, 0), color: 0x1e88e5, letter: "Z", letterColor: "#1e88e5" },
  ];

  const disposables: { dispose: () => void }[] = [];
  for (const a of axes) {
    const len = 0.95;
    const pts = [new THREE.Vector3(0, 0, 0), a.dir.clone().multiplyScalar(len)];
    const geo = new THREE.BufferGeometry().setFromPoints(pts);
    const mat = new THREE.LineBasicMaterial({ color: a.color, linewidth: 2 });
    root.add(new THREE.Line(geo, mat));
    disposables.push(geo, mat);

    const tip = new THREE.Mesh(
      new THREE.ConeGeometry(0.07, 0.18, 10),
      new THREE.MeshBasicMaterial({ color: a.color }),
    );
    tip.position.copy(a.dir.clone().multiplyScalar(len));
    tip.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), a.dir.clone().normalize());
    root.add(tip);
    disposables.push(tip.geometry, tip.material as THREE.Material);

    const sprMat = new THREE.SpriteMaterial({
      map: makeAxisLetterTexture(a.letter, a.letterColor),
      depthTest: false,
      transparent: true,
    });
    const spr = new THREE.Sprite(sprMat);
    spr.scale.set(0.32, 0.32, 1);
    spr.position.copy(a.dir.clone().multiplyScalar(len + 0.22));
    root.add(spr);
    disposables.push(sprMat.map!, sprMat);
  }

  return {
    canvas,
    sync(mainCamera: THREE.Camera) {
      // 轴随主相机旋转的逆变换，指示世界朝向
      root.quaternion.copy(mainCamera.quaternion).invert();
      renderer.render(scene, camera);
    },
    dispose() {
      for (const d of disposables) d.dispose();
      renderer.dispose();
    },
  };
}

/** 按世界系视线方向设置轨道相机（保持 target 与距离） */
export function applyWorldViewDirection(
  camera: THREE.PerspectiveCamera,
  controls: { target: THREE.Vector3; update: () => void },
  eyeWorld: THREE.Vector3,
  upWorld: THREE.Vector3,
) {
  const eyeDir = eyeWorld.clone().normalize();
  let up = upWorld.clone().normalize();
  if (up.lengthSq() < 1e-12) up = pickUpHintWorld(eyeDir);

  const threeEye = worldDirToThree(eyeDir.x, eyeDir.y, eyeDir.z).normalize();
  const threeUp = worldDirToThree(up.x, up.y, up.z).normalize();

  const target = controls.target.clone();
  const dist = Math.max(camera.position.distanceTo(target), 100);
  // 视线与 up 近平行时换兜底 up，避免万向锁
  if (Math.abs(threeEye.dot(threeUp)) > 0.95) {
    const fallback = worldDirToThree(0, 1, 0).normalize();
    threeUp.copy(Math.abs(threeEye.dot(fallback)) > 0.95 ? new THREE.Vector3(1, 0, 0) : fallback);
  }

  camera.up.copy(threeUp);
  camera.position.copy(target).add(threeEye.multiplyScalar(dist));
  camera.lookAt(target);
  controls.update();
}
