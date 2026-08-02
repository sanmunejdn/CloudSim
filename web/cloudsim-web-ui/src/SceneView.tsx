import { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import type { BackendObject } from "./api";
import { fetchMeshSoup, postSelection } from "./api";

type Props = {
  objects: BackendObject[];
  selectedId: string | null;
  onSelect: (id: string) => void;
};

/** Data 为 Z-up mm；Three 默认 Y-up，统一绕 X -90° */
const zUpToYUp = new THREE.Matrix4().makeRotationX(-Math.PI / 2);

function eulerZyxDegToQuat(ex: number, ey: number, ez: number): THREE.Quaternion {
  const rx = THREE.MathUtils.degToRad(ex);
  const ry = THREE.MathUtils.degToRad(ey);
  const rz = THREE.MathUtils.degToRad(ez);
  const qx = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), rx);
  const qy = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 1, 0), ry);
  const qz = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), rz);
  return qz.multiply(qy).multiply(qx);
}

export default function SceneView({ objects, selectedId, onSelect }: Props) {
  const mountRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef<THREE.Scene>();
  const rendererRef = useRef<THREE.WebGLRenderer>();
  const cameraRef = useRef<THREE.PerspectiveCamera>();
  const controlsRef = useRef<OrbitControls>();
  const rootRef = useRef<THREE.Group>();
  const idToMesh = useRef(new Map<string, THREE.Object3D>());

  useEffect(() => {
    const mount = mountRef.current!;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a1d23);
    const camera = new THREE.PerspectiveCamera(50, 1, 1, 1e7);
    camera.position.set(800, 600, 1000);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    mount.appendChild(renderer.domElement);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0, 0);

    const light = new THREE.DirectionalLight(0xffffff, 1.1);
    light.position.set(1, 2, 3);
    scene.add(light);
    scene.add(new THREE.AmbientLight(0xffffff, 0.35));
    scene.add(new THREE.GridHelper(2000, 20, 0x445, 0x333));

    const root = new THREE.Group();
    root.applyMatrix4(zUpToYUp);
    scene.add(root);

    sceneRef.current = scene;
    rendererRef.current = renderer;
    cameraRef.current = camera;
    controlsRef.current = controls;
    rootRef.current = root;

    const resize = () => {
      const w = mount.clientWidth;
      const h = mount.clientHeight;
      camera.aspect = w / Math.max(h, 1);
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    };
    resize();
    window.addEventListener("resize", resize);

    const raycaster = new THREE.Raycaster();
    const pointer = new THREE.Vector2();
    const onClick = (ev: MouseEvent) => {
      const rect = renderer.domElement.getBoundingClientRect();
      pointer.x = ((ev.clientX - rect.left) / rect.width) * 2 - 1;
      pointer.y = -((ev.clientY - rect.top) / rect.height) * 2 + 1;
      raycaster.setFromCamera(pointer, camera);
      const hits = raycaster.intersectObjects([...idToMesh.current.values()], true);
      if (hits.length > 0) {
        let o: THREE.Object3D | null = hits[0].object;
        while (o && !o.userData.backendId) o = o.parent;
        if (o?.userData.backendId) {
          const id = String(o.userData.backendId);
          onSelect(id);
          void postSelection(id);
        }
      }
    };
    renderer.domElement.addEventListener("click", onClick);

    let raf = 0;
    const tick = () => {
      controls.update();
      renderer.render(scene, camera);
      raf = requestAnimationFrame(tick);
    };
    tick();

    return () => {
      cancelAnimationFrame(raf);
      window.removeEventListener("resize", resize);
      renderer.domElement.removeEventListener("click", onClick);
      controls.dispose();
      renderer.dispose();
      mount.removeChild(renderer.domElement);
    };
  }, [onSelect]);

  useEffect(() => {
    const root = rootRef.current;
    if (!root) return;
    let cancelled = false;

    (async () => {
      while (root.children.length) {
        const c = root.children[0];
        root.remove(c);
        // @ts-expect-error dispose if mesh
        c.traverse?.((x: THREE.Object3D) => {
          const m = x as THREE.Mesh;
          m.geometry?.dispose?.();
          if (Array.isArray(m.material)) m.material.forEach((mm) => mm.dispose());
          else m.material?.dispose?.();
        });
      }
      idToMesh.current.clear();

      const box = new THREE.Box3();
      for (const obj of objects) {
        if (!obj.hasGeometry || obj.geometryKind !== 2 || !obj.visible) continue;
        const soup = await fetchMeshSoup(obj.id);
        if (cancelled || !soup || soup.length < 9) continue;
        const geo = new THREE.BufferGeometry();
        geo.setAttribute("position", new THREE.BufferAttribute(soup, 3));
        geo.computeVertexNormals();
        const mat = new THREE.MeshStandardMaterial({
          color: obj.id === selectedId ? 0x4ea1ff : 0xb0b8c4,
          metalness: 0.15,
          roughness: 0.55,
        });
        const mesh = new THREE.Mesh(geo, mat);
        mesh.userData.backendId = obj.id;
        const [x, y, z] = obj.pose.positionMm;
        mesh.position.set(x, y, z);
        mesh.quaternion.copy(eulerZyxDegToQuat(obj.pose.eulerDeg[0], obj.pose.eulerDeg[1], obj.pose.eulerDeg[2]));
        root.add(mesh);
        idToMesh.current.set(obj.id, mesh);
        box.expandByObject(mesh);
      }
      if (!box.isEmpty() && cameraRef.current && controlsRef.current) {
        const size = box.getSize(new THREE.Vector3()).length();
        const center = box.getCenter(new THREE.Vector3());
        controlsRef.current.target.copy(center);
        cameraRef.current.position.copy(center.clone().add(new THREE.Vector3(size, size * 0.7, size)));
        controlsRef.current.update();
      }
    })();

    return () => {
      cancelled = true;
    };
  }, [objects, selectedId]);

  return <div className="scene" ref={mountRef} />;
}
