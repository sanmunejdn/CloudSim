/// @file useMeshSync.ts
/// @brief 对象网格/点云：签名变化重建，否则只同步 worldMatrix

import { useEffect } from "react";
import * as THREE from "three";
import { fetchMeshSoup, type BackendObject } from "../../api";
import {
  applyMeshSelectionStyle,
  applyObjectTransform,
  colorFromObject,
  createMeshMaterial,
  disposeObject3D,
} from "../objectMesh";
import { isSceneCoordinateFrame, makeCoordinateFrameAxes } from "../frameAxes";
import { geomSignature, loadPointCloudObject } from "../meshLoad";
import type { ViewportRefs } from "./types";

export function useMeshSync(
  refs: ViewportRefs,
  objects: BackendObject[],
  selectedId: string | null,
  pointCloudRevision: number,
) {
  const { contentRef, idToMesh, boxRef, geomSigRef, focusPendingRef, cameraRef, controlsRef } = refs;

  useEffect(() => {
    const content = contentRef.current;
    if (!content) return;
    const sig = geomSignature(objects, pointCloudRevision);
    let cancelled = false;

    const syncExisting = () => {
      const box = new THREE.Box3();
      for (const obj of objects) {
        const mesh = idToMesh.current.get(obj.id);
        if (!mesh) continue;
        applyObjectTransform(mesh, obj);
        mesh.visible = !!obj.visible;
        if (!isSceneCoordinateFrame(obj)) {
          applyMeshSelectionStyle(mesh, obj, obj.id === selectedId);
        }
        if (mesh.visible) box.expandByObject(mesh);
      }
      boxRef.current = box;
    };

    if (sig === geomSigRef.current && idToMesh.current.size > 0) {
      syncExisting();
      return;
    }

    (async () => {
      while (content.children.length) {
        const c = content.children[0];
        content.remove(c);
        disposeObject3D(c);
      }
      idToMesh.current.clear();
      const box = new THREE.Box3();
      const snapshot = objects.slice();
      for (const obj of snapshot) {
        if (cancelled) return;
        if (!obj.visible) continue;
        let mesh: THREE.Object3D | null = null;
        if (isSceneCoordinateFrame(obj)) {
          mesh = makeCoordinateFrameAxes(100);
          mesh.userData.backendId = obj.id;
          applyObjectTransform(mesh, obj);
        } else {
          if (!obj.hasGeometry) continue;
          if (obj.geometryKind === 1) {
            const pts = await loadPointCloudObject(obj);
            if (cancelled || !pts) continue;
            mesh = pts;
            applyObjectTransform(mesh, obj);
          } else {
            const soup = await fetchMeshSoup(obj.id);
            if (cancelled || !soup || soup.length < 9) continue;
            const selected = obj.id === selectedId;
            const geo = new THREE.BufferGeometry();
            geo.setAttribute("position", new THREE.BufferAttribute(soup, 3));
            geo.computeVertexNormals();
            mesh = new THREE.Mesh(geo, createMeshMaterial(colorFromObject(obj, selected), selected));
            mesh.userData.backendId = obj.id;
            applyObjectTransform(mesh, obj);
          }
        }
        if (!mesh) continue;
        content.add(mesh);
        idToMesh.current.set(obj.id, mesh);
        mesh.updateMatrixWorld(true);
        box.expandByObject(mesh);
      }
      if (cancelled) return;
      geomSigRef.current = sig;
      boxRef.current = box;
      if (focusPendingRef.current && !box.isEmpty() && cameraRef.current && controlsRef.current) {
        focusPendingRef.current = false;
        const size = Math.max(box.getSize(new THREE.Vector3()).length(), 100);
        const center = box.getCenter(new THREE.Vector3());
        controlsRef.current.target.copy(center);
        cameraRef.current.position.copy(center.clone().add(new THREE.Vector3(size, size * 0.7, size)));
        controlsRef.current.update();
      }
    })();

    return () => {
      cancelled = true;
    };
  }, [objects, selectedId, pointCloudRevision, contentRef, idToMesh, boxRef, geomSigRef, focusPendingRef, cameraRef, controlsRef]);
}
