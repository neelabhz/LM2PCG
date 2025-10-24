export { default } from './PointCloudView2';
/*
import React, { useEffect, useMemo, useState } from 'react';
import DeckGL from '@deck.gl/react';
import { OrbitView, COORDINATE_SYSTEM } from '@deck.gl/core';
import { PointCloudLayer } from '@deck.gl/layers';
import { SimpleMeshLayer } from '@deck.gl/mesh-layers';
import { loadPly } from '../loaders/ply';
import { loadMeshPly } from '../loaders/mesh';
import type { UnifiedManifest, ManifestItem, LoadedPointCloud } from '../types';

type Props = { manifest: UnifiedManifest };

type LayerEntry = ManifestItem & { cloud?: LoadedPointCloud; mesh?: { positions: Float32Array; normals?: Float32Array; indices?: Uint16Array | Uint32Array } };

const INITIAL_VIEW_STATE = {
  target: [0, 0, 0],
  rotationOrbit: 0,
  rotationX: 30,
  zoom: 6
};

function computeCenter(positions: Float32Array): [number, number, number] {
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (let i = 0; i < positions.length; i += 3) {
    const x = positions[i], y = positions[i + 1], z = positions[i + 2];
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
    if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
  }
  return [0.5 * (minX + maxX), 0.5 * (minY + maxY), 0.5 * (minZ + maxZ)];
}

function computeSize(positions: Float32Array): [number, number, number] {
  let minX = Infinity, minY = Infinity, minZ = Infinity;
  let maxX = -Infinity, maxY = -Infinity, maxZ = -Infinity;
  for (let i = 0; i < positions.length; i += 3) {
    const x = positions[i], y = positions[i + 1], z = positions[i + 2];
    if (x < minX) minX = x; if (x > maxX) maxX = x;
    if (y < minY) minY = y; if (y > maxY) maxY = y;
    if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;
  }
  return [maxX - minX, maxY - minY, maxZ - minZ];
}

function recenterPositions(positions: Float32Array, center: [number, number, number]): Float32Array {
  const out = new Float32Array(positions.length);
  const [cx, cy, cz] = center;
  for (let i = 0; i < positions.length; i += 3) {
    out[i] = positions[i] - cx;
    out[i + 1] = positions[i + 1] - cy;
    out[i + 2] = positions[i + 2] - cz;
  }
  return out;
}

function buildBinaryAccessors(cloud: LoadedPointCloud, { includeColor = true }: { includeColor?: boolean } = {}) {
  const attrs: any = {
    getPosition: { value: cloud.attributes.positions, size: 3 }
  };
  if (includeColor && cloud.attributes.colors) {
    const isUint8 = cloud.attributes.colors instanceof Uint8Array;
    attrs.getColor = { value: cloud.attributes.colors, size: 3, normalized: isUint8 };
  }
  if (cloud.attributes.normals) attrs.getNormal = { value: cloud.attributes.normals, size: 3 };
  return attrs;
}

function useItems(manifest: UnifiedManifest) {
  const [items, setItems] = useState<LayerEntry[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [center, setCenter] = useState<[number, number, number] | null>(null);
  const [sceneSize, setSceneSize] = useState<[number, number, number] | null>(null);

  useEffect(() => {
    let cancelled = false;
    (async () => {
      try {
        setLoading(true);
        const entries: LayerEntry[] = (manifest.items || []).map((it) => ({ ...it }));
        for (let i = 0; i < entries.length; i++) {
          } else if (entry.kind === 'mesh') {
          if (it.kind === 'pointcloud') {
            const include = it.filters?.labelInclude ? new Set(it.filters.labelInclude) : undefined;
            const exclude = it.filters?.labelExclude ? new Set(it.filters.labelExclude) : undefined;
            const rgba = [...baseColor, Math.round(alpha * 255)];
            const cloud = await loadPly(it.source.url, { filterLabelIn: include, filterLabelNotIn: exclude, dropColor });
            if (cancelled) return;
            entries[i] = { ...it, cloud };
          } else if (it.kind === 'mesh') {
            const mesh = await loadMeshPly(it.source.url);
            if (cancelled) return;
            entries[i] = { ...it, mesh };
              getColor: rgba as any,
        if (!cancelled) setItems(entries);

        if (!cancelled) {
              wireframe: false,
              parameters: { depthTest: true, blend: true, cull: false },
              material: { ambient: 1.0, diffuse: 0.0, shininess: 0, specularColor: [0, 0, 0] }
            if (e.cloud && e.cloud.length > 0) { centerSource = e.cloud.attributes.positions as Float32Array; break; }
          sizeUnits: 'pixels',
          coordinateSystem: COORDINATE_SYSTEM.CARTESIAN,
          getColor: constantColor,
          onClick: (info: any) => {
            if (info && Number.isFinite(info.index)) {
              const idx = info.index as number;
              const attrs: Record<string, any> = {};
              if (c.attributes.label) attrs.label = (c.attributes.label as any)[idx];
              if (c.attributes.point_id) attrs.point_id = (c.attributes.point_id as any)[idx];
              setPicked({ itemId: entry.id, index: idx, attrs: { ...attrs, name: entry.name, group: entry.group } });
            }
          }
        }));
      } else if (entry.kind === 'mesh') {
        if (!entry.mesh) return;
        const baseColor = entry.style?.color ?? [30, 144, 255]; // dodgerblue default
        const alpha = Math.max(0, Math.min(1, uobbOpacity));
        const rgba = [...baseColor, Math.round(alpha * 255)];
        // Convert to attributes format expected by SimpleMeshLayer
        // Pass plain typed arrays the SimpleMeshLayer understands
        const mesh = {
          positions: entry.mesh.positions,
          normals: entry.mesh.normals,
          indices: entry.mesh.indices,
          mode: 4 // GL.TRIANGLES
        } as any;
        list.push(new SimpleMeshLayer({
          id: entry.id,
          mesh,
          getColor: rgba as any,
          getLineColor: rgba as any,
          opacity: alpha,
          pickable: false,
          coordinateSystem: COORDINATE_SYSTEM.CARTESIAN,
          wireframe: true,
          material: { ambient: 1.0, diffuse: 0.0, specular: 0.0, shininess: 32 },
          parameters: { depthTest: true, depthMask: true, blend: true, cull: false }
        }));

        // Edge overlay as LineLayer (robust visibility regardless of fill/lighting)
        if (entry.mesh.indices && entry.mesh.positions) {
          const idx = Array.from(entry.mesh.indices as any as Iterable<number>);
          try { console.log('[UOBB] verts', (entry.mesh.positions.length/3)|0, 'tris', (idx.length/3)|0); } catch {}
          const edges = new Map<string, [number, number]>();
          for (let i = 0; i < idx.length; i += 3) {
            const a = idx[i], b = idx[i + 1], c = idx[i + 2];
            const pairs: [number, number][] = [[a, b], [b, c], [c, a]];
            for (const [u, v] of pairs) {
              import React, { useEffect, useMemo, useState } from 'react';
              import DeckGL from '@deck.gl/react';
              import { OrbitView, COORDINATE_SYSTEM } from '@deck.gl/core';
              import { PointCloudLayer } from '@deck.gl/layers';
              import { SimpleMeshLayer } from '@deck.gl/mesh-layers';
              import { loadPly } from '../loaders/ply';
              import { loadMeshPly } from '../loaders/mesh';
              import type { UnifiedManifest, ManifestItem, LoadedPointCloud } from '../types';

              type Props = { manifest: UnifiedManifest };

              type LayerEntry = ManifestItem & { cloud?: LoadedPointCloud; mesh?: { positions: Float32Array; normals?: Float32Array; indices?: Uint16Array | Uint32Array } };

              const INITIAL_VIEW_STATE = {
                target: [0, 0, 0],
                rotationOrbit: 0,
                rotationX: 30,
                zoom: 6
              };

              function updateBoundsFrom(positions: Float32Array, bounds: { min: number[]; max: number[] }) {
                for (let i = 0; i < positions.length; i += 3) {
                  const x = positions[i], y = positions[i + 1], z = positions[i + 2];
                  if (x < bounds.min[0]) bounds.min[0] = x; if (x > bounds.max[0]) bounds.max[0] = x;
                  if (y < bounds.min[1]) bounds.min[1] = y; if (y > bounds.max[1]) bounds.max[1] = y;
                  if (z < bounds.min[2]) bounds.min[2] = z; if (z > bounds.max[2]) bounds.max[2] = z;
                }
              }

              function centerFromBounds(bounds: { min: number[]; max: number[] }): [number, number, number] {
                return [
                  0.5 * (bounds.min[0] + bounds.max[0]),
                  0.5 * (bounds.min[1] + bounds.max[1]),
                  0.5 * (bounds.min[2] + bounds.max[2])
                ];
              }

              function sizeFromBounds(bounds: { min: number[]; max: number[] }): [number, number, number] {
                return [
                  bounds.max[0] - bounds.min[0],
                  bounds.max[1] - bounds.min[1],
                  bounds.max[2] - bounds.min[2]
                ];
              }

              function buildBinaryAccessors(cloud: LoadedPointCloud, { includeColor = true }: { includeColor?: boolean } = {}) {
                const attrs: any = {
                  getPosition: { value: cloud.attributes.positions, size: 3 }
                };
                if (includeColor && cloud.attributes.colors) {
                  const isUint8 = cloud.attributes.colors instanceof Uint8Array;
                  attrs.getColor = { value: cloud.attributes.colors, size: 3, normalized: isUint8 };
                }
                if (cloud.attributes.normals) attrs.getNormal = { value: cloud.attributes.normals, size: 3 };
                return attrs;
              }

              export default function PointCloudView({ manifest }: Props) {
                const [items, setItems] = useState<LayerEntry[]>([]);
                const [loading, setLoading] = useState(true);
                const [error, setError] = useState<string | null>(null);
                const [center, setCenter] = useState<[number, number, number] | null>(null);
                const [sceneSize, setSceneSize] = useState<[number, number, number] | null>(null);
                const [viewState, setViewState] = useState<any>(INITIAL_VIEW_STATE);
                const [globalPointSize, setGlobalPointSize] = useState<number>(4);
                const [uobbOpacity, setUobbOpacity] = useState<number>(0.35);
                const [picked, setPicked] = useState<{ itemId: string; index: number; attrs: Record<string, any> } | null>(null);

                // Load items from manifest
                useEffect(() => {
                  let cancelled = false;
                  (async () => {
                    try {
                      setLoading(true);
                      setError(null);
                      const entries: LayerEntry[] = (manifest.items || []).map((it) => ({ ...it }));
                      for (let i = 0; i < entries.length; i++) {
                        const it = entries[i];
                        if (it.kind === 'pointcloud') {
                          const include = it.filters?.labelInclude ? new Set(it.filters.labelInclude) : undefined;
                          const exclude = it.filters?.labelExclude ? new Set(it.filters.labelExclude) : undefined;
                          const dropColor = it.style?.colorMode === 'constant';
                          const cloud = await loadPly(it.source.url, { filterLabelIn: include, filterLabelNotIn: exclude, dropColor });
                          if (cancelled) return;
                          entries[i] = { ...it, cloud };
                        } else if (it.kind === 'mesh') {
                          const mesh = await loadMeshPly(it.source.url);
                          if (cancelled) return;
                          entries[i] = { ...it, mesh };
                        }
                      }
                      if (cancelled) return;

                      // Compute overall bounds across all loaded geometries
                      const bounds = { min: [Infinity, Infinity, Infinity], max: [-Infinity, -Infinity, -Infinity] };
                      let anyGeom = false;
                      for (const e of entries) {
                        if (e.cloud?.attributes?.positions) { updateBoundsFrom(e.cloud.attributes.positions, bounds); anyGeom = true; }
                        else if (e.mesh?.positions) { updateBoundsFrom(e.mesh.positions, bounds); anyGeom = true; }
                      }
                      if (anyGeom) {
                        const c = centerFromBounds(bounds);
                        const sz = sizeFromBounds(bounds);
                        setCenter(c);
                        setSceneSize(sz);
                        // Heuristic zoom from size
                        const maxDim = Math.max(sz[0], sz[1], sz[2]);
                        const z = 8 - Math.log2(Math.max(1e-6, maxDim));
                        const zoom = Math.max(2, Math.min(14, z));
                        setViewState((vs: any) => ({ ...vs, target: c, zoom }));
                      }

                      setItems(entries);
                    } catch (e: any) {
                      if (!cancelled) setError(String(e?.message || e));
                    } finally {
                      if (!cancelled) setLoading(false);
                    }
                  })();
                  return () => { cancelled = true; };
                }, [manifest]);

                const layers = useMemo(() => {
                  const list: any[] = [];
                  for (const entry of items) {
                    if (entry.visible === false) continue;
                    if (entry.kind === 'pointcloud') {
                      const c = entry.cloud;
                      if (!c) continue;
                      const includeFileColor = entry.style?.colorMode !== 'constant';
                      const constantColor = (entry.style?.color ?? [128, 128, 128]) as [number, number, number];
                      list.push(new PointCloudLayer({
                        id: entry.id,
                        ...buildBinaryAccessors(c, { includeColor: includeFileColor }),
                        pointSize: globalPointSize,
                        sizeUnits: 'pixels',
                        coordinateSystem: COORDINATE_SYSTEM.CARTESIAN,
                        getColor: includeFileColor ? undefined : constantColor,
                        pickable: true,
                        onClick: (info: any) => {
                          if (info && Number.isFinite(info.index)) {
                            const idx = info.index as number;
                            const attrs: Record<string, any> = {};
                            if (c.attributes.label) attrs.label = (c.attributes.label as any)[idx];
                            if (c.attributes.point_id) attrs.point_id = (c.attributes.point_id as any)[idx];
                            setPicked({ itemId: entry.id, index: idx, attrs: { ...attrs, name: entry.name, group: entry.group } });
                          }
                        }
                      }));
                    } else if (entry.kind === 'mesh') {
                      if (!entry.mesh) continue;
                      const baseColor = entry.style?.color ?? [30, 144, 255]; // dodgerblue default
                      const alpha = Math.max(0, Math.min(1, uobbOpacity));
                      const rgba = [...baseColor, Math.round(alpha * 255)];
                      // Convert to attributes format expected by SimpleMeshLayer (luma.gl Geometry)
                      const attributes: any = { positions: { value: entry.mesh.positions, size: 3 } };
                      if (entry.mesh.normals) attributes.normals = { value: entry.mesh.normals, size: 3 };
                      const mesh = { attributes, indices: entry.mesh.indices, mode: 4 } as any; // 4 = GL.TRIANGLES
                      list.push(new SimpleMeshLayer({
                        id: entry.id,
                        mesh,
                        getColor: rgba as any,
                        opacity: alpha,
                        pickable: false,
                        coordinateSystem: COORDINATE_SYSTEM.CARTESIAN,
                        wireframe: false,
                        parameters: { depthTest: true, blend: true, cull: false },
                        material: { ambient: 1.0, diffuse: 0.0, shininess: 0, specularColor: [0, 0, 0] }
                      }));
                    }
                  }
                  return list;
                }, [items, globalPointSize, uobbOpacity]);

                const view = useMemo(() => new OrbitView({ far: 100000 }), []);

                const grouped = useMemo(() => {
                  const g = new Map<string, LayerEntry[]>();
                  for (const it of items) {
                    const key = it.group || 'ungrouped';
                    if (!g.has(key)) g.set(key, []);
                    g.get(key)!.push(it);
                  }
                  return g;
                }, [items]);

                return (
                  <div style={{ height: '100%', position: 'relative' }}>
                    {loading && <div style={{ position: 'absolute', top: 8, left: 12, background: 'rgba(255,255,255,0.9)', padding: '4px 8px', borderRadius: 4, zIndex: 10 }}>Loading...</div>}
                    {error && <div style={{ position: 'absolute', top: 8, left: 12, color: '#b00', background: 'rgba(255,255,255,0.9)', padding: '4px 8px', borderRadius: 4, zIndex: 10 }}>Error: {error}</div>}

                    <DeckGL
                      views={view}
                      viewState={viewState}
                      controller={true}
                      onViewStateChange={(e: any) => setViewState(e.viewState)}
                      layers={layers}
                      style={{ position: 'absolute', inset: '0' }}
                    />

                    <div style={{ position: 'absolute', right: 12, top: 12, width: 380, background: 'rgba(255,255,255,0.95)', border: '1px solid #ddd', borderRadius: 6, padding: 8 }}>
                      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 6 }}>
                        <div style={{ fontWeight: 600 }}>Layers</div>
                        <button onClick={() => setViewState((vs: any) => ({ ...vs, target: center ?? vs.target, zoom: 6, rotationOrbit: 0, rotationX: 30 }))}>
                          重置视图
                        </button>
                      </div>
                      <div style={{ display: 'grid', gridTemplateColumns: 'auto 1fr auto', alignItems: 'center', gap: 8, marginBottom: 8 }}>
                        <div style={{ fontSize: 12, color: '#555' }}>点大小</div>
                        <input type="range" min={1} max={10} step={1} value={globalPointSize} onChange={(e) => setGlobalPointSize(Number(e.target.value))} />
                        <div style={{ fontFamily: 'ui-monospace, Menlo, monospace', fontSize: 12, width: 28, textAlign: 'right' }}>{globalPointSize}</div>
                        {items.some(i => i.kind === 'mesh') && (
                          <>
                            <div style={{ fontSize: 12, color: '#555' }}>UOBB透明度</div>
                            <input type="range" min={0} max={1} step={0.05} value={uobbOpacity} onChange={(e) => setUobbOpacity(Number(e.target.value))} />
                            <div style={{ fontFamily: 'ui-monospace, Menlo, monospace', fontSize: 12, width: 28, textAlign: 'right' }}>{uobbOpacity.toFixed(2)}</div>
                          </>
                        )}
                      </div>
                      <div style={{ maxHeight: 220, overflow: 'auto', marginBottom: 8 }}>
                        {[...grouped.entries()].map(([group, arr]) => (
                          <div key={group} style={{ marginBottom: 6 }}>
                            <div style={{ fontWeight: 600, fontSize: 12, color: '#555', margin: '4px 0' }}>{group}</div>
                            {arr.map((it) => (
                              <label key={it.id} style={{ display: 'flex', alignItems: 'center', gap: 8, marginBottom: 4 }}>
                                <input
                                  type="checkbox"
                                  checked={it.visible !== false}
                                  onChange={(e) => {
                                    const copy = items.map((x) => x.id === it.id ? { ...x, visible: e.target.checked } : x);
                                    setItems(copy);
                                  }}
                                />
                                <span style={{ whiteSpace: 'nowrap', overflow: 'hidden', textOverflow: 'ellipsis' }}>{it.name}</span>
                                {it.style?.colorMode === 'constant' && (
                                  <span style={{ fontSize: 11, color: '#888' }}>gray</span>
                                )}
                              </label>
                            ))}
                          </div>
                        ))}
                      </div>
                      <div style={{ fontWeight: 600, marginBottom: 6 }}>Stats</div>
                      <div style={{ fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace', fontSize: 12, marginBottom: 8 }}>
                        <div>items loaded: {items.filter((x) => x.cloud || x.mesh).length} / {items.length}</div>
                        <div>total points: {items.reduce((s, x) => s + (x.cloud?.length || 0), 0)}</div>
                        {sceneSize && (<div>scene size: {sceneSize.map((v) => v.toFixed(2)).join(' × ')}</div>)}
                      </div>
                      <div style={{ fontWeight: 600, marginBottom: 6 }}>Inspector</div>
                      {!picked && <div style={{ color: '#666' }}>Click a point to inspect attributes.</div>}
                      {picked && (
                        <div style={{ fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace', fontSize: 12 }}>
                          <div>item: {picked.attrs.name ?? picked.itemId}</div>
                          {picked.attrs.group && <div>group: {picked.attrs.group}</div>}
                          {picked.attrs.point_id !== undefined && <div>point_id: {String(picked.attrs.point_id)}</div>}
                          {picked.attrs.label !== undefined && <div>label: {String(picked.attrs.label)}</div>}
                          <div>index: {picked.index}</div>
                        </div>
                      )}
                    </div>
                  </div>
                );
              }
              */
