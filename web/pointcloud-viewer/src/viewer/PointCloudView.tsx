import React, { useEffect, useMemo, useState } from 'react';
import DeckGL from '@deck.gl/react';
import { OrbitView, COORDINATE_SYSTEM } from '@deck.gl/core';
import { PointCloudLayer } from '@deck.gl/layers';
import { loadPly } from '../loaders/ply';
import type { ClusterEntry, RoomManifest, LoadedPointCloud } from '../types';

type Props = {
  manifest: RoomManifest;
};

type LayerEntry = {
  id: string;
  name: string;
  url: string;
  visible: boolean;
  cloud?: LoadedPointCloud;
};

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

function useRoomLayersWithFilter(manifest: RoomManifest, filterShell: boolean, dropShellColor: boolean) {
  const [shell, setShell] = useState<LoadedPointCloud | null>(null);
  const [clusters, setClusters] = useState<LayerEntry[]>([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [center, setCenter] = useState<[number, number, number] | null>(null);
  const [shellSize, setShellSize] = useState<[number, number, number] | null>(null);

  useEffect(() => {
    let cancelled = false;
    (async () => {
      try {
        setLoading(true);
        // Load shell with label filtering: exclude 1, 3 (ceiling & wall)
        const shellCloud = await loadPly(manifest.shell, {
          filterLabelNotIn: filterShell ? new Set([1, 3]) : undefined,
          dropColor: dropShellColor
        });
        if (!cancelled) setShell(shellCloud);

        // Prepare cluster entries
        const entries: LayerEntry[] = manifest.clusters.map((c: ClusterEntry, i) => ({
          id: `cluster-${i}`,
          name: c.name,
          url: c.url,
          visible: true
        }));

        // Load clusters sequentially (can parallelize if needed)
        for (let i = 0; i < entries.length; i++) {
          const cloud = await loadPly(entries[i].url);
          if (cancelled) return;
          entries[i] = { ...entries[i], cloud };
        }

        if (!cancelled) setClusters(entries);

        // Compute a shared center for all clouds and recenter positions to origin
        if (!cancelled) {
          let centerSource: Float32Array | null = null;
          if (shellCloud && shellCloud.length > 0) centerSource = shellCloud.attributes.positions as Float32Array;
          if (!centerSource) {
            for (const e of entries) {
              if (e.cloud && e.cloud.length > 0) { centerSource = e.cloud.attributes.positions as Float32Array; break; }
            }
          }
          if (centerSource) {
            const c = computeCenter(centerSource);
            setCenter(c);
            try {
              const size = computeSize(centerSource);
              setShellSize(size);
            } catch {}

            // recenter shell
            if (shellCloud && shellCloud.length > 0) {
              shellCloud.attributes.positions = recenterPositions(shellCloud.attributes.positions as Float32Array, c);
            }
            // recenter clusters
            for (const e of entries) {
              if (e.cloud && e.cloud.length > 0) {
                e.cloud.attributes.positions = recenterPositions(e.cloud.attributes.positions as Float32Array, c);
              }
            }
          }
        }
      } catch (e: any) {
        if (!cancelled) setError(String(e?.message || e));
      } finally {
        if (!cancelled) setLoading(false);
      }
    })();
    return () => {
      cancelled = true;
    };
  }, [manifest.shell, manifest.clusters, filterShell, dropShellColor]);

  return { shell, clusters, setClusters, loading, error, center, shellSize };
}

export default function PointCloudView({ manifest }: Props) {
  const [filterShell, setFilterShell] = useState(false);
  const [shellNoColor, setShellNoColor] = useState(false);
  const { shell, clusters, setClusters, loading, error, center, shellSize } = useRoomLayersWithFilter(manifest, filterShell, shellNoColor);
  const [picked, setPicked] = useState<{ source: 'shell' | 'cluster'; index: number; attrs: Record<string, any> } | null>(null);
  const [shellPointSize, setShellPointSize] = useState(3);
  const [clusterPointSize, setClusterPointSize] = useState(3);
  const [viewState, setViewState] = useState<any>(INITIAL_VIEW_STATE);

  const layers = useMemo(() => {
    const list: any[] = [];
    if (shell) {
      list.push(
        new PointCloudLayer({
          id: 'shell',
          data: { length: shell.length, attributes: buildBinaryAccessors(shell, { includeColor: !shellNoColor }) },
          pickable: true,
          autoHighlight: true,
          pointSize: shellPointSize,
          sizeUnits: 'pixels',
          coordinateSystem: COORDINATE_SYSTEM.CARTESIAN,
          // If we excluded color attribute, use a constant color to avoid sending per-vertex color
          getColor: shellNoColor ? [180, 180, 180] : undefined,
          onClick: (info: any) => {
            if (info && Number.isFinite(info.index)) {
              const idx = info.index as number;
              const attrs: Record<string, any> = {};
              if (shell.attributes.label) attrs.label = (shell.attributes.label as any)[idx];
              if (shell.attributes.point_id) attrs.point_id = (shell.attributes.point_id as any)[idx];
              setPicked({ source: 'shell', index: idx, attrs });
            }
          }
        })
      );
    }
    clusters.forEach((entry: LayerEntry) => {
      if (!entry.visible || !entry.cloud) return;
      const c = entry.cloud;
      list.push(
        new PointCloudLayer({
          id: entry.id,
          data: { length: c.length, attributes: buildBinaryAccessors(c) },
          pickable: true,
          autoHighlight: true,
          pointSize: clusterPointSize,
          sizeUnits: 'pixels',
          coordinateSystem: COORDINATE_SYSTEM.CARTESIAN,
          onClick: (info: any) => {
            if (info && Number.isFinite(info.index)) {
              const idx = info.index as number;
              const attrs: Record<string, any> = {};
              if (c.attributes.label) attrs.label = (c.attributes.label as any)[idx];
              if (c.attributes.point_id) attrs.point_id = (c.attributes.point_id as any)[idx];
              setPicked({ source: 'cluster', index: idx, attrs: { ...attrs, cluster: entry.name } });
            }
          }
        })
      );
    });
    return list;
  }, [shell, clusters, shellPointSize, clusterPointSize, shellNoColor]);

  const view = useMemo(() => new OrbitView({ far: 100000 }), []);
  const initialView = useMemo(() => {
    if (center) {
      // We already recentred all positions by subtracting `center`,
      // so the geometry is around the origin. Look at [0,0,0].
      return { ...INITIAL_VIEW_STATE, target: [0, 0, 0], zoom: 6 } as any;
    }
    return INITIAL_VIEW_STATE as any;
  }, [center]);

  useEffect(() => {
    // When initial view is computed, sync it into controlled state
    setViewState(initialView);
  }, [initialView]);

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

      <div style={{ position: 'absolute', right: 12, top: 12, width: 340, background: 'rgba(255,255,255,0.95)', border: '1px solid #ddd', borderRadius: 6, padding: 8 }}>
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 6 }}>
          <div style={{ fontWeight: 600 }}>Layers</div>
          <button onClick={() => setViewState((vs: any) => ({ ...vs, target: [0, 0, 0], zoom: 6, rotationOrbit: 0, rotationX: 30 }))}>
            重置视图
          </button>
        </div>
        <div style={{ maxHeight: 180, overflow: 'auto', marginBottom: 8 }}>
          <div style={{ display: 'flex', alignItems: 'center', gap: 8, marginBottom: 4 }}>
            <input id="shell-visible" type="checkbox" checked={true} readOnly />
            <span>shell</span>
          </div>
          <div style={{ display: 'flex', alignItems: 'center', gap: 8, margin: '4px 0 8px 22px' }}>
            <input id="shell-filter" type="checkbox" checked={filterShell} onChange={(e) => setFilterShell(e.target.checked)} />
            <label htmlFor="shell-filter">Hide labels 1,3</label>
          </div>
          <div style={{ display: 'flex', alignItems: 'center', gap: 8, margin: '0 0 8px 22px' }}>
            <input id="shell-nocolor" type="checkbox" checked={shellNoColor} onChange={(e) => setShellNoColor(e.target.checked)} />
            <label htmlFor="shell-nocolor">No color (gray)</label>
          </div>
          <div style={{ display: 'flex', alignItems: 'center', gap: 8, margin: '0 0 8px 22px' }}>
            <label htmlFor="shell-psize" style={{ minWidth: 70 }}>Point size</label>
            <input id="shell-psize" type="range" min={1} max={8} value={shellPointSize}
                   onChange={(e) => setShellPointSize(parseInt(e.target.value))} />
            <span style={{ width: 22, textAlign: 'right' }}>{shellPointSize}</span>
          </div>
          {clusters.map((c: LayerEntry, i: number) => (
            <label key={c.id} style={{ display: 'flex', alignItems: 'center', gap: 8, marginBottom: 4 }}>
              <input
                type="checkbox"
                checked={c.visible}
                onChange={(e) => {
                  const copy = clusters.slice();
                  copy[i] = { ...copy[i], visible: e.target.checked };
                  setClusters(copy);
                }}
              />
              <span style={{ whiteSpace: 'nowrap', overflow: 'hidden', textOverflow: 'ellipsis' }}>{c.name}</span>
            </label>
          ))}
          {clusters.length > 0 && (
            <div style={{ display: 'flex', alignItems: 'center', gap: 8, marginLeft: 22 }}>
              <label htmlFor="cluster-psize" style={{ minWidth: 70 }}>Point size</label>
              <input id="cluster-psize" type="range" min={1} max={8} value={clusterPointSize}
                     onChange={(e) => setClusterPointSize(parseInt(e.target.value))} />
              <span style={{ width: 22, textAlign: 'right' }}>{clusterPointSize}</span>
            </div>
          )}
        </div>
        <div style={{ fontWeight: 600, marginBottom: 6 }}>Stats</div>
        <div style={{ fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace', fontSize: 12, marginBottom: 8 }}>
          <div>shell points: {shell ? shell.length : 0}</div>
          <div>clusters loaded: {clusters.filter((c: LayerEntry) => !!c.cloud).length}</div>
          {shellSize && (
            <div>shell size: {shellSize.map((v) => v.toFixed(2)).join(' × ')}</div>
          )}
        </div>
        <div style={{ fontWeight: 600, marginBottom: 6 }}>Inspector</div>
        {!picked && <div style={{ color: '#666' }}>Click a point to inspect attributes.</div>}
        {picked && (
          <div style={{ fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace', fontSize: 12 }}>
            <div>source: {picked.source}</div>
            {picked.attrs.cluster && <div>cluster: {picked.attrs.cluster}</div>}
            {picked.attrs.point_id !== undefined && <div>point_id: {String(picked.attrs.point_id)}</div>}
            {picked.attrs.label !== undefined && <div>label: {String(picked.attrs.label)}</div>}
            <div>index: {picked.index}</div>
          </div>
        )}
      </div>
    </div>
  );
}
