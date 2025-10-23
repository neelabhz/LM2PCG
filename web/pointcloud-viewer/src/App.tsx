import React, { useEffect, useMemo, useState } from 'react';
import PointCloudView from './viewer/PointCloudView';
import { loadRoomManifest } from './utils/manifest';
import type { RoomManifest } from './types';

function useQuery() {
  return useMemo(() => new URLSearchParams(window.location.search), []);
}

export default function App() {
  const query = useQuery();
  const [manifest, setManifest] = useState<RoomManifest | null>(null);
  const [error, setError] = useState<string | null>(null);

  const project = query.get('project') ?? 'Full House';
  const floor = query.get('floor') ?? 'floor_0';
  const room = query.get('room') ?? 'room_007';
  const manifestUrl =
    query.get('manifest') ?? `/manifests/${room}.json`;

  useEffect(() => {
    loadRoomManifest(manifestUrl)
      .then((m) => setManifest(m))
      .catch((e) => setError(String(e)));
  }, [manifestUrl]);

  return (
    <div style={{ height: '100%', display: 'flex', flexDirection: 'column' }}>
      <header style={{ padding: '8px 12px', borderBottom: '1px solid #eee' }}>
        <strong>Point Cloud Viewer</strong>
        <span style={{ marginLeft: 8, color: '#666' }}>
          {project} / {floor} / {room}
        </span>
      </header>
      <div style={{ flex: 1, minHeight: 0 }}>
        {error && (
          <div style={{ padding: 12, color: '#b00' }}>Failed to load manifest: {error}</div>
        )}
        {manifest && (
          <PointCloudView manifest={manifest} />
        )}
      </div>
    </div>
  );
}
