import React, { useEffect, useState } from 'react';
import DeckGL from '@deck.gl/react';
import { OrbitView } from '@deck.gl/core';
import { PointCloudLayer } from '@deck.gl/layers';
import { loadPly } from '../loaders/ply';

// MINIMAL test component - no complexity, just raw deck.gl
export default function MinimalTest() {
  const [data, setData] = useState<any>(null);
  
  useEffect(() => {
    loadPly('/data/room_007/shell.ply', {}).then(cloud => {
      console.log('[MinimalTest] Loaded', cloud.length, 'points');
      setData(cloud);
    });
  }, []);

  const layer = data ? new PointCloudLayer({
    id: 'test',
    data: { length: data.length, attributes: { getPosition: { value: data.attributes.positions, size: 3 } } },
    getColor: [255, 0, 0], // BRIGHT RED
    pointSize: 10, // HUGE points
    sizeUnits: 'pixels'
  }) : null;

  console.log('[MinimalTest] Rendering with layer:', layer ? 'YES' : 'NO', 'points:', data?.length || 0);

  return (
    <div style={{ width: '100vw', height: '100vh', position: 'relative' }}>
      <div style={{ position: 'absolute', top: 10, left: 10, background: 'white', padding: 10, zIndex: 10 }}>
        Points loaded: {data?.length || 0}
      </div>
      <DeckGL
        views={new OrbitView({ far: 100000 })}
        initialViewState={{ target: [0, 0, 0], zoom: 0, rotationOrbit: 0, rotationX: 0 }}
        controller={true}
        layers={layer ? [layer] : []}
      />
    </div>
  );
}
