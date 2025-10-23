import type { RoomManifest } from '../types';

export async function loadRoomManifest(url: string): Promise<RoomManifest> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`HTTP ${res.status} for ${url}`);
  const json = (await res.json()) as RoomManifest;
  return json;
}
