# Point Cloud Viewer (deck.gl + loaders.gl)

Web viewer for local PLY point clouds with picking. Includes performance-oriented options for large rooms (shell color stripping, label-based filtering, downsampling).

## Dev quickstart

1) Ensure Node.js >= 18
2) Install deps and run the dev server

```bash
cd web/pointcloud-viewer
npm install
npm run dev
```

3) Prepare data with downsampling (recommended)

Use the provided script to copy shell and downsample clusters, then generate a manifest:

```bash
# show usage
npm run prepare:room

# example: prepare room_007 (downsample shell to 5%, clusters to 20%)
node scripts/downsample_and_prepare_room.mjs \
	--room room_007 \
	--shell "../../output/Full House/floor_0/room_007/results/shell/shell_007/0-7-0_shell.ply" \
	--clustersDir "../../output/Full House/floor_0/room_007/results/filtered_clusters" \
	--ratio 0.2 \
	--ratioShell 0.05 \
	--outDir public/data

# optional: write shell without RGB columns (smaller file, faster parsing)
node scripts/downsample_and_prepare_room.mjs \
	--room room_007 \
	--shell "../../output/Full House/floor_0/room_007/results/shell/shell_007/0-7-0_shell.ply" \
	--clustersDir "../../output/Full House/floor_0/room_007/results/filtered_clusters" \
	--ratio 0.2 \
	--ratioShell 0.05 \
	--outDir public/data \
	--shellNoColor
```

Outputs:
- PLYs in `public/data/<room>/shell.ply` and `public/data/<room>/clusters/*.ply`
- Manifest in `public/manifests/<room>.json`

4) Open the app

- Dev: http://localhost:5173/?manifest=/manifests/room_007.json

## What it does

- Loads shell PLY and (optionally) hides labels {1,3} via toggle (label is preserved during downsampling)
- Optional UI toggle: render shell with a constant gray ("No color (gray)") to reduce GPU uploads and improve FPS
- Loads all cluster PLYs listed in the manifest and renders as toggleable layers
- Picking: click a point to see its index and attributes (`point_id`, `label` where available)

## Performance notes

- For large rooms, consider both:
	- Generating shell without color columns (`--shellNoColor`), and
	- Enabling the viewer toggle "No color (gray)" for the shell layer.
- Downsample shell more aggressively than clusters (e.g., `--ratioShell 0.02` and `--ratio 0.2`).
- Prefer voxel downsampling first for spatial uniformity, then apply a ratio if needed.

## Notes

- Coordinates are CARTESIAN with `OrbitView` (units: meters recommended)
- PLY attributes are auto-detected; if files use different field names, adjust `src/loaders/ply.ts`
- For massive inputs, prefer voxel downsampling first, then ratio

## Production hosting

- Build with `npm run build`
- Serve `dist/` and make sure `/data` is reachable on the same origin
