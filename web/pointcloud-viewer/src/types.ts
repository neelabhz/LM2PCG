export type ClusterEntry = {
  name: string;
  url: string;
};

export type RoomManifest = {
  project: string;
  floor: string;
  room: string;
  shell: string; // URL to shell ply
  clusters: ClusterEntry[]; // URLs to cluster plys
};

export type LoadedPointCloud = {
  length: number;
  attributes: {
    positions: Float32Array;
    colors?: Uint8Array | Uint16Array | Float32Array;
    normals?: Float32Array;
    label?: Int32Array | Uint32Array | Uint16Array | Uint8Array;
    point_id?: Int32Array | Uint32Array | Uint16Array | Uint8Array;
    [key: string]: any;
  };
  meta: {
    hasColor: boolean;
    hasNormal: boolean;
    hasLabel: boolean;
    propertyNames: string[];
  };
};
