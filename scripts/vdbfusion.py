#!/usr/bin/env python3
"""
Reconstruct a triangle mesh from glim save_scans output using VDBFusion.

Reads:
  <path>/submap_N.ply   -- submap point cloud in submap-origin frame
  <path>/poses.txt      -- LC-corrected T_world_origin per submap
         format: submap_id  tx ty tz  qx qy qz qw

Usage:
  python3 vdbfusion.py --path /tmp/glim_scans --output mesh.ply

Install:
  pip install vdbfusion open3d scipy
"""

import argparse
import os
import numpy as np
import open3d as o3d
import vdbfusion
from scipy.spatial.transform import Rotation


def load_poses(poses_file: str) -> dict:
    poses = {}
    with open(poses_file) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            sid = int(parts[0])
            tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            T = np.eye(4)
            T[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
            T[:3, 3] = [tx, ty, tz]
            poses[sid] = T
    return poses


def main():
    parser = argparse.ArgumentParser(
        description='VDBFusion TSDF mesh reconstruction from glim save_scans submaps'
    )
    parser.add_argument('--path', default='/tmp/glim_scans',
                        help='Directory containing submap_N.ply and poses.txt')
    parser.add_argument('--output', default='mesh.ply',
                        help='Output triangle mesh file (.ply / .obj / .stl)')
    parser.add_argument('--voxel_size', type=float, default=0.1,
                        help='TSDF voxel size in metres (default: 0.1)')
    parser.add_argument('--sdf_trunc', type=float, default=0.3,
                        help='TSDF truncation distance in metres (default: 3x voxel_size)')
    parser.add_argument('--space_carving', action='store_true',
                        help='Enable space carving (removes free space behind surfaces)')
    args = parser.parse_args()

    poses_file = os.path.join(args.path, 'poses.txt')
    if not os.path.exists(poses_file):
        print(f'ERROR: poses.txt not found in {args.path}')
        return

    poses = load_poses(poses_file)
    print(f'Loaded {len(poses)} submap poses')

    vdb_volume = vdbfusion.VDBVolume(args.voxel_size, args.sdf_trunc, args.space_carving)

    total_points = 0
    for sid in sorted(poses.keys()):
        ply_path = os.path.join(args.path, f'submap_{sid}.ply')
        if not os.path.exists(ply_path):
            print(f'  WARNING: {ply_path} not found, skipping')
            continue

        pcd = o3d.io.read_point_cloud(ply_path)
        if len(pcd.points) == 0:
            print(f'  WARNING: submap_{sid}.ply is empty, skipping')
            continue

        T = poses[sid]
        R = T[:3, :3]
        t = T[:3, 3]

        # Points are in submap-origin frame; transform to world frame
        pts_local = np.asarray(pcd.points)           # (N, 3)
        pts_world = (R @ pts_local.T).T + t          # (N, 3)

        # Sensor origin = submap centre in world frame (translation of T_world_origin)
        origin = t

        vdb_volume.integrate(pts_world, origin)
        total_points += len(pts_world)
        print(f'  submap {sid:>4d}: {len(pts_world):>7} pts  origin=({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f})')

    print(f'\nTotal integrated: {total_points} points across {len(poses)} submaps')
    print('Extracting triangle mesh...')

    verts, tris = vdb_volume.extract_triangle_mesh()
    if len(verts) == 0:
        print('ERROR: empty mesh — try reducing --voxel_size or check that submaps contain points')
        return

    mesh = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(verts),
        o3d.utility.Vector3iVector(tris),
    )
    mesh.compute_vertex_normals()

    o3d.io.write_triangle_mesh(args.output, mesh)
    print(f'Saved: {args.output}  ({len(verts)} vertices, {len(tris)} triangles)')


if __name__ == '__main__':
    main()
