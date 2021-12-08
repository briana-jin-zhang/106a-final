#!/usr/bin/env python

import numpy as np
import os
import argparse
import imageio
import matplotlib.pyplot as plt
from tqdm import tqdm

# Arguments
def parse():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d", "--dir",
        help="Path to the trajectory directory",
        type=str,
        required=True
    )
    parser.add_argument(
        "--visualize",
        help="If set, safes a gif visualizing the cloth",
        action='store_true'
    )
    return parser.parse_known_args()

def mag_coord(pts, eef_pose):
    num_cloth_points = pts.shape[0]
    pts_eef = np.linalg.inv(eef_pose) @ np.hstack((pts, np.ones((num_cloth_points, 1)))).T
    pts_eef = (pts_eef.T - pts_eef[:, 0]).T
    return np.array([pts_eef[0], pts_eef[2]]).T

def save_vis(data_dir, points):
    with imageio.get_writer(os.path.join(data_dir, 'cloth_points.gif'), mode='I') as writer:
        for pts in tqdm(points):
            fname = '_tmp.png'
            plt.scatter(*pts.T)
            plt.savefig(fname)
            plt.clf()
            image = imageio.imread(fname)
            writer.append_data(image)

def main(data_dir, visualize):
    hand_poses = np.load(os.path.join(data_dir, 'hand_poses.npy'))
    cloth_pts = np.load(os.path.join(data_dir, 'cloth_points.npy'))
    mount = np.load(os.path.join(data_dir, 'gripper_sites.npy'))[0] - hand_poses[0, :3, 3]
    mount_g = np.zeros((4, 4))
    mount_g[:3, 3] = mount
    hand_poses = hand_poses + mount_g
    horizon = cloth_pts.shape[0]
    cloth_pts_transformed = np.array([mag_coord(pts, pose) for pose, pts in zip(hand_poses, cloth_pts)])
    np.save(os.path.join(data_dir, 'cloth_points_transformed'), cloth_pts_transformed)
    if visualize:
        save_vis(data_dir, cloth_pts_transformed)

if __name__ == '__main__':
    args, _ = parse()
    main(args.dir, args.visualize)