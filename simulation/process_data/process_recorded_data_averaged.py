#!/usr/bin/env python

import numpy as np
from scipy.spatial.transform import Rotation
import os
import argparse
import imageio
import matplotlib.pyplot as plt
from tqdm import tqdm

NUM_TAGS = 6

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
        "-s", "--scale",
        help="Scale factor to apply to the data",
        type=float,
        default=1.4
    )
    parser.add_argument(
        "--visualize",
        help="If set, safes a gif visualizing the cloth",
        action='store_true'
    )
    return parser.parse_known_args()

g_ttp = np.array([
    [-1, 0, 0, 0],
    [0, 0, 1, 0],
    [0, 1, 0, 0],
    [0, 0, 0, 1]
])


def g(rvec, tvec):
    R = Rotation.from_rotvec(rvec).as_matrix()
    t = tvec.reshape(3, 1)
    return np.block([[R, t], [np.zeros(3), 1]]).reshape(4, 4)

# calculates coords and centers around the 5th tag
def mag_coords(rvec, tvec, g_ttp=None, base=5):
    coords = tool_coords(rvec, tvec, g_ttp=g_ttp)
    return (coords-coords[base-1])

def tool_coords(rvec, frame_tvecs, g_ttp=None):
    # rvec: (3,)
    # frame_tvecs: (6, 3) 
    if g_ttp is None:
        g_ttp = np.eye(4)
    g_ct = g(rvec, frame_tvecs[0])
    g_tc = g_ttp @ np.linalg.inv(g_ct)
    # (5,4)
    homog = np.hstack((frame_tvecs[1:], np.ones(NUM_TAGS-1).reshape(-1, 1)))
    return ((g_tc @ homog.T).T)[:, :3]

def save_vis(data_dir, points):
    with imageio.get_writer(os.path.join(data_dir, 'cloth_points.gif'), mode='I') as writer:
        for pts in tqdm(points):
            fname = '_tmp.png'
            plt.scatter(*pts.T)
            plt.savefig(fname)
            plt.clf()
            image = imageio.imread(fname)
            writer.append_data(image)

def main(data_dir, scale, visualize):
    rvecs = np.load(os.path.join(data_dir, 'rvecs_averaged.npy'), allow_pickle=True)
    tvecs = np.load(os.path.join(data_dir, 'tvecs_averaged.npy'), allow_pickle=True)
    tvecs *= scale
    N = tvecs.shape[1]
    rec_coords = np.array([mag_coords(rvecs[i], tvecs[:,i], g_ttp=g_ttp).T[[0, 2]].T for i in range(N)])
    print('rec_coords', rec_coords.shape)
    np.save(os.path.join(data_dir, 'recorded_cloth_points'), rec_coords)
    if visualize:
        save_vis(data_dir, rec_coords)

if __name__ == '__main__':
    args, _ = parse()
    main(args.dir, args.scale, args.visualize)