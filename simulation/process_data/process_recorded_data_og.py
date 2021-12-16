#!/usr/bin/env python

import numpy as np
from scipy.spatial.transform import Rotation
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
        "-s", "--scale",
        help="Scale factor to apply to the data",
        type=float,
        default=1.0
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

def split_vec(vec, tool=0):
    idx = np.where(vec[:, 0] == tool)[0][0]
    return vec[idx].reshape(1, 4), np.delete(vec, idx, axis=0)

def g(rvec, tvec):
    R = Rotation.from_rotvec(rvec).as_matrix()
    t = tvec.reshape(3, 1)
    return np.block([[R, t], [np.zeros(3), 1]]).reshape(4, 4)

def mag_coords(rvec, tvec, g_ttp=None, base=5):
    coords = tool_coords(rvec, tvec, g_ttp=g_ttp, indexed=True)
    idx = np.where(np.array(coords)[:, 0] == base)[0][0]
    return (np.array(coords) - coords[idx])[:, 1:]

def tool_coords(rvec, tvec, g_ttp=None, indexed=False):
#     print('tool_coords')
    eef_tvec, _ = split_vec(tvec)
    eef_rvec, _ = split_vec(rvec)
#     print('eef_tvec', eef_tvec.shape)
#     print('eef_rvec', eef_rvec.shape)
    if g_ttp is None:
        g_ttp = np.eye(4)
#     print('[:,1:]')
#     print(eef_rvec[:,1:].shape)
#     print(eef_tvec[:,1:].shape)
    g_ct = g(eef_rvec[:, 1:], eef_tvec[:, 1:])
    g_tc = g_ttp @ np.linalg.inv(g_ct)
    coords = []
    for i in range(len(rvec)):
        if tvec[i, 0] == 0:
            continue
        # appear to be the rvecs and tvecs 
        # rvec is wrong one cuz just want translation
        g_ca = g(rvec[i, 1:], tvec[i, 1:])
        g_ta = g_tc @ g_ca
        if indexed:
            coords.append(np.concatenate(([tvec[i, 0]], g_ta[:3, 3])))
        else:
            coords.append(g_ta[:3, 3])
    return coords

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
    rvecs = np.load(os.path.join(data_dir, 'rvecs.npy'), allow_pickle=True)
    tvecs = np.load(os.path.join(data_dir, 'tvecs.npy'), allow_pickle=True)
    tvecs[:, :, 1:] *= scale
    N = len(tvecs)
    mag_coords_result = mag_coords(rvecs[0], tvecs[0], g_ttp=g_ttp)
    rec_coords = np.array([mag_coords(rvecs[i], tvecs[i], g_ttp=g_ttp).T[[0, 2]].T for i in range(N)])
    print('mag_coords', mag_coords_result.shape)
    print('rec_coords', rec_coords.shape)
    np.save(os.path.join(data_dir, 'recorded_cloth_points'), rec_coords)
    if visualize:
        save_vis(data_dir, rec_coords)

if __name__ == '__main__':
    args, _ = parse()
    main(args.dir, args.scale, args.visualize)