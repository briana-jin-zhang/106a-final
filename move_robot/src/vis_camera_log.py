#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
from glob import glob
import sys
import os

def main(log_dir):
    im_paths = sorted(glob(os.path.join(log_dir, '*.npy')))
    for path in im_paths:
        plt.imshow(np.load(path))
        plt.show()

if __name__ == '__main__':
    main(sys.argv[1])