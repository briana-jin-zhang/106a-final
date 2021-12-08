# Data Processing Scripts

This directory contains scripts to process data before training / running inference on the model. At a high level, these convert the given cloth positions into a frame centered at an achor point on the magazine.

# Simulation Data

`process_sim_data.py` takes in the collected cloth positions in a simulated trajectory as a numpy array of shape `(horizon, num_cloth_points, 3)` and transforms them into an array of shape `(horizon, num_cloth_points, 2)`.

The outputs are saved to the same directory as the cloth points.

## Usage

`python process_sim_data.py -d simulated_trajectories/100 -c simulated_trajectories/100`

To visualize the results as a gif, also saved in the trajectory directory:

`python process_sim_data.py -d simulated_trajectories/100 -c simulated_trajectories/100 --visualize`


# Parsed Video Data

`process_recorded_data.py` takes in the reconstructed AR tag poses as `rvecs.npy` and `tvecs.npy` files, each of shape `(frames, num_cloth_points + 1, 4)`. and transforms them into an array of shape `(frames, num_cloth_points, 2)`.

The outputs are saved to the same directory the numpy arrays are stored in.

## Usage

`python process_recorded_data.py -d video_data`

To visualize the results as a gif, also saved in the trajectory directory:

`python process_recorded_data.py -d video_data --visualize`
