# You can use this file to plot the logged sensor data
# Currently supports LiDAR, IMU, and Odometry logging
# Inputs file paths using command line argument --files

import matplotlib.pyplot as plt
from utilities import FileReader
import numpy as np

def plot_errors(filename):
    
    headers, values=FileReader(filename).read_file() 
    time_list=[]
    first_stamp=values[0][-1]
    print(len(headers))
    for val in values:
        time_list.append(val[-1] - first_stamp)

    for i in range(0, len(headers) - 1):
        plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")
    
    plt.legend()
    plt.grid()
    plt.show()

def plot_laser(filename):

    headers, values=FileReader(filename).read_file()
    time_list=[]
    print(values)
    first_stamp=values[0][-1]
    
    # assumes all rows have same entries
    laser_data = np.array(values)[:, :-1]
    times = np.array(values)[:, -1] - first_stamp
    times_tiled = np.tile(times[np.newaxis].T, (laser_data.shape[1]))

    # angle between range measurements
    angle = 2 * np.pi / laser_data.shape[1]

    angles = np.arange(stop=2 * np.pi, step=angle)

    # data starts at front of robot (positive x)
    x: np.ndarray = laser_data * np.cos(angles)
    y: np.ndarray = laser_data * np.sin(angles)

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')


    ax.scatter(x.flatten(), y.flatten(), times_tiled.flatten(), marker="o")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("time")

    plt.show()
    

import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        filenames: str
        if filename.endswith("laser.csv"):
            plot_laser(filename)
        else:
            plot_errors(filename)
