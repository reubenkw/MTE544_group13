# You can use this file to plot the loged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import os

import matplotlib.pyplot as plt
from utilities import FileReader
import numpy as np

def get_path_sensor_from_filename(fname: str) -> tuple[str, str]:
    # filename should be in format data/{path}/{sensor}
    # returns capitalized path and sensor names
    filename_parts = fname.replace("/", "\\").replace(".csv", "").split("\\")
    assert filename_parts[0] == "data" and len(filename_parts) == 3
    return filename_parts[1].capitalize(), filename_parts[2].capitalize()

def plot_errors(filename: str):
    path_type, sensor = get_path_sensor_from_filename(filename)
    
    headers, values=FileReader(filename).read_file() 
    time_list=[]
    first_stamp=values[0][-1]
    for val in values:
        # convert time from [ns] to [s]
        time_list.append((val[-1] - first_stamp) * 1e-9)

    plot_order = range(0, len(headers) - 1)
    # required for all data to be visible
    if path_type == "Line" and sensor == "Imu":
        plot_order = [1, 0, 2]

    plt.figure()
    for i in plot_order:
        plt.plot(time_list, [lin[i] for lin in values], label= f"{headers[i]} linear")
    
    #plt.plot([lin[0] for lin in values], [lin[1] for lin in values])
    plt.title(f"{path_type} {sensor} Data")
    plt.xlabel("Time [s]")
    plt.ylabel(f"{sensor} Reading")
    plt.legend()
    plt.grid()

    plt.savefig(f"plots/{path_type.lower()}_{sensor.lower()}.png", transparent=True, bbox_inches='tight')


def plot_laser(filename):
    path_type, _ = get_path_sensor_from_filename(filename)

    _, values=FileReader(filename).read_file()
    # headers assumed to be (ranges, stamp, ) 
    # which doesn't fit the data but matches the file reader format
    first_stamp=values[0][-1]
    
    # assumes all rows have same entries
    laser_data = np.array(values)[:, :-1]
    # convert times from [ns] to [s]
    times = (np.array(values)[:, -1] - first_stamp) * 1e-9
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
    # ax.scatter(x[0], y[0], times_tiled[0], marker="o")

    ax.set_title(f"{path_type} Lidar Readings")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("time [s]")
    ax.set_box_aspect(aspect=None, zoom=0.8)

    plt.savefig(f"plots/{path_type.lower()}_lidar_3d.png", transparent=True, bbox_inches='tight')

    # 2d plotting
    # slice of 3d plot at t=0
    plt.figure()
    plt.scatter(x[0], y[0])

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")

    plt.title("Initial Lidar Reading")

    plt.savefig(f"plots/{path_type.lower()}_lidar.png", transparent=True, bbox_inches='tight')
    

import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=False, help='List of files to process. If not given, searches \\data dir.')
    
    args = parser.parse_args()
    
    filenames=args.files
    if not filenames:
        # if files not given, search data folder for csv files
        print("No files given. Searching data dir.")
        filenames = [os.path.join(dp, f) for dp, dn, filenames in os.walk("data") for f in filenames if os.path.splitext(f)[1] == '.csv']

    print("plotting the files", args.files)

    for filename in filenames:
        filenames: str
        if filename.endswith("laser.csv"):
            plot_laser(filename)
        else:
            plot_errors(filename)

    print("Done.")
