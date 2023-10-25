import matplotlib.pyplot as plt
from utilities import FileReader
from pathlib import Path


def plot_errors(dir):
    dir = Path(dir)

    f_linear = dir.joinpath("linear.csv")
    f_angular = dir.joinpath("angular.csv")
    f_traj = dir.joinpath("trajectory.csv")
    f_pose = dir.joinpath("robot_pose.csv")
    f_save = dir.joinpath("graph.png")

    fig, (ax0, ax1, ax2, ax3) = plt.subplots(1, 4, figsize=(20,6))
    fig.suptitle("Robot Path Following")
    # plot robot pose
    headers, values=FileReader(f_traj).read_file()
    ax0.plot([lin[0] for lin in values], [lin[1] for lin in values], label="Ideal")
    
    headers, values=FileReader(f_pose).read_file()
    
    ax0.plot([lin[0] for lin in values], [lin[1] for lin in values], label="Actual")
    ax0.set_title("Trajectory")
    ax0.set_xlabel("X [m]")
    ax0.set_ylabel("Y [m]")
    ax0.legend()
    ax0.grid()
    ax0.set_aspect("equal")

    # plot pose variables individually
    first_stamp=values[0][-1]
    
    time_list = []
    for val in values:
        time_list.append(val[-1] - first_stamp)

    for i in range(0, len(headers) - 1):
        ax1.plot(time_list, [lin[i] for lin in values], label= headers[i])
    
    ax1.set_title("Pose States over Time")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Sate Value")
    ax1.legend()
    ax1.grid()


    # linear error
    headers, values=FileReader(f_linear).read_file()
    
    first_stamp=values[0][-1]
    
    time_list = []
    for val in values:
        time_list.append(val[-1] - first_stamp)

    for i in range(0, len(headers) - 1):
        ax2.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")

    ax2.set_title("Linear Error")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Error Value")
    ax2.legend()
    ax2.grid()

    # angular error
    headers, values=FileReader(f_angular).read_file()

    time_list = []
    for val in values:
        time_list.append(val[-1] - first_stamp)

    for i in range(0, len(headers) - 1):
        ax3.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " angular")
    
    ax3.set_title("Angular Error")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Error Value")
    ax3.legend()
    ax3.grid()

    plt.show()

    plt.savefig(f_save, transparent=True, bbox_inches="tight")


import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--dir', type=str, required=True, help='dir containing robot_pose.csv, linear.csv, angular.csv files')
    
    args = parser.parse_args()
    
    print("plotting the files", args.dir)

    plot_errors(args.dir)



