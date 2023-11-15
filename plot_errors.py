import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from utilities import FileReader

def plot_errors(filename):
    # imu_ax, imu_ay, odom_v, odom_w, kf_ax, kf_ay, kf_vx, kf_w, x, y, stamp, 
    headers, values=FileReader(filename).read_file()
    
    time_list=[]
    
    first_stamp=values[0][-1]
    
    for val in values:
        time_list.append(val[-1] - first_stamp)

    
    fig = plt.figure(layout="constrained", figsize=(10, 10))

    gs = GridSpec(3, 2, figure=fig)
    
    title = filename.split("/")[-1].split(".")[0].capitalize()
    fig.suptitle(title)

    ax1 = fig.add_subplot(gs[0, :])
    ax1.plot([lin[8] for lin in values], [lin[9] for lin in values])
    ax1.set_title("State Space")
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("y [m]")
    ax1.grid()
    ax1.set_aspect("equal")

    ax2 = fig.add_subplot(gs[1, 0])
    ax2.set_title("Sensor vs Filtered (ax)")
    ax2.plot(time_list, [lin[0] for lin in values], label=headers[0])
    ax2.plot(time_list, [lin[4] for lin in values], label=headers[4])
    ax2.set_ylabel("Value [$m/s^2$]")
    ax2.set_xlabel("Time [ns]")
    ax2.legend()
    ax2.grid()

    ax3 = fig.add_subplot(gs[1, 1])
    ax3.set_title("Sensor vs Filtered (ay)")
    ax3.plot(time_list, [lin[1] for lin in values], label=headers[1])
    ax3.plot(time_list, [lin[5] for lin in values], label=headers[5])
    ax3.set_ylabel("Value [$m/s^2$]")
    ax3.set_xlabel("Time [ns]")
    ax3.legend()
    ax3.grid()

    ax4 = fig.add_subplot(gs[2, 0])
    ax4.set_title("Sensor vs Filtered (v)")
    ax4.plot(time_list, [lin[2] for lin in values], label=headers[2])
    ax4.plot(time_list, [lin[6] for lin in values], label=headers[6])
    ax4.set_ylabel("Value [$m/s$]")
    ax4.set_xlabel("Time [ns]")
    ax4.legend()
    ax4.grid()

    ax5 = fig.add_subplot(gs[2, 1])
    ax5.set_title("Sensor vs Filtered ($\omega$)")
    ax5.plot(time_list, [lin[3] for lin in values], label=headers[3])
    ax5.plot(time_list, [lin[7] for lin in values], label=headers[7])
    ax3.set_ylabel("Value [$rad/s^2$]")
    ax3.set_xlabel("Time [ns]")
    ax5.legend()
    ax5.grid()

    # saving
    f_save = filename.replace(".csv", ".png")
    plt.savefig(f_save)


import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename)


