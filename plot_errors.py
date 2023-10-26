import matplotlib.pyplot as plt
from utilities import FileReader
from pathlib import Path


def plot_motion(dir: Path, ctrlr_type: str, motion_name: str) -> None:
    ctrlr_names = {"straight": "linear", "turn": "angular"}
    ctrlr_name: str = ctrlr_names[motion_name]

    f_error = dir.joinpath(f"{ctrlr_name}.csv")
    f_pose = dir.joinpath("robot_pose.csv")
    f_save = dir.joinpath("graph.png")

    fig, (ax0, ax1, ax2, ax3) = plt.subplots(1, 4, figsize=(20, 6))
    fig.suptitle(f"{ctrlr_name.capitalize()} {ctrlr_type} Point Controller")

    # {e-t, edot-t}
    headers, values = FileReader(f_error).read_file()
    first_stamp = values[0][-1]

    time_list = []
    for val in values:
        time_list.append(val[-1] - first_stamp)

    for i in range(0, len(headers) - 1):
        ax0.plot(time_list, [lin[i] for lin in values], label=headers[i])

    ax0.set_title("Pose States over Time")
    ax0.set_xlabel("Time [s]")
    ax0.set_ylabel("Sate Value")
    ax0.legend()
    ax0.grid()

    # {x-t, y-t, th-t}
    headers, values = FileReader(f_pose).read_file()
    first_stamp = values[0][-1]

    time_list = []
    for val in values:
        time_list.append(val[-1] - first_stamp)

    for i in range(0, len(headers) - 1):
        ax1.plot(time_list, [lin[i] for lin in values], label=headers[i])

    ax1.set_title("Pose States over Time")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Sate Value")
    ax1.legend()
    ax1.grid()

    # {x-y}
    headers, values = FileReader(f_pose).read_file()

    ax2.plot([lin[0] for lin in values], [lin[1] for lin in values])
    ax2.set_title("Trajectory")
    ax2.set_xlabel("X [m]")
    ax2.set_ylabel("Y [m]")
    ax2.grid()
    ax2.set_aspect("equal")

    # {e-edot}
    headers, values = FileReader(f_error).read_file()

    ax3.plot([lin[0] for lin in values], [lin[1] for lin in values])
    ax3.set_title("Error")
    ax3.set_xlabel("e")
    ax3.set_ylabel("$\dot{e}$")
    ax3.grid()
    ax3.set_aspect("equal")

    # saving
    plt.savefig(f_save)


def plot_traj(dir: Path, traj_name: str, ctrlr_name: str) -> None:
    f_traj = dir.joinpath("trajectory.csv")
    f_pose = dir.joinpath("robot_pose.csv")
    f_save = dir.joinpath("graph.png")

    fig, ax0 = plt.subplots(1, 1)
    fig.suptitle(f"Robot {traj_name.capitalize()} Path Following Using {ctrlr_name}")

    # plot ideal traj
    headers, values = FileReader(f_traj).read_file()
    ax0.plot([lin[0] for lin in values], [lin[1] for lin in values], label="Ideal")

    # plot robot pose
    headers, values = FileReader(f_pose).read_file()

    ax0.plot([lin[0] for lin in values], [lin[1] for lin in values], label="Actual")
    ax0.set_title("Trajectory")
    ax0.set_xlabel("X [m]")
    ax0.set_ylabel("Y [m]")
    ax0.legend()
    ax0.grid()
    ax0.set_aspect("equal")

    plt.savefig(f_save)


def main() -> None:
    dir_data = Path("./data")
    dir_point = dir_data.joinpath("point")
    dir_traj = dir_data.joinpath("trajectory")

    # for each trajectories
    for d in dir_traj.iterdir():
        d: Path
        if d.is_dir():
            traj = d.name

            # for each controller
            for c in d.iterdir():
                c: Path
                if c.is_dir():
                    ctrlr = c.name
                    plot_traj(c, traj, ctrlr)

    # for each controller
    for c in dir_point.iterdir():
        c: Path
        if c.is_dir():
            ctrlr = c.name

            # for each motion (straight = linear ctrlr, turn = angular ctrlr)
            for m in c.iterdir():
                m: Path
                if m.is_dir():
                    motion = m.name
                    plot_motion(m, ctrlr, motion)


if __name__ == "__main__":
    main()
