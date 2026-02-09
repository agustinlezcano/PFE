from .orchestrator import Orchestrator
from .traj_utils import TrajectoryUtils as traj_utils#impor w/ as
import matplotlib.pyplot as plt
import numpy as np

def initialize_and_generate_trajectory():
    # Initialize the orchestrator and start the system
    orchestrator = Orchestrator()
    print("System initialized.")

    traj_utils_instance = traj_utils("LSPB")
    traj_utils_instance.setMaxParams(4.0, 2.0, 0.5, 1.0)

    # Set trajectory parameters
    traj_utils_instance.setParams(maxSpeed=1.0, maxAcc=2.0, accTime=0.5, distance=1.0)

    maxSpeed = traj_utils_instance.setPointSpeed
    maxAcc = traj_utils_instance.setPointAcc
    distance = traj_utils_instance.setPointDistance
    totalTime = traj_utils_instance.setPointTime
    accTime = traj_utils_instance.setPointTimeAcc

    # Calculate total time and acceleration time if not defined
    if totalTime is None or accTime is None:
        totalTime, accTime = traj_utils_instance.calculateTimeFromSpeedAndDistance(maxSpeed, maxAcc, distance)

    t = totalTime - accTime

    (v, a) = traj_utils_instance.calculateParams(accTime, t)
    #print(f"Calculated velocity: {v}, acceleration: {a}")

    # Validate trajectory parameters
    validate = traj_utils_instance.validateParams(maxSpeed, maxAcc)
    if not validate:
        print("Invalid parameters for trajectory generation.")
        exit(1)

    # Generate trajectory profile
    sArray, sdArray, sddArray, t = generate_trajectory_profile(traj_utils_instance, totalTime, accTime, maxSpeed, maxAcc, distance)

    # Plot the trajectory
    #plot_trajectory(t, sArray, sdArray, sddArray)
    return sArray, sdArray, sddArray, t

def generate_trajectory_profile(traj_utils_instance, totalTime, accTime, maxSpeed, maxAcc, distance, n=2000):
    sArray, sdArray, sddArray, t = [], [], [], []
    for i in range(n):
        time = i / n * totalTime
        (s, sd, sdd) = traj_utils_instance.generateLSPBProfile(maxSpeed, maxAcc, time, accTime, totalTime, distance)
        sArray.append(s)
        sdArray.append(sd)
        sddArray.append(sdd)
        t.append(time)
    #print(f"Generated LSPB profile with S(t) values ranging from {sArray[0]} to {sArray[-1]}")
    return sArray, sdArray, sddArray, t

def plot_trajectory(t, sArray, sdArray, sddArray):
    fig, ax = plt.subplots()
    fig.suptitle('LSPB Trajectory')
    ax.set_xlabel('Time [s]')
    ax.plot(t, sArray, linewidth=0.5, color='blue', label='$s$')
    ax.plot(t, sdArray, linewidth=1.0, color='red', label='sd')
    ax.plot(t, sddArray, linewidth=1.0, color='green', label='sdd')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    # Call the refactored method
    initialize_and_generate_trajectory()

# plt.style.use('_mpl-gallery')

# # make data
# x = np.linspace(0, 10, 100)
# y = 4 + 1 * np.sin(2 * x)
# x2 = np.linspace(0, 10, 25)
# y2 = 4 + 1 * np.sin(2 * x2)

# # plot
# fig, ax = plt.subplots()

# ax.plot(x2, y2 + 2.5, 'x', markeredgewidth=2)
# ax.plot(x, y, linewidth=2.0)
# ax.plot(x2, y2 - 2.5, 'o-', linewidth=2)

# ax.set(xlim=(0, 8), xticks=np.arange(1, 8),
#        ylim=(0, 8), yticks=np.arange(1, 8))

# plt.show()