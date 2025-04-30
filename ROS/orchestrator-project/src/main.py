from orchestrator import Orchestrator
import traj_utils
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    # Initialize the orchestrator and start the system
    orchestrator = Orchestrator()
    print("System initialized.")

    traj_utils = traj_utils.TrajectoryUtils("LSPB")
    traj_utils.setMaxParams(4.0, 2.0, 0.5, 1.0) #traj_utils.setParams(maxSpeed=1.0, maxAcc=1.0, accTime=0.5, distance=1.0)
    
    # setpoints
    traj_utils.setParams(maxSpeed=1.0, maxAcc=2.0, accTime=0.5, distance=1.0)

    maxSpeed = traj_utils.setPointSpeed  # Maximum speed (test 1.0)
    maxAcc = traj_utils.setPointAcc  # Maximum acceleration (test 2.0)
    distance = traj_utils.setPointDistance  # Distance to cover (test 1.0)
    totalTime = traj_utils.setPointTime # Time total (test 2.5)
    accTime = traj_utils.setPointTimeAcc  # Acceleration time (test 0.5)
    # distance = 1.0  # Adjusted distance to cover (test 1.0)
    # TODO: make s max = 1 --> not strictly necessary, but it would be nice to have a function that does this    

    # Calculate total time and acceleration time if not defined
    if totalTime is None or accTime is None:
        totalTime, accTime = traj_utils.calculateTimeFromSpeedAndDistance(maxSpeed, maxAcc, distance)
    
    t = totalTime - accTime  # Time to decelerate
    
    (v,a) = traj_utils.calculateParams(accTime, t) 
    print(f"Calculated velocity: {v}, acceleration: {a}")

    # TODO: make same for time
    validate = traj_utils.validateParams(maxSpeed, maxAcc)
    if not validate:
        print("Invalid parameters for trajectory generation.")
        exit(1)
    else:

        sArray = []  # Placeholder for S(t) values
        sdArray = []  # Placeholder for S'(t) values
        sddArray = []  # Placeholder for S''(t) values
        j = []  # Placeholder for the index of the trajectory
        n = 2000  # Number of points to generate
        # TODO: Check for acc time (before recalculating the speed)
        t = []
        for i in range(n):
            time = i / n * totalTime  # Example time for the motion
            # TODO: use refactored distance
            (s, sd, sdd) = traj_utils.generateLSPBProfile(maxSpeed, maxAcc, time, accTime, totalTime, distance)
            sArray.append(s)
            sdArray.append(sd)
            sddArray.append(sdd)
            j.append(i)
            t.append(time)
        print(f"Generated LSPB profile with S(t) values ranging from {sArray[0]} to {sArray[-1]}")
        

        fig, ax = plt.subplots()
        fig.suptitle('LSPB Trajectory')
        ax.set_xlabel('Time [s]')
        ax.plot(t, sArray,linewidth=0.5, color='blue', label = '$s$')
        ax.plot(t, sdArray, linewidth=1.0, color='red', label = 'sd')
        ax.plot(t, sddArray, linewidth=1.0, color='green', label = 'sdd')
        plt.legend()
        plt.show()

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