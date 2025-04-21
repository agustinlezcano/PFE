from orchestrator import Orchestrator
import traj_utils
import matplotlib.pyplot as plt
import numpy as np

if __name__ == "__main__":
    # Initialize the orchestrator and start the system
    orchestrator = Orchestrator()
    print("System initialized.")

    traj_utils = traj_utils.TrajectoryUtils("LSPB")
    maxSpeed = 5.0  # Adjusted maximum speed (test 2)
    maxAcc = 4.0  # Adjusted maximum acceleration
    totalTime = 2.0  # Adjusted total time for the motion
    accTime = 0.5  # Adjusted acceleration time (test 0.5)
    distance = 1.0  # Adjusted distance to cover (test 1.0)
    sArray = []  # Placeholder for S(t) values
    sdArray = []  # Placeholder for S'(t) values
    j = []  # Placeholder for the index of the trajectory
    n = 1000  # Number of points to generate
    # TODO: Check for acc time (before recalculating the speed)
    t = []
    for i in range(n):
        time = i / n * totalTime  # Example time for the motion
        s = traj_utils.generateLSPBProfile(maxSpeed, maxAcc, time, accTime, totalTime, distance)
        sd = traj_utils.generateLSPBProfileDerivative(maxSpeed, maxAcc, time, accTime, totalTime, distance)
        sArray.append(s)
        sdArray.append(sd)
        j.append(i)
        t.append(time)
    print(f"Generated LSPB profile with S(t) values ranging from {sArray[0]} to {sArray[-1]}")
    

    fig, ax = plt.subplots()
    ax.plot(t, sArray, linewidth=0.5, color='blue') # markeredgewidth
    ax.plot(t, sdArray, linewidth=1.0, color='red')
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