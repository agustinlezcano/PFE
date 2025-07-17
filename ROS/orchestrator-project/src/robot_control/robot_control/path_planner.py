import math

class PathPlanner:
    def __init__(self):
        pass

    def generateLSPBProfile(self, maxSpeed, time, targetPoint):
        """
        Generates a Linear Segment with Parabolic Blends (LSPB) profile.
        :param maxSpeed: Maximum speed
        :param time: Total time for the motion
        :param targetPoint: Target position
        :return: Dictionary containing LSPB profile parameters
        """
        tAcc = min(time / 2, targetPoint / maxSpeed)  # Acceleration time
        vMax = targetPoint / (time - tAcc)  # Adjusted max velocity
        return {
            "tAcc": tAcc,
            "vMax": vMax,
            "targetPoint": targetPoint,
            "time": time
        }

    def determinePointType(self, pointIndex, totalPoints):
        """
        Determines if the point is initial, final, or intermediate.
        :param pointIndex: Index of the current point
        :param totalPoints: Total number of points in the path
        :return: String indicating the type of point
        """
        if pointIndex == 0:
            return "initial"
        elif pointIndex == totalPoints - 1:
            return "final"
        else:
            return "intermediate"

    def generate3DOFTrajectory(self, maxSpeeds, times, targetPoints):
        """
        Generates a point-to-point trajectory for a 3-degree-of-freedom system.
        :param maxSpeeds: List of maximum speeds for each DOF [vx, vy, vz]
        :param times: List of total times for each DOF [tx, ty, tz]
        :param targetPoints: List of target positions for each DOF [px, py, pz]
        :return: List of dictionaries containing LSPB profiles for each DOF
        """
        if len(maxSpeeds) != 3 or len(times) != 3 or len(targetPoints) != 3:
            raise ValueError("maxSpeeds, times, and targetPoints must all have 3 elements for 3 DOF.")

        trajectories = []
        for i in range(3):
            profile = self.generateLSPBProfile(maxSpeeds[i], times[i], targetPoints[i])
            trajectories.append(profile)
        return trajectories

    def saveTrajectoryPoints(self, targetPoints):
        """
        Saves every 3D point obtained from path planning into a matrix.
        :param targetPoints: List of target positions for each DOF [px, py, pz]
        :return: Matrix containing 3D points for the trajectory
        """
        matrix = []
        for point in targetPoints:
            matrix.append([point[0], point[1], point[2]])  # x, y, z target points
        return matrix
