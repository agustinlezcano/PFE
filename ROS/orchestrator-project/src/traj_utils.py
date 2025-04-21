import math


class TrajectoryUtils:
    def __init__(self, trajectoryType):
        """
        Initializes the TrajectoryUtils class with a trajectory.
        :param trajectoryType: List of target positions for each DOF [px, py, pz]
        """
        self.trajectory = trajectoryType
        self.trajectoryMatrix = []


    def generateLSPBProfile(self, maxSpeed, maxAcc, time, accTime, totalTime, distance) -> float:
        """
        Generates a Linear Segment with Parabolic Blends (LSPB) profile.
        :param maxSpeed: Maximum speed     
        :param maxAcc: Maximum acceleration
        :param totalTime: Total time for the motion
        :param time: Current time for the motion
        :param accTime: Acceleration time
        :return: float containing S(t) value [0,1]
        """
        if time < 0:
            raise ValueError("Time must be greater than 0")
        if accTime <= 0 or accTime >= totalTime:
            raise ValueError("Acceleration time must be greater than 0 and less than total time")

        if (not self.__isValidSpeed(maxSpeed, maxAcc, distance)):
            (maxSpeed, maxAcc) = self.__calculateSpeed(distance, maxAcc, totalTime, accTime)
        
        tAcc = min(time / 2, accTime)
        # TODO: check for real S value
        # TODO: check for intermediate time values
        if  (time >= 0 and time <= tAcc):
            s = 0.5 * maxAcc * time ** 2
        elif (time > tAcc and time <= totalTime - tAcc):
            s = maxSpeed * (time - tAcc) + 0.5 * maxAcc * tAcc ** 2
        elif (time > totalTime - tAcc and time <= totalTime):
            tDec = time - (totalTime - tAcc)
            s = maxSpeed * (time - tAcc) + 0.5 * maxAcc * tAcc ** 2 - 0.5 * maxAcc * tDec ** 2
        else:
            raise ValueError("Time is out of range")
        return s
    
    def generateLSPBProfileDerivative(self, maxSpeed, maxAcc, time, accTime, totalTime, distance) -> float:
        """
        Generates a Linear Segment with Parabolic Blends (LSPB) profile derivative.
        :param maxSpeed: Maximum speed     
        :param maxAcc: Maximum acceleration
        :param totalTime: Total time for the motion
        :param time: Current time for the motion
        :param accTime: Acceleration time
        :return: float containing S(t) value [0,1]
        """
        if time < 0:
            raise ValueError("Time must be greater than 0")
        if accTime <= 0 or accTime >= totalTime:
            raise ValueError("Acceleration time must be greater than 0 and less than total time")

        # If the distance is less than twice the needed distance to accelerate,
        # then the speed is recalculated.
        if (not self.__isValidSpeed(maxSpeed, maxAcc, distance)):
            # TODO: warning message with speed
            print("Distance is too short for the given max speed. Recalculating max speed.")
            (maxSpeed, maxAcc) = self.__calculateSpeed(distance, maxAcc, totalTime, accTime)
            print(f"New max speed: {maxSpeed}")

        tAcc = accTime
        
        if (time >= 0 and time <= tAcc):
            s = maxAcc * time
        elif (time > tAcc and time <= totalTime - tAcc):
            s = maxSpeed
        elif (time > totalTime - tAcc and time <= totalTime):
            tDec = time - (totalTime - tAcc)
            s = maxSpeed - maxAcc * tDec
        else:
            raise ValueError("Time is out of range")
        
        return s

    def __isValidSpeed(self, maxSpeed: float, maxAcc: float, distance: float) -> bool:
        """
        Checks if the given speed is valid 
        (given delta position, max speed is possible).
        :param speed: Speed to check
        :return: True if valid, False otherwise
        """
        dx = (maxSpeed**2)/(2 * maxAcc)
        print(f"dx: {dx}, distance: {distance}")
        return (2 * dx <= (distance))
    

    def __calculateSpeed(self, distance: float, maxAcc: float, time: float, timeAcc: float) -> tuple[float, float]:
        """
        Calculates the maximum speed based on the given distance and maximum acceleration.

        This method computes the maximum speed achievable when covering a specified distance
        with a given maximum acceleration. It assumes uniform acceleration and that the 
        distance is split equally for acceleration and deceleration phases.

        :param distance: The total distance to be covered (float). Must be non-zero.
        :param maxAcc: The maximum acceleration (float).
        :return: The calculated maximum speed (float).
        :raises ValueError: If the distance is zero.
        """
        
        # TODO: Check if distance is different from 0 (raise warning)
        # TODO: Check for updated speed and acceleration values to make to 100% of the distance
        # 
        try:
            if distance == 0:
                raise ValueError("Distance must be non-zero")
            # Calculate the maximum speed based on the distance and maximum acceleration
            maxSpeed = distance / time
            maxAccel = maxSpeed / timeAcc
            maxSpeed = math.sqrt(2 * maxAccel * abs((distance)/2))
            
        except ValueError as e:
            print(f"Error: {e}")
            return (0.0, 0.0)
        except TypeError:       
            print("Error: Invalid input types. Distance and acceleration must be numbers.")
            return (0.0, 0.0)
        except ZeroDivisionError:
            print("Error: Division by zero. Distance cannot be zero.")
            return (0.0, 0.0)
        
        return (maxSpeed, maxAccel)
