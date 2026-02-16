

class TrajectoryUtils:
    
    def __init__(self, trajectoryType):
        """
        Initializes the TrajectoryUtils class with a trajectory.
        :param trajectoryType: List of target positions for each DOF [px, py, pz]
        """
        self.trajectory = trajectoryType
        self.trajectoryMatrix = []
        self.maxSpeed = None
        self.maxAcc = None
        self.timeAcc = None
        self.distance = None
        # optional vars
        self.setPointSpeed = None
        self.setPointAcc = None
        self.setPointTime = None
        self.setPointDistance = None
        self.setPointTimeAcc = None
        
     
    def generateLSPBProfile(self, maxSpeed, maxAcc, time, timeAcc, totalTime, distance) -> tuple[float, float, float]:
        """
        Generates a Linear Segment with Parabolic Blends (LSPB) profile.
        :param maxSpeed: Maximum speed     
        :param maxAcc: Maximum acceleration
        :param totalTime: Total time for the motion
        :param time: Current time for the motion
        :param accTime: Acceleration time
        :return: float containing S(t) value [0,1]
        """

        #(distance, maxSpeed, maxAcc) = self.__factorizeDistance(distance, maxSpeed, maxAcc)

        if time < 0:
            raise ValueError("Time must be greater than 0")
        if timeAcc <= 0 or timeAcc >= totalTime:
            raise ValueError("Acceleration time must be greater than 0 and less than total time")

        if (not self.__isValidSpeed(maxSpeed, maxAcc, distance)):
            distance = 1.0
            (maxSpeed, timeAcc, maxAcc) = self.__calculateSpeed(distance, maxAcc, totalTime, timeAcc)
            #print(f"New max speed: {maxSpeed}")
        else:
            timeAcc = timeAcc

        tAcc = min(totalTime / 2, timeAcc)

        if  (time >= 0 and time <= tAcc):
            s = 0.5 * maxAcc * time ** 2
            sd = maxAcc * time
            sdd = maxAcc
        elif (time > tAcc and time <= totalTime - tAcc):
            s = maxSpeed * (time - tAcc) + 0.5 * maxAcc * tAcc ** 2
            sd = maxSpeed
            sdd = 0
        elif ((time > totalTime - tAcc) and time <= totalTime):
            tDec = time - (totalTime - tAcc)
            s = maxSpeed * (time - tAcc) + 0.5 * maxAcc * tAcc ** 2 - 0.5 * maxAcc * tDec ** 2
            sd = maxSpeed - maxAcc * tDec
            sdd = - maxAcc
        else:
            raise ValueError("Time is out of range")
        return s, sd, sdd
    

    def __isValidSpeed(self, maxSpeed: float, maxAcc: float, distance: float) -> bool:
        """
        Checks if the given speed is valid 
        (given delta position, max speed is possible).
        :param speed: Speed to check
        :return: True if valid, False otherwise
        """
        dx = (maxSpeed**2)/(2 * maxAcc)
        #print(f"dx: {dx}, distance: {distance}")
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
        
        # TODO: Check for updated speed and acceleration values to make to 100% of the distance
        # TODO: includes time to equations (timeAcc)
        try:
            if distance == 0:
                raise ValueError("Distance must be non-zero")
            # Calculate the maximum speed based on the distance and maximum acceleration
            maxSpeed = distance / (time - timeAcc)
            maxAcc = maxSpeed / timeAcc # I could not reach this speed in timeAcc
            
            if timeAcc > time:
                raise ValueError("Time to reach max speed is greater than total time")
            #ACTUALIZAR LA VELOCIDAD MAXIMA. DEBE SER MAYOR QUE LA ANTERIOR PARA CUBRIR EL MISMO ESPACIO PERO ACELERANDO
            # QUE V Y A SE MANTENGAN DENTRO DE LIMITES. TENGOB DISTANCIAS DE TRAMOS CON A, SACAR V DE TRAMO V=CTE (Y T)
            #X1 = 0.5 * maxAcc * timeAcc ** 2 # distance covered during acceleration
            #X2 = maxSpeed * (time - 2 * timeAcc) # distance covered during constant speed
            #X3 = 0.5 * maxAcc * timeAcc ** 2 # distance covered during deceleration
            #maxSpeed = (1 - X1 - X3) / (time - 2 * timeAcc) # distance covered during constant speed
            
            # maxAccel = maxSpeed / timeAcc # I could not reach this speed in timeAcc
            #maxSpeed = maxAcc * timeAcc # max speed is equal to max acceleration times time to reach max speed
            #s = 0.5 * maxAcc * timeAcc ** 2 + maxSpeed * (time - 2 * timeAcc) + 0.5 * maxAcc * timeAcc **2 # distance covered during acceleration
            #maxSpeed = (1 - (maxAcc * timeAcc ** 2)) / (time - 2 * timeAcc) # distance covered during deceleration

            if (not self.__isTimeAccFeaseable(timeAcc, maxSpeed, maxAcc)):
                # raise ValueError("Time to accelerate is not feasible")
                timeAcc = maxSpeed / maxAcc # time to reach max speed
                #print(f"New time to accelerate: {timeAcc}")

        except ValueError as e:
            print(f"Error: {e}")
            return (0.0, 0.0)
        except TypeError:       
            print("Error: Invalid input types. Distance and acceleration must be numbers.")
            return (0.0, 0.0)
        except ZeroDivisionError:
            print("Error: Division by zero. Distance cannot be zero.")
            return (0.0, 0.0)
        
        
        return (maxSpeed, timeAcc, maxAcc)
    
    def __isTimeAccFeaseable(self, timeAcc: float, maxSpeed: float, maxAcc: float) -> bool:
        """
        Checks if the given time to accelerate is feasible based on the maximum speed and acceleration.
        :param timeAcc: Time to accelerate
        :param maxSpeed: Maximum speed
        :param maxAcc: Maximum acceleration
        :return: True if feasible, False otherwise
        """
        return (maxSpeed / maxAcc <= timeAcc)
    
    def setMaxParams(self, maxSpeed: float, maxAcc: float, timeAcc: float, distance: float) -> None:
        """
        Sets the parameters for the trajectory.
        :param maxSpeed: Maximum speed
        :param maxAcc: Maximum acceleration
        :param timeAcc: Time to accelerate
        :param distance: Distance to cover
        """
        self.maxSpeed = maxSpeed
        self.maxAcc = maxAcc
        self.timeAcc = timeAcc
        self.distance = distance

    def validateParams(self, v: float, a: float) -> bool:
        """
        Validates the parameters for the trajectory.
        :raises ValueError: If any of the parameters are invalid
        """
        return (v <= self.maxSpeed) and (a <= self.maxAcc)

    def calculateParams(self, timeAccel: float, time: float) -> tuple[float, float]:
        """
        Calculates the parameters for the trajectory.
        :param timeAccel: Time to accelerate
        :param time: Total time for the motion before decceleration
        :return: Tuple containing the calculated parameters
        """
        v = 1 / time
        a = 1 / (time * timeAccel)
        return (v,a)
    
    def calculateTimeFromSpeedAndDistance(self, maxSpeed: float, maxAcc: float, distance: float) -> tuple[float, float]:
        accTime = maxSpeed / maxAcc  # Time to reach max speed
        accDistance = 0.5 * maxAcc * accTime**2  # Distance covered during acceleration

        if accDistance * 2 > distance:
            # If the distance is too short to reach max speed, adjust accTime
            accTime = (distance / (2 * maxAcc))**0.5
            totalTime = 2 * accTime
        else:
            # Otherwise, calculate total time with constant speed phase
            cruiseDistance = distance - 2 * accDistance
            cruiseTime = cruiseDistance / maxSpeed
            totalTime = 2 * accTime + cruiseTime

        return totalTime, accTime
    
    def setParams(self, **kwargs) -> None:
        """
        Sets the parameters for the trajectory.
        :param kwargs: Dictionary containing the parameters
        """
        self.setPointSpeed = kwargs.get("maxSpeed", 1.0)
        self.setPointAcc = kwargs.get("maxAcc", 2.0)
        self.setPointTime = kwargs.get("time", None)
        self.setPointDistance = kwargs.get("distance", 1.0)
        self.setPointTimeAcc = kwargs.get("timeAcc", None)

    @staticmethod
    def interpolate(s: float, thetaIni: float, thetaFin: float) -> float:
        """
        Interpolates the trajectory based on the given parameters.
        :param s: S
        :param thetaIni: Initial Position
        :param thetaFin: Final Position
        :return: Tuple containing the interpolated values
        :raises: ValueError: If s is not between 0 and 1
        
        thetaIni - thetaIni * s + thetaFin * s = theta
        theta = thetaIni + (thetaFin - thetaIni) * s
        """
        # Aca sería ángulo o posición. Si la cinemática se define en el robot, pasar posición
        # Tener en cuenta consigna de homing (paso ángulos)
        if s < 0 or s > 1:
            raise ValueError("s must be between 0 and 1")
        
        theta = (1 - s) * thetaIni + s * thetaFin
        return theta
    
    
