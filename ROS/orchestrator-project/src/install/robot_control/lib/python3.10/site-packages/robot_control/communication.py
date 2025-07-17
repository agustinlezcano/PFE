class Communication:
    def __init__(self, message: str = ""):
        """
        Initialize the Communication class with an optional message.
        """
        self.message = message

    def sendMessage(self, message: str) -> None:
        """
        Simulate sending a message.
        :param message: The message to send.
        """
        self.message = message
        print(f"Message sent: {self.message}")

    def receiveMessage(self) -> str:
        """
        Simulate receiving a message.
        :return: The received message.
        """
        print(f"Message received: {self.message}")
        return self.message
    
    def createDataStructure(self, **kwargs) -> str:
        """
        Create a data structure from the given data.
        :param data: The data to structure.
        :return: A string representation of the structured data.
        """
        s = kwargs.get('s', '0.0000')
        sd = kwargs.get('sd', '0.0000')
        sdd = kwargs.get('sdd', '0.0000')

        # Truncate the values to 4 decimal places
        s = self.__truncate(self.__round(s,8), 4)
        sd = self.__truncate(self.__round(sd,8), 4)
        sdd = self.__truncate(self.__round(sdd,8), 4)

        # Create the structured data string
        structured_data = f":T//P{s}/V{sd}/A{sdd}\n"
        print(structured_data)
        return structured_data

    def __truncate(self, value: float, n: int) -> str:
        """
        Truncate a float to a specified number of decimal places.
        :param value: The float to truncate.
        :param decimal_places: The number of decimal places to keep.
        :return: The truncated float.
        """
        # '%.3f'%(value)  # '{0:.{1}f}'.format(f, n)
        return '{0:.{1}f}'.format(value, n)
        
    
    def __round(self, value: float, decimal: int) -> float:
        """
        Round a float to a specified number of decimal places.
        :param value: The float to round.
        :param decimal: The number of decimal places to keep.
        :return: The rounded float.
        """
        factor = 10 ** decimal
        return round(value * factor) / factor