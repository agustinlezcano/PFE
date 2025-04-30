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