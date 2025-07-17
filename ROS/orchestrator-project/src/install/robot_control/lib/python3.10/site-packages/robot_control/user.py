class User:
    def __init__(self, stock: dict, ordersQuantity: int, orders: dict):
        """
        Initialize the User class with stock, order quantity, and orders.
        """
        self.stock = stock
        self.ordersQuantity = ordersQuantity
        self.orders = orders

    def initGUI(self) -> None:
        pass

    def displayStock(self) -> None:
        pass

    def isOrderCorrect(self) -> bool:
        pass

    def isOrderDelivered(self) -> bool:
        pass

    def addOrder(self, order: dict) -> None:
        pass

    def deleteOrder(self, orderId: str) -> None:
        pass