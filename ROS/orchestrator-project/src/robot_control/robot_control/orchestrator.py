class Orchestrator:
    """
    Orchestrator class implementing the Singleton pattern.
    Manages tasks and commands while maintaining a task history.
    """

    _instance = None  # Singleton

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            print("⚙️ Creating a new instance of Orchestrator")
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        print("🚀 Initializing Orchestrator")
        self.tasks = []
        self._initialized = True

    def __call__(self, command: str):
        print(f"🔧 Executing command: {command}")
        self.tasks.append(command)

    def showTasks(self):
        print("📋 Registered tasks:")
        for i, task in enumerate(self.tasks, 1):
            print(f"{i}. {task}")
