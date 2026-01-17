from abc import ABC, abstractmethod

class Controller(ABC):
    @abstractmethod
    def do_control(self):
        pass

    @abstractmethod
    def reset(self):
        pass

class PIDController(Controller):

    def __init__(self):
        pass

    def do_control(self):
        pass

    def reset(self):
        pass

class LQRController(Controller):

    def __init__(self):
        pass

    def do_control(self):
        pass

    def reset(self):
        pass