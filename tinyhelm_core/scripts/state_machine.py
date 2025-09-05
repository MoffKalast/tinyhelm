from enum import Enum, auto

class HelmState(Enum):
    IDLE = auto()
    NAVIGATING = auto()
    HOLD_POSITION = auto()

class StateMachine:
    def __init__(self):
        self.state = HelmState.IDLE

    def set_state(self, state: HelmState):
        self.state = state

    def get_state(self):
        return self.state

    def is_active(self):
        return self.state != HelmState.IDLE