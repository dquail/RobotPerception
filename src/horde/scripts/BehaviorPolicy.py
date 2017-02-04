from random import randint

class BehaviorPolicy:
    """
    0 = stay,
    1 = move left
    2 = move right
    """
    def __init__(self):
        self.lastAction = 0

    def policy(self):
        if (randint(0,9) < 5):
            #50 of the time, do the previous action
            return self.lastAction
        else:
            #else do a random action
            self.lastAction = randint(0,2)
            return self.lastAction
