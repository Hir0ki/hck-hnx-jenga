from random import randint

class Brain():

    def __init__(self, matrix):
        self._matrix = matrix  
        self._available = matrix.getRemaining()

    def get_next_move(self):
        pass

    def _get_random_move(self):
        return randint(0, self._matrix.shape[0] * self._matrix.shape [1])
