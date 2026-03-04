from commander.location import Location
from commander.vectors import Vectors

class ActionFrame:
    def __init__(self, location: Location, vector: Vectors, action: int, translation_speed: float = 0.1):
        self.location = location
        self.vector = vector
        self.action = action
        self.translation_speed = translation_speed
