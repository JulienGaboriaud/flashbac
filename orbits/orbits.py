import matplotlib.pyplot as plt
import numpy as np


class Planet:
    def __init__(self, initpos, initspeed, mass):
        self.position = initpos
        self.speed = initspeed
        self.mass = mass

    def advance(self, force, time):
        acceleration = force / self.mass
        self.speed += acceleration * time
        self.position += acceleration * time ** 2 / 2 + self.speed * time
