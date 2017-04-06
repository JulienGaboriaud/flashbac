import matplotlib
matplotlib.use("Qt5Agg")
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


class System:
    def __init__(self, initposs, initspeeds, masses, masssun,
                 timeevol, Ndt):
        # timeevol = total evolution time, Ndt = number of time steps
        self.planets = []
        self._nplanets = len(initposs)  # number of planets
        self._ndim = len(initposs[0])  # number of dimensions to the problem
        for initpos, initspeed, mass in zip(initposs, initspeeds, masses):
            self.planets.append(Planet(initpos, initspeed, mass))
        # plenet trajectories
        self.trajectories = np.zeros((self._nplanets, self._ndim, Ndt))
        # position zero
        self.trajectories[:, :, 0] = np.array([p.position for p in
                                               self.planets])

    def show_trajectories(self, timestepmax=None):
        if self._ndim != 2:
            print("Cannot show other than 2 dimensions")
            return
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title("Orbites planétaires")
        ax.set_xlabel("Position X")
        ax.set_ylabel("Position Y")
        for p in self.trajectories:
            # here p = array of positions in function of time
            x = p[0, :timestepmax]
            y = p[1, :timestepmax]
            ax.plot(x, y)
        plt.show()
        plt.close(fig)



if __name__ == "__main__":
    s = System([[0, 1], [2, 0]], [[1, 0], [0, -0.5]], [1, 2], 10, 1, 10)
    s.show_trajectories()
