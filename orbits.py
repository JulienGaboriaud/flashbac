#import matplotlib
#matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


G = 1.15e-4  # in units of earth mass, astronomical units and years


class Planet:
    def __init__(self, initpos, initspeed, mass):
        self.position = np.array(initpos, dtype=float)
        self.speed = np.array(initspeed, dtype=float)
        self.mass = mass

    def advance(self, force, time):
        acceleration = np.array(force, dtype=float) / self.mass
        self.speed += acceleration * time
        self.position += acceleration * time ** 2 / 2 + self.speed * time

    def __eq__(self, planet):
        posequal = (self.position == planet.position).all
        speedequal = (self.speed == planet.speed).all
        massequal = self.mass == planet.mass
        if posequal and speedequal and massequal:
            return True
        return False


if __name__ == "__main__":
    # create sun (I AM GOD!)
    sun = Planet([0, 0], [5, 0], 1)
    pos = [sun.position.copy(), ]
    for i in range(10):
        sun.advance([0, 5], 1)
        pos.append(sun.position.copy())
    pos = np.array(pos)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlabel("Position X (UA)")
    ax.set_ylabel("Position Y (UA)")
    ax.set_title("Position soleil")
    ax.plot(pos[:, 0], pos[:, 1])
    plt.show()
    plt.close(fig)


"""
class System:
    def __init__(self, initposs, masses, masssun,
                 timeevol, Ndt):
        # timeevol = total evolution time, Ndt = number of time steps
        self.planets = []
        self.dt = timeevol / Ndt
        self.Ndt = Ndt
        self._instant_time = 0
        self._nplanets = len(initposs) + 1  # number of planets, +1 because sun
        self._ndim = len(initposs[0])  # number of dimensions to the problem
        if self._ndim != 2:
            raise NotImplementedError("ndim != 2 not implemented...")
        for initpos, mass in zip(initposs, masses):
            initspeed = self._get_planet_speed(initpos, masssun)
            self.planets.append(Planet(initpos, initspeed, mass))

        # sun is like a planet that start at center
        self.planets.append(Planet([0, 0], [0, 0], masssun))
        # plenet trajectories
        self.trajectories = np.zeros((self._nplanets, self._ndim, Ndt))
        # position zero
        self.trajectories[:, :, 0] = np.array([p.position for p in
                                               self.planets])

    def _get_planet_speed(self, initpos, masssun):
        if not initpos[0]:
            # x0 = 0 => theta = pi / 2
            theta = np.pi / 2
        else:
            theta = np.arctan(initpos[1] / initpos[0])
        r0 = np.linalg.norm(initpos)
        v0 = np.sqrt(G * masssun / r0)  # circular orbit
        initspeed = [-v0 * np.sin(theta), v0 * np.cos(theta)]  # anticlockwise
        return initspeed

    def to_the_end(self):
        for i in range(self.Ndt - 1):
            self.advance_one_step()

    def advance_one_step(self):
        for planet in self.planets:
            force = self.force_on(planet)
            planet.advance(force, self.dt)

        # advance time
        self._instant_time += 1
        newtraj = np.array([p.position for p in self.planets])
        self.trajectories[:, :, self._instant_time] = newtraj

    def force_on(self, planet):
        # total force on planet
        F = np.zeros(self._ndim)
        for p in self.planets:
            if p == planet:
                # don't count force on itself!
                continue
            F += self.force_on_by(planet, p)
        return F

    def force_on_by(self, p1, p2):
        # return force on p1 by p2
        pos1 = p1.position
        pos2 = p2.position
        m1 = p1.mass
        m2 = p2.mass
        deltapos = pos1 - pos2
        F = -m1 * m2 * G / (np.linalg.norm(deltapos) ** 3) * deltapos
        return F

    def show_trajectories(self, timestepmax=None):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.set_title("Orbites planétaires")
        ax.set_xlabel("Position X (UA)")
        ax.set_ylabel("Position Y (UA)")
        ax.axhline(0.0, linestyle=":", color="k")
        ax.axvline(0.0, linestyle=":", color="k")
        for p in self.trajectories:
            # here p = array of positions in function of time
            x = p[0, :timestepmax]
            y = p[1, :timestepmax]
            ax.plot(x, y)
        plt.show()
        plt.close(fig)


if __name__ == "__main__":
    tottime = 2
    dt = tottime / 10000
    Ndt = int(tottime / dt)
    initpos = [[0.38, 0],  # mercury
               [0, 0.72],  # venus
               [-1, 0],  # earth
               [0, -1.52],  # mars
               [5.20, 0],  # jupiter
               ]
    masses = [0.055,
              0.815,
              1,
              0.107,
              317.8]
    masssun = 330000  # in earth masses
    s = System(initpos, masses, masssun, tottime, Ndt)
    s.to_the_end()
    s.show_trajectories()
"""
