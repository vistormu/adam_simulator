import numpy as np

from ....entities import System, Point, Vector, Configuration

a: list[float] = [-1.0, 0.0, 0.24365, 0.21325, 0.0, 0.0, 0.0]
d: list[float] = [-1.0, 0.1519, 0.0, 0.0, 0.11235, 0.08535, 0.0819]
alpha: list[float] = [-1.0, -np.pi/2, 0.0, 0.0, -np.pi/2, np.pi/2, 0.0]
phi: list[int] = [-1, 1, 0, 0, 1, 1, 1]
mu: list[int] = [-1, 1, -1, -1, 1, -1, 0]
base_to: int = 3

MAX_ITERATIONS: int = 20
TOLERANCE: float = 0.01


class Farm:
    @classmethod
    def iterate(cls, systems: list[System], target: System) -> Configuration | None:
        new_configuration: Configuration | None = None

        distance: float = np.linalg.norm(target.position-systems[-1].position).astype(float)
        for iteration in range(1, MAX_ITERATIONS+1):
            # Termination condition
            if distance <= TOLERANCE:
                break

            # Perform iteration
            backward_systems: list[System] = cls.backward(systems, target)
            forward_systems, new_configuration = cls.forward(backward_systems, systems[0])

            # Update variables
            target = System(target.position, forward_systems[-1].x_axis, forward_systems[-1].y_axis, forward_systems[-1].z_axis)
            distance: float = np.linalg.norm(target.position-systems[-1].position).astype(float)

            # if new_configuration[2] > 0.0:
            #     new_configuration[1] = new_configuration[1] + new_configuration[2]
            #     new_configuration[2] = -new_configuration[2]

            # if iteration == MAX_ITERATIONS:
            #     new_configuration = None

        return new_configuration

    @staticmethod
    def backward(systems: list[System], target: System) -> list[System]:
        # Initialize lists
        p: list[Point] = [system.position for system in systems]
        x: list[Vector] = [system.x_axis for system in systems]
        y: list[Vector] = [system.y_axis for system in systems]
        z: list[Vector] = [system.z_axis for system in systems]

        new_p: list[Point] = [systems[0].position] + [target.position]*(len(p)-1)
        new_x: list[Vector] = [systems[0].x_axis] + [target.x_axis]*(len(x)-1)
        new_y: list[Vector] = [systems[0].y_axis] + [target.y_axis]*(len(y)-1)
        new_z: list[Vector] = [systems[0].z_axis] + [target.z_axis]*(len(z)-1)

        for i in reversed(range(0, len(p)-1)):
            new_z[i] = new_y[i+1]*np.sin(alpha[i+1]) + new_z[i+1]*np.cos(alpha[i+1])
            new_p[i] = new_p[i+1] - Point(*new_x[i+1])*a[i+1] - Point(*new_y[i+1])*d[i+1]*np.sin(alpha[i+1]) - Point(*new_z[i+1])*d[i+1]*np.cos(alpha[i+1])

            j: int = i-2 if a[i]+d[i] == 0 else i-1

            v: np.ndarray = np.array(p[j] - new_p[i]) - np.dot(p[j] - new_p[i], new_z[i])*np.array(new_z[i])

            if a[i]+d[i] == 0:
                mu[i] = np.sign(np.dot(v, x[i]))

            new_x[i] = Vector(*((1-phi[i])*mu[i]*v + phi[i]*mu[i]*np.cross(v, new_z[i]))).normalize()
            new_y[i] = Vector(*((1-phi[i])*mu[i]*np.cross(new_z[i], v) + phi[i]*mu[i]*v)).normalize()

        return [System(p_i, x_i, y_i, z_i) for (p_i, x_i, y_i, z_i) in zip(new_p, new_x, new_y, new_z)]

    @ staticmethod
    def forward(systems: list[System], base: System) -> tuple[list[System], Configuration]:
        # Initialize lists
        p: list[Point] = [oriented_point.position for oriented_point in systems]
        x: list[Vector] = [oriented_point.x_axis for oriented_point in systems]
        y: list[Vector] = [oriented_point.y_axis for oriented_point in systems]
        z: list[Vector] = [oriented_point.z_axis for oriented_point in systems]

        theta: list[float] = [0.0]*(len(p))

        new_p: list[Point] = [base.position]*len(p)
        new_x: list[Vector] = [base.x_axis]*len(x)
        new_y: list[Vector] = [base.y_axis]*len(y)
        new_z: list[Vector] = [base.z_axis]*len(z)

        for i in range(1, len(p)):
            u: np.ndarray = np.sign(np.dot(x[i], p[base_to] - new_p[i-1]))*np.array(p[base_to] - new_p[i-1]) if i == 1 and base_to != 0 else np.array(x[i])

            new_x[i] = Vector(*(u-np.dot(u, new_z[i-1])*new_z[i-1])).normalize()

            theta[i] = float(np.arctan2(np.dot(np.cross(new_x[i-1], new_x[i]), new_z[i-1]), np.dot(new_x[i], new_x[i-1])))

            new_p[i] = new_p[i-1] + Point(*new_x[i-1])*a[i]*np.cos(theta[i]) + Point(*new_y[i-1])*a[i]*np.sin(theta[i]) + Point(*new_z[i-1])*d[i]
            new_y[i] = (new_x[i-1]*(-np.cos(alpha[i])*np.sin(theta[i])) + new_y[i-1]*np.cos(alpha[i])*np.cos(theta[i]) + new_z[i-1]*np.sin(alpha[i])).normalize()
            new_z[i] = (new_x[i-1]*np.sin(alpha[i])*np.sin(theta[i]) + new_y[i-1]*(-np.sin(alpha[i])*np.cos(theta[i])) + new_z[i-1]*np.cos(alpha[i])).normalize()

        return [System(p_i, x_i, y_i, z_i) for (p_i, x_i, y_i, z_i) in zip(new_p, new_x, new_y, new_z)], Configuration(*theta[1:])
