import numpy as np

class TrajectoryGenerator:
    def __init__(self, dt=0.01, velocity_limit=1.0):
        self.dt = dt
        self.velocity_limit = velocity_limit
        self.current_trajectory = None

        
    def _compute_yaw(self, dx, dy, yaw_mode):
        if yaw_mode == "velocity":
            return np.arctan2(dy, dx)
        elif yaw_mode == "fixed":
            return np.zeros_like(dx)
        elif yaw_mode == "rate":
            return np.linspace(0, len(dx)*self.dt*0.1, len(dx))
        else:
            raise ValueError("Unknown yaw_mode")

    def circle(self, radius=1.0, z=1.0, duration=10.0, yaw_mode="velocity"):
        omega = min(self.velocity_limit / radius, 2 * np.pi / duration)
        timesteps = int(duration / self.dt)
        t = np.linspace(0, duration, timesteps)

        x = radius * np.cos(omega * t)
        y = radius * np.sin(omega * t)
        z = np.ones_like(t) * z

        dx = -radius * omega * np.sin(omega * t)
        dy =  radius * omega * np.cos(omega * t)
        yaw = self._compute_yaw(dx, dy, yaw_mode)

        return np.stack([t, x, y, z, yaw], axis=1)

    def line(self, start, end, duration=5.0, z=1.0, yaw_mode="velocity"):
        start = np.array(start, dtype=float)
        end = np.array(end, dtype=float)
        displacement = end - start
        distance = np.linalg.norm(displacement)

        min_time = distance / self.velocity_limit
        duration = max(duration, min_time)

        timesteps = int(duration / self.dt)
        t = np.linspace(0, duration, timesteps)
        alpha = t / duration

        pos = start[None, :] + alpha[:, None] * displacement
        x, y = pos[:,0], pos[:,1]
        z = np.ones_like(t) * z

        dx = np.gradient(x, self.dt)
        dy = np.gradient(y, self.dt)
        yaw = self._compute_yaw(dx, dy, yaw_mode)

        return np.stack([t, x, y, z, yaw], axis=1)

    def figure_eight(self, radius=1.0, z=1.0, duration=10.0, yaw_mode="velocity"):
        omega = min(self.velocity_limit / radius, 2 * np.pi / duration)
        timesteps = int(duration / self.dt)
        t = np.linspace(0, duration, timesteps)

        x = radius * np.sin(omega * t)
        y = radius * np.sin(2 * omega * t)
        z = np.ones_like(t) * z

        dx = radius * omega * np.cos(omega * t)
        dy = 2 * radius * omega * np.cos(2 * omega * t)
        yaw = self._compute_yaw(dx, dy, yaw_mode)

        return np.stack([t, x, y, z, yaw], axis=1)

    def ellipsoid(self, rx=1.0, ry=0.5, rz=0.3, duration=10.0, yaw_mode="velocity"):
        # Parametrize with spherical-like coords
        omega = min(self.velocity_limit / max(rx, ry, rz), 2 * np.pi / duration)
        timesteps = int(duration / self.dt)
        t = np.linspace(0, duration, timesteps)

        theta = omega * t
        phi = omega * t / 2  # slower change in elevation

        x = rx * np.cos(theta) * np.cos(phi)
        y = ry * np.sin(theta) * np.cos(phi)
        z = rz * np.sin(phi)

        dx = np.gradient(x, self.dt)
        dy = np.gradient(y, self.dt)
        yaw = self._compute_yaw(dx, dy, yaw_mode)

        return np.stack([t, x, y, z, yaw], axis=1)
