from common.utils import clamp


class JerkSlewLimiter:
    def __init__(self, dt, max_accel, max_decel, max_jerk):
        self.dt = dt
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.max_jerk = max_jerk
        self.velocity = 0
        self.accel = 0

    def update(self, target_velocity):
        desired_accel = (target_velocity - self.velocity) / self.dt

        same_direction = target_velocity * self.velocity >= 0
        increasing_magnitude = abs(target_velocity) > abs(self.velocity)
        accel_limit = self.max_accel if same_direction and increasing_magnitude else self.max_decel

        desired_accel = clamp(desired_accel, -accel_limit, accel_limit)

        max_delta_accel = self.max_jerk * self.dt
        accel_delta = clamp(desired_accel - self.accel, -max_delta_accel, max_delta_accel)
        self.accel += accel_delta

        self.velocity += self.accel * self.dt
        return self.velocity
