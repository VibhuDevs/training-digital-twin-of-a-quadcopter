"""Small PID with derivative on measurement, output limiting, and conditional integration."""

class PID:
    def __init__(self, kp, ki=0.0, kd=0.0, output_limit=None, integral_limit=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.previous_measurement = None

    def reset(self):
        self.integral = 0.0
        self.previous_measurement = None

    def update(self, setpoint, measurement, dt, feedforward=0.0):
        derivative = 0.0 if self.previous_measurement is None else -(measurement - self.previous_measurement) / max(dt, 1e-6)
        self.previous_measurement = measurement
        candidate = self.integral + (setpoint - measurement) * dt
        if self.integral_limit is not None:
            candidate = max(-self.integral_limit, min(self.integral_limit, candidate))
        output = feedforward + self.kp * (setpoint - measurement) + self.ki * candidate + self.kd * derivative
        clipped = output if self.output_limit is None else max(-self.output_limit, min(self.output_limit, output))
        if clipped == output or (output > clipped and setpoint < measurement) or (output < clipped and setpoint > measurement):
            self.integral = candidate
        return clipped
