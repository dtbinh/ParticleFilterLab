#!/usr/bin/env python
import math


class PDController:
    def __init__(self, kp, kd, min_output, max_output):
        self.kp = kp
        self.kd = kd
        self.minOutput = min_output
        self.maxOutput = max_output
        self.previousError = 0.0
        self.previousTime = None

    def update(self, value, target_value, time):
        error = target_value - value
        p = self.kp * error
        d = 0
        if self.previousTime is not None:
            dt = time - self.previousTime
            if dt > 0:
                d = self.kd * (error - self.previousError) / dt
        output = p + d
        self.previousTime = time
        self.previousError = error
        return max(min(output, self.maxOutput), self.minOutput)
