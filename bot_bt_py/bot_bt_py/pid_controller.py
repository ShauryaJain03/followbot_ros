#!/usr/bin/env python3
import time


class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(-1.0, 1.0)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
        
    def update(self, error, current_time=None):
        if current_time is None:
            current_time = time.time()
            
        if self.prev_time is None:
            self.prev_time = current_time
            dt = 0.0
        else:
            dt = current_time - self.prev_time
            
        if dt <= 0.0:
            dt = 0.01  
            
        proportional = self.kp * error
        
        self.integral += error * dt
        integral_term = self.ki * self.integral
        
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        derivative_term = self.kd * derivative
        
        output = proportional + integral_term + derivative_term
        
        output = max(min(output, self.output_limits[1]), self.output_limits[0])
        
        self.prev_error = error
        self.prev_time = current_time
        
        return output
    
    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None