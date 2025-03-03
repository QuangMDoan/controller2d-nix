#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""
import sys
import os
import cutils
import numpy as np

# Script level imports
# for accessing util functions in ./../carla/controller/utils.py

sys.path.append(os.path.abspath(sys.path[0] + '/..'))
from carla.controller import utils

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        desired_speed = 0
        min_idx, _ = utils.find_closest_point(self._waypoints, [self._current_x, self._current_y])
        desired_speed = self._waypoints[min_idx][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def get_history(self):
        return self.vars.pid_terms_history

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        # current robot/vehicle pose (x, y, yaw) 
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        ######################################################
        ######################################################
        # MODULE 7: DECLARE USAGE VARIABLES HERE
        ######################################################
        ######################################################
        """
            Use 'self.vars.create_var(<variable name>, <default value>)'
            to create a persistent variable (not destroyed at each iteration).
            This means that the value can be stored for use in the next
            iteration of the control loop.

            Example: Creation of 've_prev', default value to be 0
            self.vars.create_var('ve_prev', 0.0)

            Example: Setting 've_prev' to be 1.0
            self.vars.ve_prev = 1.0

            Example: Accessing the value from 've_prev' to be used
            throttle_output = 0.5 * self.vars.ve_prev
        """
        self.vars.create_var('ve_prev', 0.0) # previous velocity error for PID control
        self.vars.create_var('ve_integral_prev', 0.0) # accumulated velocity errors from time 0 to time t-1 
        self.vars.create_var('pid_terms_history', [])
        
        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            """
                Controller iteration code block.

                Controller Feedback Variables:
                    x               : Current X position (meters)
                    y               : Current Y position (meters)
                    yaw             : Current yaw pose (radians)
                    v               : Current forward speed (meters per second)
                    t               : Current time (seconds)
                    v_desired       : Current desired speed (meters per second)
                                      (Computed as the speed to track at the
                                      closest waypoint to the vehicle.)
                    waypoints       : Current waypoints to track
                                      (Includes speed to track at each x,y
                                      location.)
                                      Format: [[x0, y0, v0],
                                               [x1, y1, v1],
                                               ...
                                               [xn, yn, vn]]
                                      Example:
                                          waypoints[2][1]: 
                                          Returns the 3rd waypoint's y position

                                          waypoints[5]:
                                          Returns [x5, y5, v5] (6th waypoint)
                
                Controller Output Variables:
                    throttle_output : Throttle output (0 to 1)
                    steer_output    : Steer output (-1.22 rad to 1.22 rad)
                    brake_output    : Brake output (0 to 1)
            """

            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
            # https://drive.google.com/file/d/14X4L41lthW6HZRXBn6ENHGUyQCJbCQsn/view?usp=sharing
            ######################################################
            ######################################################
            """
                Implement a longitudinal controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_prev like a "global variable".
            """
            
            # Change these outputs with the longitudinal controller. Note that
            # brake_output is optional and is not required to pass the
            # assignment, as the car will naturally slow down over time.
            throttle_output = 0
            brake_output    = 0
            
            # Longitunal PID control            
            kp = 2.5
            ki = 0.006
            kd = 0.2

            # Remove dt because dt is a constant: dt = 0.033 for 30 fps
            
            # PID error term, velocity error ve
            velocity_error = v_desired - v  

            # PID I term
            velocity_error_integral = self.vars.ve_integral_prev + abs(velocity_error)

            # PID D term
            error_derivative = velocity_error - self.vars.ve_prev

            # acceleration x_dotdot is based on the difference between desired velocity (from preset waypoints) and current carla speed
            x_dotdot = kp * velocity_error + ki * velocity_error_integral + kd * error_derivative

            # use sigmoid function to bound the acceleration to [-1, 1] and use it as throttle_output
            throttle_output = np.tanh(x_dotdot)
            
            self.vars.pid_terms_history.append([v_desired, v, velocity_error, velocity_error_integral, error_derivative, x_dotdot, throttle_output])
            if throttle_output > 0:
                throttle_output = throttle_output
                brake_output    = 0
            else:
                brake_output    = -throttle_output
                throttle_output = 0
            
            ######################################################
            ######################################################
            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
            # https://drive.google.com/file/d/1j6rJHBH3GFLkEpsuLau9tCMuAMcrthR2/view?usp=sharing
            ######################################################
            ######################################################
            """
                Implement a lateral controller here. Remember that you can
                access the persistent variables declared above here. For
                example, can treat self.vars.v_prev like a "global variable".
            """
            
            # Change the steer output with the lateral controller. 
            steer_output    = 0
            # Compute steering angle delta for pure pursuit controller
            if len(waypoints) > 1:
                robot_pose = [x, y, yaw]
                waypt_counter, _ = utils.find_closest_point(waypoints, robot_pose)
                waypoint_1, waypoint_2, update_trajectory = utils.update_waypoint_trajectory(waypoints, waypt_counter)
                
                # With the desired speed 22 m/s, and 30 fps, Carla drives 0.74 meters per frame 
                # so we use 4 meters lookahead, and limiting the steering by 90 degrees
                if update_trajectory:
                    lookahead_distance = 4
                    line_vec = [waypoint_1, waypoint_2]
                    look_ahead_carrot = utils.next_carrot(line_vec, robot_pose, lookahead_distance)
                    steer_output = utils.calculate_delta(robot_pose, look_ahead_carrot, np.radians(90))

            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_brake(brake_output)        # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)

        ######################################################
        ######################################################
        # MODULE 7: STORE OLD VALUES HERE (ADD MORE IF NECESSARY)
        ######################################################
        ######################################################
        """
            Use this block to store old values (for example, we can store the
            current x, y, and yaw values here using persistent variables for use
            in the next iteration)
        """
        # Store current values to be used in next step
        self.vars.ve_prev = velocity_error 
        self.vars.ve_integral_prev = velocity_error_integral