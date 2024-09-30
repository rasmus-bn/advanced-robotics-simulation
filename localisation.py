import numpy as np
from shapely.ops import nearest_points
from shapely.geometry import Point, LineString
import random
import math
import pygame
from robot import RobotPose
from copy import deepcopy
import cv2

class ParticleFilterLocalization:
    def __init__(self, num_particles, motion_model_stddev, environment):
        self.num_particles = num_particles
        self.motion_model_stddev = motion_model_stddev
        self.environment = environment
        self.particles = []
   
        self.DRAW_PARTICLES = True
        self.DRAW_ROBOT = False
        # Initialize particles
        self._initialize_particles(environment.width, environment.height)

        #this is just for potential visualization, REMEMBER VISUALIZATION IS NOT THE SIMULATION
        self.image = pygame.image.load('thymio_small.png')
        self.rect = self.image.get_rect()

    def _initialize_particles(self, map_width, map_height):
        #insert your code here
        pass
    
    def _add_noise_to_pose(self, pose):
        """
        Add a small amount of noise to the particle's pose.

        Parameters:
            - pose (Point): The particle's current pose.

        Returns:
            - Point: Updated particle pose with noise.
        """
        noise_x = random.gauss(0, self.motion_model_stddev)
        noise_y = random.gauss(0, self.motion_model_stddev)
        noise_theta = random.gauss(0, self.motion_model_stddev)
        # return Particle(pose.x, pose.y, pose.theta)
        return Particle(pose.x + noise_x, pose.y + noise_y, pose.theta + noise_theta)

    def _measurement_update(self, landmark_sightings):
        #Insert your code here
        pass

    
    def resample_particles(self):
        #insert your code here
        pass

    #This one is given to the students.
    def update(self, delta_time, landmark_sightings, current_motor_speeds):
        # Update particles based on motion model, given to the students.
        self.particles = self._motion_model(self.particles, current_motor_speeds, delta_time)

        # Calculate weights based on measurement model
        self._measurement_update(landmark_sightings)
       
        # Resample particles based on weights
        self.resample_particles()

        # Reset weights to uniform after resampling
        self.weights = np.ones(self.num_particles) / self.num_particles


    def get_estimate(self):
        #insert your code here
        return RobotPose(0, 0, 0)

     #This one is the exact same as the one used in the robot. 
    def _motion_model(self, particles, motor_speeds, delta_time):
        L = 10 #axle distance in cm
        wheel_radius = 2.2 #wheel radius in cm 
        for i in range(len(self.particles)):
            particle = particles[i]
            # Assume maximum linear velocity at motor speed 500
            v_max = 10  # pixels/second
            axl_dist = 5
            # Calculate the linear velocity of each wheel
            left_motor_speed, right_motor_speed = motor_speeds
            left_wheel_velocity = (left_motor_speed / 500) * v_max
            right_wheel_velocity = (right_motor_speed / 500) * v_max
            
            v_x = math.cos(particle.theta) * (wheel_radius * (left_wheel_velocity + right_wheel_velocity) / 2)
            v_y = math.sin(particle.theta) * (wheel_radius * (left_wheel_velocity + right_wheel_velocity) / 2)
            omega = (wheel_radius * (left_wheel_velocity - right_wheel_velocity)) / (2 * axl_dist)
            
            particle.x += v_x * delta_time
            particle.y += v_y * delta_time
            particle.theta += omega * delta_time

            # Ensure the orientation is within the range [0, 2*pi)
            particle.theta = particle.theta % (2 * math.pi)

            particles[i] = particle
        return particles
    
    def draw(self, surface, robot_pose):
        if self.DRAW_ROBOT:
            rotated_image = pygame.transform.rotate(self.image, math.degrees(-1*robot_pose.theta))
            self.rect.center = (int(robot_pose.x), int(robot_pose.y))
            new_rect = rotated_image.get_rect(center=self.rect.center)
            surface.blit(rotated_image, new_rect)
      
        if self.DRAW_PARTICLES:
            for i in range(len(self.particles)):
                particle = self.particles[i]
                weight =  self.weights[i]
                pygame.draw.circle(surface, (0,100,100),(particle.x,particle.y),5,2)
  
        
class Particle:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta



   
