import pygame
import random, math
from shapely import LineString, Point
from lidar import LidarSensor
from pygame.locals import QUIT, KEYDOWN
from environment import Environment
from robot import DifferentialDriveRobot
from localisation import ParticleFilterLocalization
from landmarks import LandmarkHandler
from robot import RobotPose
from ending import BoomAnimation

# Initialize Pygame
pygame.init()

# Set up environment
width, height = 1200, 800
env = Environment(width, height)

#initialize lidar and robot
robot = DifferentialDriveRobot(width/2,height/2,2.6,'thymio_small.png')
# Create a Lidar sensor with 60 beams and a max distance of 500 units
max_lidar_beam_distance = 500
lidar = LidarSensor(num_beams=60, max_distance_cm=500)

#create landmarkhandler
landmark_handler = LandmarkHandler()    

# Create ParticleFilterLocalization instance
particle_filter = ParticleFilterLocalization(num_particles=200, motion_model_stddev=3, environment=env)

#for potential visualization
USE_VISUALIZATION = True
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Robotics Simulation")

#timestep counter in milliseconds
last_time = pygame.time.get_ticks()

#collision notifier, that show you if you run into a wall
boom_animation = BoomAnimation(width // 2, height // 2)
showEnd = False

if __name__ == "__main__":
    # Game loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
        
        #calculate timestep
        time_step = (pygame.time.get_ticks() - last_time)/1000
        last_time = pygame.time.get_ticks()

        #this is the odometry where we use the wheel size and speed to calculate
        #where we approximately end up.
        robot_pose = robot.predict(time_step)

         # Generate Lidar scans - for these exercises, you wil be given these.
        lidar_scans, _intersect_points = lidar.generate_scans(robot_pose, env.get_environment())
        
        #This is what you will use in landmark detection
        landmark_sightings = landmark_handler.find_landmarks(_intersect_points, env.get_environment(), robot_pose)
      
 
        #this is where the particle filtering localisation happens       
        particle_filter.update(time_step, landmark_sightings, robot.getMotorspeeds())

        # this is where we get the estimated position from the particles.
        particle_estimated_pose = particle_filter.get_estimate()

        #mix the update, sense data into one position
        mixed_pose = RobotPose((0.1 * robot_pose.x + 0.9 * particle_estimated_pose.x), \
                               (0.1 * robot_pose.y + 0.9 * particle_estimated_pose.y), \
                               (robot_pose.theta))
                               
        #EXERCISE 6.1: make the robot move and navigate the environment based on our current sensor information and our current map.
        robot.explore_environment(lidar_scans)
        
        if USE_VISUALIZATION:
            screen.fill((0, 0, 0))
            env.draw(screen)
            robot.draw(screen)
            
            lidar.draw(robot_pose, _intersect_points, screen)

            particle_filter.draw(screen, mixed_pose)
            
            
            
            
               # Update the display
            collided = env.checkColision(robot_pose)
            if collided:
                # Draw the animation
                boom_animation.draw(screen)
                ended = boom_animation.update()
                if ended:
                    pygame.quit()
                    
            pygame.display.flip()
            pygame.display.update()

    # Quit Pygame
    pygame.quit()



   
  

