import math
import numpy as np

def update_particles_with_motion_model(x_move, y_move, yaw_move, yaw_original):

    #set standard deviations for measurement noise from odometry
    #generate gaussians centered on movements
    #sd_xy = 0.1
    #sd_yaw = np.pi/180 #about 1 degree
    
    rot1 = np.arctan2(y_move, x_move) - yaw_original
    trans = math.sqrt((x_move)**2 + (y_move)**2)
    rot2 = yaw_move - rot1

    print(rot1)
    print(trans)
    print(rot2)

    return

        


xo = 0
yo = 0
theta = 0

x1 = math.sqrt(3)
y1 = 1
theta1 = 40 * (np.pi/180)

update_particles_with_motion_model(x1, y1, theta1, theta)