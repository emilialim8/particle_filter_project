#!/usr/bin/env python3

from cmath import pi
import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion

import numpy as np
from numpy.random import random_sample
import math
import astropy 

from likelihood_field import LikelihoodField

from random import randint, random, uniform



def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
            p.orientation.x,
            p.orientation.y,
            p.orientation.z,
            p.orientation.w])
            [2])

    return yaw

def compute_prob_zero_centered_gaussian(dist, sd):
    # from class exercise
    """ Takes in distance from zero (dist) and standard deviation (sd) for gaussian
        and returns probability (likelihood) of observation """
    c = 1.0 / (sd * math.sqrt(2 * math.pi))
    prob = c * math.exp((-math.pow(dist,2))/(2 * math.pow(sd, 2)))
    return prob

def draw_random_sample(n, choices, probs):
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.

    n- number of elements to draw
    choices - the choices to draw from
    probs - their relative probabilties

    returns the random sample
    tested and works
    """
    sample = np.random.choice(choices, size = n, replace = True, p = probs)
    return sample


def normalize_radian(rad):
    """Recursive function to normalize a radian so
    it is between (-pi, pi]
    
    rad - an angle in radians"""
    pi2 = 2 * np.pi
    
    #if angle is too big, subtract 2pi
    if rad > np.pi:
        rad -= pi2
        rad = normalize_radian(rad)
    
    #if angle is too small, add 2pi
    if rad <= (-1 * np.pi):
        rad += pi2
        rad = normalize_radian(rad)
    return(rad)

"""
for testing draw_random_sample
samp_array = ["a", "b", "c", "d"]
samp_weight = [.5, .25, .25, 0]
ret = draw_random_sample(20, samp_array, samp_weight)
print(ret)
"""

class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map and likelihood field
        self.map = OccupancyGrid()
        print("map intialized")
        self.likelihood_field = LikelihoodField()
        print("map and field intialized")

        # the number of particles used in the particle filter
        self.num_particles = 10
        #they have it at 10000, TODO change back

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()


        # intialize the particle cloud
        self.initialize_particle_cloud()
        print("particle cloud intialized")
        '''
        for testing particle cloud intialization
        
        for i in range(10):
            print(self.particle_cloud[i].pose.position)
            print(self.particle_cloud[i].pose.orientation)
        '''

        self.initialized = True



    def get_map(self, data):

        self.map = data
    

    def initialize_particle_cloud(self):
        
        #min and max coordinates from map
        #max_width = self.map.info.width
        #max_height = self.map.info.height
        max_width = 100
        max_height = 50

        #iterate through number of particles to 
        #randomly intialize each particle
        for i in range(self.num_particles):
            this_point = Point()

            #random x in map
            this_point.x = randint(0, max_width - 1)

            #random y in map
            this_point.y = randint(0, max_height - 1)
            this_point.z = 0 #2D

            #yaw is the angle between -pi and pi 
            yaw = uniform(-np.pi, np.pi)

            #intialize quanternion with converted yaw, yay math
            this_quant = Quaternion(*quaternion_from_euler(0,0,yaw))

            #create pose from position and quanternion
            this_pose = Pose(position = this_point, orientation = this_quant)

            #create particle from pose and uniform weight
            this_part = Particle(pose = this_pose, w = 1)

            #add to array
            self.particle_cloud.append(this_part)


        self.normalize_particles()

        self.publish_particle_cloud()


    def normalize_particles(self):
        
        norm = 1 / sum(part.w for part in self.particle_cloud)
        #print(norm)
        for part in self.particle_cloud:
            part.w = norm * part.w
            #print(part.w)
        
        return
        



    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses

        for part in self.particle_cloud:
            particle_cloud_pose_array.poses.append(part.pose)

        self.particles_pub.publish(particle_cloud_pose_array)


    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):
        
        #TODO test this code
        
        #create array of weights
        weights = [part.w for part in self.particle_cloud]

        #draw a random sample of num_particle particles from the particle cloud
        #where probability are the weights
        self.particle_cloud = draw_random_sample(self.num_particles,
                self.particle_cloud, weights)
        return 



    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:

            print("main robot time")

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            x_move = np.abs(curr_x - old_x)
            y_move = np.abs(curr_y - old_y)
            yaw_move = np.abs(curr_yaw - old_yaw)

            if (x_move > self.lin_mvmt_threshold or 
                y_move > self.lin_mvmt_threshold or
                yaw_move > self.ang_mvmt_threshold):

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model(x_move, y_move, yaw_move)

                self.update_particle_weights_with_measurement_model(data)

                self.normalize_particles()

                self.resample_particles()

                self.update_estimated_robot_pose()

                self.publish_particle_cloud()
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        
        #for now, make this a weighted average of all the particles poses
        #create array of weights

        sum_x = 0
        sum_y = 0
        yaw_array = []
        weight_array = []

        #loop through patricle cloud
        #getting weighted sum of x and y
        #and an array of the yaws and weights
        for part in self.particle_cloud:
            sum_x += part.pose.position.x * part.w
            sum_y += part.pose.position.y * part.w
            yaw_array.append(get_yaw_from_pose(part.pose))
            weight_array.append(part.w)
        
        #divide, this works because the weights are normalized
        x_mean = sum_x / self.num_particles
        y_mean = sum_y / self.num_particles

        #to aggregate yaw, need weighted circular mean
        yaw_mean = astropy.stats.circmean(yaw_array, weights = weight_array)
        #quanternion with converted yaw, yay math
        self.robot_estimate.orientation = Quaternion(
            *quaternion_from_euler(0,0,yaw_mean))
            #can you just make a new line like this in python
        self.robot_estimate.position.x = x_mean
        self.robot_estimate.position.y = y_mean
        return

    
    def update_particle_weights_with_measurement_model(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        #       Compute the importance weights (w) for the particles 
        #       in this environment using the likelihood field measurement
        #       algorithm. 
    
        #loop through particle cloud
        for part in self.particle_cloud:
            q = 1 
            print(part)
            for k in range(0,359): #for all 360 degrees
                #z is the measurement at range k
                z = data.ranges[k]
                print(k)
                #theta is the particles current yaw
                theta = euler_from_quaternion([
                    part.pose.orientation.x, 
                    part.pose.orientation.y, 
                    part.pose.orientation.z, 
                    part.pose.orientation.w])[2]
                if z == 0:
                    z = 3.5 
                if z != 0: #TODO redunant code, ask Emilia
                    #translate and rotate minimum values to x and y of particle
                    x = part.pose.position.x + z * np.cos(theta + k*np.pi/180)
                    y = part.pose.position.y + z * np.sin(theta + k*np.pi/180)

                    #calculate the mimimum distance at each end point using helper function
                    dist = self.likelihood_field.get_closest_obstacle_distance(x,y)

                    #TODO incorporate random and miss z probabilities
                    sd_scan = 0.1
                    print(compute_prob_zero_centered_gaussian(dist, sd_scan))
                    q = q * compute_prob_zero_centered_gaussian(dist, sd_scan)
                    print(q)
            part.w = q
        return

        

    def update_particles_with_motion_model(self, x_move, y_move, yaw_move):

        #set standard deviations for measurement noise from odometry
        #generate gaussians centered on movements
        sd_xy = 0.1
        sd_yaw = np.pi/180 #about 1 degree
       
        for part in self.particle_cloud:

            #Do we need error checking for the edge of the map?
            part.pose.position.x += np.random.normal(x_move, sd_xy)
            part.pose.position.y += np.random.normal(y_move, sd_xy)

            part_yaw = get_yaw_from_pose(part.pose)

            #if yaw falls outside of (-pi, pi], renormalize back inside range
            part_yaw += normalize_radian(np.random.normal(yaw_move, sd_yaw))
            part.pose.orientation = Quaternion(*quaternion_from_euler(0,0,part_yaw))
        return


if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()

    









