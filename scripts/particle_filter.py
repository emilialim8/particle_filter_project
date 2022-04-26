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
from scipy.stats import circmean 

from likelihood_field import LikelihoodField

from random import randint, random, uniform, choices


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

'''
#for testing draw_random_sample
samp_array = ["a", "b", "c", "d"]
samp_weight = [.5, .25, .25, 0]
for i in range(4):
    ret = draw_random_sample(20, samp_array, samp_weight)   
    print(ret)
'''

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


        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # inialize our map and likelihood field
        self.map = OccupancyGrid()
        print("map intialized")
        
        self.likelihood_field = LikelihoodField()
        print("map and field intialized")


        # the number of particles used in the particle filter
        self.num_particles = 10000
      
        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.1        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None

        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()


        print("width: " + str(self.map.info.width) + ", height: " + str(self.map.info.width) )
        print("origin " + str(self.map.info.origin.position.x) + ", " +str(self.map.info.origin.position.y))
        
        self.grid = np.array(self.map.data)
        print("the gird is")
        print(self.grid)
        
        # intialize the particle cloud
        self.initialize_particle_cloud()
        print("particle cloud intialized")
        
        
        #for testing particle cloud intialization
        r = rospy.Rate(1)
        r.sleep()
        #publish to make sure robot has time to publish
        self.publish_particle_cloud()

        '''
        for i in range(10):
            print(self.particle_cloud[i].pose.position)
            print(self.particle_cloud[i].pose.orientation)
            r.sleep()
            #publish to make sure robot has time to publish
            self.publish_particle_cloud()
        '''

        self.initialized = True



    def get_map(self, data):

        self.map = data

        '''
        for debugging map info
        print("width: " + str(self.map.info.width) + ", height: " + str(self.map.info.width) )
        print("origin " + str(self.map.info.origin.position.x) + ", " +str(self.map.info.origin.position.y))
        '''
    


    def initialize_particle_cloud(self):
 
        #min and max coordinates from map
        res = self.map.info.resolution
        max_width = self.map.info.width
        max_height = self.map.info.height
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        #max_width = 10
        #max_height = 10

        on_map = [] #array for indexes of occupancy grid that are part of maze
        for i in range(len(self.grid)): #loop through entire occupancy grid
            if (self.grid[i] >= 0):
                on_map.append(i) #add indexes that are known
        #print(on_map)
        #iterate through number of particles to 
        #randomly intialize each particle
        for i in range(self.num_particles):
            this_point = Point()

            #random x in map
            #this_point.x = (randint(0, 60) * res + origin_x) 
            #these values are hardcoded by examining the boundaries of the map in RVIZ
            this_index = choices(on_map)[0] #choose a random point on the map
            
            #convert indexes to positions
            this_point.x = (this_index % max_width) * res + origin_x
            this_point.y = int(this_index / max_width) * res + origin_y
            #random y in map
            #this_point.y = (randint(0, 60) * res + origin_y) 
            this_point.z = 0 #2D
            #this_point.x = 0
            #this_point.y = 0

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
        
        #occupancy grid information
        res = self.map.info.resolution
        max_width = self.map.info.width
        #max_height = self.map.info.height
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        grid = self.grid

        weight_sum = 0
        for part in self.particle_cloud:
            

            #convert particle poses from meters to pixels
            pixel_x = int((part.pose.position.x - origin_x) / res)
            pixel_y = int((part.pose.position.y - origin_y) / res)
            index = pixel_x + pixel_y * max_width
            if(grid[index] < 0):
                part.w = 0
            weight_sum += part.w

        norm = 1 / weight_sum
 
        
        for part in self.particle_cloud:
            part.w = norm * part.w
            #print(part.w)
        #print("sum of weights after normalized:")
        #print(sum(part.w for part in self.particle_cloud))
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
        
      
        #create array of weights
        weights = [part.w for part in self.particle_cloud]
        #print("sum of weights:")
        #print(sum(weights))
        #draw a random sample of num_particle particles from the particle cloud
        #where probability are the weights
        #MOVE THIS TO OTHER FUNCTION
        new_cloud = []
        for i in range(self.num_particles):
            this_choice = choices(self.particle_cloud, weights, k = 1)[0]

            new_pos = Point(x = this_choice.pose.position.x,
                            y = this_choice.pose.position.y,
                            z = 0)
            new_orientation = Quaternion(x = 0,y=0,
                                z = this_choice.pose.orientation.z,
                                w = this_choice.pose.orientation.w)
            pose = Pose(new_pos, new_orientation)
            w = this_choice.w
            new_part = Particle(pose,w)
            new_cloud.append(new_part)
        self.particle_cloud = new_cloud
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


        if self.particle_cloud[0]:

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
            yaw_move = normalize_radian(curr_yaw - old_yaw)

            if (x_move > self.lin_mvmt_threshold or 
                y_move > self.lin_mvmt_threshold or
                np.abs(yaw_move) > self.ang_mvmt_threshold):
                print("updating particles")

                # This is where the main logic of the particle filter is carried out

                self.update_particles_with_motion_model(x_move, y_move, yaw_move)
                print("updated motion")
                self.update_particle_weights_with_measurement_model(data)
                print("updated weight")
                self.normalize_particles()
                print("Normalized")
                self.resample_particles()
                '''
                for i in range(10):
                    print(self.particle_cloud[i].pose.position)
                    print(self.particle_cloud[i].pose.orientation)'''


                print("resampled")
                self.update_estimated_robot_pose()
                print("updated estimation")
                self.publish_particle_cloud()
                print("pubblished particle cloud")
                self.publish_estimated_robot_pose()

                self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate
        
        #for now, make this a weighted average of all the particles poses
        #create array of weights

        sum_x = 0
        sum_y = 0
        yaw_array = []
        #weight_array = []

        #loop through patricle cloud
        #getting sum of x and y
        #we are not taking a weighted sum because the particles have
        #already been resampled
        #and an array of the yaws
        for part in self.particle_cloud:
            sum_x += part.pose.position.x #* part.w
            sum_y += part.pose.position.y #* part.w
            yaw_array.append(get_yaw_from_pose(part.pose))
            #weight_array.append(part.w)
        
        #divide, this works because the weights are normalized
        x_mean = sum_x / self.num_particles
        y_mean = sum_y / self.num_particles

        #to aggregate yaw use circular mean
        yaw_mean = circmean(yaw_array) 
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
            for k in [0,45,90,135,180,225,270,315]: #range(360): #for all 360 degrees
                #z is the measurement at range k
                z = data.ranges[k]
                #theta is the particles current yaw
                theta = euler_from_quaternion([
                    part.pose.orientation.x, 
                    part.pose.orientation.y, 
                    part.pose.orientation.z, 
                    part.pose.orientation.w])[2] 
                if z != 0: 
                    #translate and rotate minimum values to x and y of particle
                    x = part.pose.position.x + z * np.cos(theta + k*np.pi/180)
                    y = part.pose.position.y + z * np.sin(theta + k*np.pi/180)

                    #calculate the mimimum distance at each end point using helper function
                    dist = self.likelihood_field.get_closest_obstacle_distance(x,y)
                    if math.isnan(dist):
                        q = 0 #dist will be nan if out of the map, so set weight to 0
                    else:
                        sd_scan = 0.1

                        #approximation for z hit, z random, and z max probailities of laser scan
                        zhit = 0.95
                        zmax = 0.04
                        zrand = 0.01
                        #incorporate probability of a random scan or max scan into the weight
                        q = q * (zhit * compute_prob_zero_centered_gaussian(dist, sd_scan) + (zrand/zmax))
                        #print(q)
            part.w = q
            #print(q)
        return

        

    def update_particles_with_motion_model(self, x_move, y_move, yaw_move):

        #set standard deviations for measurement noise from odometry
        #generate gaussians centered on movements
        sd_xy = 0.2
        sd_yaw = np.pi/90 #about 1 degree
       
        for part in self.particle_cloud:

            #this is the rotate then move than rotate model from class
            #the theta from the robot's current estimated pose
            theta_robo = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)
            theta_part = get_yaw_from_pose(part.pose)
            rot1 = np.arctan2(y_move, x_move) - theta_robo
            trans = math.sqrt((x_move)**2 + (y_move)**2)
            rot2 = yaw_move - rot1

            #add noise to each of these movements
            rot1 += np.random.normal(0, sd_yaw)
            rot2 += np.random.normal(0, sd_yaw)
            trans += np.random.normal(0, sd_xy)

            #update the positions
            part.pose.position.x += trans * math.cos(theta_part + rot1)
            part.pose.position.y += trans * math.sin(theta_part + rot1)
            theta_part += rot1 + rot2

            part.pose.orientation = Quaternion(*quaternion_from_euler(0,0,theta_part))
        return


if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()

    









