# particle_filter_project

##  Implementation Plan
Emilia Lim and Sam Nitkin

### How will you initialize your particle cloud (initialize_particle_cloud)?
Generate a random distribution of particles over the space is the number of particles proportional to the size of the space.

To test that this function works, we will print the particles onto the map and view them by hand.

### How will you update the position of the particles based on the movements of the robot (update_particles_with_motion_model)?
We will generate a normal distribution for each dimension of the robots movement (x, y, orientation) based on proposed movement, with a mean of the distance of movement and a SD to be calibrated by observing error in the robot. We will sample from this distribution and add values to each particle.

To test this code, we will give the robot movement inputs and sample outputs from the distribution, plotting the outputs to check for a reasonable normal distribution for both dimensions.

### How will you compute the importance weights of each particle after receiving the robot's laser scan data?(update_particle_weights_with_measurement_model)?

Set zeros to the maximum scan distance + 0.1. For each particle, find the sum of the absolute value of the differences of the robots range readings and the particles (+.001 to avoid divide by 0 errors).Weight each particle by the inverse of this error term. 

To test this code, we will compare a robot’s laser scan data with a small set of particles and check if the weighting aligns with what we would expect.

### How will you normalize the particles' importance weights (normalize_particles) and resample the particles (resample_particles)?

Calculate the normalizer by taking the inverse of the sum of the weights. Multiply each weight by the normalizer. Then do a random choice of the weighted sample with replacement using numpy.random.choice

To test this code, we can make sure that the weights all add up to one. We can also run the resample particles code on a set of particles and see if the resample seems reasonable. 

###  How will you update the estimated pose of the robot (update_estimated_robot_pose)?

Take the average of all the positions of our particles in the newly sampled cloud to generate an estimate of the robot’s position.

To test this, we can see if it is able to correctly average a set of particle locations.

### How will you incorporate noise into your particle filter localization?

Noise will be incorporated into the steps of our robots that deal with the motors and sensors of the robot. Noise is assumed to be distributed normally. When moving, a random sample from a normal distribution centered on the robot's movement will be added to each particle. Similarly, when reading from the laser scan sensors, we will add a normally distributed error to each comparison with our particles in order to account for the noise in the sensor. 

### A brief timeline sketching out when you would like to have accomplished each of the components listed above.

By 4/17, initialize the particle cloud and update particles with motion model
By end of lab 4/20, have update particles with measurement model working
By 4/23 have normalizing, resampling, and estimated position ready
Due April 26, testing and debugging, writeup 

## Writeup

### Objectives Description
    
The goal of this project is to construct a working particle filter that can help a robot estimate it's position in a known maze enviornment. The particle filter is an implementation of the Bayes filter algorithm for solving Robot state estimation problems.

### High-level description

Our robot uses a particle filter and known map to estimate its state. We added the map to the robot's knowledge using SLAM in lab. Using object-oriented programming we intialized a cloud of particles within the confines of our map, giving each a random pose and uniform weight. Particles move following the robots odometry and are weighted using a liklihood field algorithm. Noise is incorporated during both the movement and the liklihood field steps. The weighted particles are then resampled to form a new particle cloud and this process is repeated. The average position of the particle cloud is calculated to form an estimation of the robot's pose.

### Intialization of Particle Cloud

The code is located in intialize_particle_cloud(). 

The map is imported and all the indexes of the occupancy grid that are known (not negative 1) are stored. These indexes are converted to coordinates in meters, and particles are randomly selected in these positions. Each particle is also given a weight of 1 and a random yaw between pi and -pi, which is converted into a quanterion. All these particles are published to the particle cloud.

### Movement Model

This code is located in the update_particles_with_motion_model() function, with parameters being passed in from robot_scan_recieved(). A helper function, normalize_radian(), is used.

The movement model takes in the robot's odometry to estimate the previous pose and the current pose in robot_scan_recieve(). The change in the robot's position is calculated in terms of x, y, and theta. The theta must be normalized between -pi and pi using the helper function normalize_radian(). updated_particles_with_motion_model() uses the rotate, move, rotate model from class, where each particle is rotated and translated based off the robot's change in position. To each change (rotation 1 & 2, transation x & y), noise is added from a standard distribution centered at zero, with standard deviations calibrated by trial and error. These new x, y, and thetas are added to the particle's current position and converted back to a pose object, updated in the particle cloud.

### Resampling

The weights of the particles are normalized in normalize_particles() and the particles are resampled in resample_particles(), both of which are called in called in robot_scan_received(). .

To normalize the particles, the normalizer is calculated by taking the inverse of the sum of the weights. Each weight is then multiplied by the normalizer. To resample, we indepently drew particles from the particle cloud using weighted choice N times where N is the number of particles in the cloud. To do this, we used the choices function from the python package random. When a particle is draw, we make a deep copy, intializing a new particle with it's pose and weight to avoid editing the particle directly in the particle cloud. These new particles are used to generate a new particle cloud, which is sent to the robot. 

### Incorporation of Noise 

Noise is incorporated in two parts of the code, the measurment model (update_particle_weights_with_measurement_model) and the motion model (update_particles_with_motion_model). 

To see how noise is incorporated into the motion model, read the motion model section above.

Noise was incorporated into the measurement model by calculation the probability that the sensor reading the particle's closest distance matched. Zhit accounted for gaussian error in the reading of the sensor. Z random is the chance that the sensor picked up a random reading and z max the chance it read nothing when an object was there. Both of these give particles slightly higher weights if they don't match the sensor readings well, allowing some particles which aren't exact hits to survive.

### Updating estimated robot pose

The robot's pose is estimated in update_estimated_robot_pose() which is called in robot_scan_received(). 

To update the robot's estimated pose, we take the average of the x's, y's and yaws of all the particles (using the cicrular mean function from scipy for the average yaw) and set the robot's estimated pose, converting yaw back to a quanternion using the helper function.

### Optimization of parameters

When testing our code, we realized we needed more noise in our movement model so that we could correct our particles if they started getting off course from out robot. We also decreased the probabilities of random and max hits in the measurement model so that our particles would converge faster, by taking the laser scan readings to be true values. We also had to adjust the number of angles we read in our measurement model. If we tried to test the laser scanner for all 360 degrees, we encountered many run time errors, so we reduced the angle checking to every 45 degrees to ease the computational load of the code so it could run quicker.

### Challenges

There were several challenges we encountered with this project. Attempting to run the robot with the full number of particles and the liklihood field scanning 360 degrees resulted in many run time errors, and thus we had to decrease values to test it. Another challenge was working with the PGM map and visualization. RVIZ was an unfamiliar tool and the robot lagged in publishing the particle cloud, causing our cloud to either not appear or too show up not centered on our map. An additional big challenged we encounter was with resampling with replacement. Intially, our resampled cloud would duplicate particles, but when we updated the weight of the same particle in two different locations in the cloud, it would update for both particles. Consequently, we had to redo our resampling code to make a new particle cloud with deep copies. The trial and error process for setting parameters also proved to be difficult. Each run of the robot takes a long time to setup, and it is not always clear whether a tweak in a parameter led to improved performance due to the idiosyncratic nature of each run. 

### Future Work

Given more time, it would be nice to run even more tests to optimize the parameters of our robot further, our current solutions are "good enough," but it is difficult to determine if they represent true values of the robot. We would also like to implement the path finding for the maze if we had more time. It would also be possible to not intialize particles on walls, which would allow us to converge to our true location even quicker.

### Takeaways


