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
    Our robot uses a particle filter and known map to estimate its state. We added the map to the robot's knowledge using SLAM in lab. Using object-oriented programming 

### Intialization of Particle Cloud

### Movement Model

### Resampling

### Incorporation of Noise 

### Updating estimated robot pose

### Optimization of parameters

### Challenges

### Future Work

### Takeaways
