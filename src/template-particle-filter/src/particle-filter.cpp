/*********************
MIT License

Copyright (c) [year] [fullname]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
**************************/
#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include "particle_filter.h"

using namespace std;


void ParticleFilter::init(float x, float y, float theta, float std[]) {
	// Set the number of particles. Initialize all particles to first position (based on estimates of
	//   x, y, theta and their uncertainties from servo motor) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    num_particles = 100; //set to number of files in observation directory
    weights.resize(num_particles);
    particles.resize(num_particles);
    halfAngleOffset = 70;

    float std_r, std_p; // Standard deviations for roll, pitch
    std_r = std[0];
    std_p = std[1];

    // Normal distribution for roll and pitch
    normal_distribution<float> dist_r(x, std_r); // mean is centered around the new measurement
    normal_distribution<float> dist_p(y, std_p);
    default_random_engine gen; //http://www.cplusplus.com/reference/random/default_random_engine/

    // create particles and set their values, should be run kernel GPUSS
    for(int i=0; i<num_particles; ++i){
        Particle p;
        p.id = i;
        p.angle_res = 0.5 * 180/M_PI;
        p.roll_idx = static_cast<int>((dist_r(gen)+halfAngleOffset)/(p.angle_res)+0.5); // take a random value from the Gaussian Normal distribution and update it
        p.pitch_idx = static_cast<int>((dist_p(gen)+halfAngleOffset)/(p.angle_res)+0.5);
        p.weight = 1;
        particles[i] = p;
        weights[i] = p.weight;
    }
    is_initialized = true;
}

void ParticleFilter::prediction(float delta_t, float std_rot[], float pitch_rate, float roll_rate) {
	// Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    float std_r, std_p; // Standard deviations for x, y, and theta
    std_r = std_rot[0];
    std_p = std_rot[1];

    default_random_engine gen;

    for(int i=0; i<num_particles; ++i){
        Particle *p = &particles[i]; // get address of particle to update

        // motion model use for prediction by some constant velocity model
        float new_p  = p->roll_idx*p->angle_res + roll_rate*delta_t; // p->x + (velocity/yaw_rate) * (sin(p->theta + yaw_rate*delta_t) - sin(p->theta));
        float new_r = p->pitch_idx*p->angle_res + pitch_rate*delta_t;// p->y + (velocity/yaw_rate) * (cos(p->theta) - cos(p->theta + yaw_rate*delta_t));

        // add Gaussian Noise to each measurement
        // Normal distribution for x, y and theta
        normal_distribution<float> dist_r(new_r, std_r);
        normal_distribution<float> dist_p(new_p, std_p);

        // update the particle attributes ans discretize the continuous values
        p->roll_idx = static_cast<int>((dist_r(gen)+halfAngleOffset)/(p->angle_res)+0.5);
        p->roll_idx = static_cast<int>((dist_p(gen)+halfAngleOffset)/(p->angle_res)+0.5);
    }
}

void ParticleFilter::updateWeights(int ObsFlag, float std_rot[], float obs_rp[]) {
	// Update the weights of each particle using a multi-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution

    float weights_sum = 0.0;
    float ang_resolution = 0.0;

    float std_obs_r, std_obs_p; //// Standard deviations for x, y, and theta
    std_obs_r = std_rot[0];
    std_obs_p = std_rot[1];
    float std_theta = 0.0;
    // if (ObsFlag = -1)    ////measuerments from IMU only
    // {

    // }

    //////// one pipeline results only
    if (ObsFlag == 1)      /////skyline only
    {
        ang_resolution = 0.005;
        std_theta = 3*ang_resolution; 
    }
    else if (ObsFlag == 2)      //// ground plane only
    {
        ang_resolution = 0.002;
        std_theta = 3*ang_resolution; 
    }
    else if (ObsFlag == 3)         //////////two pipeline results available at the same time
     {
        ang_resolution = 0.005*0.002/(0.005+0.002);
        std_theta = 3*ang_resolution;
    }
    for(int i=0; i<num_particles; ++i)
    {
            Particle *p = &particles[i];
            float wt = 1.0;

            float distance = dist(p->roll_idx*p->angle_res, p->pitch_idx*p->angle_res, obs_rp[0], obs_rp[1]);
            float num = exp(-0.5 * (pow((p->roll_idx*p->angle_res - obs_rp[0]), 2) / pow(std_theta, 2) + pow((p->pitch_idx*p->angle_res - obs_rp[1]), 2) / pow(std_theta, 2)));
            float denom = 2 * M_PI * std_theta * std_theta;
            wt *= num/denom;
            if(distance <= 1.41421 * ang_resolution)
            {
                /************resize the particle number*******************/
                weights.resize(num_particles+4);
                particles.resize(num_particles+4);
  
                // p.id = i;
                p->roll_idx = static_cast<int>((p->angle_res*p->roll_idx)/(ang_resolution)+0.5);
                p->pitch_idx = static_cast<int>((p->angle_res*p->pitch_idx)/(ang_resolution)+0.5);
                p->angle_res = ang_resolution;
                p->weight = wt;
                particles[i] = *p;
                weights[i] = p->weight;

                /***************new paticle surrouding the observation*************/
                Particle new_p;
                new_p.id = num_particles+0;
                new_p.roll_idx = p->roll_idx + 1;
                new_p.pitch_idx = p->pitch_idx;
                new_p.angle_res = ang_resolution;
                num = exp(-0.5 * (pow(((p->roll_idx+1)*p->angle_res - obs_rp[0]), 2) / pow(std_theta, 2) + pow((p->pitch_idx*p->angle_res - obs_rp[1]), 2) / pow(std_theta, 2)));
                new_p.weight = wt*num/denom;
                particles.push_back(new_p);
                weights.push_back(new_p.weight);

                new_p.id = num_particles+1;
                new_p.roll_idx = p->roll_idx-1;
                new_p.pitch_idx = p->pitch_idx;
                new_p.angle_res = ang_resolution;
                num = exp(-0.5 * (pow(((p->roll_idx-1)*p->angle_res - obs_rp[0]), 2) / pow(std_theta, 2) + pow((p->pitch_idx*p->angle_res - obs_rp[1]), 2) / pow(std_theta, 2)));
                new_p.weight = wt*num/denom;
                particles.push_back(new_p);
                weights.push_back(new_p.weight);

                new_p.id = num_particles+2;
                new_p.roll_idx = p->roll_idx;
                new_p.pitch_idx = p->pitch_idx+1;
                new_p.angle_res = ang_resolution;
                num = exp(-0.5 * (pow((p->roll_idx*p->angle_res - obs_rp[0]), 2) / pow(std_theta, 2) + pow(((p->pitch_idx+1)*p->angle_res - obs_rp[1]), 2) / pow(std_theta, 2)));
                new_p.weight = wt*num/denom;
                particles.push_back(new_p);
                weights.push_back(new_p.weight);

                new_p.id = num_particles+3;
                new_p.roll_idx = p->roll_idx;
                new_p.pitch_idx = p->pitch_idx-1;
                new_p.angle_res = ang_resolution;
                num = exp(-0.5 * (pow((p->roll_idx*p->angle_res - obs_rp[0]), 2) / pow(std_theta, 2) + pow(((p->pitch_idx-1)*p->angle_res - obs_rp[1]), 2) / pow(std_theta, 2)));
                new_p.weight = wt*num/denom;
                particles.push_back(new_p);
                weights.push_back(new_p.weight);

                weights_sum += 1; ////selected particle itself
                weights_sum += 4*wt;                                 
                num_particles += 4;                                
            }
            else
            {
                ////// update weights using Multivariate Gaussian Distribution
                // float num = exp(-0.5 * (pow((p->roll_idx*p->angle_res - obs_rp[0]), 2) / pow(std_theta, 2) + pow((p->pitch_idx*p->angle_res - obs_rp[1]), 2) / pow(std_theta, 2)));
                // float denom = 2 * M_PI * std_theta * std_theta;
                // wt *= num/denom;
                weights_sum += wt;
                p->weight = wt;
            }

    }

    // normalize weights to bring them in (0, 1]
    for (int i = 0; i < num_particles; i++) {
        Particle *p = &particles[i];
        p->weight /= weights_sum;
        weights[i] = p->weight;
    }
}

void ParticleFilter::resample() {
	// Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    default_random_engine gen;

    // Random integers on the [0, n) range
    // the probability of each individual integer is its weight of the divided by the sum of all weights.
    discrete_distribution<int> distribution(weights.begin(), weights.end());
    vector<Particle> resampled_particles;

    for (int i = 0; i < num_particles; i++){
        resampled_particles.push_back(particles[distribution(gen)]);
    }
    particles = resampled_particles;

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].roll_idx*particles[i].angle_res << " " << particles[i].pitch_idx*particles[i].angle_res << "\n";
	}
	dataFile.close();
}
