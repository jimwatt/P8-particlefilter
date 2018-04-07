/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>


#include "particle_filter.h"



ParticleFilter::ParticleFilter(const Map& map) : num_particles(0), is_initialized(false) {

	const int N = map.landmark_list.size();
	cloud_ = Eigen::MatrixXf::Zero(2,N);
	for (size_t i = 0; i < N; i++) {
		cloud_(0,i) = map.landmark_list[i].x_f;
		cloud_(1,i) = map.landmark_list[i].y_f;
	}
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.

	num_particles = 10;

	rng.seed(123456);

	one_over_N = 1.0/double(num_particles);

	for(int pp=0;pp<num_particles;++pp) {
		const Particle part{.id=pp,
			.x = x + std[0] * ndist(rng),
			.y = y + std[1] * ndist(rng),
			.theta = theta + std[2] * ndist(rng),
			.weight = one_over_N};
		particles.push_back(part);
	}

	is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	double px_p, py_p;
	for(int pp=0;pp<num_particles;++pp) {

		const double yaw = particles[pp].theta;

		if (fabs(yaw_rate) > 0.001) {
			px_p = particles[pp].x + velocity/yaw_rate * ( sin (yaw + yaw_rate*delta_t) - sin(yaw));
			py_p = particles[pp].y + velocity/yaw_rate * ( cos(yaw) - cos(yaw+yaw_rate*delta_t) );
		}
		else {
			px_p = particles[pp].x + velocity*delta_t*cos(yaw);
			py_p = particles[pp].y + velocity*delta_t*sin(yaw);
		}

		particles[pp].x = px_p + std_pos[0]*ndist(rng);
		particles[pp].y = py_p + std_pos[1]*ndist(rng);
		particles[pp].theta = yaw + yaw_rate*delta_t + std_pos[2]*ndist(rng);
	}

}



void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	

	const int numlandmarks = cloud_.cols();
	const int numobservations = observations.size();

	const double xfactor = 1.0 / (2.0 * std_landmark[0]*std_landmark[0]);
	const double yfactor = 1.0 / (2.0 * std_landmark[1]*std_landmark[1]);

	const int K = 1;
	Eigen::VectorXi ret_indexes(K);
	Eigen::VectorXf out_dists_sqr(K);
	double weightsum = 0.0;
	for(int pp=0;pp<num_particles;++pp) {	//for each particle

		const double th = particles[pp].theta;

		Eigen::Matrix2f R;
		R << cos(th), -sin(th), sin(th), cos(th);

		Eigen::Vector2f r;
		r << particles[pp].x , particles[pp].y;

		for(int ii=0;ii<numobservations;++ii) {	//for each observation

			// convert observation to map frame
			Eigen::Vector2f obsxy;
			obsxy << observations[ii].x, observations[ii].y;
			Eigen::Vector2f obs_mapframe = R * obsxy + r;

			int mindex;
			double mindistance2 = std::numeric_limits<double>::infinity();
			for(int jj=0;jj<numlandmarks;++jj) {
				const double d2 = (cloud_.col(jj) - obs_mapframe).squaredNorm();
				if(d2<mindistance2) {
					mindex = jj;
					mindistance2 = d2;
				} 
			}

			const Eigen::Vector2f ds = obs_mapframe - cloud_.col(mindex);
			
			const double exponent = (ds[0]*ds[0]) * xfactor + (ds[1]*ds[1]) * yfactor;
			particles[pp].weight *= exp(-exponent);
			weightsum += particles[pp].weight;
		}
	}

	// normalize particle weights
	for(int pp=0;pp<num_particles;++pp) {
		particles[pp].weight /= weightsum;
	}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	std::vector<double> weights(num_particles,0.0);
	for(int pp=0;pp<num_particles;++pp) {
		weights[pp] = particles[pp].weight;
	}
	std::discrete_distribution<int> discretedist(weights.begin(), weights.end());

	std::vector<Particle> newparticles(num_particles);
	for(int ii=0;ii<num_particles;++ii) {
		newparticles[ii] = particles[discretedist(rng)];
		newparticles[ii].weight = one_over_N;
	}

	particles = newparticles;

}


std::string ParticleFilter::getAssociations(Particle best)
{
	std::vector<int> v = best.associations;
	std::stringstream ss;
    std::copy( v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    std::string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
std::string ParticleFilter::getSenseX(Particle best)
{
	std::vector<double> v = best.sense_x;
	std::stringstream ss;
    copy( v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    std::string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
std::string ParticleFilter::getSenseY(Particle best)
{
	std::vector<double> v = best.sense_y;
	std::stringstream ss;
    std::copy( v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    std::string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
