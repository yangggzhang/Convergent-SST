/**
 * @file climb_hill.cpp
 * 
 * 
 */

#include "systems/climb_hill.hpp"
#include "utilities/random.hpp"


#define _USE_MATH_DEFINES

#include <cmath>

#define MIN_X -2
#define MAX_X 2
#define MIN_Y 0
#define MAX_Y 2.5

#define SPEED 0.5

double climb_hill_t::distance(double* point1,double* point2)
{
	double height1 = hill_height(point1);
	double height2 = hill_height(point2);
	return std::sqrt( (point1[0]-point2[0]) * (point1[0]-point2[0]) + (point1[1]-point2[1]) * (point1[1]-point2[1]) + (height1 - height2) * (height1 - height2) );
}

void climb_hill_t::random_particles(double* destination, double* state, double radius)
{
	destination[0] = state[0] + uniform_random(-radius, radius);
	destination[1] = state[1] + uniform_random(-radius, radius);
}

void climb_hill_t::random_state(double* state)
{
	state[0] = uniform_random(MIN_X,MAX_X);
	state[1] = uniform_random(MIN_Y,MAX_Y);
}

void climb_hill_t::random_control(double* control)
{
	control[0] = uniform_random(-M_PI,M_PI);
}

bool climb_hill_t::propagate( double* start_state, double* control, int min_step, int max_step, double* result_state, double& duration )
{
	temp_state[0] = start_state[0]; temp_state[1] = start_state[1];

	int num_steps = uniform_int_random(min_step,max_step);
	bool validity = true;
	for(int i=0;i<num_steps;i++)
	{
		double temp0 = temp_state[0];
		double temp1 = temp_state[1];
		
		double dhdx = hill_gradient_x(temp_state);
		double dhdy = hill_gradient_y(temp_state);

		double norm_ = std::sqrt(dhdx * dhdx + dhdy * dhdy);

		temp_state[0] += params::integration_step * SPEED * (dhdx*cos(control[0])/norm_ - dhdy*sin(control[0])/norm_);
		temp_state[1] += params::integration_step * SPEED * (dhdx*sin(control[0])/norm_ + dhdy*cos(control[0])/norm_);
		enforce_bounds();
		validity = validity && valid_state();
	}
	result_state[0] = temp_state[0];
	result_state[1] = temp_state[1];
	result_state[2] = temp_state[2];
	duration = num_steps*params::integration_step;
	return validity;
}

bool climb_hill_t::propagate_with_particles( double* start_state, std::vector<double*> &particles, double* control, int min_step, int max_step, double* result_state, std::vector<double*> &result_particles, double& duration )
{
	temp_state[0] = start_state[0]; temp_state[1] = start_state[1];
	for (int i = 0; i < number_of_particles; ++i)
	{
		temp_particles[i][0] = particles[i][0];
		temp_particles[i][1] = particles[i][1];
	}

	int num_steps = uniform_int_random(min_step,max_step);
	bool validity = true;
	for(int i=0;i<num_steps;i++)
	{		
		double dhdx = hill_gradient_x(temp_state);
		double dhdy = hill_gradient_y(temp_state);

		double norm_ = std::sqrt(dhdx * dhdx + dhdy * dhdy);

		temp_state[0] += params::integration_step * SPEED * (dhdx*cos(control[0])/norm_ - dhdy*sin(control[0])/norm_);
		temp_state[1] += params::integration_step * SPEED * (dhdx*sin(control[0])/norm_ + dhdy*cos(control[0])/norm_);
		enforce_bounds();
		
		//Propagate all particles (No bound enforcement)
		for (int j = 0; j < number_of_particles; ++j)
		{
			double temp_dhdx = hill_gradient_x(temp_particles[j]);
			double temp_dhdy = hill_gradient_y(temp_particles[j]);

			double temp_norm_ = std::sqrt(dhdx * dhdx + dhdy * dhdy);

			temp_particles[j][0] += params::integration_step * SPEED * (temp_dhdx*cos(control[0])/temp_norm_ - temp_dhdy*sin(control[0])/temp_norm_);
			temp_particles[j][1] += params::integration_step * SPEED * (dhdx*sin(control[0])/temp_norm_ + temp_dhdy*cos(control[0])/temp_norm_);
		}

		validity = validity && valid_state();
	}
	result_state[0] = temp_state[0];
	result_state[1] = temp_state[1];

	// std::cout << "climb_hill_t:: propagate_with_particles: result_state: " << result_state[0] << " " << result_state[1] << std::endl;

	for (int i = 0; i < number_of_particles; ++i)
	{
		result_particles[i][0] = temp_particles[i][0];
		result_particles[i][1] = temp_particles[i][1];

		// std::cout << "climb_hill_t:: propagate_with_particles: result_particles: " << result_particles[i][0] << " " << result_particles[i][1] << std::endl;
	}

	duration = num_steps*params::integration_step;
	return validity;

}


void climb_hill_t::enforce_bounds()
{
	if(temp_state[0]<MIN_X)
		temp_state[0]=MIN_X;
	else if(temp_state[0]>MAX_X)
		temp_state[0]=MAX_X;

	if(temp_state[1]<MIN_Y)
		temp_state[1]=MIN_Y;
	else if(temp_state[1]>MAX_Y)
		temp_state[1]=MAX_Y;
}


bool climb_hill_t::valid_state()
{
	return 	(temp_state[0]!=MIN_X) &&
			(temp_state[0]!=MAX_X) &&
			(temp_state[1]!=MIN_Y) &&
			(temp_state[1]!=MAX_Y);
}

double climb_hill_t::hill_height(double* point) {
	return 3*point[1] + sin(point[0] + point[0]*point[1]);
}

double climb_hill_t::hill_gradient_x(double* point) {
	return cos(point[0] + point[0]*point[1])*(1+point[1]);
}

double climb_hill_t::hill_gradient_y(double* point) {
	return 3 + cos(point[0]+point[0]*point[1])*point[0];
}

svg::Point climb_hill_t::visualize_point(double* state, svg::Dimensions dims)
{
	double x = (state[0]-MIN_X)/(MAX_X-MIN_X) * dims.width; 
	double y = (state[1]-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
	return svg::Point(x,y);
}