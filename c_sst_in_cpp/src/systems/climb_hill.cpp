/**
 * @file climb_hill.cpp
 * 
 * 
 */

#include "systems/climb_hill.hpp"
#include "utilities/random.hpp"
#include "utilities/convexhull.hpp"


#define _USE_MATH_DEFINES

#include <cmath>
#include <limits>
#include <math.h>
#include <sstream>

#define MIN_X -3
#define MAX_X 3
#define MIN_Y -3
#define MAX_Y 3

#define MIN_SPEED 0.5
#define MAX_SPEED 0.5

#define D_min std::numeric_limits<double>::min()
#define D_max std::numeric_limits<double>::max()



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
	control[1] = uniform_random(MIN_SPEED, MAX_SPEED);
}

bool climb_hill_t::propagate( double* start_state, double* control, int min_step, int max_step, double* result_state, double& duration )
{	
	temp_state[0] = start_state[0]; temp_state[1] = start_state[1];

	double theta = control[0]; double u = control[1];

	int num_steps = uniform_int_random(min_step,max_step);
	bool validity = true;
	for(int i=0;i<num_steps;i++)
	{		
		double dhdx = hill_gradient_x(temp_state);
		double dhdy = hill_gradient_y(temp_state);
		double delta_h = dhdx * cos(theta) + dhdy * sin(theta);

		temp_state[0] += params::integration_step * u * (-2/M_PI * atan(delta_h) + 1) * cos(theta);
		temp_state[1] += params::integration_step * u * (-2/M_PI * atan(delta_h) + 1) * sin(theta);
		enforce_bounds();
		validity = validity && valid_state();
	}
	result_state[0] = temp_state[0];
	result_state[1] = temp_state[1];

	duration = num_steps*params::integration_step;
	return validity;
}

bool climb_hill_t::convergent_propagate( const bool &random_time, double* start_state,std::vector<double*> &start_particles, double* control, int min_step, int max_step, double* result_state, std::vector<double*> &result_particles, double& duration, double& cost )
{	
	double local_cost = 0;
	temp_state[0] = start_state[0]; temp_state[1] = start_state[1];

	double theta = control[0]; double u = control[1];

	int num_steps = 0; // adjust time stamp according to input
	if (random_time)	num_steps = uniform_int_random(min_step,max_step);
	else num_steps =  (int)(min_step + max_step)/2;

	bool validity = true;

	//std::cout<<"update state"<<std::endl;
	//propagate state
	for(int i=0;i<num_steps;i++)
	{		
		double dhdx = hill_gradient_x(temp_state);
		double dhdy = hill_gradient_y(temp_state);
		double delta_h = dhdx * cos(theta) + dhdy * sin(theta);

		temp_state[0] += params::integration_step * u * (-2/M_PI * atan(delta_h) + 1) * cos(theta);
		temp_state[1] += params::integration_step * u * (-2/M_PI * atan(delta_h) + 1) * sin(theta);
		enforce_bounds();
		validity = validity && valid_state();
	}
	result_state[0] = temp_state[0];
	result_state[1] = temp_state[1];

	duration = num_steps*params::integration_step;
	local_cost = duration; //default setting


	
	if ( start_particles.size() > 0 ) 
	{
		local_cost = 0;
		// iterate through particles for convergent planning
		for (size_t i = 0; i < start_particles.size(); i++)
		{
			temp_particles[i][0] = start_particles[i][0];
			temp_particles[i][1] = start_particles[i][1];
			temp_particles[i][2] = hill_height(start_particles[i]);
		}

		ConvexHull* conv;
		double init_vol = 0;
		init_vol = (double) conv->ConvexHull_Volume(temp_particles);
		for (int i = 0; i < num_steps; i++)
		{
			for (size_t j = 0; j < start_particles.size(); j++)
			{
				double temp_dhdx = hill_gradient_x(temp_particles[j]);
				double temp_dhdy = hill_gradient_y(temp_particles[j]);

				double temp_delta_h = temp_dhdx * cos(theta) + temp_dhdy * sin(theta);

				temp_particles[j][0] += params::integration_step * u * (-2/M_PI * atan(temp_delta_h) + 1) * cos(theta);
				temp_particles[j][1] += params::integration_step * u * (-2/M_PI * atan(temp_delta_h) + 1) * sin(theta);
				temp_particles[j][2] = hill_height(temp_particles[j]);
				
			}
			//local_cost += (double)conv->ConvexHull_Volume(temp_particles) * params::integration_step;
		}

		double end_vol = (double)conv->ConvexHull_Volume(temp_particles);
		
		local_cost = (init_vol + end_vol)*duration/2.0;
		//std::cout<<init_vol <<" "<<end_vol<<std::endl;
		local_cost = local_cost/init_vol;
		//std::cout<<local_cost<<std::endl;
		for (size_t i = 0; i < start_particles.size(); i++)
		{
			result_particles[i][0] = temp_particles[i][0];
			result_particles[i][1] = temp_particles[i][1];
			//std::cout << "climb_hill_t:: propagate_with_particles: result_particles: " << result_particles[i][0] << " " << result_particles[i][1] << " " << temp_particles[i][2] << std::endl;
		}
	}

	cost = local_cost;
	
//	std::cout<<cost<<std::endl;
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
	//return 3.0*point[1] + sin(point[0] + point[0]*point[1]);
	double height;
	double dist = std::sqrt(point[0] * point[0] + point[1] * point[1]);
	if (dist <= 1) height = 1 - dist*dist;
	else height = 0.0;
	return height;
	//if height = 18 - dist * dist;

	//return 18.0 - point[0] * point[0] - point[1] * point[1];
	//return 3*point[1] + sin(point[0] + point[0]*point[1]);
}

double climb_hill_t::hill_gradient_x(double* point) {
	// return cos(point[0] + point[0]*point[1])*(1+point[1]);
	//return cos(point[0] + point[0]*point[1])*(1+point[1]);
	double dx;
	double dist = std::sqrt(point[0] * point[0] + point[1] * point[1]);
	if (dist <= 1) dx = -2 * point[0];
	else dx  = 0.0;
	return dx;
	//return -2.0 * point[0];
}

double climb_hill_t::hill_gradient_y(double* point) {
	// return 3 + cos(point[0]+point[0]*point[1])*point[0];
	//return 3.0 + cos(point[0]+point[0]*point[1])*point[0];
	double dy;
	double dist = std::sqrt(point[0] * point[0] + point[1] * point[1]);
	if (dist <= 1) dy = -2 * point[1];
	else dy  = 0.0;
	return dy;
	//return -2.0 * point[1];
}

svg::Point climb_hill_t::visualize_point(double* state, svg::Dimensions dims)
{
	double x = (state[0]-MIN_X)/(MAX_X-MIN_X) * dims.width; 
	double y = (state[1]-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
	return svg::Point(x,y);
}