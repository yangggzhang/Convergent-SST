/**
 * @file climb_hill.cpp
 * 
 * 
 */

#include "systems/gripper.hpp"
#include "utilities/random.hpp"
#include "utilities/convexhull.hpp"
#include <openrave-core.h>

#define _USE_MATH_DEFINES

#include <cmath>
#include <limits>
#include <math.h>
#include <sstream>

#define MIN_X -2
#define MAX_X 2
#define MIN_Y -2
#define MAX_Y 2
#define MIN_Z -2
#define MAX_Z 2

#define MIN_SPEED -0.5
#define MAX_SPEED 0.5

#define D_min std::numeric_limits<double>::min()
#define D_max std::numeric_limits<double>::max()

using namespace OpenRAVE;


double gripper_t::distance(double* point1,double* point2)
{
	return std::sqrt( (point1[0]-point2[0]) * (point1[0]-point2[0]) + (point1[1]-point2[1]) * (point1[1]-point2[1]) + (point1[2] - point2[2]) * (point1[2] - point2[2]) );
}

void gripper_t::random_particles(double* destination, double* state, double radius)
{
	destination[0] = state[0] + uniform_random(-radius, radius);
	destination[1] = state[1] + uniform_random(-radius, radius);
	destination[2] = state[2] + uniform_random(-radius, radius);
}

void gripper_t::random_state(double* state)
{
	state[0] = uniform_random(MIN_X,MAX_X);
	state[1] = uniform_random(MIN_Y,MAX_Y);
	state[2] = uniform_random(MIN_Z,MAX_Z);
}

void gripper_t::random_control(double* control)
{
	control[0] = uniform_random(MIN_SPEED, MAX_SPEED);
	control[1] = uniform_random(MIN_SPEED, MAX_SPEED);
	control[2] = uniform_random(MIN_SPEED, MAX_SPEED);
}

bool gripper_t::propagate( double* start_state, double* control, int min_step, int max_step, double* result_state, double& duration )
{	
	temp_state[0] = start_state[0]; temp_state[1] = start_state[1]; temp_state[2] = start_state[2];

	double ux = control[0]; double uy = control[1]; double uz = control[2];

	int num_steps = uniform_int_random(min_step,max_step);
	bool validity = true;
	for(int i=0;i<num_steps;i++)
	{		
		temp_state[0] += params::integration_step * ux;
		temp_state[1] += params::integration_step * uy;
		temp_state[2] += params::integration_step * uz;
		enforce_bounds(temp_state);
		validity = validity && valid_state();
		if (check_collision(temp_state))
		{
			temp_state[0] += -params::integration_step * ux;
			temp_state[1] += -params::integration_step * uy;
			temp_state[2] += -params::integration_step * uz;
		}
	}
	result_state[0] = temp_state[0];
	result_state[1] = temp_state[1];
	result_state[2] = temp_state[2];

	duration = num_steps*params::integration_step;
	return validity;
}

bool gripper_t::convergent_propagate( const bool &random_time, double* start_state,std::vector<double*> &start_particles, double* control, int min_step, int max_step, double* result_state, std::vector<double*> &result_particles, double& duration, double& cost )
{	
	double local_cost = 0;
	temp_state[0] = start_state[0]; temp_state[1] = start_state[1]; temp_state[2] = start_state[2];

	double ux = control[0]; double uy = control[1]; double uz = control[2];

	// std::cout << "ux: " << ux << " uy: " << uy << " uz: " << uz << std::endl;

	int num_steps = 0; // adjust time stamp according to input
	if (random_time)	num_steps = uniform_int_random(min_step,max_step);
	else num_steps =  (int)(min_step + max_step)/2;

	bool validity = true;

	double init_De = 0;
	if (start_particles.size() > 0)
	{	
		for (size_t i = 0; i < start_particles.size(); i++)
		{
			temp_particles[i][0] = start_particles[i][0];
			temp_particles[i][1] = start_particles[i][1];
			temp_particles[i][2] = start_particles[i][2];
		}

		for (size_t i = 0; i < start_particles.size(); ++i)
		{
			init_De += distance(temp_state, temp_particles[i]);
		}
	}

	//std::cout<<"update state"<<std::endl;
	//propagate state
	for(int i=0;i<num_steps;i++)
	{		

		// temp_state[0] += params::integration_step * ux;
		// temp_state[1] += params::integration_step * uy;
		// temp_state[2] += params::integration_step * uz;
		// enforce_bounds(temp_state);
		// validity = validity && valid_state();
		// if (check_collision(temp_state))
		// {
		// 	temp_state[0] += -params::integration_step * ux;
		// 	temp_state[1] += -params::integration_step * uy;
		// 	temp_state[2] += -params::integration_step * uz;
		// }

		if (start_particles.size() > 0)
		{
			for (size_t j = 0; j < start_particles.size(); j++)
			{
				temp_particles[j][0] += params::integration_step * ux;
				temp_particles[j][1] += params::integration_step * uy;
				temp_particles[j][2] += params::integration_step * uz;				
				enforce_bounds(temp_particles[j]);
				if (check_collision(temp_particles[j]))
				{
					temp_particles[j][0] += -params::integration_step * ux;
					temp_particles[j][1] += -params::integration_step * uy;
					temp_particles[j][2] += -params::integration_step * uz;
				}
				
				// local_cost += distance(temp_state, temp_particles[j]) * params::integration_step;
			}
		}

	}

	double end_De = 0;
	temp_state[0] = 0; temp_state[1] = 0; temp_state[2] = 0;
	if (start_particles.size() > 0)
	{
		for (size_t j = 0; j < start_particles.size(); j++)
		{
			temp_state[0] += temp_particles[j][0];
			temp_state[1] += temp_particles[j][1];
			temp_state[2] += temp_particles[j][2];
		}

		temp_state[0] = temp_state[0]/number_of_particles;
		temp_state[1] = temp_state[1]/number_of_particles;
		temp_state[2] = temp_state[2]/number_of_particles;

		for (size_t j = 0; j < start_particles.size(); j++)
		{
			end_De += distance(temp_state, temp_particles[j]);
		}
	}

	enforce_bounds(temp_state);
	validity = validity && valid_state();

	result_state[0] = temp_state[0];
	result_state[1] = temp_state[1];
	result_state[2] = temp_state[2];

	duration = num_steps*params::integration_step;
	// local_cost = duration; //default setting
	local_cost = (init_De+end_De)/2*duration;

	
	if ( start_particles.size() > 0 ) 
	{
		for (size_t i = 0; i < start_particles.size(); i++)
		{
			result_particles[i][0] = temp_particles[i][0];
			result_particles[i][1] = temp_particles[i][1];
			result_particles[i][2] = temp_particles[i][2];
			//std::cout << "climb_hill_t:: propagate_with_particles: result_particles: " << result_particles[i][0] << " " << result_particles[i][1] << " " << temp_particles[i][2] << std::endl;
		}
	}

	cost = local_cost;
	
//	std::cout<<cost<<std::endl;
	return validity;
}

void gripper_t::enforce_bounds(double* state)
{
	if(state[0]<MIN_X)
		state[0]=MIN_X;
	else if(state[0]>MAX_X)
		state[0]=MAX_X;

	if(state[1]<MIN_Y)
		state[1]=MIN_Y;
	else if(state[1]>MAX_Y)
		state[1]=MAX_Y;

	if(state[2]<MIN_Z)
		state[2]=MIN_Z;
	else if(state[2]>MAX_Z)
		state[2]=MAX_Z;
}

bool gripper_t::check_collision(double* state)
{
	if (state[0]<1.0 && state[0]>-1.0 && state[1]<1.0 && state[1]>-1.0 && state[2]<0.1 && state[2]>-0.1)
	{
		// std::cout << state[0] << " " << state[1] << " " << state[2] << std::endl;
		return true;
	}
	else
		return false;
}


bool gripper_t::valid_state()
{
	return 	(temp_state[0]!=MIN_X) &&
			(temp_state[0]!=MAX_X) &&
			(temp_state[1]!=MIN_Y) &&
			(temp_state[1]!=MAX_Y) &&
			(temp_state[2]!=MIN_Z) &&
			(temp_state[2]!=MAX_Z);
}


svg::Point gripper_t::visualize_point(double* state, svg::Dimensions dims)
{
	double x = (state[0]-MIN_X)/(MAX_X-MIN_X) * dims.width; 
	double y = (state[1]-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
	// double y = (state[2]-MIN_Z)/(MAX_Z-MIN_Z) * dims.height; 
	return svg::Point(x,y);
}

std::string gripper_t::export_point(double* state)
{
	std::stringstream s;
	s << state[0] << "," << state[1] << "," << state[2] << std::endl;
	return s.str();
}
