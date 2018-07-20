/**
 * @file gripper.cpp
 * 
 * 
 */

#include "systems/gripper.hpp"
#include "utilities/random.hpp"


#define _USE_MATH_DEFINES

#include <cmath>
#include <limits>
#include <math.h>
#include <sstream>
#include <vector>
#include <openrave-core.h>

#define MIN_X -1.5
#define MAX_X 1.5
#define MIN_Y -1.5
#define MAX_Y 1.5
#define MIN_Z -1.5
#define MAX_Z 1.5

#define MIN_SPEED 0.5
#define MAX_SPEED 0.5

#define D_min std::numeric_limits<double>::min()
#define D_max std::numeric_limits<double>::max()

using namespace std;
using namespace OpenRAVE;

double gripper_t::distance(double* point1,double* point2)
{
	return std::sqrt( (point1[0]-point2[0]) * (point1[0]-point2[0]) + (point1[1]-point2[1]) * (point1[1]-point2[1]) + (point1[2]-point2[2]) * (point1[2]-point2[2]));
}

void gripper_t::random_particles(double* destination, double* state, double radius)
{
	double rand_r = uniform_random(0,radius);
	double rand_theta = uniform_random(-M_PI,M_PI);
    double rand_alpha = uniform_random(-M_PI/2.0,M_PI/2.0);
	destination[0] = state[0] + rand_r * cos(rand_alpha) * cos(rand_theta);
	destination[1] = state[1] + rand_r * cos(rand_alpha) * sin(rand_theta);
    destination[2] = state[2] + rand_r * sin(rand_alpha);
	enforce_bounds_once(destination);
}

void gripper_t::random_state(double* state)
{
	state[0] = uniform_random(MIN_X,MAX_X);
	state[1] = uniform_random(MIN_Y,MAX_Y);
	state[2] = uniform_random(MIN_Z,MAX_Z);
}

void gripper_t::random_control(double* control)
{
    double rand_speed = uniform_random(MIN_SPEED, MAX_SPEED);
    double rand_theta = uniform_random(-M_PI,M_PI);
    double rand_alpha = uniform_random(-M_PI/2.0,M_PI/2.0);
	control[0] = rand_speed * cos(rand_alpha) * cos(rand_theta);
	control[1] = rand_speed * cos(rand_alpha) * sin(rand_theta);
    control[2] = rand_speed * sin(rand_alpha);
}

// for test 
bool gripper_t::propagate( double* start_state, double* control, int min_step, int max_step, double* result_state, double& duration )
{	
	temp_state[0] = start_state[0]; temp_state[1] = start_state[1]; temp_state[2] = start_state[2];

	double control_x = control[0]; double control_y = control[1]; double control_z = control[2];

	int num_steps = uniform_int_random(min_step,max_step);
	bool validity = true;
	for(int i=0;i<num_steps;i++)
	{		
		temp_state[0] += params::integration_step * control_x;
		temp_state[1] += params::integration_step * control_y;
        temp_state[2] += params::integration_step * control_z;
		enforce_bounds();
		validity = validity && valid_state();
	}
	result_state[0] = temp_state[0];
	result_state[1] = temp_state[1];
    result_state[2] = temp_state[2];

	duration = num_steps*params::integration_step;
	return validity;
}

bool gripper_t::convergent_propagate( const int &num_steps, double* start_state,std::vector<double*> &start_particles, double* control, double* result_state, std::vector<double*> &result_particles, double& duration, double& cost )
{	
	//std::cout<<"start convergent propagate"<<std::endl;
	cost = 0.0;

	double init_vol = 0.0;
	double final_vol = 0.0;

	temp_state[0] = start_state[0]; temp_state[1] = start_state[1]; temp_state[2] = start_state[2];

	for (size_t i = 0; i < start_particles.size(); i++)
	{
		temp_particles[i][0] = start_particles[i][0];
		temp_particles[i][1] = start_particles[i][1];
		temp_particles[i][2] = start_particles[i][2];
	}

	init_vol = cost_function(temp_state,temp_particles);


	double control_x = control[0]; double control_y = control[1]; double control_z = control[2];
	//std::cout<<"Control : "<<control_x<<","<<control_y<<","<<control_z<<std::endl;
	bool validity = true;
	//propagate state
	for(int i=0;i<num_steps;i++)
	{		
		temp_state[0] += params::integration_step * control_x;
		temp_state[1] += params::integration_step * control_y;
		temp_state[2] += params::integration_step * control_z;
		
		for (size_t j = 0; j < start_particles.size(); j++)
		{
			temp_particles[j][0] += params::integration_step * control_x;
			temp_particles[j][1] += params::integration_step * control_y;
			temp_particles[j][2] += params::integration_step * control_z;
		}
		//std::cout<<temp_state[0]<<","<<temp_state[1]<<","<<temp_state[2]<<std::endl;

		final_vol = cost_function(temp_state,temp_particles);
		cost += (init_vol + final_vol)/2.0*params::integration_step;
		init_vol = final_vol;
		enforce_bounds();
		//enforce_bound_particles();
		validity = validity && valid_state();
	}

	if (validity) 
	{
		result_state[0] = temp_state[0];
		result_state[1] = temp_state[1];
		result_state[2] = temp_state[2];

		for (size_t i = 0; i < start_particles.size(); i++)
		{
			result_particles[i][0] = temp_particles[i][0];
			result_particles[i][1] = temp_particles[i][1];
			result_particles[i][2] = temp_particles[i][2];
		}

		duration = num_steps*params::integration_step;
		//std::cout<<start_state[0]<<","<<start_state[1]<<","<<start_state[2]<<std::endl;
		//std::cout<<result_state[0]<<","<<result_state[1]<<","<<result_state[2]<<std::endl;
		//std::cout<<std::endl;
	}
	
	
	return validity;
}

void gripper_t::enforce_bounds()
{
    // add collision check
	if(temp_state[0]<MIN_X)
		temp_state[0]=MIN_X;
	else if(temp_state[0]>MAX_X)
		temp_state[0]=MAX_X;

	if(temp_state[1]<MIN_Y)
		temp_state[1]=MIN_Y;
	else if(temp_state[1]>MAX_Y)
		temp_state[1]=MAX_Y;

    if(temp_state[2]<MIN_Z)
		temp_state[2]=MIN_Z;
	else if(temp_state[2]>MAX_Z)
		temp_state[2]=MAX_Z;
}

void gripper_t::enforce_bounds_once(double* sample)
{
	if(sample[0]<MIN_X)
		sample[0]=MIN_X;
	else if(sample[0]>MAX_X)
		sample[0]=MAX_X;

	if(sample[1]<MIN_Y)
		sample[1]=MIN_Y;
	else if(sample[1]>MAX_Y)
		sample[1]=MAX_Y;
    
    if(sample[2]<MIN_Z)
		sample[2]=MIN_Z;
	else if(sample[2]>MAX_Z)
		sample[2]=MAX_Z;
}

void gripper_t::enforce_bound_particles()
{
	for (size_t i = 0; i < temp_particles.size(); i++)
	{
		if(temp_particles[i][0]<MIN_X)			temp_particles[i][0]=MIN_X;
		else if(temp_particles[i][0]>MAX_X)		temp_particles[i][0]=MAX_X;
		if(temp_particles[i][1]<MIN_Y)			temp_particles[i][1]=MIN_Y;
		else if(temp_particles[i][1]>MAX_Y)		temp_particles[i][1]=MAX_Y;
		if(temp_particles[i][2]<MIN_Z)			temp_particles[i][1]=MIN_Z;
		else if(temp_particles[i][2]>MAX_Z)		temp_particles[i][1]=MAX_Z;
		//temp_particles[i][2] = hill_height(temp_particles[i]);
	}
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

bool gripper_t::valid_particles()
{
	for (size_t i = 0; i < temp_particles.size(); i++)
	{
		if(temp_particles[i][0]!= MIN_X && 
		temp_particles[i][0] != MAX_X && 
		temp_particles[i][1] != MIN_Y && 
		temp_particles[i][1] != MAX_Y &&
		temp_particles[i][2] != MIN_Z &&
		temp_particles[i][2] != MAX_Z) return true;
	}
	return false;
}

svg::Point gripper_t::visualize_point(double* state, svg::Dimensions dims)
{
	double x = (state[0]-MIN_X)/(MAX_X-MIN_X) * dims.width; 
	double y = (state[1]-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
	return svg::Point(x,y);
}

void gripper_t::write_point(double* state,std::ofstream &myfile)
{
	myfile<<state[0]<<","<<state[1]<<","<<state[2]<<std::endl;
}

double gripper_t::cost_function(double* state, std::vector<double*> particles)
{
	double var_cost = 0.0;
	for (size_t i = 0; i < particles.size(); i++)
	{
		double local_distance = 0.0;
		for (int j = 0; j < 3; j++)
		{
			local_distance += (state[j] - particles[i][j]) * (state[j] - particles[i][j]);
		}

		var_cost += std::sqrt(local_distance);
	}
	return var_cost;
}

void gripper_t::load_openrave()
{
	RaveInitialize(true); 
	penv = RaveCreateEnvironment();
	return;
}