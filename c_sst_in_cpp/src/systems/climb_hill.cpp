/**
 * @file climb_hill.cpp
 * 
 * 
 */

#include "systems/climb_hill.hpp"
#include "utilities/random.hpp"


#define _USE_MATH_DEFINES

#include <cmath>
#include <limits>
#include <math.h>
#include <sstream>
#include <vector>

#define MIN_X -1.5
#define MAX_X 1.5
#define MIN_Y -1.5
#define MAX_Y 1.5

#define MIN_SPEED 0.5
#define MAX_SPEED 0.5

#define EPSILON 0.0000000001

#define D_min std::numeric_limits<double>::min()
#define D_max std::numeric_limits<double>::max()

double climb_hill_t::distance(double* point1,double* point2)
{
	// double height1 = hill_height(point1);
	// double height2 = hill_height(point2);
	return std::sqrt( (point1[0]-point2[0]) * (point1[0]-point2[0]) + (point1[1]-point2[1]) * (point1[1]-point2[1]));
}

void climb_hill_t::random_particles(double* destination, double* state, double radius)
{
	double rand_r = uniform_random(0,radius);
	double rand_theta = uniform_random(-M_PI,M_PI);
	destination[0] = state[0] + rand_r * cos(rand_theta);
	destination[1] = state[1] + rand_r * sin(rand_theta);
	enforce_bounds_once(destination);
	destination[2] = hill_height(destination);
}

void climb_hill_t::random_state(double* state)
{
	state[0] = uniform_random(MIN_X,MAX_X);
	state[1] = uniform_random(MIN_Y,MAX_Y);
	state[2] = hill_height(state);
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

bool climb_hill_t::convergent_propagate( const int &num_steps, double* start_state,std::vector<double*> &start_particles, double* control, double* result_state, std::vector<double*> &result_particles, double& duration, double& cost )
{	
//	double init_cost = 0;
//	double local_cost = 0;


	//ConvexHull* conv;
	cost = 0.0;

	double init_vol = 0.0;
	double final_vol = 0.0;

	temp_state[0] = start_state[0]; 
	temp_state[1] = start_state[1];
	temp_state[2] = hill_height(temp_state);

	for (size_t i = 0; i < start_particles.size(); i++)
	{
		temp_particles[i][0] = start_particles[i][0];
		temp_particles[i][1] = start_particles[i][1];
		temp_particles[i][2] = hill_height(start_particles[i]);
		//init_cost += distance(temp_particles[i],temp_state);
			//std::cout<< temp_particles[i][0] <<","<< temp_particles[i][1]<<","<<temp_particles[i][2]<<std::endl;
	}

	init_vol = cost_function(temp_state,temp_particles);

	double theta = control[0]; double u = control[1];

	bool validity = true;
	//propagate state
	for(int i=0;i<num_steps;i++)
	{		
		double dhdx = hill_gradient_x(temp_state);
		double dhdy = hill_gradient_y(temp_state);
		double delta_h = dhdx * cos(theta) + dhdy * sin(theta);
		
		//temp_state[0] += params::integration_step * u *  cos(theta);
		//temp_state[1] += params::integration_step * u *  sin(theta);
		temp_state[0] += params::integration_step * u * (-2/M_PI * atan(delta_h) + 1) * cos(theta);
		temp_state[1] += params::integration_step * u * (-2/M_PI * atan(delta_h) + 1) * sin(theta);
		temp_state[2] = hill_height(temp_state);
		
		for (size_t j = 0; j < start_particles.size(); j++)
		{
			double temp_dhdx = hill_gradient_x(temp_particles[j]);
			double temp_dhdy = hill_gradient_y(temp_particles[j]);

			double temp_delta_h = temp_dhdx * cos(theta) + temp_dhdy * sin(theta);

			//temp_particles[j][0] += params::integration_step * u  * cos(theta);
			//temp_particles[j][1] += params::integration_step * u  * sin(theta);

			temp_particles[j][0] += params::integration_step * u * (-2/M_PI * atan(temp_delta_h) + 1) * cos(theta);
			temp_particles[j][1] += params::integration_step * u * (-2/M_PI * atan(temp_delta_h) + 1) * sin(theta);
			temp_particles[j][2] = hill_height(temp_particles[j]);
			//local_cost += distance(temp_state,temp_particles[j])*params::integration_step;
		}
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
		result_state[2] = hill_height(result_state);

		for (size_t i = 0; i < start_particles.size(); i++)
		{
			result_particles[i][0] = temp_particles[i][0];
			result_particles[i][1] = temp_particles[i][1];
			result_particles[i][2] = hill_height(result_particles[i]);
			//std::cout << "climb_hill_t:: propagate_with_particles: result_particles: " << result_particles[i][0] << " " << result_particles[i][1] << " " << temp_particles[i][2] << std::endl;
		}

		//final_vol = cost_function(result_state, result_particles);

		duration = num_steps*params::integration_step;
		//cost = params::lambda*duration + duration*(init_vol + final_vol) / 2.0;
		//cost = duration*(init_vol + final_vol) / 2.0;

	}
	
	//if (validity) final_vol = (double) conv->ConvexHull_Volume(temp_particles) + EPSILON;
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

void climb_hill_t::enforce_bounds_once(double* sample)
{
	if(sample[0]<MIN_X)
		sample[0]=MIN_X;
	else if(sample[0]>MAX_X)
		sample[0]=MAX_X;

	if(sample[1]<MIN_Y)
		sample[1]=MIN_Y;
	else if(sample[1]>MAX_Y)
		sample[1]=MAX_Y;
}

void climb_hill_t::enforce_bound_particles()
{
	for (size_t i = 0; i < temp_particles.size(); i++)
	{
		if(temp_particles[i][0]<MIN_X)			temp_particles[i][0]=MIN_X;
		else if(temp_particles[i][0]>MAX_X)		temp_particles[i][0]=MAX_X;
		if(temp_particles[i][1]<MIN_Y)			temp_particles[i][1]=MIN_Y;
		else if(temp_particles[i][1]>MAX_Y)		temp_particles[i][1]=MAX_Y;
		//temp_particles[i][2] = hill_height(temp_particles[i]);
	}
}


bool climb_hill_t::valid_state()
{
	return 	(temp_state[0]!=MIN_X) &&
			(temp_state[0]!=MAX_X) &&
			(temp_state[1]!=MIN_Y) &&
			(temp_state[1]!=MAX_Y);
}

bool climb_hill_t::valid_particles()
{
	for (size_t i = 0; i < temp_particles.size(); i++)
	{
		if(temp_particles[i][0]!= MIN_X && temp_particles[i][0] != MAX_X && temp_particles[i][1] != MIN_Y && temp_particles[i][1] != MAX_Y) return true;
	}
	return false;
}

double climb_hill_t::hill_height(double* point) {

	//return sin(0.5 * point[0]) + cos(point[1]) + 0.5*cos(0.2*point[0]*point[1]);
	//return 3.0*point[1] + sin(point[0] + point[0]*point[1]);
	return sin(point[0] * point[1]);

	// double height;
	// double dist = point[0] * point[0] + point[1] * point[1];
	// if (dist <= 1) height = 1.0 - dist;
	// else height = 0.0;
	// return height;
}

double climb_hill_t::hill_gradient_x(double* point) {
	//return 0.5*cos(0.5 * point[0]) - 0.1 * (point[1] * sin(0.2 * point[0] * point[1]));
	//return cos(point[0] + point[0]*point[1])*(1.0+point[1]);

	return point[1] * cos(point[0] * point[1]);

	// double dx;
	// double dist = point[0] * point[0] + point[1] * point[1];
	// if (dist <= 1) dx = -2 * point[0];
	// else dx  = 0.0;
	// return dx;
}

double climb_hill_t::hill_gradient_y(double* point) {
	//return - sin(point[1]) - 0.1 * (point[0] * sin(0.2 * point[0] * point[1]));
	//return 3.0 + cos(point[0]+point[0]*point[1])*point[0];
	return point[0] * cos(point[0] * point[1]);

	// double dy;
	// double dist = point[0] * point[0] + point[1] * point[1];
	// if (dist <= 1) dy = -2 * point[1];
	// else dy  = 0.0;
	// return dy;
}

svg::Point climb_hill_t::visualize_point(double* state, svg::Dimensions dims)
{
	double x = (state[0]-MIN_X)/(MAX_X-MIN_X) * dims.width; 
	double y = (state[1]-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
	return svg::Point(x,y);
}

void climb_hill_t::write_point(double* state,std::ofstream &myfile)
{
	myfile<<state[0]<<","<<state[1]<<","<<state[2]<<std::endl;
}

double climb_hill_t::cost_function(double* state, std::vector<double*> particles)
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

double climb_hill_t::go_through_path(std::vector<tree_node_t*> path)
{
	std::vector<double*> path_particles;
	path_particles.resize(path[0]->particles.size());
	for (size_t i = 0; i < path_particles.size(); i++)  
	{
		path_particles[i] = alloc_state_point();
		copy_state_point(path_particles[i],path[0]->particles[i]);
	}
	double path_cost = 0.0;
	double* current_state = alloc_state_point();
	double* next_state = alloc_state_point();
	double* step_state = alloc_state_point();
	copy_state_point(current_state,path[0]->point);
	copy_state_point(step_state,current_state);
	for (size_t i = 1; i < path.size(); i++)
	{
		double init_vol = cost_function(current_state,path_particles);
		copy_state_point(next_state,path[i]->point);
		double theta = atan2(next_state[1] - current_state[1], next_state[0] - current_state[0]);
		double u = MAX_SPEED;
		double local_duration = 0.0;
		bool forward = true;
		
		while(forward){

			double dhdx = hill_gradient_x(step_state);
			double dhdy = hill_gradient_y(step_state);
			double delta_h = dhdx * cos(theta) + dhdy * sin(theta);
			
			double step_size = params::integration_step;
			double local_integration_step = (next_state[0] - step_state[0])/ ( u * (-2/M_PI * atan(delta_h) + 1) * cos(theta));
			
			if (local_integration_step < params::integration_step) 
			{
				step_size = local_integration_step;
				forward = false;
			}

			step_state[0] += step_size * u * (-2/M_PI * atan(delta_h) + 1) * cos(theta);
			step_state[1] += step_size * u * (-2/M_PI * atan(delta_h) + 1) * sin(theta);
		
			for (size_t j = 0; j < path_particles.size(); j++)
			{
				double temp_dhdx = hill_gradient_x(path_particles[j]);
				double temp_dhdy = hill_gradient_y(path_particles[j]);

				double temp_delta_h = temp_dhdx * cos(theta) + temp_dhdy * sin(theta);

				path_particles[j][0] += step_size * u * (-2/M_PI * atan(temp_delta_h) + 1) * cos(theta);
				path_particles[j][1] += step_size * u * (-2/M_PI * atan(temp_delta_h) + 1) * sin(theta);
			//local_cost += distance(temp_state,temp_particles[j])*params::integration_step;
			}
			local_duration += step_size;
		}


		for (size_t j = 0; j < path_particles.size(); j++)
		{
				path_particles[j][2] = hill_height(path_particles[j]);
		}

		double final_vol = cost_function(next_state,path_particles);
		double local_cost = local_duration*(init_vol + final_vol) / 2.0;
		path_cost += local_cost;
		copy_state_point(current_state,next_state);
		copy_state_point(step_state,current_state);
	}
	return path_cost;
}


