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

#define MIN_X -5
#define MAX_X 5
#define MIN_Y -5
#define MAX_Y 5

#define MIN_SPEED 0.5
#define MAX_SPEED 0.5


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

bool climb_hill_t::propagate_with_particles( double* start_state, std::vector<double*> &particles, double* control, int min_step, int max_step, double* result_state, std::vector<double*> &result_particles, double& duration, double& Da )
{

	temp_state[0] = start_state[0]; temp_state[1] = start_state[1];
	for (int i = 0; i < number_of_particles; ++i)
	{
		temp_particles[i][0] = particles[i][0];
		temp_particles[i][1] = particles[i][1];
		temp_particles[i][2] = hill_height(particles[i]);
	}

	double temp_cost = 0;
	ConvexHull* conv;
	double init_vol = conv->ConvexHull_Volume(temp_particles);
	
	double theta = control[0]; double u = control[1];

	int num_steps = uniform_int_random(min_step,max_step);
	bool validity = true;
	for(int i=0;i<num_steps;i++)
	{		
		double dhdx = hill_gradient_x(temp_state);
		double dhdy = hill_gradient_y(temp_state);
		double delta_h = dhdx * cos(theta) + dhdy * sin(theta);
		// std::cout << "delta_h: " << delta_h << " -2/pi*atan(delta_h) " << -2/M_PI * atan(delta_h) << std::endl;
		// std::cout << "theta: " << theta << "cos: " << cos(theta) << "sin: " << sin(theta) << std::endl;
		temp_state[0] += params::integration_step * u * (-2/M_PI * atan(delta_h) + 1) * cos(theta);
		temp_state[1] += params::integration_step * u * (-2/M_PI * atan(delta_h) + 1) * sin(theta);
		enforce_bounds();
		
		//Propagate all particles (No bound enforcement)
		for (int j = 0; j < number_of_particles; ++j)
		{
			double temp_dhdx = hill_gradient_x(temp_particles[j]);
			double temp_dhdy = hill_gradient_y(temp_particles[j]);

			double temp_delta_h = temp_dhdx * cos(theta) + temp_dhdy * sin(theta);

			temp_particles[j][0] += params::integration_step * u * (-2/M_PI * atan(temp_delta_h) + 1) * cos(theta);
			temp_particles[j][1] += params::integration_step * u * (-2/M_PI * atan(temp_delta_h) + 1) * sin(theta);
			temp_particles[j][2] = hill_height(temp_particles[j]);
		}

		validity = validity && valid_state();

		if (validity) {
			temp_cost += conv->ConvexHull_Volume(temp_particles) * params::integration_step / init_vol;
		}
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
	Da = temp_cost;
	return validity;

}

bool climb_hill_t::propagate_fixed_duration( double* start_state, std::vector<double*> &particles, double* control, int step_size, double* result_state, std::vector<double*> &result_particles, double& duration, double& Da)
{

	temp_state[0] = start_state[0]; temp_state[1] = start_state[1];
	for (int i = 0; i < number_of_particles; ++i)
	{
		temp_particles[i][0] = particles[i][0];
		temp_particles[i][1] = particles[i][1];
		temp_particles[i][2] = hill_height(particles[i]);
	}

	double temp_cost = 0;
	ConvexHull* conv;
	double init_vol = conv->ConvexHull_Volume(temp_particles);
	
	double theta = control[0]; double u = control[1];

	int num_steps = step_size;
	bool validity = true;
	for(int i=0;i<num_steps;i++)
	{		
		double dhdx = hill_gradient_x(temp_state);
		double dhdy = hill_gradient_y(temp_state);
		double delta_h = dhdx * cos(theta) + dhdy * sin(theta);
		// std::cout << "delta_h: " << delta_h << " -2/pi*atan(delta_h) " << -2/M_PI * atan(delta_h) << std::endl;
		// std::cout << "theta: " << theta << "cos: " << cos(theta) << "sin: " << sin(theta) << std::endl;
		temp_state[0] += params::integration_step * u * (-2/M_PI * atan(delta_h) + 1) * cos(theta);
		temp_state[1] += params::integration_step * u * (-2/M_PI * atan(delta_h) + 1) * sin(theta);
		enforce_bounds();
		
		//Propagate all particles (No bound enforcement)
		for (int j = 0; j < number_of_particles; ++j)
		{
			double temp_dhdx = hill_gradient_x(temp_particles[j]);
			double temp_dhdy = hill_gradient_y(temp_particles[j]);

			double temp_delta_h = temp_dhdx * cos(theta) + temp_dhdy * sin(theta);

			temp_particles[j][0] += params::integration_step * u * (-2/M_PI * atan(temp_delta_h) + 1) * cos(theta);
			temp_particles[j][1] += params::integration_step * u * (-2/M_PI * atan(temp_delta_h) + 1) * sin(theta);
			temp_particles[j][2] = hill_height(temp_particles[j]);
		}

		validity = validity && valid_state();

		if (validity) {
			temp_cost += conv->ConvexHull_Volume(temp_particles) * params::integration_step / init_vol;
		}
	}
	result_state[0] = temp_state[0];
	result_state[1] = temp_state[1];

	// std::cout << "climb_hill_t:: propagate_with_particles: result_state: " << result_state[0] << " " << result_state[1] << std::endl;
	// std::cout<<temp_particles.size()<<std::endl;
	for (int i = 0; i < number_of_particles; ++i)
	{
		result_particles[i][0] = temp_particles[i][0];
		result_particles[i][1] = temp_particles[i][1];

		// std::cout << "climb_hill_t:: propagate_with_particles: result_particles: " << result_particles[i][0] << " " << result_particles[i][1] << std::endl;
	}

	duration = num_steps*params::integration_step;
	Da = temp_cost;
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
	// return 3*point[1] + sin(point[0] + point[0]*point[1]);
	return 3*point[1] + sin(point[0] + point[0]*point[1]);
}

double climb_hill_t::hill_gradient_x(double* point) {
	// return cos(point[0] + point[0]*point[1])*(1+point[1]);
	return cos(point[0] + point[0]*point[1])*(1+point[1]);
}

double climb_hill_t::hill_gradient_y(double* point) {
	// return 3 + cos(point[0]+point[0]*point[1])*point[0];
	return 3 + cos(point[0]+point[0]*point[1])*point[0];
}

svg::Point climb_hill_t::visualize_point(double* state, svg::Dimensions dims)
{
	double x = (state[0]-MIN_X)/(MAX_X-MIN_X) * dims.width; 
	double y = (state[1]-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
	return svg::Point(x,y);
}