/**
 * @file climb_hill.cpp
 * 
 * 
 */

#include "systems/gripper_2D_EG.hpp"
#include "utilities/random.hpp"
#include "utilities/convexhull.hpp"


#define _USE_MATH_DEFINES

#include <cmath>
#include <limits>
#include <math.h>
#include <sstream>

#define MIN_X -4
#define MAX_X 4
#define MIN_Y -2
#define MAX_Y 3

#define MIN_SPEED -0.5
#define MAX_SPEED 0.5

double gripper_2D_EG_t::distance(double* point1,double* point2)
{
	return std::sqrt( (point1[0]-point2[0]) * (point1[0]-point2[0]) + (point1[1]-point2[1]) * (point1[1]-point2[1]));
}

void gripper_2D_EG_t::random_particles(double* destination, double* state, double radius)
{
	double theta = uniform_random(0, 2*M_PI);
	double p = uniform_random(0,1);
	destination[0] = state[0] + p*radius * cos(theta); 
	destination[1] = state[1] + p*radius * sin(theta);
}

void gripper_2D_EG_t::random_state(double* state)
{
	state[0] = uniform_random(MIN_X,MAX_X);
	state[1] = uniform_random(MIN_Y,MAX_Y);
}

void gripper_2D_EG_t::random_control(double* control)
{
	if (flag == false) {
		control[0] = MAX_SPEED/2;
		control[1] = 0;
		count++;
	}
	else {
		control[0] = MAX_SPEED/2;
		control[1] = 0.2*MAX_SPEED/2;
	}

	if (count >= 1) flag = true;

	// control[0] = uniform_random(MIN_SPEED, MAX_SPEED);
	// control[1] = uniform_random(MIN_SPEED, MAX_SPEED);
}

bool gripper_2D_EG_t::propagate( double* start_state, double* control, int min_step, int max_step, double* result_state, double& duration )
{	
	temp_state[0] = start_state[0]; temp_state[1] = start_state[1];

	double ux = control[0]; double uy = control[1];

	int num_steps = uniform_int_random(min_step,max_step);
	bool validity = true;
	for(int i=0;i<num_steps;i++)
	{		
		temp_state[0] += params::integration_step * ux;
		temp_state[1] += params::integration_step * uy;
		enforce_bounds(temp_state);
		validity = validity && valid_state();
		check_collision(temp_state);
		// if (check_collision(temp_state))
		// {
		// 	temp_state[0] += -params::integration_step * ux;
		// 	temp_state[1] += -params::integration_step * uy;
		// }
	}
	result_state[0] = temp_state[0];
	result_state[1] = temp_state[1];

	duration = num_steps*params::integration_step;
	return validity;
}

bool gripper_2D_EG_t::convergent_propagate( const bool &random_time, double* start_state,std::vector<double*> &start_particles, double* control, int min_step, int max_step, double* result_state, std::vector<double*> &result_particles, double& duration, double& cost )
{	
	double local_cost = 0;
	temp_state[0] = start_state[0]; temp_state[1] = start_state[1];

	double ux = control[0]; double uy = control[1];

	// std::cout << "ux: " << ux << " uy: " << uy << " uz: " << uz << std::endl;

	int num_steps = 200; // adjust time stamp according to input
	// if (random_time)	num_steps = uniform_int_random(min_step,max_step);
	// else num_steps =  (int)(min_step + max_step)/2;

	bool validity = true;

	double init_De = 0;
	if (start_particles.size() > 0)
	{	
		for (size_t i = 0; i < start_particles.size(); i++)
		{
			temp_particles[i][0] = start_particles[i][0];
			temp_particles[i][1] = start_particles[i][1];
		}

		for (size_t i = 0; i < start_particles.size(); ++i)
		{
			double alpha = 1 + 10*portion_in_collision(temp_state, temp_particles[i]);
			init_De += alpha * distance(temp_state, temp_particles[i]);
		}
	}

	//std::cout<<"update state"<<std::endl;
	//propagate state
	for(int i=0;i<num_steps;i++)
	{		

		temp_state[0] += params::integration_step * ux;
		temp_state[1] += params::integration_step * uy;
		enforce_bounds(temp_state);
		validity = validity && valid_state();
		check_collision(temp_state);
		// if (enforce_obstacle(temp_state))
		// {
		// 	temp_state[0] += -params::integration_step * ux;
		// 	temp_state[1] += -params::integration_step * uy;
		// }

		if (start_particles.size() > 0)
		{
			for (size_t j = 0; j < start_particles.size(); j++)
			{
				temp_particles[j][0] += params::integration_step * ux;
				temp_particles[j][1] += params::integration_step * uy;
				check_collision(temp_particles[j]);
				// if (check_collision(temp_particles[j]))
				// {
				// 	temp_particles[j][0] += -params::integration_step * ux;
				// 	temp_particles[j][1] += -params::integration_step * uy;
				// }
				
				// local_cost += distance(temp_state, temp_particles[j]) * params::integration_step;
			}
		}

	}

	double end_De = 0;
	if (start_particles.size() > 0)
	{
		for (size_t j = 0; j < start_particles.size(); j++)
		{
			double alpha = 1 + 10*portion_in_collision(temp_state, temp_particles[j]);
			end_De += alpha * distance(temp_state, temp_particles[j]);
		}
	}

	result_state[0] = temp_state[0];
	result_state[1] = temp_state[1];

	std::cout << "Start De: " << init_De << ", End De: " << end_De << std::endl;

	duration = num_steps*params::integration_step;
	// local_cost = duration; //default setting
	local_cost = (init_De+end_De)/2*duration;
	// local_cost = log(end_De/init_De);

	
	if ( start_particles.size() > 0 ) 
	{
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

void gripper_2D_EG_t::enforce_bounds(double* state)
{
	if(state[0]<MIN_X)
		state[0]=MIN_X;
	else if(state[0]>MAX_X)
		state[0]=MAX_X;

	if(state[1]<MIN_Y)
		state[1]=MIN_Y;
	else if(state[1]>MAX_Y)
		state[1]=MAX_Y;
}

bool gripper_2D_EG_t::check_collision(double* state)
{
	bool obstacle_collision = false;
	for(unsigned i=0;i<obstacles.size() && !obstacle_collision;i++)
	{
		double corners[2];
		double curv_cen[2];
		int pos_record = 0; //1: upper right, 2: lower right, 3: upper left, 4: lower left
		if(	state[0]>obstacles[i].low_x && state[0]<obstacles[i].high_x && 
			state[1]>obstacles[i].low_y && state[1]<obstacles[i].high_y)
		{
			pos_record = location_in_obstacle(state, i, corners, curv_cen);

			if( distance(corners, state) < obstacles[i].curv_radi && 
			    distance(curv_cen, state) > obstacles[i].curv_radi)
			{
				continue;
			}
			else
			{
				double lower_angle_bound = 0;
				double upper_angle_bound = 0;
				switch (pos_record)
				{
					case 1: // Upper Right
					lower_angle_bound = 0;
					upper_angle_bound = M_PI/2;
					enforce_obstacle_bounds(state, i, corners, curv_cen, lower_angle_bound, upper_angle_bound);
					break;

					case 2: // Lower Right
					lower_angle_bound = -M_PI/2;
					upper_angle_bound = 0;
					enforce_obstacle_bounds(state, i, corners, curv_cen, lower_angle_bound, upper_angle_bound);
					break;

					case 3: // Upper Left
					lower_angle_bound = M_PI/2;
					upper_angle_bound = M_PI;
					enforce_obstacle_bounds(state, i, corners, curv_cen, lower_angle_bound, upper_angle_bound);
					break;

					case 4: // Lower Left
					lower_angle_bound = -M_PI;
					upper_angle_bound = -M_PI/2;
					enforce_obstacle_bounds(state, i, corners, curv_cen, lower_angle_bound, upper_angle_bound);
					break;

					default:
					std::cout << "[gripper_2D.cpp] Pos_record is not valid." << std::endl;
				}
			}

			obstacle_collision = true;
		}
	}

	return obstacle_collision;
}

int gripper_2D_EG_t::location_in_obstacle(double* state, int obstacles_id, double* corners, double* curv_cen)
{
	int pos_record = 0;
	if (state[0] > obstacles[obstacles_id].cen_x) // In the right side
	{
		if (state[1] > obstacles[obstacles_id].cen_y) // In the upper right side
		{
			pos_record = 1;
			corners[0] = obstacles[obstacles_id].high_x; corners[1] = obstacles[obstacles_id].high_y;
			curv_cen[0] = corners[0] - obstacles[obstacles_id].curv_radi; curv_cen[1] = corners[1] - obstacles[obstacles_id].curv_radi;
		}
		else // In the lower right side
		{
			pos_record = 2;
			corners[0] = obstacles[obstacles_id].high_x; corners[1] = obstacles[obstacles_id].low_y;
			curv_cen[0] = corners[0] - obstacles[obstacles_id].curv_radi; curv_cen[1] = corners[1] + obstacles[obstacles_id].curv_radi;
		}
	}
	else // In the left hand side
	{
		if (state[1] > obstacles[obstacles_id].cen_y) // In the upper left side
		{
			pos_record = 3;
			corners[0] = obstacles[obstacles_id].low_x; corners[1] = obstacles[obstacles_id].high_y;
			curv_cen[0] = corners[0] + obstacles[obstacles_id].curv_radi; curv_cen[1] = corners[1] - obstacles[obstacles_id].curv_radi;
		}
		else // In the lower left side
		{
			pos_record = 4;
			corners[0] = obstacles[obstacles_id].low_x; corners[1] = obstacles[obstacles_id].low_y;
			curv_cen[0] = corners[0] + obstacles[obstacles_id].curv_radi; curv_cen[1] = corners[1] + obstacles[obstacles_id].curv_radi;
		}
	}

	return pos_record;
}

int gripper_2D_EG_t::enforce_obstacle_bounds(double* state, int obstacles_id, double* corners, double* curv_cen, double lower_angle_bound, double upper_angle_bound)
{
	int enforce_pos = -1;
	double angle2curv = std::atan2(state[1]-curv_cen[1], state[0]-curv_cen[0]);
	if (angle2curv > lower_angle_bound && angle2curv < upper_angle_bound)
	{
		state[0] = curv_cen[0] + cos(angle2curv) * obstacles[obstacles_id].curv_radi;
		state[1] = curv_cen[1] + sin(angle2curv) * obstacles[obstacles_id].curv_radi;
		enforce_pos = 0;
	}
	else
	{
		if (fabs(state[0] - corners[0]) < fabs(state[1]-corners[1])) 
			state[0] = corners[0];
		else
			state[1] = corners[1];

		enforce_pos = 1;
	}

	return enforce_pos;
}

bool gripper_2D_EG_t::valid_state()
{
	return 	(temp_state[0]!=MIN_X) &&
			(temp_state[0]!=MAX_X) &&
			(temp_state[1]!=MIN_Y) &&
			(temp_state[1]!=MAX_Y);
}

double gripper_2D_EG_t::cost_function(double* state, std::vector<double*> particles) {
	double var_cost = 0.0;
	for (size_t i = 0; i < particles.size(); i++)
	{
		double alpha = 1 + 10*portion_in_collision(state, particles[i]);
		var_cost += alpha * distance(state, particles[i]);
	}
	return var_cost;
}

double gripper_2D_EG_t::portion_in_collision(double* point1, double* point2)
{
	// line approximation: a*x + b*y + c = 0
	double a,b,c;
	if(point1[0] == point2[0] && point1[1] == point2[1])
		return 0;
	else if(point1[0] == point2[0])
	{
		a = 1; b = 0; c = -point1[0];
	}
	else
	{
		b = 1; a = - (point1[1]-point2[1])/(point1[0]-point2[0]); c = -a*point1[0]-b*point1[1];
	}

	double portion = 0;
	double temp_min_x = std::min(point1[0], point2[0]);
	double temp_max_x = std::max(point1[0], point2[0]);
	double temp_min_y = std::min(point1[1], point2[1]);
	double temp_max_y = std::max(point1[1], point2[1]);
	for(unsigned i=0;i<obstacles.size();i++)
	{
		std::vector<double> pos_neg_check; //Order: Top-right, top-left, bot-left, bot-right		
		std::vector<double> contact_pt_x;
		std::vector<double> contact_pt_y;

		double temp_val = a*obstacles[i].high_x + b*obstacles[i].high_y + c;
		pos_neg_check.push_back(temp_val);
		if (temp_val == 0)
		{
			contact_pt_x.push_back(obstacles[i].high_x);
			contact_pt_y.push_back(obstacles[i].high_y);
			if(contact_pt_x.back()< temp_min_x || contact_pt_x.back() > temp_max_x ||
			   contact_pt_y.back()< temp_min_y || contact_pt_y.back() > temp_max_y ||
			   a == 0 || b == 0)
			    continue;
		}

		temp_val = a*obstacles[i].low_x + b*obstacles[i].high_y + c;
		pos_neg_check.push_back(temp_val);
		if(temp_val == 0)
		{
			contact_pt_x.push_back(obstacles[i].low_x);
			contact_pt_y.push_back(obstacles[i].high_y);
			if(contact_pt_x.back()< temp_min_x || contact_pt_x.back() > temp_max_x ||
			   contact_pt_y.back()< temp_min_y || contact_pt_y.back() > temp_max_y ||
			   a == 0 || b == 0)
				continue;
		}

		temp_val = a*obstacles[i].low_x + b*obstacles[i].low_y + c;
		pos_neg_check.push_back(temp_val);
		if(temp_val == 0)
		{
			contact_pt_x.push_back(obstacles[i].low_x);
			contact_pt_y.push_back(obstacles[i].low_y);
			if(contact_pt_x.back()< temp_min_x || contact_pt_x.back() > temp_max_x ||
			   contact_pt_y.back()< temp_min_y || contact_pt_y.back() > temp_max_y ||
			   a == 0 || b == 0)
			    continue;
		}		

		temp_val = a*obstacles[i].high_x + b*obstacles[i].low_y + c;
		pos_neg_check.push_back(temp_val);
		if(temp_val == 0)
		{
			contact_pt_x.push_back(obstacles[i].high_x);
			contact_pt_y.push_back(obstacles[i].low_y);
			if(contact_pt_x.back()< temp_min_x || contact_pt_x.back() > temp_max_x ||
			   contact_pt_y.back()< temp_min_y || contact_pt_y.back() > temp_max_y ||
			   a == 0 || b == 0)
			    continue;
		}

		//Start checking where the collision happens
		if(pos_neg_check[0] * pos_neg_check[1] < 0)
		{
			contact_pt_x.push_back((-b*obstacles[i].high_y-c)/a);
			contact_pt_y.push_back(obstacles[i].high_y);
			if(contact_pt_x.back()< temp_min_x || contact_pt_x.back() > temp_max_x ||
			   contact_pt_y.back()< temp_min_y || contact_pt_y.back() > temp_max_y)
			    continue;
		}
		if(pos_neg_check[1]*pos_neg_check[2] < 0)
		{
			contact_pt_x.push_back(obstacles[i].low_x);
			contact_pt_y.push_back((-a*obstacles[i].low_x-c)/b);
			if(contact_pt_x.back()< temp_min_x || contact_pt_x.back() > temp_max_x ||
			   contact_pt_y.back()< temp_min_y || contact_pt_y.back() > temp_max_y)
			    continue;
		}
		if(pos_neg_check[2] * pos_neg_check[3] < 0)
		{
			contact_pt_x.push_back((-b*obstacles[i].low_y-c)/a);
			contact_pt_y.push_back(obstacles[i].low_y);
			if(contact_pt_x.back()< temp_min_x || contact_pt_x.back() > temp_max_x ||
			   contact_pt_y.back()< temp_min_y || contact_pt_y.back() > temp_max_y)
			    continue;
		}
		if(pos_neg_check[3]*pos_neg_check[0] < 0)
		{
			contact_pt_x.push_back(obstacles[i].high_x);
			contact_pt_y.push_back((-a*obstacles[i].high_x-c)/b);
			if(contact_pt_x.back()< temp_min_x || contact_pt_x.back() > temp_max_x ||
			   contact_pt_y.back()< temp_min_y || contact_pt_y.back() > temp_max_y)
			    continue;
		}

		if (contact_pt_x.size() != 0 && contact_pt_y.size() != 0)
		{
			double contact_pt1[] = {contact_pt_x[0], contact_pt_y[0]};
			double contact_pt2[] = {contact_pt_x[1], contact_pt_y[1]};
			// std::cout << contact_pt_x.size() << std::endl;
			// std::cout << contact_pt1[0] << ", " << contact_pt1[1] << std::endl;
			// std::cout << contact_pt2[0] << ", " << contact_pt2[1] << std::endl;
			portion += distance(contact_pt1, contact_pt2);
		}
	}

	portion = portion/distance(point1,point2);

	// if(portion > 0)
	// {
	// 	std::cout << point1[0] << ", " << point1[1] << ", " << point2[0] << ", " << point2[1] << ": " << portion << std::endl;
	// }

	return portion;

}

svg::Point gripper_2D_EG_t::visualize_point(double* state, svg::Dimensions dims)
{
	double x = (state[0]-MIN_X)/(MAX_X-MIN_X) * dims.width; 
	double y = (state[1]-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
	// double y = (state[2]-MIN_Z)/(MAX_Z-MIN_Z) * dims.height; 
	return svg::Point(x,y);
}

std::string gripper_2D_EG_t::export_point(double* state)
{
	std::stringstream s;
	s << state[0] << "," << state[1] << std::endl;
	return s.str();
}