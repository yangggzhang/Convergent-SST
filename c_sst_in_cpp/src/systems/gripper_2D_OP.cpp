/**
 * @file climb_hill.cpp
 * 
 * 
 */

#include "systems/gripper_2D_OP.hpp"
#include "utilities/random.hpp"
#include "utilities/convexhull.hpp"
#include <openrave-core.h>

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

#define DEPTH_TOLERENCE 0.001
#define CLAW_GRIPPER_OFFSET 0.3

#define D_min std::numeric_limits<double>::min()
#define D_max std::numeric_limits<double>::max()

using namespace OpenRAVE;


double gripper_2D_OP_t::distance(double* point1,double* point2)
{
	return std::sqrt( (point1[0]-point2[0]) * (point1[0]-point2[0]) + (point1[1]-point2[1]) * (point1[1]-point2[1]) );
}

void gripper_2D_OP_t::random_particles(double* destination, double* state, double radius)
{
	destination[0] = state[0] + uniform_random(-radius, radius);
	destination[1] = state[1] + uniform_random(-radius, radius);
}

void gripper_2D_OP_t::random_state(double* state)
{
	state[0] = uniform_random(MIN_X,MAX_X);
	state[1] = uniform_random(MIN_Y,MAX_Y);
}

void gripper_2D_OP_t::random_control(double* control)
{
	control[0] = uniform_random(MIN_SPEED, MAX_SPEED);
	control[1] = uniform_random(MIN_SPEED, MAX_SPEED);
}

bool gripper_2D_OP_t::propagate( double* start_state, double* control, int min_step, int max_step, double* result_state, double& duration )
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
		if(validity == false) return validity;
		check_collision(temp_state);
		// if (check_collision(temp_state))
		// {
		// 	temp_state[0] += -params::integration_step * ux;
		// 	temp_state[1] += -params::integration_step * uy;
		// 	temp_state[2] += -params::integration_step * uz;
		// }
	}
	result_state[0] = temp_state[0];
	result_state[1] = temp_state[1];

	duration = num_steps*params::integration_step;
	return validity;
}

bool gripper_2D_OP_t::convergent_propagate( const bool &random_time, double* start_state,std::vector<double*> &start_particles, double* control, int min_step, int max_step, double* result_state, std::vector<double*> &result_particles, double& duration, double& cost )
{	
	double local_cost = 0;
	temp_state[0] = start_state[0]; temp_state[1] = start_state[1];

	double ux = control[0]; double uy = control[1];

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
		}

		for (size_t i = 0; i < start_particles.size(); ++i)
		{
			double alpha = 1 + 1000*portion_in_collision(temp_state, temp_particles[i]);
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
		if(validity == false) return validity;
		check_collision(temp_state);
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
				check_collision(temp_particles[j]);
				// if (check_collision(temp_particles[j]))
				// {
				// 	temp_particles[j][0] += -params::integration_step * ux;
				// 	temp_particles[j][1] += -params::integration_step * uy;
				// 	temp_particles[j][2] += -params::integration_step * uz;
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
			double alpha = 1 + 1000*portion_in_collision(temp_state, temp_particles[j]);
			end_De += alpha * distance(temp_state, temp_particles[j]);
		}
	}

	result_state[0] = temp_state[0];
	result_state[1] = temp_state[1];

	duration = num_steps*params::integration_step;
	// local_cost = duration; //default setting
	local_cost = (init_De+end_De)/2*duration;

	
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

void gripper_2D_OP_t::enforce_bounds(double* state)
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

bool gripper_2D_OP_t::check_collision(double* state)
{
	// std::cout << "Collision_checking" << std::endl;
	std::vector<dReal> values;
	values.resize(state_dimension);
	for(int i = 0; i < state_dimension; ++i) {
		if(probot->GetName() == "4Claw-Gripper"){
			values[i] = state[i] - CLAW_GRIPPER_OFFSET;
		}
		else {
			values[i] = state[i];
		}
		// std::cout << values[i] << ", ";
	}
	// std::cout << std::endl;
	probot->SetActiveDOFValues(values,true);

	// std::vector<dReal> values_temp;
	// probot->GetActiveDOFValues(values_temp);
	// for(int i = 0; i < values_temp.size(); ++i) {
	// 	std::cout<< values_temp[i] <<", ";
	// }
	// std::cout << std::endl;

	CollisionReportPtr report(new CollisionReport());
	penv->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);

	bool obstacle_collision = false;
	while(penv->CheckCollision(probot,report)){
		int contactpoints = (int) report->contacts.size();
		if (contactpoints <= 1) break;
		double depth_max = -0.0;
		double normx = 0.0;
		double normy = 0.0;
		for (int i = 0; i < contactpoints; ++i){
			CollisionReport::CONTACT& c = report->contacts[i];
			// std::cout << "contact " << i << "depth: " << c.depth << std::endl;
			if(fabs(depth_max) < fabs(c.depth)){
				depth_max = c.depth;
				normx = c.norm.x; normy = c.norm.y;
			}
			// std::cout << "contact" << i << ": pos=("
			// 	<< c.pos.x << ", " << c.pos.y << ", " << c.pos.z << "), norm=("
			// 	<< c.norm.x << ", " << c.norm.y << ", " << c.norm.z << ")" << std::endl;
		}

		if (fabs(depth_max) < DEPTH_TOLERENCE) break;

		state[0] += depth_max * normx; state[1] += depth_max * normy;
		// std::cout << "depth: " << depth_max << std::endl;
		// std::cout << "norm: " << normx << ", " << normy << ", " << normz << std::endl;
		for(int i = 0; i < state_dimension; ++i) {
			if(probot->GetName() == "4Claw-Gripper"){
				values[i] = state[i] - CLAW_GRIPPER_OFFSET;
			}
			else {
				values[i] = state[i];
			}
			// std::cout << values[i] << ", ";
		}
		// std::cout << std::endl;
		probot->SetActiveDOFValues(values,true);

		obstacle_collision = true;
	}

	return obstacle_collision;
}

double gripper_2D_OP_t::portion_in_collision(double* point1, double* point2)
{
	double total_dist = distance(point1, point2);
	double colli_dist = 0.;
	RaveVector<double> RV1(point1[0], point1[1], 0.0);
	RaveVector<double> RV2(point2[0], point2[1], 0.0);
	geometry::ray<double> ray1(RV1, RV2 - RV1);
	geometry::ray<double> ray2(RV2, RV1 - RV2);
	CollisionReportPtr report1(new CollisionReport());
	CollisionReportPtr report2(new CollisionReport());
	penv->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);

	for(int i = 1; i < pbodies.size(); ++i) { //From 1 because 0 is the robot
		penv->CheckCollision(ray1, pbodies[i], report1);
		if(report1->contacts.size() == 0) continue;
		penv->CheckCollision(ray2, pbodies[i], report2);
		if(report2->contacts.size() == 0) continue; 
		double contact1[3] = {report1->contacts[0].pos.x, report1->contacts[0].pos.y, report1->contacts[0].pos.z};
		double contact2[3] = {report2->contacts[0].pos.x, report2->contacts[0].pos.y, report2->contacts[0].pos.z};
		colli_dist += distance(contact1, contact2);
	}

	if(colli_dist/total_dist >= 1) std::cout << "Something wrong in portion_in_collision calculation." << std::endl;
	// if(colli_dist/total_dist > 0 ) {
	// 	std::cout << "Portion in collision: " <<  colli_dist/total_dist << std::endl;
	// 	std::cout << point1[0] << ", " << point1[1] << ", " << point1[2] << ", " ;
	// 	std::cout << point2[0] << ", " << point2[1] << ", " << point2[2] << std::endl;
	// }
	return colli_dist/total_dist;

}

bool gripper_2D_OP_t::valid_state()
{
	return 	(temp_state[0]!=MIN_X) &&
			(temp_state[0]!=MAX_X) &&
			(temp_state[1]!=MIN_Y) &&
			(temp_state[1]!=MAX_Y);
}


svg::Point gripper_2D_OP_t::visualize_point(double* state, svg::Dimensions dims)
{
	double x = (state[0]-MIN_X)/(MAX_X-MIN_X) * dims.width; 
	double y = (state[1]-MIN_Y)/(MAX_Y-MIN_Y) * dims.height; 
	// double y = (state[2]-MIN_Z)/(MAX_Z-MIN_Z) * dims.height; 
	return svg::Point(x,y);
}

std::string gripper_2D_OP_t::export_point(double* state)
{
	std::stringstream s;
	s << state[0] << "," << state[1] << std::endl;
	return s.str();
}

void gripper_2D_OP_t::load_openrave() 
{
    RaveInitialize(true); 
	penv = RaveCreateEnvironment();

	pchecker = RaveCreateCollisionChecker(penv,"ode");
	if( !pchecker ) {
		RAVELOG_ERROR("failed to create checker\n");
		return;
	}
	penv->SetCollisionChecker(pchecker);

	if(!penv->Load("../OpenraveEnv/gripper_sys_4claw_simplified.env.xml")) {
		std::cout << "gripper.cpp:: Error loading scene.";
		return;
	}

	EnvironmentMutex::scoped_lock lock(penv->GetMutex());

	std::vector<RobotBasePtr> vrobots;
	penv->GetRobots(vrobots);
	// get the first body
	if( vrobots.size() == 0 ) {
		RAVELOG_ERROR("no robots loaded\n");
		return;
	}
	probot = vrobots.at(0);

	penv->GetBodies(pbodies);

	std::vector<int> v;
	v.clear();
	probot->SetActiveDOFs(v, DOF_X | DOF_Y);
	std::cout << "Robot's name: " << probot->GetName() << std::endl;
	std::cout << "Robot's Active DOF: " << probot->GetActiveDOF() << std::endl;
	
	return;
}
