/**
 * @file run.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "utilities/parameter_reader.hpp"
#include "utilities/condition_check.hpp"
#include "utilities/random.hpp"
#include "utilities/timer.hpp"

#include "systems/climb_hill.hpp"
#include "systems/gripper.hpp"
#include "systems/gripper_2D.hpp"
#include "systems/gripper_2D_OP.hpp"
#include "motion_planners/sst.hpp"
#include "motion_planners/rrt.hpp"

#include <openrave/openrave.h>

#include <iostream>
#include "omp.h"

#define NUM_THREADS 2

bool check_collision_parallel(double* state, int ID, EnvironmentBasePtr temp_penv, RobotBasePtr temp_probot);

int main(int ac, char* av[])
{

	RaveInitialize(true); 
	EnvironmentBasePtr penv = RaveCreateEnvironment();

	CollisionCheckerBasePtr pchecker = RaveCreateCollisionChecker(penv,"ode");
	if( !pchecker ) {
		RAVELOG_ERROR("failed to create checker\n");
		return 0;
	}
	penv->SetCollisionChecker(pchecker);
	penv->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);

	if(!penv->Load("/home/parallels/Desktop/Convergent-SST/c_sst_in_cpp/OpenraveEnv/gripper_sys_puma.env.xml")) {
		std::cout << "gripper.cpp:: Error loading scene.";
		return 0;
	}

	// EnvironmentMutex::scoped_lock lock(penv->GetMutex());

	std::vector<RobotBasePtr> vrobots;
	penv->GetRobots(vrobots);
	// get the first body
	if( vrobots.size() == 0 ) {
		RAVELOG_ERROR("no robots loaded\n");
		return 0;
	}
	RobotBasePtr probot = vrobots.at(0);

	std::vector<int> v;
	v.clear();
	probot->SetActiveDOFs(v, DOF_X | DOF_Y);
	std::cout << "Robot's name: " << probot->GetName() << std::endl;
	std::cout << "Robot's Active DOF: " << probot->GetActiveDOF() << std::endl;

	// EnvironmentBasePtr clone_penv = penv->CloneSelf(Clone_Bodies);
	EnvironmentBasePtr clone_penv = RaveCreateEnvironment();
	CollisionCheckerBasePtr clone_pchecker = RaveCreateCollisionChecker(clone_penv,"ode");
	if( !clone_pchecker ) {
		RAVELOG_ERROR("failed to create checker\n");
		return 0;
	}
	clone_penv->SetCollisionChecker(clone_pchecker);
	clone_penv->GetCollisionChecker()->SetCollisionOptions(CO_Contacts);

	if(!clone_penv->Load("/home/parallels/Desktop/Convergent-SST/c_sst_in_cpp/OpenraveEnv/gripper_sys_4claw.env.xml")) {
		std::cout << "gripper.cpp:: Error loading scene.";
		return 0;
	}

	// EnvironmentMutex::scoped_lock clone_lock(clone_penv->GetMutex());

	vrobots.clear();
	clone_penv->GetRobots(vrobots);
	// get the first body
	if( vrobots.size() == 0 ) {
		RAVELOG_ERROR("no robots loaded\n");
		return 0;
	}
	RobotBasePtr clone_probot = vrobots.at(0);

	clone_probot->SetActiveDOFs(v, DOF_X | DOF_Y);
	std::cout << "Robot's name: " << clone_probot->GetName() << std::endl;
	std::cout << "Robot's Active DOF: " << clone_probot->GetActiveDOF() << std::endl;

	std::vector<double*> temp_particles;
	temp_particles.clear();
	for (int i = 0; i < 50; ++i)
	{
		temp_particles.push_back(new double[2]); //+1 to store the height
	}

	omp_set_num_threads(NUM_THREADS);
	double start_time = omp_get_wtime();
	// printf("%d\n",omp_get_max_threads());
	// printf("Num Threads:%d\n ",omp_get_num_threads());
	#pragma omp parallel
	{
		// printf("Num Threads:%d\n ",omp_get_num_threads());
		int ID = omp_get_thread_num();
		#pragma omp for schedule(auto)
		for (size_t j = 0; j < 1000000; j++)
		{
			// printf("j: %d, ID: %d\n", j, ID);
			// temp_particles[j][0] += params::integration_step * ux;
			// temp_particles[j][1] += params::integration_step * uy;
			if(ID == 0) 
				{
					// printf("One\n");
					check_collision_parallel(temp_particles[j], ID, penv, probot);
				}
			else {
				// printf("Two\n");
				check_collision_parallel(temp_particles[j], ID, clone_penv, clone_probot);
			}
		}
	}
	double end_time = omp_get_wtime();
	std::cout << "Parallel with " << NUM_THREADS << " threads: " << end_time - start_time << std::endl;

	start_time = omp_get_wtime();

	for (size_t j = 0; j < 1000000; j++)
	{
		check_collision_parallel(temp_particles[j], 0, clone_penv, clone_probot);
	}

	end_time = omp_get_wtime();
	std::cout << "Serial computation: " << end_time - start_time << std::endl;

	return 0;

	/////////////////////////////////////////////////////////////////////////////////
	read_parameters(ac,av);
	//****************After reading in from input, we need to instantiate classes
	init_random(time(NULL));
	system_t* system;
	if(params::system=="climb_hill")
	{
		if(params::number_of_particles == 0) system = new climb_hill_t();
		else system = new climb_hill_t(params::number_of_particles);
	}
	else if(params::system=="gripper")
	{
		if(params::number_of_particles == 0) system = new gripper_t();
		else system = new gripper_t(params::number_of_particles);
	}
	else if(params::system=="gripper_2D")
	{
		if(params::number_of_particles ==0) system = new gripper_2D_t();
		else system = new gripper_2D_t(params::number_of_particles);
	}
	else if(params::system=="gripper_2D_OP")
	{
		if(params::number_of_particles ==0) system = new gripper_2D_OP_t();
		else system = new gripper_2D_OP_t(params::number_of_particles, NUM_THREADS);
	}


	planner_t* planner;
	if(params::planner=="rrt")
	{
		if (params::number_of_particles != 0 && params::number_of_control != 0)
		{
			planner = new rrt_t(system, params::number_of_particles, params::particle_radius,params::number_of_control);
		}
		else planner = new rrt_t(system);
		
	}
	else if(params::planner=="sst")
	{
		if(params::number_of_particles == 0) planner = new sst_t(system);
		else planner = new sst_t(system, params::number_of_particles, params::particle_radius,params::number_of_control);
	}

	planner->set_start_state(params::start_state);
	planner->set_goal_state(params::goal_state,params::goal_radius);
	planner->setup_planning();

	condition_check_t checker(params::stopping_type,params::stopping_check);
	condition_check_t* stats_check=NULL;
	if(params::stats_check!=0)
	{
		stats_check = new condition_check_t(params::stats_type,params::stats_check);
	}

	checker.reset();
	std::cout<<"Starting the planner: "<<params::planner<<" for the system: "<<params::system<<std::endl;
	if(stats_check==NULL)
	{
		do
		{
			planner->step();
			// std::cout << "Finish one step" << std::endl;
		}
		while(!checker.check());
		std::vector<std::pair<double*,double> > controls;
		planner->get_solution(controls);
		double solution_cost = 0;
		for(unsigned i=0;i<controls.size();i++)
		{
			solution_cost+=controls[i].second;
		}
		std::cout<<checker.time()<<" "<<checker.iterations()<<" "<<planner->number_of_nodes<<" " <<solution_cost<<std::endl ;
		std::cout<<"check!!!"<<params::trial<<std::endl;
		// planner->visualize_tree(params::trial);
		// planner->visualize_nodes(params::trial);
		planner->export_solution_path(params::trial);
		// planner->export_nodes(params::trial);
		// planner->export_tree(params::trial);
	}
	else
	{
		int count = 0;
		bool execution_done = false;
		bool stats_print = false;
		while(true)
		{
			do
			{
				planner->step();
				execution_done = checker.check();
				stats_print = stats_check->check();
			}
			while(!execution_done && !stats_print);
			if(stats_print)
			{
				std::vector<std::pair<double*,double> > controls;
				planner->get_solution(controls);
				double solution_cost = 0;
				for(unsigned i=0;i<controls.size();i++)
				{
					solution_cost+=controls[i].second;
				}
				// std::cout<<checker.time()<<" "<<checker.iterations()<<" "<<planner->number_of_nodes<<" " <<solution_cost<<std::endl ;
				std::cout<<"Time: "<<checker.time()<<" Iterations: "<<checker.iterations()<<" Nodes: "<<planner->number_of_nodes<<" Solution Quality: " <<solution_cost<<std::endl ;
				stats_print = false;
				if(params::intermediate_visualization)
				{
					// planner->visualize_tree(count);
					// planner->visualize_nodes(count);
					planner->export_solution_path(count);
					planner->export_nodes(count);
					// planner->export_tree(count);
					count++;
				}				
				stats_check->reset();
			}
			if (execution_done)
			{
				std::vector<std::pair<double*,double> > controls;
				planner->get_solution(controls);
				double solution_cost = 0;
				for(unsigned i=0;i<controls.size();i++)
				{
					solution_cost+=controls[i].second;
				}
				// std::cout<<checker.time()<<" "<<checker.iterations()<<" "<<planner->number_of_nodes<<" " <<solution_cost<<std::endl ;
				std::cout<<"Time: "<<checker.time()<<" Iterations: "<<checker.iterations()<<" Nodes: "<<planner->number_of_nodes<<" Solution Quality: " <<solution_cost<<std::endl ;
				// planner->visualize_tree(params::trial);
				// planner->visualize_nodes(params::trial);
				planner->export_solution_path(params::trial);
				// planner->export_nodes(params::trial);
				// planner->export_tree(params::trial);
				break;
			}
		}
	}
	std::cout<<"Done planning."<<std::endl;

}



bool check_collision_parallel(double* state, int ID, EnvironmentBasePtr temp_penv, RobotBasePtr temp_probot)
{
	// std::cout << "Collision_checking" << std::endl;
	// std::cout << ID << std::endl;
	std::vector<dReal> values;
	values.resize(2);
	for(int i = 0; i < 2; ++i) {
		// if(clone_probot[ID]->GetName() == "4Claw-Gripper"){
			// values[i] = state[i] - CLAW_GRIPPER_OFFSET;
		// }
		// else {
			values[i] = -1;
			// values[i] = state[i];
		// }
		// std::cout << values[i] << ", ";
	}

	// EnvironmentMutex::scoped_lock lock(temp_penv->GetMutex());
	temp_probot->SetActiveDOFValues(values,true);
	// std::cout << std::endl;
	// if (ID == 0) probot->SetActiveDOFValues(values,true);
	// else clone_probot->SetActiveDOFValues(values,true);

	// std::vector<dReal> values_temp;
	// probot->GetActiveDOFValues(values_temp);
	// for(int i = 0; i < values_temp.size(); ++i) {
	// 	std::cout<< values_temp[i] <<", ";
	// }
	// std::cout << std::endl;

	CollisionReportPtr report(new CollisionReport());

	bool obstacle_collision = false;
	#pragma omp critical
	{
	temp_penv->CheckCollision(temp_probot, report);
	}
	// if(ID == 0) penv->CheckCollision(probot);
	// else clone_penv->CheckCollision(clone_probot);
	// while(clone_penv[ID]->CheckCollision(clone_probot[ID],report)){
	// 	int contactpoints = (int) report->contacts.size();
	// 	if (contactpoints <= 1) break;
	// 	double depth_max = -0.0;
	// 	double normx = 0.0;
	// 	double normy = 0.0;
	// 	for (int i = 0; i < contactpoints; ++i){
	// 		CollisionReport::CONTACT& c = report->contacts[i];
	// 		// std::cout << "contact " << i << "depth: " << c.depth << std::endl;
	// 		if(fabs(depth_max) < fabs(c.depth)){
	// 			depth_max = c.depth;
	// 			normx = c.norm.x; normy = c.norm.y;
	// 		}
	// 		// std::cout << "contact" << i << ": pos=("
	// 		// 	<< c.pos.x << ", " << c.pos.y << ", " << c.pos.z << "), norm=("
	// 		// 	<< c.norm.x << ", " << c.norm.y << ", " << c.norm.z << ")" << std::endl;
	// 	}

	// 	if (fabs(depth_max) < DEPTH_TOLERENCE) break;

	// 	state[0] += depth_max * normx; state[1] += depth_max * normy;
	// 	// std::cout << "depth: " << depth_max << std::endl;
	// 	// std::cout << "norm: " << normx << ", " << normy << ", " << normz << std::endl;
	// 	for(int i = 0; i < state_dimension; ++i) {
	// 		if(clone_probot[ID]->GetName() == "4Claw-Gripper"){
	// 			values[i] = state[i] - CLAW_GRIPPER_OFFSET;
	// 		}
	// 		else {
	// 			values[i] = state[i];
	// 		}
	// 		// std::cout << values[i] << ", ";
	// 	}
	// 	// std::cout << std::endl;
	// 	clone_probot[ID]->SetActiveDOFValues(values,true);

	// 	obstacle_collision = true;

	// 	break;
	// }

	return obstacle_collision;
}