/**
 * @file rrt.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "motion_planners/rrt.hpp"
#include "nearest_neighbors/graph_nearest_neighbors.hpp"

#include <iostream>
#include <deque>
#include <limits>
#include <math.h>
#include <openrave-core.h>
#include "utilities/random.hpp"

//EDIT
void rrt_t::setup_planning()
{
	//init internal variables
	sample_state = system->alloc_state_point();
	temp_sample_state = system->alloc_state_point();
	control_temp_state = system->alloc_state_point();

	cost = 0;

	for (int i = 0; i < number_of_particles; ++i)
	{
		sample_particles.push_back(system->alloc_state_point());
		control_temp_particles.push_back(system->alloc_state_point());
	}

	sample_control = system->alloc_control_point();
	for (int i = 0; i < number_of_control; ++i)
	{
		sample_control_sequence.push_back(system->alloc_control_point());
	}

	//initialize output candidates
	candidate_states.resize(number_of_control);
	candidate_states_particles.resize(number_of_control);
	for (int i = 0; i < number_of_control; ++i)
	{
		candidate_states[i] = system->alloc_state_point();
		for (int j = 0; j < number_of_particles; j++) candidate_states_particles[i].push_back(system->alloc_state_point());
	}
	
	last_state = system->alloc_state_point();

	selected_state = system->alloc_state_point();
	// //finish


	metric_query = new tree_node_t();
	metric_query->point = system->alloc_state_point();

    close_nodes = (proximity_node_t**)malloc(MAX_KK * sizeof (proximity_node_t*));
    distances = (double*)malloc(MAX_KK * sizeof (double));

	//initialize the metric
	metric = new graph_nearest_neighbors_t();
	metric->set_system(system);
	//create the root of the tree
	root = new tree_node_t();
	number_of_nodes++;
	root->point = system->alloc_state_point();
	root->cost = 0;
	system->copy_state_point(root->point,start_state);
	for (int i = 0; i < number_of_particles; ++i)
	{
		root->particles.push_back(system->alloc_state_point());
		system->random_particles(root->particles.back(), start_state, particle_radius);
	}
	//add root to nearest neighbor structure
	add_point_to_metric(root);
	system->load_openrave();
}

void rrt_t::restart_planning()
{
	delete metric;
	metric = new graph_nearest_neighbors_t();
	metric->set_system(system);
	delete root;
	root = new tree_node_t();
	root->cost = 0.0;
	root->point = system->alloc_state_point();
	system->copy_state_point(root->point,start_state);
	for (int i = 0; i < number_of_particles; ++i)
	{
		root->particles.push_back(system->alloc_state_point());
		system->random_particles(root->particles.back(), start_state, particle_radius);
	}
	number_of_nodes = 1;
	add_point_to_metric(root);

}

void rrt_t::get_solution(std::vector<std::pair<double*,double> >& controls)
{
	last_solution_path.clear();
	system->copy_state_point(sample_state,goal_state);
	system->copy_state_point(metric_query->point,sample_state);
	unsigned val = metric->find_delta_close_and_closest(metric_query,close_nodes,distances,goal_radius);

    double length = 999999999;
    for(unsigned i=0;i<val;i++)
    {
        tree_node_t* v = (tree_node_t*)(close_nodes[i]->get_state());
        double temp = v->cost ;
        if( temp < length)
        {
            length = temp;
            nearest = v;
        }
    }
	//now nearest should be the closest node to the goal state
	if(system->distance(goal_state,nearest->point) < goal_radius)
	{
		std::deque<tree_node_t*> path;
		while(nearest->parent!=NULL)
		{
			path.push_front(nearest);
			nearest = nearest->parent;
		}
		last_solution_path.push_back(root);
		for(unsigned i=0;i<path.size();i++)
		{
			last_solution_path.push_back(path[i]);
			controls.push_back(std::pair<double*,double>(NULL,0));
			controls.back().first = system->alloc_control_point();
			system->copy_control_point(controls.back().first,path[i]->parent_edge->control);
			controls.back().second = path[i]->parent_edge->cost;
		}
	}
}

void rrt_t::smooth()
{
	// std::vector<tree_node_t*> last_solution_path;
	size_t times = last_solution_path.size();
	size_t current_step = 0;
	while (current_step < times)
	{
		//std::cout<<current_step<<"/"<<times<<std::endl;
		int current_size = last_solution_path.size();
		//std::vector<tree_edge_t*> temp_solution_path(last_solution_path);
		int start_vertice = uniform_int_random(0 , current_size - 1);
		int end_vertice = uniform_int_random(0 , current_size - 1);
		if (start_vertice > end_vertice)
		{
			int temp_vertice = start_vertice;
			start_vertice = end_vertice;
			end_vertice = temp_vertice;
		}

		std::vector<tree_node_t*> temp_solution_path;
		//temp_solution_path.resize(current_size - end_vertice + start_vertice + 1);

		for (size_t i = 0; i <= start_vertice; i++)
		{
			temp_solution_path.push_back(last_solution_path[i]);
		}

		for (size_t i = end_vertice; i < current_size; i++)
		{
			temp_solution_path.push_back(last_solution_path[i]);
		}
		//std::cout<<start_vertice<<","<<end_vertice<<","<<current_size<<"propose"<<temp_solution_path.size()<<std::endl;
		//double temp_cost = uniform_random(0,100);

		double temp_cost = system->go_through_path(temp_solution_path);
		if (temp_cost < last_solution_cost)
		{
			//std::cout<<"update"<<std::endl;
			last_solution_path.clear();
			for (size_t i = 0; i < temp_solution_path.size(); i++)	last_solution_path.push_back(temp_solution_path[i]);
			last_solution_cost = temp_cost;
			//std::cout<<"update length : "<<last_solution_path.size()<<std::endl;
		}
		current_step ++;
	}
	//std::cout<<"updated length"<<last_solution_path.size()<<std::endl;
}

void rrt_t::step()
{
	random_sample();
	nearest_vertex();
	if(propagate())
	{
		add_to_tree();
	}
}

void rrt_t::add_point_to_metric(tree_node_t* state)
{
	proximity_node_t* new_node = new proximity_node_t(state);
	state->prox_node = new_node;
	metric->add_node(new_node);
}


void rrt_t::random_sample()
{
	int rand_int = uniform_int_random(0,99);
	if (rand_int <= 2) system->copy_state_point(sample_state,goal_state);
	else	system->random_state(sample_state);
	if (number_of_control == 0) system->random_control(sample_control);
	else 
	{
		for (int i = 0; i < number_of_control; ++i)
		{
			system->random_control(sample_control_sequence[i]);
		}
	}
}
void rrt_t::nearest_vertex()
{
	system->copy_state_point(metric_query->point,sample_state);
	double distance;
	nearest = (tree_node_t*)metric->find_biased_closest(metric_query,&distance)->get_state();
}
//EDIT
bool rrt_t::propagate()
{
	double best_cost = std::numeric_limits<double>::infinity();
	bool local_valid = false;
	double temp_duration = std::numeric_limits<double>::infinity();
	double best_biased_cost = std::numeric_limits<double>::infinity();
	double temp_cost;

	system->copy_state_point(temp_sample_state, sample_state);
	system->copy_state_point(last_state, sample_state);

	int num_steps = 0;
	if (params::random_time)	num_steps = uniform_int_random(params::min_time_steps,params::max_time_steps);
	else num_steps =  (int)(params::min_time_steps + params::max_time_steps)/2;

	int temp_flag_selection = uniform_int_random(0,1);

	for (int i = 0; i < number_of_control; ++i)
	{
		
		temp_cost = best_cost;
		bool temp_valid = system->convergent_propagate( num_steps, nearest->point, nearest->particles, sample_control_sequence[i], control_temp_state,control_temp_particles, temp_duration, temp_cost );		
		
		double local_biased_cost = 0.0;
		if (temp_flag_selection == 0) local_biased_cost = system->distance(sample_state,control_temp_state);
		else local_biased_cost = temp_cost;

		if (temp_valid && local_biased_cost < best_biased_cost)
		{
			local_valid = true;
			system->copy_state_point(temp_sample_state, control_temp_state);
			for (int j = 0; j < number_of_particles; ++j)
			{
				system->copy_state_point(sample_particles[j],control_temp_particles[j]);
			}
			best_cost = temp_cost;
			best_biased_cost = local_biased_cost;
			duration = temp_duration;
		}
	}

	system->copy_state_point(sample_state, temp_sample_state);

	system->copy_state_point(selected_state, temp_sample_state);

	cost = best_cost;
	
	return local_valid;
	
}
void rrt_t::add_to_tree()
{
	//create a new tree node
	tree_node_t* new_node = new tree_node_t();
	new_node->point = system->alloc_state_point();
	system->copy_state_point(new_node->point,sample_state);
	//Pass the propagated particles to the next node
	//ADD_KAIWEN
	for (int i = 0; i < number_of_particles; ++i)
	{
		new_node->particles.push_back(system->alloc_state_point());
		system->copy_state_point(new_node->particles.back(), sample_particles[i]);
	}
	//create the link to the parent node
	new_node->parent_edge = new tree_edge_t();
	new_node->parent_edge->control = system->alloc_control_point();
	system->copy_control_point(new_node->parent_edge->control,sample_control);
	new_node->parent_edge->duration = duration;
	new_node->parent_edge->cost = cost;
	//set this node's parent
	new_node->parent = nearest;
	new_node->cost = nearest->cost + cost;
	//set parent's child
	nearest->children.insert(nearest->children.begin(),new_node);
	add_point_to_metric(new_node);
	number_of_nodes++;


	if (system->distance(new_node->point,goal_state) < goal_radius)
	{
		last_solution_cost = 0.0;
		last_solution_path.clear();
		std::deque<tree_node_t*> path;
		nearest = new_node;
		while(nearest->parent!=NULL)
		{
			path.push_front(nearest);
			nearest = nearest->parent;
		}
		last_solution_path.push_back(root);
		for(unsigned i=0;i<path.size();i++)
		{
			last_solution_path.push_back(path[i]);
		}
		//std::cout<<"Current: "<<last_solution_cost<<", Best: "<<best_solution_cost<<std::endl;
		//smooth();
		last_solution_cost = new_node->cost;
		update_path();
		restart_planning();
	}

}

