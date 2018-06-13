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
#include "utilities/random.hpp"

#include <iostream>
#include <deque>
#include <limits>
#include <math.h>

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
	system->copy_state_point(root->point,start_state);
	for (int i = 0; i < number_of_particles; ++i)
	{
		root->particles.push_back(system->alloc_state_point());
		system->random_particles(root->particles.back(), start_state, particle_radius);
	}
	//add root to nearest neighbor structure
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
	system->random_state(sample_state);
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
	nearest = (tree_node_t*)metric->find_closest(metric_query,&distance)->get_state();
}
//EDIT
bool rrt_t::propagate()
{
	double best_cost = std::numeric_limits<double>::infinity();
	bool local_valid = false;
	double temp_duration = std::numeric_limits<double>::infinity();
	double best_biased_cost = std::numeric_limits<double>::infinity();
	double temp_cost;
	int temp_fixed_time_step = uniform_random(params::min_time_steps, params::max_time_steps);
	int temp_flag_selection = uniform_int_random(0,1);

	system->copy_state_point(temp_sample_state, sample_state);

	for (int i = 0; i < number_of_control; ++i)
	{
		
		temp_cost = best_cost;
		bool temp_valid = system->convergent_propagate( params::random_time, nearest->point, nearest->particles, sample_control_sequence[i], temp_fixed_time_step,temp_fixed_time_step, control_temp_state,control_temp_particles, temp_duration, temp_cost );
		double local_distance = system->distance(sample_state,control_temp_state);
		double local_biased_cost; 
		if (temp_flag_selection == 0) local_biased_cost = local_distance;
		else local_biased_cost = temp_cost;
		//double local_biased_cost = temp_cost;
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

}

