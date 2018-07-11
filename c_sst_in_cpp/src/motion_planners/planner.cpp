/**
 * @file planner.cpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#include "motion_planners/planner.hpp"

void write_point(std::ofstream &myfile, double* point, double cost, int type)
{
	myfile<<point[0]<<","<<point[1]<<","<<point[2]<<","<<cost<<","<<type<<std::endl;
	
}

void planner_t::set_start_state(double* in_start)
{
	if(start_state==NULL)
		start_state = system->alloc_state_point();
	system->copy_state_point(start_state,in_start);
}

void planner_t::set_goal_state(double* in_goal,double in_radius)
{
	if(goal_state==NULL)
		goal_state = system->alloc_state_point();
	system->copy_state_point(goal_state,in_goal);
	goal_radius = in_radius;
}

void planner_t::visualize_tree(int image_counter)
{
	std::stringstream s;
    s<<"/home/parallels/Documents/Convergent-SST/c_sst_in_cpp/data/"<<params::planner<<"_"<<image_counter<<"_tree.svg";
    std::string dir(s.str());
    svg::Dimensions dimensions(params::image_width, params::image_height);
    svg::Document doc(dir, svg::Layout(dimensions, svg::Layout::BottomLeft));

    visualize_edge(root,doc,dimensions);

	svg::Circle circle(system->visualize_point(start_state,dimensions),params::solution_node_diameter,svg::Fill( svg::Color(255,0,0) ));
	doc<<circle;
	svg::Circle circle2(system->visualize_point(goal_state,dimensions),params::solution_node_diameter,svg::Fill( svg::Color(0,255,0) ));
	doc<<circle2;

	visualize_solution_path(doc,dimensions);
    system->visualize_obstacles(doc,dimensions);

    doc.save();
}

void planner_t::write_tree(std::ofstream &myfile)
{
	write_edge(root,myfile);

	write_point(myfile,start_state,0,2);
	write_point(myfile, goal_state,0,3);

	write_solution_path(myfile);
}

void sort(std::vector<tree_node_t*>& nodes)
{
	for(unsigned i=0;i<nodes.size();i++)
	{
		tree_node_t* x = nodes[i];
		unsigned j = i;
		while(j>0 && nodes[j-1]->cost > x->cost)
		{
			nodes[j] = nodes[j-1];
			j--;
		}
		nodes[j] = x;
	}
}

void planner_t::visualize_nodes(int image_counter)
{
	std::stringstream s;
    s<<"/home/parallels/Documents/Convergent-SST/c_sst_in_cpp/data/"<<params::planner<<"_"<<image_counter<<"_node.svg";
    std::string dir(s.str());
    svg::Dimensions dimensions(params::image_width, params::image_height);
    svg::Document doc(dir, svg::Layout(dimensions, svg::Layout::BottomLeft));
    sorted_nodes.clear();
    get_max_cost();
    sort(sorted_nodes);

    for(unsigned i=sorted_nodes.size()-1;i!=0;i--)
    {
	    visualize_node(sorted_nodes[i],doc,dimensions);
	}

	svg::Circle circle(system->visualize_point(start_state,dimensions),params::solution_node_diameter,svg::Fill( svg::Color(255,0,0) ));
	doc<<circle;
	svg::Circle circle2(system->visualize_point(goal_state,dimensions),params::solution_node_diameter,svg::Fill( svg::Color(0,255,0) ));
	doc<<circle2;

	visualize_solution_nodes(doc,dimensions);
    system->visualize_obstacles(doc,dimensions);

    doc.save();
}

void planner_t::write_nodes(std::ofstream &myfile)
{
	sorted_nodes.clear();
    get_max_cost();
   // sort(sorted_nodes);

    for(unsigned i=sorted_nodes.size()-1;i!=0;i--)
    {
		write_node(sorted_nodes[i], myfile);
	}

	write_point(myfile,start_state,0,2);
	write_point(myfile, goal_state,0,3);

	write_solution_nodes(myfile);
}

void planner_t::visualize_edge(tree_node_t* node, svg::Document& doc, svg::Dimensions& dim)
{

	for (std::list<tree_node_t*>::iterator i = node->children.begin(); i != node->children.end(); ++i)
	{
		svg::Polyline traj_line(svg::Stroke(params::tree_line_width, svg::Color::Blue));

		traj_line<<system->visualize_point(node->point,dim);
		traj_line<<system->visualize_point((*i)->point,dim);
		doc<<traj_line;

		visualize_edge(*i,doc,dim);

	}

}

void planner_t::write_edge(tree_node_t* node, std::ofstream &myfile)
{

	for (std::list<tree_node_t*>::iterator i = node->children.begin(); i != node->children.end(); ++i)
	{
		//svg::Polyline traj_line(svg::Stroke(params::tree_line_width, svg::Color::Blue));

		write_point(myfile, node->point,-1.0, 1);
		write_point(myfile, (*i)->point,-1.0, 1);

		write_edge(*i,myfile);
	}
}

void planner_t::visualize_node(tree_node_t* node, svg::Document& doc, svg::Dimensions& dim)
{

	svg::Circle circle(system->visualize_point(node->point,dim),params::node_diameter,svg::Fill( svg::Color((node->cost/max_cost)*255,(node->cost/max_cost)*255,(node->cost/max_cost)*255) ) );
	doc<<circle;
	// for (std::list<tree_node_t*>::iterator i = node->children.begin(); i != node->children.end(); ++i)
	// {
	// 	visualize_node(*i,doc,dim);
	// }

}

void planner_t::write_node(tree_node_t* node, std::ofstream &myfile)
{
	write_point(myfile, node->point,node->cost,0);
}

void planner_t::visualize_solution_path( svg::Document& doc, svg::Dimensions& dim)
{
	if(best_solution_path.size()!=0)
	{
		svg::Polyline traj_line(svg::Stroke(params::solution_line_width, svg::Color::Black));
		for(unsigned i=0;i<best_solution_path.size();i++)
		{
			traj_line<<system->visualize_point(best_solution_path[i]->point,dim);
		}
		doc<<traj_line;
	}
}

void planner_t::write_solution_path(std::ofstream &myfile)
{
	if(best_solution_path.size()!=0)
	{
		for(unsigned i=0;i<best_solution_path.size();i++)
		{
			write_point(myfile,best_solution_path[i]->point,0,4);
		}
	}
}

void planner_t::visualize_solution_nodes( svg::Document& doc, svg::Dimensions& dim)
{

	if(best_solution_path.size()!=0)
	{
		for(unsigned i=0;i<best_solution_path.size();i++)
		{
			svg::Circle circle(system->visualize_point(best_solution_path[i]->point,dim),params::solution_node_diameter,svg::Fill( svg::Color(0,255,0) ));
			doc<<circle;
		}
	}
}

void planner_t::write_solution_nodes( std::ofstream &myfile )
{

	if(best_solution_path.size()!=0)
	{
		for(unsigned i=0;i<best_solution_path.size();i++)
		{
			write_point(myfile, best_solution_path[i]->point, 0, 5);
		}
	}
}

void planner_t::write_control(std::ofstream &myfile)
{
	write_point(myfile,last_state,0,6);
	for (int i = 0; i < number_of_control; i++)
	{
		write_point(myfile,candidate_states[i],0,10+i);
		for (int j = 0; j < number_of_particles; j++)
		{
			write_point(myfile,candidate_states_particles[i][j],0,20+i);
		}
	}
	write_point(myfile,selected_state,0,9);
}

void planner_t::record(std::ofstream &myfile)
{
	write_tree(myfile);
	write_nodes(myfile);
	write_control(myfile);
	myfile<<std::endl;
}

void planner_t::get_max_cost()
{
	max_cost = 0;
	get_max_cost(root);
}

void planner_t::get_max_cost(tree_node_t* node)
{
	sorted_nodes.push_back(node);
	if(node->cost > max_cost)
		max_cost = node->cost;
	for (std::list<tree_node_t*>::iterator i = node->children.begin(); i != node->children.end(); ++i)
	{
		get_max_cost(*i);
	}
}


void planner_t::update_path()
{
	if (last_solution_cost < best_solution_cost && last_solution_cost > 0)
	{
		best_solution_path.clear();
		for (size_t i = 0 ; i < last_solution_path.size(); i++)
		{
			best_solution_path.push_back(last_solution_path[i]);
		}
		best_solution_cost = last_solution_cost;
	}
}
