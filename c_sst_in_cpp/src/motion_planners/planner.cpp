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
    s<<"tree_"<<image_counter<<".svg";
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

void sort(std::vector<tree_node_t*>& nodes)
{
	for(unsigned i=0;i<nodes.size();i++)
	{
		tree_node_t* x = nodes[i];
		unsigned j = i;
		while(j>0 && nodes[j-1]->path_cost > x->path_cost)
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
    s<<"nodes_"<<image_counter<<".svg";
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

void planner_t::visualize_node(tree_node_t* node, svg::Document& doc, svg::Dimensions& dim)
{

	svg::Circle circle(system->visualize_point(node->point,dim),params::node_diameter,svg::Fill( svg::Color((node->path_cost/max_cost)*255,(node->path_cost/max_cost)*255,(node->path_cost/max_cost)*255) ) );
	doc<<circle;
	// for (std::list<tree_node_t*>::iterator i = node->children.begin(); i != node->children.end(); ++i)
	// {
	// 	visualize_node(*i,doc,dim);
	// }

}

void planner_t::visualize_solution_path( svg::Document& doc, svg::Dimensions& dim)
{
	if(last_solution_path.size()!=0)
	{
		svg::Polyline traj_line(svg::Stroke(params::solution_line_width, svg::Color::Black));
		for(unsigned i=0;i<last_solution_path.size();i++)
		{
			traj_line<<system->visualize_point(last_solution_path[i]->point,dim);
		}
		doc<<traj_line;
	}
}
void planner_t::visualize_solution_nodes( svg::Document& doc, svg::Dimensions& dim)
{

	if(last_solution_path.size()!=0)
	{
		for(unsigned i=0;i<last_solution_path.size();i++)
		{
			svg::Circle circle(system->visualize_point(last_solution_path[i]->point,dim),params::solution_node_diameter,svg::Fill( svg::Color(0,255,0) ));
			doc<<circle;
		}
	}
}



void planner_t::get_max_cost()
{
	max_cost = 0;
	get_max_cost(root);
}

void planner_t::get_max_cost(tree_node_t* node)
{
	sorted_nodes.push_back(node);
	if(node->path_cost > max_cost)
		max_cost = node->path_cost;
	for (std::list<tree_node_t*>::iterator i = node->children.begin(); i != node->children.end(); ++i)
	{
		get_max_cost(*i);
	}
}

void planner_t::export_solution_path(int csv_counter)
{
    if(last_solution_path.size()!=0)
	{
		std::stringstream s;
	    s<<"solution_path_"<<csv_counter<<".csv";
	    std::string dir(s.str());
	    std::ofstream doc;
	    doc.open(dir.c_str(),std::ofstream::out | std::ofstream::trunc);

	    doc << system->get_state_dimension() << std::endl;

	    doc << last_solution_path.size() << "," << number_of_particles << std::endl;

	    for(unsigned i=0;i<last_solution_path.size();i++)
		{
			doc << system->export_point(last_solution_path[i]->point);
			for(int j=0;j<number_of_particles;j++)
			{
				doc << system->export_point(last_solution_path[i]->particles[j]);
			}
		}

		doc.close();
	}
}

void planner_t::export_nodes(int csv_counter)
{
	std::stringstream s;
    s<<"nodes_"<<csv_counter<<".csv";
    std::string dir(s.str());
    std::ofstream doc;
    doc.open(dir.c_str(),std::ofstream::out | std::ofstream::trunc);

    sorted_nodes.clear();
    get_max_cost();
    sort(sorted_nodes);

    doc << system->get_state_dimension() << std::endl;
    doc << sorted_nodes.size() << std::endl;
    for(unsigned i=sorted_nodes.size()-1;i!=0;i--)
    {
	    doc << system->export_point(sorted_nodes[i]->point);
	}

	doc.close();
}

void planner_t::export_tree(int csv_counter)
{
	std::stringstream s;
    s<<"tree_"<<csv_counter<<".csv";
    std::string dir(s.str());
    std::ofstream doc;
    doc.open(dir.c_str(),std::ofstream::out | std::ofstream::trunc);

    doc << system->get_state_dimension() << std::endl;
    doc << system->export_point(start_state) << std::endl;
    doc << system->export_point(goal_state) << std::endl;
    doc << std::endl;

    export_edge(root,doc);

	doc.close();

}

void planner_t::export_edge(tree_node_t* node, std::ofstream& doc)
{
    for (std::list<tree_node_t*>::iterator i = node->children.begin(); i != node->children.end(); ++i)
	{
		doc << system->export_point(node->point);
		doc << system->export_point((*i)->point);
		
		export_edge(*i,doc);
	}
}
