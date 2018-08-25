/**
 * @file rrt.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#ifndef SPARSE_PLANNER_RRT_RESTART_HPP
#define SPARSE_PLANNER_RRT_RESTART_HPP

#include "systems/system.hpp"
#include "motion_planners/planner.hpp"

/**
 * @brief The motion planning algorithm RRT (Rapidly-exploring Random Tree)
 * @details The motion planning algorithm RRT (Rapidly-exploring Random Tree)
 */
class rrt_restart_t : public planner_t
{
public:
	/**
	 * @copydoc planner_t::planner_t(system_t*)
	 */
	rrt_restart_t(system_t* in_system) : planner_t(in_system)
	{
		number_of_control = 0;
	}
	
	/**
	 * @copydoc planner_t::planner_t(system_t* in_system, unsigned in_number_of_particles, double in_particle_radius)
	 */
	//ADD_KAIWEN
	rrt_restart_t(system_t* in_system, unsigned in_number_of_particles, double in_particle_radius, unsigned in_number_of_control) : planner_t(in_system, in_number_of_particles, in_particle_radius)
	{
		number_of_control = in_number_of_control;
	}
	virtual ~rrt_restart_t(){}

	/**
	 * @copydoc planner_t::setup_planning()
	 */
	virtual void setup_planning();

	/**
	 * 
	 */
	void restart_planning();

	/**
	 * @copydoc planner_t::get_solution(std::vector<std::pair<double*,double> >&)
	 */
	virtual void get_solution(std::vector<std::pair<double*,double> >& controls, double &end_div);

	/**
	 * @copydoc planner_t::step()
	 */
	virtual void step();

protected:
	
	/**
	 * @brief A randomly sampled state.
	 */
	double* sample_state;

	double* temp_sample_state;
	/**
	 * @brief Particles associated with the propagated state (used in b-rrt).
	 */
	//ADD_KAIWEN
	std::vector<double*> sample_particles;

	/**
	 * @brief A randomly sampled control.
	 */
	double* sample_control;

	/**
	 * @brief Number of control randomed for each iteration (used in b-rrt).
	 */
	//ADD_KAIWEN
	unsigned number_of_control;

	/**
	 * @brief A series of random control (used in b-rrt).
	 */
	//ADD_KAIWEN
	std::vector<double*> sample_control_sequence;

	/**
	 * @brief A temporary state to store locally best state for each iteration (used in b-rrt).
	 */
	//ADD_KAIWEN
	double* control_temp_state;

	/**
	 * @brief Temporary particles to store locally best state for each iteration (used in b-rrt).
	 */
	//ADD_KAIWEN
	std::vector<double*> control_temp_particles;	

	/**
	 * @brief A resulting duration of a propagation step.
	 */
	double duration;

	/**
	 * Yang
	 * @brief A cost of a propagation step.
	 */
	double cost;

	/**
	 * @brief Average divergence measure.
	 */
	//ADD_KAIWEN
	double Da;

	/**
	 * @brief Storage used to query the nearest neighbor structure.
	 */
	tree_node_t* metric_query;

	/**
	 * @brief The result of a query in the nearest neighbor structure.
	 */
	tree_node_t* nearest;
	
	/**
	 * @brief A set of nodes used to get solutions.
	 */
	proximity_node_t** close_nodes;
	/**
	 * @brief A set of distances used to get solutions.
	 */
	double* distances;

	/**
	 * @brief Perform the random sampling step of RRT.
	 * @details Perform the random sampling step of RRT.
	 */
	void random_sample();

	/**
	 * @brief Find the nearest node to the randomly sampled state.
	 * @details Find the nearest node to the randomly sampled state.
	 */
	void nearest_vertex();

	/**
	 * @brief Perform a local propagation from the nearest state.
	 * @details Perform a local propagation from the nearest state.
	 * @return If the trajectory is collision free or not.
	 */
	bool propagate();

	/**
	 * @brief If propagation was successful, add the new state to the tree.
	 * @details If propagation was successful, add the new state to the tree.
	 */
	void add_to_tree();

	/**
	 * @brief Add a state into the nearest neighbor structure for retrieval in later iterations.
	 * @details Add a state into the nearest neighbor structure for retrieval in later iterations.
	 * 
	 * @param node The node to add.
	 */
	void add_point_to_metric(tree_node_t* node);


};

#endif