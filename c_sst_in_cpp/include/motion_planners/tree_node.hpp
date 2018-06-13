/**
 * @file tree_node.hpp
 * 
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick  
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 * 
 * Authors: Zakary Littlefield, Kostas Bekris 
 * 
 */

#ifndef SPARSE_TREE_NODE_HPP
#define SPARSE_TREE_NODE_HPP

#include <list>
#include <vector>
#include <cstddef>

class proximity_node_t;

/**
 * @brief An edge of the tree.
 * @details An edge of the tree.
 */
class tree_edge_t
{
public:
    /**
     * @brief The control for this edge.
     */
	double* control;
    /**
     * @brief The duration to execute control.
     */
	double duration;

    /**
	 * Yang
	 * @brief The cost to execute control.
	 */
	double cost;
};

/**
 * @brief A node of the tree.
 * @details A node of the tree.
 */
class tree_node_t
{
public:
    //EDIT
	tree_node_t()
	{
		point = NULL;
        particles.clear();
		prox_node = NULL;
		parent = NULL;
		parent_edge = NULL;
		children.clear();
		node_cost = 0;
        path_cost = 0;
	}

    /**
     * @brief The state represented by this node.
     */
    double* point;

    /**
     * @brief The potential positions of the current state.
     */
    //ADD_KAIWEN
    std::vector<double*> particles;

    /**
     * @brief A pointer to the node in the nearest neighbor structure for easy removal.
     */
    proximity_node_t* prox_node;

    /**
     * @brief Parent node.
     */
    tree_node_t* parent;

    /**
    * @brief Parent edge
    */
    tree_edge_t* parent_edge;
        
    /**
     * @brief Children vertices
     */
    std::list<tree_node_t*> children;

    /**
     * Kaiwen
     * @brief The path cost to this node.
     */
    double path_cost;

    /**
     * Kaiwen
     * @brief The node cost of this node.
     */
    double node_cost;

};

#endif