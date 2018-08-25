/**
 * @file climb_hill.hpp
 * 
 * 
 */

#ifndef SPARSE_GRIPPER_2D_HPP
#define SPARSE_GRIPPER_2D_HPP

#include "systems/system.hpp"
#include <math.h>

class Rectangle_t
{
public:
	/**
	 * @brief Create a rectangle using two corners
	 * 
	 * @param lx Bottom Left X coordinate
	 * @param ly Bottom Left Y coordinate
	 * @param hx Top Right X coordinate
	 * @param hy Top Right Y coordinate
	 * @param curvature Fraction of length or height that are curved
	 */
	Rectangle_t(double lx,double ly,double hx,double hy, double curvature)
	{
		low_x = lx;
		low_y = ly;
		high_x = hx;
		high_y = hy;
		cen_x = (lx+hx)/2;
		cen_y = (ly+hy)/2;
		curv = curvature;
		curv_radi = std::min(fabs(hx - lx), fabs(hy - ly)) * curvature / 2;
	}
	/**
	 * @brief Create a rectangle with center position and dimensions.
	 * 
	 * @param pos_x Center X coordinate.
	 * @param pos_y Center Y coordinate.
	 * @param dim_x X Dimension
	 * @param dim_y Y Dimension
	 * @param curvature Fraction of length or height that are curved
	 * @param value Just a flag to denote which constructor is used.
	 */
	Rectangle_t(double pos_x,double pos_y,double dim_x,double dim_y, double curvature, bool value)
	{
		low_x = pos_x-dim_x/2;
		low_y = pos_y-dim_y/2;
		high_x = pos_x+dim_x/2;
		high_y = pos_y+dim_y/2;
		cen_x = pos_x;
		cen_y = pos_y;
		curv = curvature;
		curv_radi = std::min(dim_x, dim_y) * curvature / 2;
	}
	
	double low_x;
	double low_y;
	double high_x;
	double high_y;
	double cen_x;
	double cen_y;
	double curv;
	double curv_radi;
};

class gripper_2D_t : public system_t
{
public:
	gripper_2D_t()
	{
		state_dimension = 2;
		control_dimension = 2;
		temp_state = new double[state_dimension];
		number_of_particles = 0;
		temp_particles.clear();
		obstacles.push_back(Rectangle_t(   1,  	  1,     2,   3,  0.5));
		// obstacles.push_back(Rectangle_t(  2.5,  -0.5,   3.5, 1.5, 0.5));
		obstacles.push_back(Rectangle_t(  2.5,  -0.5,   3.5, 3, 0.5));
		obstacles.push_back(Rectangle_t(   2,    -2,   2.25,  0,  0.5));
	}

	gripper_2D_t(unsigned in_number_of_particles)
	{
		state_dimension = 2;
		control_dimension = 2;
		temp_state = new double[state_dimension];
		number_of_particles = in_number_of_particles;
		temp_particles.clear();
		for (int i = 0; i < number_of_particles; ++i)
		{
			temp_particles.push_back(new double[state_dimension]); //+1 to store the height
		}
		obstacles.push_back(Rectangle_t(   1,  	  1,     2,   3,  0.5));
		// obstacles.push_back(Rectangle_t(  2.5,  -0.5,   3.5, 1.5, 0.5));
		obstacles.push_back(Rectangle_t(  2.5,  -0.5,   3.5, 3, 0.5));
		obstacles.push_back(Rectangle_t(   2,    -2,   2.25,  0,  0.5));
	}

	virtual ~gripper_2D_t(){}

	virtual double distance(double* point1, double* point2);

	virtual void random_particles(double* destination, double* state, double radius);

	virtual void random_state(double* state);

	virtual void random_control(double* control);

	virtual bool propagate(double* start_state, double* control, int min_step, int max_step, double* result_state, double& duration );

	virtual bool convergent_propagate( const bool &random_time, double* start_state,std::vector<double*> &start_particles, double* control, int min_step, int max_step, double* result_state, std::vector<double*> &result_particles, double& duration, double& cost );
	//virtual bool propagate_with_particles( double* start_state, std::vector<double*> &particles, double* control, int min_step, int max_step, double* result_state, std::vector<double*> &result_particles, double& duration, double& Da );

	//virtual bool propagate_fixed_duration( double* start_state, std::vector<double*> &particles, double* control, int step_size, double* result_state, std::vector<double*> &result_particles, double& duration, double& Da );

	virtual void enforce_bounds(double* state);
	
	virtual bool valid_state();

	virtual double cost_function(double* state, std::vector<double*> particles);

	bool check_collision(double* state);

	int location_in_obstacle(double* state, int obstacles_id, double* corners, double* curv_cen);

	int enforce_obstacle_bounds(double* state, int obstacles_id, double* corners, double* curv_cen, double lower_angle_bound, double upper_angle_bound);

	double portion_in_collision(double* point1, double* point2);

	svg::Point visualize_point(double* state, svg::Dimensions dims);

	std::string export_point(double* state);

	virtual void load_openrave(){
		return;
	};

protected:

	std::vector<Rectangle_t> obstacles;
};


#endif