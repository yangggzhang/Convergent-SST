/**
 * @file climb_hill.hpp
 * 
 * 
 */

#ifndef SPARSE_GRIPPER_HPP
#define SPARSE_GRIPPER_HPP

#include "systems/system.hpp"

class gripper_t : public system_t
{
public:
	gripper_t()
	{
		state_dimension = 3;
		control_dimension = 3;
		temp_state = new double[state_dimension];
		number_of_particles = 0;
		num_threads = 0;
		temp_particles.clear();
	}

	gripper_t(unsigned in_number_of_particles, int in_num_threads)
	{
		state_dimension = 3;
		control_dimension = 3;
		temp_state = new double[state_dimension];
		number_of_particles = in_number_of_particles;
		temp_particles.clear();
		for (int i = 0; i < number_of_particles; ++i)
		{
			temp_particles.push_back(new double[state_dimension]); //+1 to store the height
		}
		num_threads = in_num_threads;
	}

	virtual ~gripper_t(){
		RaveDestroy();
	}

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

	bool check_collision(double* state);

	bool check_collision_parallel(double* state, int ID, EnvironmentBasePtr temp_penv, RobotBasePtr temp_probot);

	double portion_in_collision(double* point1, double* point2);

	svg::Point visualize_point(double* state, svg::Dimensions dims);

	std::string export_point(double* state);

	virtual void load_openrave();

protected:
	EnvironmentBasePtr penv;

	std::vector<EnvironmentBasePtr> clone_penv;

	// EnvironmentBasePtr clone_penv;

	CollisionCheckerBasePtr pchecker;

	// CollisionCheckerBasePtr clone_pchecker;

	RobotBasePtr probot;

	// RobotBasePtr clone_probot;

	std::vector<RobotBasePtr> clone_probot;

	std::vector<KinBodyPtr> pbodies;

	int num_threads;
};


#endif