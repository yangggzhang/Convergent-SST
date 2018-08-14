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
		temp_particles.clear();
	}

	gripper_t(unsigned in_number_of_particles)
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
		int mid_par = number_of_particles/4;
		for (int i = 0; i < mid_par; i++) thread1_temp_particles.push_back(new double[state_dimension]);
		for (int i = mid_par; i < 2*mid_par; i++) thread2_temp_particles.push_back(new double[state_dimension]);
		for (int i = 2*mid_par; i < 3*mid_par; i++) thread3_temp_particles.push_back(new double[state_dimension]);
		for (int i = 3*mid_par; i < number_of_particles; i++) thread4_temp_particles.push_back(new double[state_dimension]);

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

	bool env_check_collision(EnvironmentBasePtr local_penv, RobotBasePtr local_probot, double* state);


	double portion_in_collision(double* point1, double* point2);

	svg::Point visualize_point(double* state, svg::Dimensions dims);

	std::string export_point(double* state);

	virtual void load_openrave();

	void thread1_particle_propagate(double* control);
	void thread2_particle_propagate(double* control);
	void thread3_particle_propagate(double* control);
	void thread4_particle_propagate(double* control);

	//double thread_propagate(EnvironmentBasePtr &env, RobotBasePtr &robot, std::vector<double*> &particles, double* state,double* control);

protected:
	EnvironmentBasePtr penv;

	EnvironmentBasePtr thread1_penv;

	EnvironmentBasePtr thread2_penv;

	EnvironmentBasePtr thread3_penv;

	EnvironmentBasePtr thread4_penv;

	CollisionCheckerBasePtr pchecker;

	CollisionCheckerBasePtr thread1_pchecker;

	CollisionCheckerBasePtr thread2_pchecker;

	CollisionCheckerBasePtr thread3_pchecker;

	CollisionCheckerBasePtr thread4_pchecker;

	RobotBasePtr probot;

	RobotBasePtr thread1_probot;

	RobotBasePtr thread2_probot;

	RobotBasePtr thread3_probot;

	RobotBasePtr thread4_probot;


	std::vector<KinBodyPtr> pbodies;

	std::vector<KinBodyPtr> thread1_pbodies;

	std::vector<KinBodyPtr> thread2_pbodies;

	std::vector<KinBodyPtr> thread3_pbodies;

	std::vector<KinBodyPtr> thread4_pbodies;

};


#endif