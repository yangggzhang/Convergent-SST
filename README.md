# Convergent-SST

Sparse methods for convergent planning

All source codes are in src
The main files are Convergent_SST.m and B_RRT.m(biased convergent planning without using sparse methods)

Previous simulation results are in Data folder
Evaluate_res.m can be used to visualize simulation results

The dynamical system for hill climbing simulation is described is HillClimbingSys.pdf

The implementation of convergent-sst and biased-rrt in c++ can be found in c_sst_in_cpp.

The code is modified based on Pracsys lab at Rutgers' sparse rrt code. The source code of sparse rrt can be found in https://bitbucket.org/pracsys/sparse_rrt.

## COMPILING
To compile the convergent sst in C++, follow these steps in the git directory:

```
cd c_sst_in_cpp
mkdir build
cd build
cmake ..
make
```

## EXECUTING
To run with default parameters from input/default.cfg, just run 
this executable.

```
cd ../bin
./run
```

you may check the default configuration file:

```
more ../input/default.cfg
```

All of the parameters can be found by running

```
./run --help
```

The output of this command is shown here for reference: 

```
Options:
  --help                                Print available options.
  --config arg (=../input/default.cfg)  The name of a file to read for options 
                                        (default is ../input/default.cfg). 
                                        Command-line options override the ones 
                                        in the config file. A config file may 
                                        contain lines with syntax
                                        'long_option_name = value'
                                        and comment lines that begin with '#'.
  --integration_step arg                Integration step for propagations.
  --stopping_type arg                   Condition for terminating planner 
                                        (iterations or time).
  --stopping_check arg                  Amount of time or iterations to 
                                        execute.
  --stats_type arg                      Condition for printing statistics of a 
                                        planner (iterations or time).
  --stats_check arg                     Frequency of statistics gathering.
  --intermediate_visualization arg (=0) Flag denoting generating images during 
                                        statistics gathering.
  --min_time_steps arg                  Minimum number of simulation steps per 
                                        local planner propagation.
  --max_time_steps arg                  Maximum number of simulation steps per 
                                        local planner propagation.
  --bacon arg (=0)                      bacon strips and bacon strips and bacon
                                        strips
  --random_seed arg                     Random seed for the planner.
  --sst_delta_near arg                  The radius for BestNear in SST.
  --sst_delta_drain arg                 The radius for witness nodes in SST.
  --planner arg                         A string for the planner to run.
  --system arg                          A string for the system to plan for.
  --start_state arg                     The given start state. Input is in the 
                                        format of "0 0"
  --goal_state arg                      The given goal state. Input is in the 
                                        format of "0 0"
  --goal_radius arg                     The radius for the goal region.
  --tree_line_width arg                 Line thickness for tree visualization.
  --solution_line_width arg             Line thickness for solution path.
  --image_width arg                     Width of output images.
  --image_height arg                    Height of output images.
  --node_diameter arg                   Diameter of visualized nodes in output 
                                        images.
  --solution_node_diameter arg          Diameter of nodes along solution path 
                                        in output images.
  --number_of_particles arg             Number of particles generated.
  --particle_radius arg                 Radius to generate particles.
  --number_of_control arg               Number of control of b-rrt for each 
                                        iteration.
  --fixed_time_step arg                 Fixed number of simulation steps per 
                                        local planner propagation.
```
