#TODO

- use roslaunch files to startup code

- in practice, invariants_ros should ask for the robots current pose

### Running the code

spyder

roscore

param set /use_sim_time true

roslaunch launch/initialize_rviz_ur10.launch

python path_visualisation.py 

rosbag play --clock  -l -s 4.2 2020-05-05-09-15-48.bag 

### Simulations

- first experiment on trajectory generalization: I want to show that we can also change the desired tangent at then end of the trajectory. Show for the same target position/orientation a different tangent direction
- second experiment: I would like to show something of the UR10 in rviz as well to make it less abstract
- show effect of increasing window size on invariants: less jumps but longer calculation time

### Methodology

- check influence of using slack variable instead of current soft constraint approach for desired target state X_N
- the way in which the global angular scale Theta is determined by the optimization is not really geometrically correct, but it works...
- implement as splines
