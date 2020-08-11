# Installation



# Running the code

...

# Visualize results


spyder

roscore

param set /use_sim_time true

roslaunch launch/initialize_rviz_ur10.launch

python path_visualisation.py 

rosbag play --clock  -l -s 4.2 2020-05-05-09-15-48.bag 

### TODO

- check influence of using slack variable instead of current soft constraint approach for desired target state X_N
- check the way in which the global angular scale Theta is determined by the optimization
- implement as splines
- in practice, invariants_ros should ask for the robots current pose