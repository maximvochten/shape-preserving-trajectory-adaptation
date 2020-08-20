# Etasl_invariants_integration

Integration of the invariant trajectory generator into eTaSL.


## Installation


## Running the code

        roscore

Start eTaSL controller first:

        python scripts/etasl_traj_follow_simulator.py

Then start the trajectory generator:

        python invariants_ros.py


#### Parameters

change motion of moving target in `invariants_ros.py`

enable/disable obstacle by commenting the relevant code in `etasl_traj_follow_simulator.py`

#### Optionally simultaneous visualization in rviz

        roslaunch launch/initialize_rviz_ur10.launch

        python path_visualization.py



Record results during execution (necessary for later off-line visualization in rviz):

        rosbag record -a

## Visualize results in Matlab

Run `visualization/main_process_results.m`

First change `nb` to the right number of trajectories.

## Visualize results in rviz

        roscd etasl_invariants_integration

        roscore

        rosparam set /use_sim_time true

        roslaunch launch/initialize_rviz_ur10.launch

        python path_visualisation.py

Obstacle avoidance:

        rosbag play --clock -l -s 2.0 2020-05-12-08-21-10.bag

Moving target:

        rosbag play --clock -l -s 3.0 2020-06-22-15-42-01.bag

Disable/enable obstacle in path_visualization.py, line with command `plot_obstacle(`


## Future work

- check the way in which the global angular scale Theta is determined by the optimization
- try using slack variable instead of current soft constraint approach for desired target state X_N
- implement with splines
- in practice, invariants_ros should ask for the robots current pose
- use roslaunch files to startup code, also include a ros parameter for testing standalone as well

