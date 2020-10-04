# Etasl_invariants_integration

Integration of the invariant trajectory generator into eTaSL.


## Current dependencies

- **Ubuntu 18.04** 
- **ROS melodic** 
- **Casadi library**: >3.5.1 for Python 2.7. Add the folder to your PYTHONPATH:  

    export PYTHONPATH="${PYTHONPATH}:/home/mvochten/libraries/casadi-linux-py27-v3.5.1-64bit"  

    It is advised to put this command in your bashrc script unless you want to repeat it.

- (optional) ma57 linear solver. If you don't have this, you will get an "invalid option" error in Casadi during the call to the solver. You need to replace the "linear_solver" option in the solver definition (inside invariant_descriptor_class.py) from "ma57" to "mumps"  
- **Etasl** installed (from the general etasl-install repository), but switched to the "devel" branch

Note: etasl-py is not yet updated to Python 3, so I prefer Python 2 for now, also for Casadi.


## Running the code

    roscore

Start eTaSL controller first, it will wait for the first trajectory:

    python scripts/etasl_traj_follow_simulator.py

Then start the trajectory generator:

    python invariants_ros.py

Results (trajectories, twists, joint angles) are automatically saved in text files.

#### Optional simultaneous visualization in rviz

Prior to running the code above, run the following commands:

    roslaunch launch/initialize_rviz_ur10.launch

    python path_visualization.py
        
#### Optional record results in bagfile

Record results during execution (necessary for later off-line visualization in rviz):

    rosbag record -a

TODO enable/disable this in the code as well

#### Parameters

change motion of moving target in `invariants_ros.py`

enable/disable obstacle by commenting the relevant code in `etasl_traj_follow_simulator.py`


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

TODO update to latest etasl version (master branch). 

- check the way in which the global angular scale Theta is determined by the optimization
- try using slack variable instead of current soft constraint approach for desired target state X_N
- implement with splines
- in practice, invariants_ros should ask for the robots current pose
- use roslaunch files to startup code, also include a ros parameter for testing standalone as well

