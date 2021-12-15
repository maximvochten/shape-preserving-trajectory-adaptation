# Etasl_invariants_integration

Integration of the invariant trajectory generator into eTaSL.

Tested for Ubuntu 18.04, ROS Melodic, eTaSL 1.3.2, Casadi 3.5.1, Orocos 2.9

**IMPORTANT:** Clone this package using the following command:

    git clone --recursive git@gitlab.kuleuven.be:robotgenskill/python_projects/etasl_invariants_integration.git

Otherwise, the symbolic link between this repo and the invariants\_python repository is broken, and trajectories will not be generated
## Installation

We assume that ROS and eTaSL are installed.

Clone and build this repository in your Catkin workspace

    cd ~/catkin_ws/src
    git clone git@gitlab.kuleuven.be:robotgenskill/python_projects/etasl_invariants_integration.git
    git clone git@gitlab.kuleuven.be:rob-expressiongraphs/viz_marker.git
    cd ~/catkin_ws
    catkin_make

Download the [Casadi Python 2.7 library](https://web.casadi.org/get/) and unpack it somewhere (e.g. ~/libraries/). Add the downloaded folder to your PYTHONPATH using:  

    export PYTHONPATH="${PYTHONPATH}:<path-to-casadi-library\>/casadi-linux-py27-v3.5.1-64bit"

It is advised to put this command in your `.bashrc` script unless you want to be able to switch between different versions of Casadi. If you use Spyder as your Python editor, then you need to run Spyder through the terminal to have set the correct PYTHONPATH. Alternatively, you can also set the PYTHONPATH in the local settings of Spyder.

The reason we use Python 2 is because of eTaSL 1.3.2. As soon as eTaSL upgrades to Python 3 in version 1.4, you can switch Casadi to Python 3 as well. Small adjustments to the code in this repository may be required.

To work with the robot model in Python, we need to install the following packages related to KDL (robot kinematics and dynamics library) and URDF files:

    sudo apt-get install ros-melodic-urdfdom-py 

    cd ~/catkin_ws/src/
    git clone https://github.com/gt-ros-pkg/hrl-kdl/
    cd .. && catkin_make

Actually, the two previous packages should not be required since eTaSL features the same functionality and you can request these things from eTaSL. But I like to check these things in Python too, outside of eTaSL.

### Generation of rtt typekit

Skip this step if you only want to do simulations in Python without Orocos.

To stream data from rostopics onto Orocos ports, you need so-called rtt typekits. These are already available for standard message types in ROS. However, if you define custom message types in ROS, then you need to generate the rtt typekit yourself.

First make sure that Orocos is on the ROS package path (assuming you installed it together with eTaSL):

    source ~/etasl-install/ws/etasl-rtt/devel/setup.bash 

Go to your catkin workspace and generate the typekit using

    cd ~/catkin_ws/src
    
    rosrun rtt_roscomm create_rtt_msgs etasl_invariants_integration

Compile the code and source your workspace

    cd .. ; catkin_make
    
    source devel/setup.bash 

### Example connecting with port in Orocos

TODO: take inspiration from [here](https://gitlab.kuleuven.be/rob-hardware/pickit_camera#connecting-ros-topic-and-orocos-port)

## Running the code
There are two options to run the trajectory generator: the first one uses the Python driver of eTaSL, whereas the second option contains a full robot application in a ROS/Orocos environment.

### Option 1: Python driver of eTaSL
Open a terminal and start up ROS:

    roscore

Then, start the eTaSL controller. It will wait for the first trajectory:

    rosrun etasl_invariants_integration etasl_traj_follow_simulator.py

Also, run the following script to provide a target pose for the trajectories to be calculated towards:

    rosrun etasl_invariants_integration simulate_endpose.py

Finally, start the trajectory generator:

    rosrun etasl_invariants_integration ros_communicate_trajectories.py

Results (trajectories, twists, joint angles) are automatically saved in text files.


#### Optional: record results in bagfile

Record results during execution (necessary for later off-line visualization in rviz):

    rosbag record -a
    
#### Optional: simultaneous visualization in rviz

Prior to running the code above, run the following commands, where 'robot-name' is changed by the robot of your choice. Currently 5 robots are supported (conform the [etasl application template](https://gitlab.kuleuven.be/etasl/etasl_application_template): Universal Robots UR10, Kinova Gen3, Franka Emika Panda, KUKA LWR and KUKA iiwa:

    roslaunch etasl_invariants_integration load_setup_'robot-name'.launch

    rosrun etasl_invariants_integration path_visualisation.py

I recommend not doing this together with rosbag record since then the visualization is also recorded.

#### Parameters

You can enable/disable obstacle avoidance by (un)commenting `sim.readTaskSpecificationString(obstacle_avoidance_specification)` in `etasl_traj_follow_simulator.py`

### Option 2: full robot application
Open a terminal and start up ROS:

    roscore

Then, choose whether you want to use a simulated target pose or the target pose of a real target (currently only HTC Vive supported; make sure as well that robot and Vive system are [calibrated](https://gitlab.kuleuven.be/rob-hardware/htc-vive#calibration-of-htc-vive-and-robot-system)):

    rosrun etasl_invariants_integration simulate_endpose.py

or

    rosrun etasl_invariants_integration tracker_endpose.py

Next, deploy the trajectory following application:

    roscd etasl_invariants_integration/scripts/deploy
    rttlua -i deploy_general.lua (real_robot)

The real\_robot command can be used when the application needs to be used on a real robot.

Finally, run the trajectory generator:

    rosrun etasl_invariants_integration ros_communicate_trajectories.py

#### Smoother behavior by using B-splines
As an improvement to the discrete trajectory following procedure, an alternative is presented as well, where a B-spline fit of the trajectory is done to obtain a smoother following behavior. In order to use this, make sure the [ros spline fitting trajectory](https://gitlab.kuleuven.be/robotgenskill/python_projects/ros-spline-fitting-trajectory) repository is in your catkin workspace as well.

_Note:_ if you want to use this on a real robot, make sure you generate the typekit.

Now, after you ran roscore and the target pose commands, use the following to start the spline fitter:

    rosrun ros_spline_fitting_trajectory spline_fitting.py

Next, deploy another application, namely:

    roscd etasl_invariants_integration/scripts/deploy
    rttlua -i bspline_deploy.lua (real_robot)

Then, again, run the trajectory generator:

    rosrun etasl_invariants_integration ros_communicate_trajectories.py 

## Offline visualization of results in Matlab

Run `visualization/main_process_results.m`

But first change `nb` to the number of trajectories that were generated by the invariants.

## Offline visualization of results in rviz

This is very handy because you can calculate the trajectories once and then check/visualize them in a separate step.

    roscd etasl_invariants_integration

    roscore

    rosparam set /use_sim_time true

The last command is required because the visualization messages  will have different timestamps from the earlier recorded results from the past.

    roslaunch etasl_invariants_integration initialize_rviz_ur10.launch

    python scripts/path_visualisation.py

Replay experiment with obstacle avoidance:

    rosbag play --clock -l -s 2.0 data/bagfiles/2020-05-12-08-21-10.bag

Replay experiment with moving target:

    rosbag play --clock -l -s 3.0 data/bagfiles/2020-06-22-15-42-01.bag

Disable/enable visualization of obstacle in path_visualization.py on the line with command `plot_obstacle(`





## Future work

Implementation

- use roslaunch files to startup code, also include a ros parameter for testing standalone as well
- separate roslaunch files for if you want simultaneous visualization or not
- try to remove all hardcoded stuff like the option for obstacle avoidance, ... and put it in roslaunch files
- path_visualization.py is really inefficient probably...

Methodology

- try using splines both on invariants_ros side and on etasl side to remove the effects of discretization on both sides
- try using slack variable instead of current soft constraint approach for desired target state X_N

Practical

- invariants_ros should ask for the robots current pose to set the initial state
- pose of moving target should come from another ros node doing the target estimation


