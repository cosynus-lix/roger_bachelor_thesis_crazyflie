# Pareto optimal pathfinding

## Project overview

The goal of this project is to study how multiple robots evolve in a common environment. The main approach that we will follow is a centralized one with the goal of finding the Pareto optimal path for each robot, minimizing the global time needed for every robot to reach its goal. This will be studied for when the robots have a few fixed path possibilities each and we can only vary their speed. The solution to this coordination problem will be found primarily through the use of coordination spaces. We will therefore study these spaces in depth in order to optimize both time and space taken by finding tricks to diminish their complexities. We will be testing these methods on Crazyflie drones hence all algorithms support 3 dimensional space but this could also work fine in a more restricted space with, for instance, wheeled robots.

## Installation

For simulations we will be using Gazebo with the ROS operating system on our drones. Gazebo has ROS as a necessary dependency so both can be installed following the instructions bellow. You will need to follow the SITL mode compilation method.

`https://github.com/wuwushrek/sim_cf`

An installation of Python 3 is also required to run the scripts. The code was tested and functional for Python 3.6

This repository can then be cloned anywhere and used immediately.

The installation of Terminator (`sudo apt install terminator`) is also highly recommended to keep all the required terminals organized at runtime.

## Usage

We will denote as `[path]` the path of this repository relative to the home directory of the user. Each one of the following steps will require a new terminal. Much debugging is displayed during loading and at run time so it is not recommended to run them as background tasks.

### Step 1: generating the environment
```
cd ~/[path]
python3 genMap.py maps/example.map
```

### Step 2: Launching the Gazebo environment
```
cd 
roslaunch crazyflie_gazebo multiple_cf_sim.launch world_name:=../../../../../../[path]/salle_lix
```
If another world is used, it is important to have it saved as a `.world` file extension but to not include the `.world` in the world_name.

### Step 3: Creating the drone instances
```
cd ~/catkin_ws/src/crazyflie_ros/sim_cf/crazyflie_gazebo/scripts
./run_cfs.sh 2
```
The `2` may be replaced with the amount of drones that will be launched. On the `salle_lix` world, the default maximum is 4 as the drone entities also need to be in the code of the world.

### Step 4: Linking both of them

Once the terminal has finished flashing rapidly, open the Gazebo interface and click the play button in the lower left corner.

### Step 5: Launch the pathfinding algorithms
This will calculate the trajectories of every drone, solve the coordination problem and then have the drones execute the solution
```
cd ~/[path]
python3 rosControlFile_multi2.py maps/example.map
```

## Changing world

In the case where you wish to change the world, rename the world without obstacles to `basis.world` and add the marker `<!-- obstacles -->` where the obstacles should be added. The program `genMap.py` outputs the world as `salle_lix.world`.

## Project details

This project was the work of Alexis Roger, student of Ecole Polytechnique Bachelor program during his bachelor Thesis at Lix, computer science laboratory of Ecole Polytechnique, under the supervision of Professor Eric Goubault at the start of the year 2020.
