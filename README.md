# Run the Docker Container

## Build
Source the alias to run Docker commands. Build the Docker image. (Make sure you copy your private SSH key and place it as `id_rsa` in the top level of this repository. It will be temporarily copied into the container to clone private repositories.) You can source `bash_utils` in each new terminal or add to your `.bashrc`.
```
source bash_utils
sterling_build
```

## Start
Start the Docker container
```
source bash_utils
sterling_start
```

## Shell
Open a command shell inside the Docker container
```
source bash_utils
sterling_shell
```
Here you will run all the following commands inside the container.

# Gazebo Simulation
You can choose the resolution of simulation you'd liked to run:
```
run_gazebo_low
run_gazebo_medium
run_gazebo_high
```
This should bring up a window of the Husarion Panther in a boxed in area with terrain regions and a U-shaped sidewalk.

# Nav2
To launch the navigation stack for the Husarion Panther:
```
run_nav2
```

# Sterling-Patern Deploy Costmaps
A volume is setup to have `/sterling` inside `/ros2_ws/src` in the container. To deploy Sterling-Patern, build the workspace, setup the `params.yaml` with the correct paths to trained models (you may have to grab from Ryan... or UT Box), and topic names and run the command:
```
colcon build
run_sterling_costmap
```
To see the costmaps on RViz, change the map topic to `/sterling/local_costmap/costmap` or `/sterling/global_costmap/costmap`. 

# Recording Data
If you want to record bag data, you can run this script. It'd recommend running it outside of the container for easy access to the bag. The bag will be saved in the directory you run this script.  
```
record_bag_sim.sh
```

# Exit
To close the Docker container, run this is any of the terminals and it will close the container and kick out all shell instances:
```
exit
sterling_stop
```

