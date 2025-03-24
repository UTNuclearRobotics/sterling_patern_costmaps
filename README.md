# Run the Docker Container

## Build
Source the alias to run Docker commands. Build the Docker image. (Make sure you copy your private SSH key and place it as `id_rsa` in the top level of this repository. It will be temporarily copied into the container to clone private repositories.) 
```
source bash_utils
sterling_build
```

## Start
Start the Docker container
```
sterling_start
```

## Shell
Open a command shell inside the Docker container
```
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
To deploy Sterling-Patern, make sure you setup the `params.yaml` with the correct paths to trained models (you may have to grab from UT Box) and topic names and run the command:
```
run_sterling_costmap
```
To see the costmaps on RViz, change the map topic to `/sterling/local_costmap/costmap` or `/sterling/global_costmap/costmap`. 

# Recording Data
If you want to record bag data, you can run this script. It'd recommend running it outside of the container for easy access but it doesn't matter.
```
record_bag_sim.sh
```

