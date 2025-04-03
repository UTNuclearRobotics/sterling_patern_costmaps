# Patern Simulation

This package will install and bring up all the necessary components for running Sterling-Patern simulation demos in Gazebo. 

## 1. Install Dependencies
  <details>
  <summary><a href="https://docs.docker.com/engine/install/ubuntu/">Docker</a></summary>
  <br>

  Installation Guide: 🔗 https://docs.docker.com/engine/install/ubuntu/

  </details>
  <details>
  <summary><a href="https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/1.14.5/install-guide.html">NVIDIA Container Toolkit</a></summary>
  <br>

  Installtion Guide: 🔗 https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/1.14.5/install-guide.html

  </details>
  <details>
  <summary><a href="https://docs.docker.com/engine/install/linux-postinstall/">Docker: Post Installation Steps</a></summary>
  <br>
        
   Installation Guide: 🔗 https://docs.docker.com/engine/install/linux-postinstall/
   
  </details>

## 2. Installation and Setup
1) __Clone Repository__
   
   ```shell
   git clone -b patern/husarion-packages git@github.com:UTNuclearRobotics/sterling_sim_docker.git
   ```
3) __Setup SSH Environment__

   ```shell
   ./setup.sh
    ```
3) __Build Docker Image__
   
   ```shell
   make build
   ```

## 3. Run
- Start the Docker container.

    ```shell
    make start
    ```
    
## 4. Interfacing
- Open `patern_gazebo_c` container shell.
   
    ```shell
    make shell
    ```

- 🐳 Launch the Patern costmaps. A volume is setup to have `/sterling` inside `/ros2_ws/src` in the container. To deploy Sterling-Patern, build the workspace, setup the `params.yaml` with the correct paths to trained models (you may have to grab from Ryan... or UT Box), and topic names. You should see RViz launch to view all the costmaps from Nav2 and Sterling combined.

    The [default](https://utexas.app.box.com/folder/312320392388) Patern model folder should contain:
    - terrain_rep.pt
    - cost_head.pt
    - fpro.pt
    - fvis.pt
    - upro.pt
    - uvis.pt

    ```shell
    colcon build
    run_sterling_costmap
    ```

- 🐳 Run Gazebo simulation. Pick one. This should bring up a window of the Husarion Panther in a boxed in area with terrain regions and a U-shaped sidewalk.

    ```shell
    run_gazebo_low
    run_gazebo_medium
    run_gazebo_high
    ```

- 🐳 Run the navigation stack for the Husarion Panther.

    ```shell
    run_nav2
    ```

- Stop the container.
    
    ```shell
    make stop
    ```

- Record bag data. It'd recommend running it outside of the container for easy access to the bag. The bag will be saved in the directory you run this script.

    ```shell
    record_bag_sim.sh
    ```
