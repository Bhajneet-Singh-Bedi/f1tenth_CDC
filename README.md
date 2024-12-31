# f1tenth_CDC
### This repository contains source codes for autonomous F1tenth car.
### The problem statement was to drive a small scaled F1 car as fast as possible.


_Note:-_
<ins>Files for phase_1 are in autodrive_devkit_phase_1 and for phase_2 in autodrive_devkit_phase_2. Since the name of the package is same so it is recommended to make a separate workspace for both the packages. Or if you want to take the longer way and change the name of either of the packge.... go ahead🙂🙂</ins>


Steps to run this code:- 
- Phase-1
  * Pull the simulator using the following command
  ```bash
    docker pull autodriveecosystem/autodrive_f1tenth_sim:2024-cdc-compete
  ```
  * Run the following command to allow running graphical applications on host via docker.
  ```bash
    xhost local:root
  ```
  *  Run the following command to run the docker:-
    ```bash
    docker run --name autodrive_f1tenth_sim --rm -it --entrypoint /bin/bash --network=host --ipc=host -v /tmp/.X11-unix:/tmp.X11-umix:rw --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_f1tenth_sim:phase_1
    ```
  * In the next terminal:-
    ```bash
    ros2 run ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py && ros2 run gap_follow_follow
    ```
    **Note:- Build and source the package before running these commands**
  * Connect the simulator by clicking on the connect button and make sure the mode is autonomous instead of manual.
  * The F1tenth car will start running, change the viewing angle for better experience.
 
- Phase-2
  * Pull the simulator using the following command
  ```bash
     docker pull autodriveecosystem/autodrive_f1tenth_sim:2024-cdc-compete
  ```
  * Run the following command to allow running graphical applications on host via docker.
  ```bash
    xhost local:root
  ```
  *  Run the following command to run the docker:-
    ```bash
    docker run --name autodrive_f1tenth_sim --rm -it --entrypoint /bin/bash --network=host --ipc=host -v /tmp/.X11-unix:/tmp.X11-umix:rw --env DISPLAY --privileged --gpus all autodriveecosystem/autodrive_f1tenth_sim:phase_2
    ```
  * In the next terminal:-
    ```bash
    ros2 run ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py && ros2 run gap_follow_follow
    ```
    **Note:- Build and source the package before running these commands**
  * Connect the simulator by clicking on the connect button and make sure the mode is autonomous instead of manual.
  * The F1tenth car will start running, change the viewing angle for better experience. 
