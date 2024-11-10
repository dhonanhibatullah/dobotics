# **dobotics**
My robotics research repository, based on Webots Simulation and ROS2 Framework.


## **A. Installation**
- Make sure you have ROS2 installed on your machine. Here are several installation pages for ROS2 (LTS) on Ubuntu: 
    - [20.04 (ROS2 Foxy)](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html), 
    - [22.04 (ROS2 Humble)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html),
    - [24.04 (ROS2 Jazzy)](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

- After that, proceed with the Webots Driver installation for ROS2 on: 
    - [20.04 (ROS2 Foxy)](https://docs.ros.org/en/foxy/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html), 
    - [22.04 (ROS2 Humble)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html),
    - [24.04 (ROS2 Jazzy)](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Webots/Installation-Ubuntu.html).

- Create the Python Virtual Environment with:
    ```bash
    python3 -m venv .venv --system-site-packages --symlinks
    source .venv/bin/activate
    pip install -r requirements.txt
    ```

- Build the ROS2 project with:
    ```bash
    # on ../dobotics
    colcon build
    source install/setup.bash
    ```

- If the building process is not working somehow, you may missing out some important dependencies. Please try the following:
    ```bash
    # on ../dobotics
    source /opt/ros/{YOUR_ROS_DISTRO}/setup.bash
    source .venv/bin/activate
    colcon build
    source install/setup.bash
    ```


## **B. Packages Information**


### **1. dobotics_launchers**
This package is where all the launch files are placed. Open the `launch` folder to see all the available launch files.


### **2. dobotics_controllers**
More info will be updated later.


### **3. dobotics_webots**
This package contains all the required data to run the Webots simulation. The `project` folder is used to configure the simulation world. The `dobotics_webots` folder contains the driver to bridge between the ROS2 and the Webots simulation environment.