
# FoR Project

A number of objects are stored without any specific order on a stand located within the workspace of a robotic manipulator. 

The manipulator is an
anthropomorphic arm, with a spherical wrist and a two-fingered gripper as end-effector.

The objects can belong to different classes but have a known geometry. 

The objective of the project is to use the manipulator to pick the objects in sequence and to position them on a different stand according to a specified order. 

A calibrated 3D sensor is used to locate the different objects and to detect their position in the initial stand. 

## List of contents
- [Installation and Configuration](#installation)
- [Start the simulation](#start-the-simulation)
- [Contributions](#contributions)

## Installation and Configuration

### Install the environment

There are few possible ways to install the environment

[Docker](https://github.com/mfocchi/lab-docker)

[Natively on Linux](https://github.com/mfocchi/locosim)

[Virtual Machine image](http://www.dropbox.com/sh/5trh0s5y1xzdjds/AACchznJb7606MbQKb6-fUiUa)

### Repository

Install the Eigen library:
```bash
  sudo apt install libeigen3-dev
```

Clone the repository inside the __row_ws/src__ folder:
```bash
  cd ros_ws/src
  git clone https://github.com/Fundamentals-of-robotics/ur5.git
```

Inside __ros_ws__ folder type:
```bash
  cd ..
  catkin_make install
```

And finally, in the __ros_ws__ folder:
```bash
  source devel/setup.bash
```
This is to make sure that the env variables are updated. This command is needed to be done every time you open a **new terminal**.

### World files

Copy the content of the **worlds** folder inside **ros_ws/src/locosim/ros_impedance_controller/worlds** (you should have already other world files in the same folder such as tavolo.world, tavolo_obstacles.world).

After that, compile as done before with: 

```bash
  cd ros_ws
  catkin_make install
  source devel/setup.bash
```

### Blocks models

If not already imported, copy the content of the **blocks** folder inside **ros_ws/src/locosim/ros_impedance_controller/worlds/models** (you should have already other models in the same folder such as tavolo, brick, wall).

#### Model path

It can be that the folder where you copied the files is not inside the gazebo model path. To make shure that you can use the new models, even if they are not in the gazebo path, you can edit **GAZEBO_MODEL_PATH** through the **~/.bashrc**:
```bash
  gedit ~/.bashrc
```

then you can add your model path with, you can add as much as paths as you want, you need to split them using ':' :
```bash
  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<path to your model>
```

to check if the new path has been added:
```bash
  echo $GAZEBO_MODEL_PATH
```
this will print all the path associated with the gazebo model path.

After that, compile as done before with: 

```bash
  cd ros_ws
  catkin_make install
  source devel/setup.bash
```

## Vision setup
Now you need to install the components to load and run the YOLOv5 model
```bash
  cd ~ros_ws/src/ur5/vision
  git clone https://github.com/ultralytics/yolov5.git
  cd yolov5
  pip3 install -r requirements.txt
  pip install torchvision
  pip install scikit-learn
```
After that, compile as done before with: 
```bash
  cd ros_ws
  catkin_make install
  source devel/setup.bash
```

### Configuration

In order to be able to grab all the blocks you should change the maximum gripper aperture value inside:
**ros_ws/src/locosim/robot_descriptions/gripper_description/gripper_description/urdf/soft_finger.xacro**
we change the **line 25** of this file to:

```xacro
  <limit effort="50" velocity="10.0" lower="-0.5" upper="0.90" />
```   

If not already done, you should set **gripper_sim**, inside
__ros_ws/src/locosim/robot_control/base_controllers/params.py__ at **line 45**, to **True**. This will cause the gripper to be considered as a movable body rather than a rigid one.

```python
  'gripper_sim': True, # False: the gripper is treated as a Rigid Body
```                  

Rewrite the **line 71** of the file **ros_ws/src/locosim/robot_control/base_controllers/ur5_generic.py** as

```python
  self.world_name = project_table.world
```
At the end, go to the directory ros_ws and use the following commands: 

```bash
  catkin_make install
  source devel/setup.bash
```

## Start the simulation

To start the simulation, navigate to 
__ros_ws/src/locosim/robot_control/base_controllers__ folder.
Once done type: 
```bash
  python3 ur5_generic.py
```
This will start a Gazebo and a rViz window.

After the homing procedure has finished (you can see it printed on the terminal), navigate using a new terminal, to __ros_ws__ folder and type:
```bash
  roslaunch ur5 ur5.launch
```
This command will launch automatically all the nodes (vision, task and motion).

In the terminal, a series of debugging information will be printed to keep track of the actual state of the manipulator.

### Intermediate point & Damping factor
The project implements two different solutions. 
To select a specific solution, modify the value at line **25** of **ross_ws/src/ur5/include/ur5/ur5_motion_library.h** as follow:
```c++
  #define INTERMEDIATE_POINT 0 //to use the dynamic damping factor
  #define INTERMEDIATE_POINT 1 //to use the intermediate point
```

and then compile the project as always:
```bash
  catkin_make install
```

## Contributions

Pier Guido Seno [227485] pierguido.seno@studenti.unitn.it

Pietro Giannini [202099] pietro.giannini@studenti.unitn.it

Stefano Camposilvan [226697] stefano.camposilvan@studenti.unitn.it

Dorijan Di Zepp [226865] - dorijan.dizepp@studenti.unitn.it 
