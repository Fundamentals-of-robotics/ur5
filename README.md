
# FoR Project

A number of objects are stored without any specific order on a stand located within the workspace of a robotic manipulator. 

The manipulator is an
anthropomorphic arm, with a spherical wrist and a two-fingered gripper as end-effector.

The objects can belong to different classes but have a known geometry. 

The objective of the project is to use the manipulator to pick the objects in sequence and to position them on a different stand according to a specified order. A calibrated 3D sensor is used to locate the different objects and to detect their position in the initial stand. 

## List of contents
- [Installation and Configuration](#installation)
- [Contributions](#contributions)
## Installation and Configuration

### Install the environment
There are few possible ways to install the environment

[Docker](https://github.com/mfocchi/lab-docker)

[Natively on Linux](https://github.com/mfocchi/locosim)

[Virtual Machine image](http://www.dropbox.com/sh/5trh0s5y1xzdjds/AACchznJb7606MbQKb6-fUiUa)

### Repository

Clone the Repository:

```bash
  git clone https://github.com/Fundamentals-of-robotics/robotics-project.git
```

Inside __ros_ws__ folder type:
```bash
  catkin_make install
```

And finally, in the __ros_ws/src__ folder:
```bash
  source deve/setup.bash
```
To make sure that the env variables are updated.

### Start the nodes
Inside the terminal, navigate to __row_ws__ folder and type:
```bash
  roslaunch ur5 ur5.launch
```
This command will launch automatically all the nodes (vision, task and motion).
## Contributions

Dorijan Di Zepp [226865] - dorijan.dizepp@studenti.unitn.it

Stefano Camposilvan

Pier Guido

Pietro Giannini  [202099] - pietro.giannini@studenti.unitn.it
