This project implements a modular ROS2-based robot orchestration system for executing robotic skills such as motion, pick, place, and pick-and-place operations. The architecture separates orchestration, interfaces, semantic reasoning, and skill execution into independent ROS2 packages. 


How to install: 

if you have problems check https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html

1. Install rosdep to install system dependencies :

sudo apt install python3-rosdep

Once you have ROS 2 installed, make sure you have the most up to date packages:

sudo rosdep init
rosdep update
sudo apt update
sudo apt dist-upgrade

2. Install Colcon the ROS 2 build system with mixin:

sudo apt install python3-colcon-common-extensions
sudo apt install python3-colcon-mixin
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default

Install vcstool :

sudo apt install python3-vcstool

3. Create A Colcon Workspace and Download Tutorials

For tutorials you will need to have a colcon workspace setup.

mkdir -p ~/skill_ws/src
  
4. Download Source Code of the project

Move into your Colcon workspace and pull the MoveIt tutorials source:

cd ~/ws_moveit/src
git clone -b <branch> https://github.com/moveit/moveit2_tutorials

5. Build your Colcon Workspace

First remove all previously installed moveit binaries:

sudo apt remove ros-$ROS_DISTRO-moveit*

The following will install from Debian any package dependencies not already in your workspace. This is the step that will install MoveIt and all of its dependencies:

sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

The next command will configure your Colcon workspace:

cd ~/skill_ws

Änderungen von YT Tutorial übernehmen

colcon build --mixin release

Source the Colcon workspace:

source ~/skill_ws/install/setup.bash
  

Project Structure 

robot_orchestrator 

  

The central coordination layer of the system. 

  

Responsibilities: 

  

Receives high-level task requests 

Coordinates task execution 

Communicates with the semantic layer to validate tasks 

Starts robot skills using ROS2 Actions 

Monitors execution feedback and results 

  

Main features: 

  

Task orchestration 

Workflow management 

Asynchronous communication using ROS2 Actions 

Service-based validation of executable tasks 

robot_interfaces 

  

Contains all custom ROS2 interfaces used by the system. 

  

Responsibilities: 

  

Defines ROS2 Actions 

Defines ROS2 Services 

Provides shared communication interfaces between packages 

  

Main interfaces: 

  

StartSkill.action 

ExecuteAtomicSkill.action 

CanExecuteTask.srv 

  

This package serves as the communication backbone of the project. 

  

robot_semantics 

  

Implements the semantic reasoning layer of the system. 

  

Responsibilities: 

  

Stores and evaluates semantic robot knowledge 

Validates whether a requested task can be executed 

Provides capability checks via ROS2 Services 

  

Main features: 

  

Ontology-based reasoning 

Task feasibility checking 

Semantic abstraction layer for robot capabilities 

  

Communication: 

  

Provides the /can_execute_task service interface 

robot_skills 

  

Contains the executable robot skills and low-level robot control logic. 

  

Responsibilities: 

  

Implements atomic robot skills 

Controls robot motion 

Executes manipulation tasks 

Interfaces with MoveIt and the robot controller 

  

Implemented atomic skills: 

Move 

Grip  

Release 

  

Implemented composite skills: 

Pick [move(pick_frame)-release-move(down1)-grip-move(up1)] 

Place [move(place_frame)-move(down1)-grip-move(up1)] 

PickAndPlace [move(pick_frame)-release-move(down1)-grip-move(up1)-move(place_frame)-move(down1)-grip-move(up1)] 

  

Ontology (.RDF) 

  

  

Main features: 

  

ROS2 Action Servers 

Motion planning with MoveIt 

Execution of predefined and relative target poses 

Communication Architecture 

  

The system uses ROS2 communication mechanisms: 

  

Actions for long-running asynchronous skill execution 

Services for synchronous semantic validation 

Nodes for modular distributed system components 

  

Example workflow: 

  

A task request is sent to the orchestrator 

The orchestrator checks task feasibility using robot_semantics 

If valid, the corresponding skill is started in robot_skills 

Execution feedback and results are returned asynchronously 

Technologies 

ROS2 Humble 

Python 

MoveIt 2 

RViz2 

Ontology-based semantic reasoning 

  

Skill-execution commands for terminal with /start_skill interface  

  

Pick and Place  

ros2 action send_goal /start_skill robot_interfaces/action/StartSkill "{robot_id: 'Robot1', task_type: 'PickAndPlace', pick_frame: 'PoseA', place_frame: 'PoseB'}" --feedback  

  

Pick  

ros2 action send_goal /start_skill robot_interfaces/action/StartSkill "{robot_id: 'Robot1', task_type: 'Pick', pick_frame: 'PoseA'}" --feedback  

  

Place  

ros2 action send_goal /start_skill robot_interfaces/action/StartSkill "{robot_id: 'Robot1', task_type: 'Place', place_frame: 'PoseB'}" --feedback  

  

Move  

ros2 action send_goal /start_skill robot_interfaces/action/StartSkill "{robot_id: 'Robot1', task_type: 'Move', target_frame: 'PoseA'}" --feedback  

  

Grip  

ros2 action send_goal /start_skill robot_interfaces/action/StartSkill "{robot_id: 'Robot1', task_type: 'Grip'}" --feedback  

  

Release  

ros2 action send_goal /start_skill robot_interfaces/action/StartSkill "{robot_id: 'Robot1', task_type: 'Release'}" --feedback  

  

Ontology-request (direct request to ontology with the /can_execute interface) 

  

Robot1 - PickAndPlace  

ros2 service call /can_execute_task robot_interfaces/srv/CanExecuteTask "{robot_id: 'Robot1', task_type: 'PickAndPlace'}"  

  

Robot1 - Welding  

ros2 service call /can_execute_task robot_interfaces/srv/CanExecuteTask "{robot_id: 'Robot1', task_type: "Welding"}"  

  

Robot2 - Welding  

ros2 service call /can_execute_task robot_interfaces/srv/CanExecuteTask "{robot_id: 'Robot2', task_type: "Welding"}"  
