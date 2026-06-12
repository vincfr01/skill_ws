This project implements a modular ROS2-based robot orchestration system for executing robotic skills such as motion, pick, place, and pick-and-place operations. The architecture separates orchestration, interfaces, semantic reasoning, and skill execution into independent ROS2 packages. 


How to install: 

1. You need Ubunto 22.04 (i recommend using dual boot)

2. Install ROS2 Humble on your system

3. Follow the instructions of this youtube tutorial to install the project from the moveit website:

  Tutorial: 		https://www.youtube.com/watch?v=c6Bxbq8UdaI&t=979s 
  
  Link to the website: 	https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html

Now you should have the workspace "ws_moveit" and a working RViz simulation with the panda robot model

4. Clone the following git repository:

cd

git clone https://github.com/vincfr01/skill_ws.git 

(use the link of the git repository where this readme is)
  
5. Copy the content of the src folder from the new skill_ws to the src folder of ws_moveit -> ws_moveit contains now the new extension

6. Copy the robot.rdf from the skill_ws workspace to the following path. You have to create the "ontology" folder there.

/home/frido/ws_moveit/install/robot_semantics/share/robot_semantics/ontology 

7. Open ws_moveit in Visual Studio Code. You need the following folders in you src folder:

- robot_interfaces
- robot_orchestrator
- robot_semantics
- robot_skills

8. install owlready2

pip3 install owlready2 

9. Build the new folders (you should not have any errors)

cd ws_moveit 

source install/setup.bash 

colcon build --mixin debug --packages-select robot_interfaces 

colcon build --mixin debug --packages-select robot_orchestrator 

colcon build --mixin debug --packages-select robot_semantics 

colcon build --mixin debug --packages-select robot_skills 


How to use:

1. Start RViz simulation

ros2 launch moveit2_tutorials demo.launch.py rviz_config:=panda_moveit_config_demo_empty.rviz 

2. Run ontology_node, task_orchestrator and skills.launch.py in seperat terminals

ros2 run robot_semantics ontology_node

ros2 run robot_orchestrator task_orchestrator 

ros2 launch robot_skills skills.launch.py 

3. Enter you commands in a new termial


Commands:

For semantic requests:

"Can Robot1 execute skill PickAndPlace?"

ros2 service call /can_execute_task robot_interfaces/srv/CanExecuteTask "{robot_id: 'Robot1', task_type: 'PickAndPlace'}" 

possible robot_id: "Robot1", "Robot2" and "Robot3" (depending on the robot.rdf you use)

possibls skills:   "move", "grip", "release", "pick", "place" and "pickandplace"

Skill Execution with Robot1:

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

Welding  

ros2 action send_goal /start_skill robot_interfaces/action/StartSkill "{robot_id: 'Robot1', task_type: 'Welding'}" --feedback 

Positions: "PosA", "PosB", "PosC" and "Home"

Atomic Skills: "move", "grip" and "release"

Composite Skills: "Pick", "Place" and "PickAndPlace"


Project Structure 

robot_orchestrator
  
Receives high-level task requests 

Coordinates task execution 

Communicates with the semantic layer to validate tasks 

Starts robot skills using ROS2 Actions 

Monitors execution feedback and results 

Skill-execution commands for terminal with /start_skill interface  

robot_interfaces 

Contains all custom ROS2 interfaces used by the system. 

Defines ROS2 Actions 

Defines ROS2 Services 

Provides shared communication interfaces between packages   

StartSkill.action 

ExecuteAtomicSkill.action 

CanExecuteTask.srv 

robot_semantics 

Implements the semantic reasoning layer of the system. 

Stores and evaluates semantic robot knowledge 

Validates whether a requested task can be executed 

Provides capability checks via ROS2 Services 

Ontology-based reasoning 

Task feasibility checking 

Semantic abstraction layer for robot capabilities 

Provides the /can_execute_task service interface 

robot_skills 

Contains the executable robot skills and low-level robot control logic. 

Implements atomic robot skills 

Controls robot motion 

Executes manipulation tasks 

Interfaces with MoveIt and the robot controller 

The system uses ROS2 communication mechanisms: 

Actions for long-running asynchronous skill execution 

Services for synchronous semantic validation 

Nodes for modular distributed system components 

Example workflow: 

A task request is sent to the orchestrator 

The orchestrator checks task feasibility using robot_semantics 

If valid, the corresponding skill is started in robot_skills 

Execution feedback and results are returned asynchronously 

