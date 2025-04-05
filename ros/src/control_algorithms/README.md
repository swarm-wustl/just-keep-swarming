> Control Algorithms Documentation

Updated (3/22/2025) (Jaxon):

## Testing PID AS:

>> launch sim
ros2 launch simulation simulation_launch.py n_robots:=1 robot_offset:=1.0 robot_arrangement:=CIRCLE

>> PID control
ros2 run control_algorithms pid_control_action_server --ros-args -p is_sim:=true -p debug_lvl:=1 

> debug_lvl: [0-2]: 0 is no debug print, 1 is just moving and turning commands, 2 is full debugging 


>> Test client 
ros2 run control_algorithms test_PID_AC --ros-args -p x_pos:=2.0 -p y_pos:=2.0 -p rob_id:=0 


>> Assembler Action Server:

Comes up with shape for robot to assemble given an input shape. 

Then acts as an action client to the MRAPP Acion server to turn that shape into a path

>> Multi Robot Action Path Planner:

Given a shape constructed by the assembler action path planner, it creates a path of goals for the robot to move to that can achieve the given shape.

The robots are stored modularaly, meaning as they come in they are added correctly, i think lol

The path goal is constructed as a matrix, with the rows being a timestep in the plan and the columsn being the current robot position for its respective robot id (the column position).

Then for each timestep, we create an action future queue that are goal requests to the PID action server, as they complete we remove it from the queue. Once the queue is empty we move to the next time step. 

This is iterated until we fail or reach the goal!

TODO: We may need to create a service that requests the ids of the robot observed by the overhead cv before creating a plan 

>> PID Action Server

This action server is actually in charge of robo control. Given a current position, which is recieved by listening to a robot's respective position pub, we then move the move the robot using basic PID control. We correct position as correctly, etc. This is then pbulished to its repsective robot topic

The execute is worked with a while loop, that loops until we reach the goal. We use spin_some to prevent the while loop from blocking all other background tasks.


> Overiew of arch:

Requested Shape -> ASM_AS -> MRAPP_AS -(at each iteration)-> PID_AS


> Running Our Control System

Start up: 

Assembler Action Server, Multi Robot Action Path Planner, PID Action Server (in their respective terminal)

THis will be a launch file in the future

```
ros2 run control_algorithms multi_robot_path_planner_action_server

ros2 run control_algorithms asm_action_server

ros2 run control_algorithms pid_action_server
```

Then to actuall request a goal

A client will request a shape to asm_action_server. 

Then the robot should take shape!



Unit Tests: 
```
colcon test --packages-select control_algorithms

colcon test-result --all
```




Start the control algorithms server:

```
ros2 run control_algorithms multi_robot_path_planner_action_server
```

Send a test:

```
ros2 run control_algorithms demo_mrpp_action_client
```


