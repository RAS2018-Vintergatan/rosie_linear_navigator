# rosie_linear_navigator
A ndoe for the robot Rosie to move from one pose to the other, 
by moving in a straight line and adjusting the orientation
at the start and the end of the course

## How to run
Run the following in different terminals:

**Running node**
```
rosrun rosie_linear_navigator rosie_linear_navigator
```

**Sending signal to linear navigator**
You can send target pose through RVIZ or by publishing to the topic:
```
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"
```
