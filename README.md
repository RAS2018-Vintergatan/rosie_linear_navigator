# rosie_linear_navigator
A ndoe for the robot Rosie to move from one pose to the other, 
by moving in a straight line and adjusting the orientation
at the start and the end of the course

## How to run
Run the following in different terminals:

**Running node**
```
rosrun rosie_path_navigator rosie_path_navigator
```

**Sending signal to linear navigator**
You can send target pose through RVIZ or by publishing to the topic:
```
rostopic pub -r 0.1 rosie_path geometry_msgPoseArray "{header: 
	{frame_id: 'base_frame'},
	poses: [
		{position: {x: 0.0, y: 2.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
		{position: {x: 0.2, y: 2.03, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
	]
```
