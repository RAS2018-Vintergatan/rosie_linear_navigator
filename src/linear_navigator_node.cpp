#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <rosie_path_finder/CommissionRerun.h>

#define PI 3.141592f
#define TWO_PI 6.28319f

boost::shared_ptr<sensor_msgs::PointCloud> lastPointCloud_ptr;
boost::shared_ptr<geometry_msgs::PoseStamped> targetPose_ptr;
boost::shared_ptr<nav_msgs::Odometry> lastOdom_ptr;
boost::shared_ptr<geometry_msgs::PoseStamped> lastObstaclePoint_ptr;
boost::shared_ptr<geometry_msgs::PoseStamped> lastObstacleBatPoint_ptr;

boost::shared_ptr<tf::TransformListener> odom_tfl_ptr;
boost::shared_ptr<tf::TransformListener> target_tfl_ptr;
boost::shared_ptr<tf::TransformListener> laser_tfl_ptr;
boost::shared_ptr<tf::TransformListener> obstacle_tfl_ptr;
boost::shared_ptr<tf::TransformListener> obstacle_bat_tfl_ptr;

boost::shared_ptr<tf::StampedTransform> current_odom_tf_ptr;
boost::shared_ptr<tf::StampedTransform> target_pose_tf_ptr;
boost::shared_ptr<tf::StampedTransform> laser_point_tf_ptr;
boost::shared_ptr<tf::StampedTransform> obstacle_point_tf_ptr;
boost::shared_ptr<tf::StampedTransform> obstacle_bat_point_tf_ptr;

float obstDist = 1.0f;
float obstBatDist = 1.0f;

char gotOdom = 0;
char gotTarget = 0;
char gotPointCloud = 0;

boost::shared_ptr<geometry_msgs::Twist> last_signal_ptr;
ros::Publisher motorSignal_pub;
ros::Publisher fovVisualisationPublisher;

float positionLowerThreshold = 0.05;
float positionUpperThreshold = 0.15;
float angularLowerThreshold = 0.01;
float angularUpperThreshold = 0.02;

char collisionCourseDetected = 1;
char collidingNear = 0;
char collisionFlag = 0;
char isInside = 0;
char isInsideAngular = 0;
char reverse = 0;

char checkCollisionCourse(geometry_msgs::Twist, sensor_msgs::PointCloud, nav_msgs::Odometry, float, float);

/*
geometry_msgs/Vector3 linear
  float64 x <-- 2D
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z <-- 2D
*/
float lastMotorLinearVelocitySignal = 0;
float lastMotorAngularVelocitySignal = 0;
void sendMotorSignal(geometry_msgs::Twist& msg){
	if(msg.linear.x != lastMotorLinearVelocitySignal || msg.angular.z != lastMotorAngularVelocitySignal){
		motorSignal_pub.publish(msg);
	}
	lastMotorLinearVelocitySignal = msg.linear.x;
	lastMotorAngularVelocitySignal = msg.angular.z;

	*last_signal_ptr = msg;
	//ROS_ERROR("Not really an error... but... Signal sent!");
}

void obstacleObjectBatteryCallback(const visualization_msgs::Marker& msg){
	if(msg.header.seq != lastObstacleBatPoint_ptr->header.seq){
		//*lastPointCloud_ptr = msg;
		try{
			(*obstacle_bat_tfl_ptr).waitForTransform("world", msg.header.frame_id, ros::Time(0), ros::Duration(10.0));
			(*obstacle_bat_tfl_ptr).lookupTransform("world", msg.header.frame_id, ros::Time(0), *obstacle_bat_point_tf_ptr);
		}catch(tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
		
		float obstWorldX = (*obstacle_bat_point_tf_ptr).getOrigin().x() + msg.pose.position.x;
		float obstWorldY = (*obstacle_bat_point_tf_ptr).getOrigin().y() + msg.pose.position.y;
		float obstRobotX = msg.pose.position.x;
		float obstRobotY = msg.pose.position.y;
		float Dist = sqrt(pow(obstRobotX,2) + pow(obstRobotY,2));
		obstBatDist = Dist;

		lastObstacleBatPoint_ptr->header = msg.header;
		lastObstacleBatPoint_ptr->header.frame_id = "world";
		lastObstacleBatPoint_ptr->pose.orientation = msg.pose.orientation;
		lastObstacleBatPoint_ptr->pose.position.x = obstWorldX;
		lastObstacleBatPoint_ptr->pose.position.x = obstWorldY;
		lastObstacleBatPoint_ptr->pose.position.z = 0;
	}	
}

void objectCallback(const visualization_msgs::Marker& msg){
	if(msg.header.seq != lastObstaclePoint_ptr->header.seq){
		//*lastPointCloud_ptr = msg;
		try{
			(*obstacle_tfl_ptr).waitForTransform("world", msg.header.frame_id, ros::Time(0), ros::Duration(10.0));
			(*obstacle_tfl_ptr).lookupTransform("world", msg.header.frame_id, ros::Time(0), *obstacle_point_tf_ptr);
		}catch(tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
		
		float obstWorldX = (*obstacle_point_tf_ptr).getOrigin().x() + msg.pose.position.x;
		float obstWorldY = (*obstacle_point_tf_ptr).getOrigin().y() + msg.pose.position.y;
		float obstRobotX = msg.pose.position.x;
		float obstRobotY = msg.pose.position.y;
		float Dist = sqrt(pow(obstRobotX,2) + pow(obstRobotY,2));
		obstDist = Dist;

		lastObstaclePoint_ptr->header = msg.header;
		lastObstaclePoint_ptr->header.frame_id = "world";
		lastObstaclePoint_ptr->pose.orientation = msg.pose.orientation;
		lastObstaclePoint_ptr->pose.position.x = obstWorldX;
		lastObstaclePoint_ptr->pose.position.x = obstWorldY;
		lastObstaclePoint_ptr->pose.position.z = 0;
	}	
}

void obstacleWallsCallback(const sensor_msgs::PointCloud& msg){
	if(msg.header.seq > lastPointCloud_ptr->header.seq){
		*lastPointCloud_ptr = msg;
		try{
			(*laser_tfl_ptr).waitForTransform("base_link", msg.header.frame_id, ros::Time(0), ros::Duration(10.0));
			(*laser_tfl_ptr).lookupTransform("base_link", msg.header.frame_id, ros::Time(0), *laser_point_tf_ptr);
		}catch(tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
		gotPointCloud = 1;
	}	

	/*collisionCourseDetected = checkCollisionCourse(*last_signal_ptr, *lastPointCloud_ptr, *lastOdom_ptr, obstDist, obstBatDist);
	if(collisionCourseDetected){
		geometry_msgs::Twist signal;
		signal.linear.x = 0;
		signal.angular.z = 0;
		sendMotorSignal(signal);
	}*/
}

/*
rosmsg show nav_msgs/Odometry 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
*/
void currentPoseCallback(const nav_msgs::Odometry& msg){
	if(!gotOdom || msg.header.seq > lastOdom_ptr->header.seq){
		try{
			(*odom_tfl_ptr).waitForTransform("world", msg.header.frame_id, ros::Time(0), ros::Duration(10.0));
			(*odom_tfl_ptr).lookupTransform("world", msg.header.frame_id, ros::Time(0), *current_odom_tf_ptr);
		}catch(tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
		*lastOdom_ptr = msg;
		gotOdom = 1;
	}
}

void targetPoseCallback(const geometry_msgs::PoseStamped& msg){
	*targetPose_ptr = msg;
	try{
		(*target_tfl_ptr).waitForTransform("world", msg.header.frame_id, ros::Time(0), ros::Duration(10.0));
		(*target_tfl_ptr).lookupTransform("world", msg.header.frame_id, ros::Time(0), *target_pose_tf_ptr);
	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	gotTarget = 1;
	isInside = 0;
}

/*
 * Returns the angle between -PI and +PI
 */
float capAngle(const float& angleIn){
	float angleOut = angleIn;
	while(angleOut < 0)
		angleOut += TWO_PI;
	while(angleOut > TWO_PI)
		angleOut -= TWO_PI;
	if(angleOut > PI){
		angleOut = (angleOut-TWO_PI);
	}
	return angleOut;
}

float calcAngleCenterModifier(float omega, float vel){
	float velFactor = vel/0.03f;
	if(vel<0){
		velFactor = -velFactor;
	}
	float return_val = 0.0f;
	if(omega > -0.0001 && omega < 0.0001){
		return_val = 0.0f;
	}
	else if(omega < 0){
		return_val = -((PI/2.0f) - (velFactor * (((PI+omega)/2.0f))));
	}else{
		return_val = (PI/2.0f) - (velFactor * (((PI-omega)/2.0f)));
	}
	
	//ROS_ERROR("calcAngleCenterModifier:: omega: %f, vel: %f, velFactor: %f, return: %f", omega, vel, velFactor, return_val);
	return return_val;
}

geometry_msgs::Twist escapeCloseWalls(){
	int count = 0;
	int currentOpenAreaStart = 0;
	int currentOpenAreaEnd = 360;
	int firstPoint = 0;
	int lastPoint = 0;
	int areaBiggest = 0;
	int areaBiggestEnd = 360;
	int areaBiggestStart = 0;

	char was_open = 1;
	
	for(int i = 0; i < 360; ++i){
		
		int index = i;

		geometry_msgs::Point32 pointInCloud = lastPointCloud_ptr->points[index];
		
		float distance = sqrt(pow(pointInCloud.x, 2)+pow(pointInCloud.y,2));

		char is_open = ((distance>0.35f)||(isnan(distance)))?1:0;

		if(!was_open && is_open){
			currentOpenAreaStart = i;
			//ROS_ERROR("Setting current start to: %d", currentOpenAreaStart);
			lastPoint = currentOpenAreaStart; 	//Last point where open area started before 0
		}
		if(was_open && !is_open){
			currentOpenAreaEnd = i;
			if(++count == 1){
				firstPoint = currentOpenAreaEnd;	//First point where open Area ended after 0
			}
			int areaSizeCurrent = i-currentOpenAreaStart;
			if (areaBiggest < areaSizeCurrent){
				areaBiggest = areaSizeCurrent;
				areaBiggestStart = currentOpenAreaStart;
				areaBiggestEnd = currentOpenAreaEnd;
			}
		}
		
		if(i==359 && !is_open){
			lastPoint = i;
		}
		if(i==0 && !is_open){
			firstPoint = i;
			++count;
		}
		
		was_open = is_open;
	}
	
	int areaSizeCheck = 360 - lastPoint + firstPoint;
	areaBiggest = areaBiggestEnd-areaBiggestStart;
	if (areaSizeCheck>areaBiggest){
		areaBiggest = areaSizeCheck;
		areaBiggestStart = lastPoint - 360;
		areaBiggestEnd = firstPoint;
	}
	int escapeDirection = (areaBiggestEnd+areaBiggestStart)/2;
	
	float escapeAngle = capAngle((float)escapeDirection*PI*2.0f/360.0f);

	/*if(escapeAngle > -1.9 && escapeAngle < -1.1){
		escapeAngle = -1.1;
	}
	if(escapeAngle < 1.9 && escapeAngle > 1.1){
		escapeAngle = 1.1;
	}*/
	
	geometry_msgs::Twist twistOut;

	twistOut.angular.z = std::min(std::max(escapeAngle*0.3f,-0.2f),0.2f);
	twistOut.linear.x = 0.03*cos(escapeAngle);
	
	//ROS_ERROR("adjusting:%f, openStart:%d, openEnd:%d",
	//			escapeAngle,areaBiggestStart,areaBiggestEnd);

	return twistOut;
}


geometry_msgs::Twist adjustForCollisionAvoidance(geometry_msgs::Twist twistIn){
	geometry_msgs::Twist twistOut;
	twistOut.linear.x = twistIn.linear.x;
	twistOut.angular.z = twistIn.angular.z;

	if(!gotPointCloud || (twistIn.linear.x < 0.005 && twistIn.linear.x > -0.005)){
		return twistOut;
	}

	char reversing = 0;
	if(twistIn.linear.x < 0){
		reversing = 1;
	}

	char binHistogram[180];

	float angleCenter = 0;
	if(!reversing){
		angleCenter = (PI/2.0)-calcAngleCenterModifier(twistIn.angular.z, twistIn.linear.x);
	}else{
		angleCenter = (-PI/2.0)-calcAngleCenterModifier(twistIn.angular.z, -twistIn.linear.x);
	}

	int angleCenterLidarIndex = (angleCenter*PI)/(180);

	while(angleCenterLidarIndex < 0){
		angleCenterLidarIndex+=360;
	}
	while(angleCenterLidarIndex >= 360){
		angleCenterLidarIndex-=360;
	}

	int biggestOpenAreaStart = -1;
	int biggestOpenAreaEnd = -1;

	int closestIndex = 0;
	float closestDistance = -1;

	int currentOpenAreaStart = 0;

	char was_open = 0;

	for(int i = 0; i < 180; ++i){
		int index = i+angleCenterLidarIndex-90;
		while(index>=360){
			index -= 360;
		}
		while(index < 0){
			index += 360;
		}

		geometry_msgs::Point32 pointInCloud = lastPointCloud_ptr->points[index];
		
		float distance = sqrt(pow(pointInCloud.x, 2)+pow(pointInCloud.y,2));

		char is_open = ((distance>0.35f)||(isnan(distance)))?1:0;

		if(!was_open && is_open){
			currentOpenAreaStart = i;
			//ROS_ERROR("Setting current start to: %d", currentOpenAreaStart);
		}
		if((!is_open) && was_open  || (i == 179)){
			int currentOpenAreaEnd = i-1;
			if((currentOpenAreaEnd - currentOpenAreaStart) > (biggestOpenAreaEnd - biggestOpenAreaStart)){
				biggestOpenAreaStart = currentOpenAreaStart;
				biggestOpenAreaEnd = currentOpenAreaEnd;
			}
		}

		was_open = is_open;
	}
	if(biggestOpenAreaStart < 0){
		biggestOpenAreaStart = 0;
	}if(biggestOpenAreaEnd < 0){
		biggestOpenAreaEnd = 179;
	}

	int openAreaMid = (biggestOpenAreaStart+((biggestOpenAreaEnd-biggestOpenAreaStart)>>1))-90+1;

	float angleAdjustments = capAngle(((float)openAreaMid)*(TWO_PI)/360.0f);

	twistOut.angular.z += std::min(angleAdjustments/15, 0.35f); //This could be changed

	//ROS_ERROR("Closest lidar index:%d, distance:%f, adjusting:%f, openMid:%d, openStart:%d, openEnd:%d",
	//			closestIndex,closestDistance, angleAdjustments,openAreaMid,biggestOpenAreaStart,biggestOpenAreaEnd);

	return twistOut;
}

geometry_msgs::Twist calculateTwist(const nav_msgs::Odometry& currentOdom, const geometry_msgs::PoseStamped& targetPose){

	geometry_msgs::Twist twistOut;

	float targetWorldX = (*target_pose_tf_ptr).getOrigin().x() + targetPose.pose.position.x;
	float targetWorldY = (*target_pose_tf_ptr).getOrigin().y() + targetPose.pose.position.y;
	float rosieWorldX = (*current_odom_tf_ptr).getOrigin().x() + currentOdom.pose.pose.position.x;
	float rosieWorldY = (*current_odom_tf_ptr).getOrigin().y() + currentOdom.pose.pose.position.y;
	float deltaX = targetWorldX - rosieWorldX;
	float deltaY = targetWorldY - rosieWorldY;
	float distance = sqrt(pow(deltaX,2) + pow(deltaY,2));
	
	tf::Quaternion rosieQuaternion = (*current_odom_tf_ptr).getRotation();
	tf::Matrix3x3 m_w(rosieQuaternion);
	double roll_w, pitch_w, yaw_w;
	m_w.getRPY(roll_w, pitch_w, yaw_w);

	rosieQuaternion = tf::Quaternion(currentOdom.pose.pose.orientation.x,
									currentOdom.pose.pose.orientation.y,
									currentOdom.pose.pose.orientation.z,
									currentOdom.pose.pose.orientation.w);
	tf::Matrix3x3 m(rosieQuaternion);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	yaw += yaw_w;

	tf::Quaternion targetQuaternion = (*target_pose_tf_ptr).getRotation();
	tf::Matrix3x3 m_t_w(targetQuaternion);
	double roll_t_w, pitch_t_w, yaw_t_w;
	m_t_w.getRPY(roll_t_w, pitch_t_w, yaw_t_w);

	targetQuaternion = tf::Quaternion(targetPose.pose.orientation.x,
					targetPose.pose.orientation.y,
					targetPose.pose.orientation.z,
					targetPose.pose.orientation.w);
	tf::Matrix3x3 m_t(targetQuaternion);
	double roll_t, pitch_t, yaw_t;
	m_t.getRPY(roll_t, pitch_t, yaw_t);
	yaw_t += yaw_t_w;
	
	float deltaAnglePosition = capAngle(yaw - atan2(deltaY, deltaX));
	float deltaAnglePose = capAngle(yaw_t-yaw);

	char reverse_now = 0;	

	if((!reverse && (deltaAnglePosition < -PI/2-0.1f || deltaAnglePosition > PI/2+0.1f))
	   || (reverse && ((deltaAnglePosition < -PI/2 || deltaAnglePosition > PI/2)))){
		if((!reverse && (deltaAnglePose > -PI/2  + 0.2f && deltaAnglePose < PI/2 - 0.2f))
	  	 || (reverse && (deltaAnglePose > -PI/2 && deltaAnglePose < PI/2))){
			reverse_now = 1;
		}else{
			reverse_now = 0;
		}
	}else{
		reverse_now = 0;
	}
	
	if((!isInside && distance > positionLowerThreshold) || (isInside && distance > positionUpperThreshold)){
		isInside = 0;
		float velByDistance = std::min(std::max(distance*5,0.02f),0.03f);
		if(reverse_now){
			deltaAnglePosition = capAngle(deltaAnglePosition+PI);
			if(std::abs(deltaAnglePosition)>0.8f){
				twistOut.angular.z = -std::min(std::max(deltaAnglePosition*0.3f,-0.2f),0.2f);
				twistOut.linear.x = 0.0f;
			}else{
				twistOut.angular.z = -std::min(std::max(deltaAnglePosition*0.3f,-0.2f),0.2f);
				twistOut.linear.x = -velByDistance*std::min(std::max(cos(deltaAnglePosition),0.0),1.0);
			}
		}else{
			if(std::abs(deltaAnglePosition)>1.2f){
				twistOut.angular.z = -std::min(std::max(deltaAnglePosition*0.3f,-0.2f),0.2f);
				twistOut.linear.x = 0.0f;
			}else{
				twistOut.angular.z = -std::min(std::max(deltaAnglePosition*0.3f,-0.2f),0.2f);
				twistOut.linear.x = velByDistance*std::min(std::max(cos(deltaAnglePosition),0.0),1.0);
			}
		}
		//ROS_INFO("GO TO POSITION:");
	}else{
		if(distance <= 0.10){
			isInside = 1;
		}
		twistOut.linear.x = 0.0;
		if((!isInsideAngular && std::abs(deltaAnglePose)>angularLowerThreshold) || (isInsideAngular && std::abs(deltaAnglePose)>angularUpperThreshold)){
			isInsideAngular = 0;
			twistOut.angular.z = std::min(std::max(deltaAnglePose*0.2f,-0.1f),0.1f);
		}else{
			isInsideAngular = 1;
			twistOut.angular.z = 0.0f;
		}
		//ROS_INFO("IN POSITION:");
	}

	/*ROS_ERROR("rosieX: %f, rosieY: %f, rosieAngle: %f -- targetX: %f, targetY: %f, targetAngle: %f",
		 rosieWorldX, rosieWorldY, yaw, targetWorldX, targetWorldY, yaw_t);
	ROS_ERROR("distance: %f, pointAngleDiff: %f, targetAngleDiff: %f",
		 distance, deltaAnglePosition, deltaAnglePose);*/	

	return adjustForCollisionAvoidance(twistOut);
	//return twistOut;
}

char checkCollisionCourse(geometry_msgs::Twist signal, sensor_msgs::PointCloud lastPointCloud, nav_msgs::Odometry lastOdom, float camDist, float camBatDist){
	//ROS_INFO("LinearX: %f,LinearY: %f,LinearZ: %f,AngularX: %f,AngularY: %f,AngularZ: %f",
	//		 signal.linear.x,signal.linear.y,signal.linear.z,signal.angular.x,signal.angular.y,signal.angular.z);

	if(!gotPointCloud){
		return 0;
	}

	char reversing = 0;
	if(signal.linear.x < 0){
		reversing = 1;
	}

	tf::Quaternion poseQuaternion = tf::Quaternion(lastOdom.pose.pose.orientation.x,
								lastOdom.pose.pose.orientation.y,
								lastOdom.pose.pose.orientation.z,
								lastOdom.pose.pose.orientation.w);
	tf::Matrix3x3 m(poseQuaternion);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	float angleCenter = 0;
	float angleCenterNear = (PI/2.0);
	if(!reversing){
		angleCenter = (PI/2.0)-calcAngleCenterModifier(signal.angular.z, signal.linear.x);
	}else{
		angleCenterNear = (-PI/2.0);
		angleCenter = (-PI/2.0)-calcAngleCenterModifier(signal.angular.z, -signal.linear.x);
	}

	//ROS_ERROR("camDist: %f, camBatDist: %f, angleCenter: %f, angularZ: %f", camDist, camBatDist, angleCenter, signal.angular.z);
/*	if(camDist < (0.2 * (PI/2 - abs(angleCenter-PI/2))/(PI/2))){
		collidingNear = 1;
		return 1;
	}
	if(camBatDist < (0.2 * (PI/2 - abs(angleCenter-PI/2))/(PI/2))){
		collidingNear = 1;
		return 1;
	}*/

	float angularSpeedSignal = signal.angular.z;
	if(angularSpeedSignal){
		angularSpeedSignal = -angularSpeedSignal;
	}
	float angularSpeed = lastOdom.twist.twist.angular.z;
	if(angularSpeed < 0){
		angularSpeed = -angularSpeed;
	}

	float collisionThreshold = 0.10 + lastOdom.twist.twist.linear.x*3 + angularSpeed/13;
	float collisionNearThreshold = 0.18;
	float angleWidth = 0.3f + angularSpeedSignal/12;
	float angleWidthNear = 2.5f;
	if(reversing){
		collisionThreshold = 0.10 - lastOdom.twist.twist.linear.x*3 + angularSpeed/13;
		collisionNearThreshold = 0.19;
		angleWidth = 0.3f + angularSpeedSignal/12;
		angleWidthNear = 2.5f;
	}
	
	visualization_msgs::Marker line_list;
	line_list.type = visualization_msgs::Marker::LINE_LIST;
	line_list.color.a = 1.0;
	line_list.color.r = 0.1;
	line_list.color.g = 0.2;
	line_list.color.b = 0.2;
	line_list.scale.x = 0.01;
	line_list.header.stamp = ros::Time::now();
	line_list.header.frame_id = "base_link";

	//ROS_ERROR("Checking for collision: angleCenter:%f", angleCenter);
	collisionFlag = 0;
	collidingNear = 0;
	for(int i = 0; i < 360; i+=1){
		geometry_msgs::Point32 pointInCloud = lastPointCloud.points[i];
		if(std::isnan(pointInCloud.x) || std::isnan(pointInCloud.y)){
			continue;
		}
		tf::Vector3 point(pointInCloud.x,pointInCloud.y,pointInCloud.z);
		
		tf::Vector3 point_tf = (*laser_point_tf_ptr) * point;

		float angleToPoint = atan2(point_tf.x(), point_tf.y());
		if(std::isnan(angleToPoint)){
			continue;
		}
		float angleFromBase = capAngle(angleToPoint-angleCenter);
		float angleFromBaseNear = capAngle(angleToPoint-angleCenterNear);
		float distance = sqrt(pow(point_tf.x(),2)+pow(point_tf.y(),2));

		if(angleFromBase < angleWidth && angleFromBase > -angleWidth){
			if(distance < collisionThreshold){
				ROS_INFO("Angle: %f",angleFromBase);
				ROS_ERROR("Colliding, angleFromBase:%f", angleFromBase);
				collisionFlag = 1;
			}
			geometry_msgs::Point p;
			p.x = 0;
			p.y = 0;
			line_list.points.push_back(p);
			p.x = (collisionThreshold)*cos(-(angleToPoint-1.5708));
			p.y = (collisionThreshold)*sin(-(angleToPoint-1.5708));
			line_list.points.push_back(p);
		}if((angleFromBaseNear < angleWidthNear && angleFromBaseNear > -angleWidthNear)){
			float collisionNearThreshold_angle = collisionNearThreshold;
			//ROS_ERROR("LinNav angleFromBaseNear: %f", angleFromBaseNear);
			if(angleFromBaseNear != 0.0 && ((angleFromBaseNear < -0.9 && angleFromBaseNear > -PI+0.9) || (angleFromBaseNear > 0.9 && angleFromBaseNear < PI-0.9))){
				collisionNearThreshold_angle = 0.12/sin(angleFromBaseNear);
				if(collisionNearThreshold_angle < 0){
					collisionNearThreshold_angle = -collisionNearThreshold_angle;
				}
			}

			if(distance < collisionNearThreshold_angle){
				ROS_INFO("Angle: %f",angleFromBase);
				ROS_ERROR("Colliding hard, angleFromBase:%f", angleFromBase);
				collisionFlag = 1;
				collidingNear = 1;
			}
			geometry_msgs::Point p;
			p.x = 0;
			p.y = 0;
			line_list.points.push_back(p);
			p.x = (collisionNearThreshold_angle)*cos(-(angleToPoint-1.5708));
			p.y = (collisionNearThreshold_angle)*sin(-(angleToPoint-1.5708));
			line_list.points.push_back(p);
		}	 
	}
	fovVisualisationPublisher.publish(line_list);
	
	return collisionFlag;
}
void moveTowardsPose(const nav_msgs::Odometry& currentOdom, const geometry_msgs::PoseStamped& targetPose){	
	
	geometry_msgs::Twist signal = calculateTwist(currentOdom, targetPose);
	/*	
	if(checkCollisionCourse(signal, *lastPointCloud_ptr, *lastOdom_ptr, obstDist, obstBatDist)){
		ROS_INFO("Collision detected!");
		signal.linear.x = 0;
		signal.angular.z = 0;
	}
	
	if(collidingNear){
		signal = escapeCloseWalls();
	}
	*/
	sendMotorSignal(signal);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_linear_navigator");
	
	odom_tfl_ptr.reset(new tf::TransformListener);
	target_tfl_ptr.reset(new tf::TransformListener);
	laser_tfl_ptr.reset(new tf::TransformListener);
	obstacle_tfl_ptr.reset(new tf::TransformListener);
	obstacle_bat_tfl_ptr.reset(new tf::TransformListener);

	current_odom_tf_ptr.reset(new tf::StampedTransform);
	target_pose_tf_ptr.reset(new tf::StampedTransform);
	laser_point_tf_ptr.reset(new tf::StampedTransform);
	obstacle_point_tf_ptr.reset(new tf::StampedTransform);
	obstacle_bat_point_tf_ptr.reset(new tf::StampedTransform);

	lastPointCloud_ptr.reset(new sensor_msgs::PointCloud);
	targetPose_ptr.reset(new geometry_msgs::PoseStamped);
	lastOdom_ptr.reset(new nav_msgs::Odometry);
	last_signal_ptr.reset(new geometry_msgs::Twist);
	lastObstaclePoint_ptr.reset(new geometry_msgs::PoseStamped);
	lastObstacleBatPoint_ptr.reset(new geometry_msgs::PoseStamped);

    ros::NodeHandle n;
    
    std::map<std::string,double> linearThresholds, angularThresholds;
    n.getParam("linear_thresholds", linearThresholds);
    n.getParam("angular_thresholds", angularThresholds);

    positionLowerThreshold = linearThresholds["lower"];
    positionUpperThreshold = linearThresholds["upper"];
    angularLowerThreshold = angularThresholds["lower"];
    angularUpperThreshold = angularThresholds["upper"];

    motorSignal_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist",1);
    ros::Subscriber targetPose_sub = n.subscribe("/rosie_pose_goal", 1, targetPoseCallback);
    ros::Subscriber currentPose_sub = n.subscribe("/odom", 1, currentPoseCallback);
    ros::Subscriber obstacle_sub = n.subscribe("/my_cloud", 1, obstacleWallsCallback);
	ros::Subscriber obstacle_object_sub = n.subscribe("/visualization_marker_battery", 1, obstacleObjectBatteryCallback);
	ros::Subscriber any_object_sub = n.subscribe("/visualization_marker", 1, objectCallback);
	fovVisualisationPublisher = n.advertise<visualization_msgs::Marker>("/fov_lines",1);

	
	ros::ServiceClient pathClient = n.serviceClient<rosie_path_finder::CommissionRerun>("/commission_rerun");
	
    ros::Rate loop_rate(10);

    int collisionCount = 0;
    while(ros::ok()){
		if(gotOdom && gotTarget && 
	targetPose_ptr->pose.position.x > 0 && targetPose_ptr->pose.position.y > 0){
			ROS_ERROR("LinNav: Moving to pose");
			moveTowardsPose(*lastOdom_ptr, *targetPose_ptr);
			if(collisionFlag && !collidingNear){
				ROS_ERROR("collisionCount: %d", collisionCount);
			 	if(collisionCount++ > 30){
					collisionCount = 0;
					rosie_path_finder::CommissionRerun srv;
					srv.request.command = 1;
					pathClient.call(srv);
					ROS_ERROR("========= Path responise: %d", srv.response.answer);					
				}
			}
		}
	ROS_ERROR("LinNav: SPIN");
        ros::spinOnce();
	ROS_ERROR("LinNav: SPUN");
        loop_rate.sleep();
    }
}
