#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <math.h>
#include <tf/transform_listener.h>

#define PI 3.141592f
#define TWO_PI 6.28319f

boost::shared_ptr<sensor_msgs::PointCloud> lastPointCloud_ptr;
boost::shared_ptr<geometry_msgs::PoseStamped> targetPose_ptr;
boost::shared_ptr<nav_msgs::Odometry> lastOdom_ptr;

boost::shared_ptr<tf::TransformListener> odom_tfl_ptr;
boost::shared_ptr<tf::TransformListener> target_tfl_ptr;
boost::shared_ptr<tf::TransformListener> laser_tfl_ptr;

boost::shared_ptr<tf::StampedTransform> current_odom_tf_ptr;
boost::shared_ptr<tf::StampedTransform> target_pose_tf_ptr;
boost::shared_ptr<tf::StampedTransform> laser_point_tf_ptr;

char gotOdom = 0;
char gotTarget = 0;
char gotPointCloud = 0;

boost::shared_ptr<geometry_msgs::Twist> last_signal_ptr;
ros::Publisher motorSignal_pub;

char collisionCourseDetected = 1;
char isInside = 0;

char checkCollisionCourse(geometry_msgs::Twist, sensor_msgs::PointCloud, nav_msgs::Odometry);

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
	//ROS_ERROR("Not really and error... but... Signal sent!");
}

void obstacleDistancesCallback(const sensor_msgs::PointCloud& msg){
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

	/*collisionCourseDetected = checkCollisionCourse(*last_signal_ptr, *lastPointCloud_ptr, *lastOdom_ptr);
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
	/*if(vel == 0)
		vel = 0.1f;
	float angleOut = (omega/2.0f) * (1.0f/vel);
	angleOut = std::min(std::max(angleOut, omega/2), PI/2);*/
	if(omega > -0.1 && omega < 0.1){
		return 0;
	}
	if(omega < 0){
		return ((-PI-omega)/2.0f)*((1.0f-vel)/(1.0f-0.0f)) + (omega/2.0f);
	}else{
		return ((PI-omega)/2.0f)*((1.0f-vel)/(1.0f-0.0f)) + (omega/2.0f);	
	}
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
		angleCenter = (PI/2)-calcAngleCenterModifier(twistIn.angular.z, twistIn.linear.x);
	}else{
		angleCenter = (-PI/2)-calcAngleCenterModifier(twistIn.angular.z, -twistIn.linear.x);
	}

	int angleCenterLidarIndex = (angleCenter*PI)/(180)+180;

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

		char is_open = ((distance>0.30f)||(isnan(distance)))?1:0;

		if(!was_open && is_open){
			currentOpenAreaStart = i;
			ROS_ERROR("Setting current start to: %d", currentOpenAreaStart);
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

	twistOut.angular.z += std::min(angleAdjustments/6, 0.3f); //This could be changed

	ROS_ERROR("Closest lidar index:%d, distance:%f, adjusting:%f, openMid:%d, openStart:%d, openEnd:%d",
				closestIndex,closestDistance, angleAdjustments,openAreaMid,biggestOpenAreaStart,biggestOpenAreaEnd);

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
	char reverse = 0;
	if(deltaAnglePosition < -1.57 || deltaAnglePosition > 1.57){
		if(deltaAnglePose < 1.57 && deltaAnglePose > -1.57){
			reverse = 1;
		}
	}
	
	if((!isInside && distance > 0.10) || (isInside && distance > 0.50)){
		isInside = 0;
		float velByDistance = std::min(std::max(distance*5,0.02f),0.03f);
		if(reverse){
			deltaAnglePosition = capAngle(deltaAnglePosition+PI);
			if(std::abs(deltaAnglePosition)>1.2f){
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
		if(std::abs(deltaAnglePose)>0.08f){
			twistOut.angular.z = std::min(std::max(deltaAnglePose*0.3f,-0.2f),0.2f);
		}else{
			twistOut.angular.z = 0.0f;
		}
		//ROS_INFO("IN POSITION:");
	}

	/*ROS_ERROR("rosieX: %f, rosieY: %f, rosieAngle: %f -- targetX: %f, targetY: %f, targetAngle: %f",
		 rosieWorldX, rosieWorldY, yaw, targetWorldX, targetWorldY, yaw_t);
	ROS_ERROR("distance: %f, pointAngleDiff: %f, targetAngleDiff: %f",
		 distance, deltaAnglePosition, deltaAnglePose);*/	

	return adjustForCollisionAvoidance(twistOut);
}

char checkCollisionCourse(geometry_msgs::Twist signal, sensor_msgs::PointCloud lastPointCloud, nav_msgs::Odometry lastOdom){
	//ROS_INFO("LinearX: %f,LinearY: %f,LinearZ: %f,AngularX: %f,AngularY: %f,AngularZ: %f",
	//		 signal.linear.x,signal.linear.y,signal.linear.z,signal.angular.x,signal.angular.y,signal.angular.z);

	if(!gotPointCloud){
		return 0;
	}

	char reversing = 0;
	if(signal.linear.x < 0){
		reversing = 1;
	}

	float angleWidth = 0.3927f; //std::max(0.3927 + abs(lastOdom.twist.twist.angular.z),0.5); //0.3927 ~ 45 degrees

	tf::Quaternion poseQuaternion = tf::Quaternion(lastOdom.pose.pose.orientation.x,
								lastOdom.pose.pose.orientation.y,
								lastOdom.pose.pose.orientation.z,
								lastOdom.pose.pose.orientation.w);
	tf::Matrix3x3 m(poseQuaternion);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	float angleCenter = 0;
	if(!reversing){
		angleCenter = (PI/2)-calcAngleCenterModifier(signal.angular.z, signal.linear.x);
	}else{
		angleCenter = (-PI/2)-calcAngleCenterModifier(signal.angular.z, -signal.linear.x);
	}

	//ROS_ERROR("Checking for collision: angleCenter:%f", angleCenter);
	for(int i = 0; i < 360; i+=2){
		geometry_msgs::Point32 pointInCloud = lastPointCloud.points[i];
		
		tf::Vector3 point(pointInCloud.x,pointInCloud.y,pointInCloud.z);
		
		tf::Vector3 point_tf = (*laser_point_tf_ptr) * point;

		float angleFromBase = capAngle(atan2(point_tf.x(), point_tf.y())-angleCenter);
		float distance = sqrt(pow(point_tf.x(),2)+pow(point_tf.y(),2));

		if(angleFromBase < angleWidth && angleFromBase > -angleWidth){
			if(!reversing && distance < 0.10 + 0.20*std::abs(std::cos(angleFromBase)) ||
				reversing && distance < 0.20 + 0.20*std::abs(std::cos(angleFromBase))){
				ROS_INFO("Angle: %f",angleFromBase);
				ROS_ERROR("Colliding, angleFromBase:%f", angleFromBase);
				return 1;
			}
		}	 
	}
	return 0;
}

void moveTowardsPose(const nav_msgs::Odometry& currentOdom, const geometry_msgs::PoseStamped& targetPose){	
	
	geometry_msgs::Twist signal = calculateTwist(currentOdom, targetPose);
	if(checkCollisionCourse(signal, *lastPointCloud_ptr, *lastOdom_ptr)){
		ROS_INFO("Collision detected!");
		signal.linear.x = 0;
		signal.angular.z = 0;
	}		

	sendMotorSignal(signal);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_linear_navigator");
	
	odom_tfl_ptr.reset(new tf::TransformListener);
	target_tfl_ptr.reset(new tf::TransformListener);
	laser_tfl_ptr.reset(new tf::TransformListener);

	current_odom_tf_ptr.reset(new tf::StampedTransform);
	target_pose_tf_ptr.reset(new tf::StampedTransform);
	laser_point_tf_ptr.reset(new tf::StampedTransform);

	lastPointCloud_ptr.reset(new sensor_msgs::PointCloud);
	targetPose_ptr.reset(new geometry_msgs::PoseStamped);
	lastOdom_ptr.reset(new nav_msgs::Odometry);
	last_signal_ptr.reset(new geometry_msgs::Twist);

    ros::NodeHandle n;

    motorSignal_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist",1);
    ros::Subscriber targetPose_sub = n.subscribe("/move_base_simple/goal", 1, targetPoseCallback);
    ros::Subscriber currentPose_sub = n.subscribe("/odom", 1, currentPoseCallback);
    ros::Subscriber obstacle_sub = n.subscribe("/my_cloud", 1, obstacleDistancesCallback);

    ros::Rate loop_rate(10);

    while(ros::ok()){
		if(gotOdom && gotTarget){
			moveTowardsPose(*lastOdom_ptr, *targetPose_ptr);
		}
        ros::spinOnce();
        loop_rate.sleep();
    }
}
