#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <math.h>
#include <tf/transform_listener.h>

sensor_msgs::PointCloud lastPointCloud;
geometry_msgs::PoseStamped targetPose;
nav_msgs::Odometry lastOdom;

char gotOdom = 0;
char gotTarget = 0;
char gotPointCloud = 0;

ros::Publisher motorSignal_pub;

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
}

void obstacleDisntacesCallback(const sensor_msgs::PointCloud& msg){
	if(msg.header.seq > lastPointCloud.header.seq){
		lastPointCloud = msg;
		gotPointCloud = 1;
	}
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
	if(!gotOdom || msg.header.seq > lastOdom.header.seq){
		lastOdom = msg;
		gotOdom = 1;
	}
}

void targetPoseCallback(const geometry_msgs::PoseStamped& msg){
	targetPose = msg;
	gotTarget = 1;
}

float capAngle(const float& angleIn){
	float angleOut = angleIn;
	while(angleOut < 0)
			angleOut += 6.28319f;
	while(angleOut > 6.28319f)
		angleOut -= 6.28319f;
	if(angleOut > 3.14159){
		angleOut = (angleOut-6.28319);
	}
	return angleOut;
}

geometry_msgs::Twist calculateTwist(const nav_msgs::Odometry& currentOdom, const geometry_msgs::PoseStamped& targetPose){

	geometry_msgs::Twist twistOut;
	
	tf::TransformListener listener;
	tf::StampedTransform transform;
	tf::StampedTransform transformTarget;
	
	try{
		listener.waitForTransform("world", currentOdom.child_frame_id, ros::Time(0), ros::Duration(10.0) );
		listener.lookupTransform("world", currentOdom.child_frame_id, ros::Time(0), transform);
		listener.waitForTransform("world", targetPose.header.frame_id, ros::Time(0), ros::Duration(10.0) );
		listener.lookupTransform("world", targetPose.header.frame_id, ros::Time(0), transformTarget);
	}catch(tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	
	//ROS_INFO("targetX:%f,targetY:%f",transformTarget.getOrigin().x(),transformTarget.getOrigin().y());

	float worldX = transform.getOrigin().x();
	float worldY = transform.getOrigin().y();
	float deltaX = targetPose.pose.position.x - worldX;
	float deltaY = targetPose.pose.position.y - worldY;
	float distance = sqrt(pow(deltaX,2) + pow(deltaY,2));
	
	tf::Quaternion deltaPoseQuaternion = transform.getRotation();
	tf::Matrix3x3 m(deltaPoseQuaternion);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	tf::Quaternion targetQuaternion = tf::Quaternion(targetPose.pose.orientation.x,
							targetPose.pose.orientation.y,
							targetPose.pose.orientation.z,
							targetPose.pose.orientation.w);
	tf::Matrix3x3 m_t(targetQuaternion);
	double roll_t, pitch_t, yaw_t;
	m_t.getRPY(roll_t, pitch_t, yaw_t);

	
	float deltaAnglePosition = capAngle(yaw - atan2(deltaY, deltaX));
	float deltaAnglePose = capAngle(yaw_t-yaw);
	
	if(distance > 0.10){
		float velByDistance = std::min(std::max((pow(distance,2))*20,0.08),0.15);
		if(std::abs(deltaAnglePosition)>0.1f){
			twistOut.angular.z = -std::min(std::max(deltaAnglePosition,-0.3f),0.3f);
			twistOut.linear.x = 0.0f;
		}else{
			twistOut.angular.z = -std::min(std::max(deltaAnglePosition*0.1f,-0.3f),0.3f);
			twistOut.linear.x = std::min(velByDistance*std::max(pow(cos(deltaAnglePosition),2),0.0),0.1);
		}
		//ROS_INFO("GO TO POSITION:");
	}else{
		twistOut.linear.x = 0.0;
		if(std::abs(deltaAnglePose)>0.08f){
			twistOut.angular.z = std::min(std::max(deltaAnglePose*0.7f,-0.3f),0.3f);
		}else{
			twistOut.angular.z = 0.0f;
		}
		//ROS_INFO("IN POSITION:");
	}
	//ROS_INFO("dX:%f,dY:%f,currentAngle:%f\ndistance: %f,anglePositions: %f, anglePose: %f",
	//	 deltaX,deltaY,yaw,distance,deltaAnglePosition,deltaAnglePose);

	return twistOut;
}

char checkCollisionCourse(geometry_msgs::Twist signal, sensor_msgs::PointCloud lastPointCloud, nav_msgs::Odometry lastOdom){
	//ROS_INFO("LinearX: %f,LinearY: %f,LinearZ: %f,AngularX: %f,AngularY: %f,AngularZ: %f",
	//		 signal.linear.x,signal.linear.y,signal.linear.z,signal.angular.x,signal.angular.y,signal.angular.z);
	if(!gotPointCloud)
		return 1;
	float angleWidth = std::max(1.0 + abs(lastOdom.twist.twist.angular.z),1.5);

	tf::Quaternion poseQuaternion = tf::Quaternion(lastOdom.pose.pose.orientation.x,
								lastOdom.pose.pose.orientation.y,
								lastOdom.pose.pose.orientation.z,
								lastOdom.pose.pose.orientation.w);
	tf::Matrix3x3 m(poseQuaternion);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	float angleCenter = 3.1415/2;
	
	tf::TransformListener listener;
	tf::StampedTransform transform;
	
	//ROS_INFO("Checking for collision:");
	for(int i = 0; i < 360; ++i){
		 geometry_msgs::Point32 pointInCloud = lastPointCloud.points[i];
		 
		try{
			listener.waitForTransform("base_link", lastPointCloud.header.frame_id, ros::Time(0), ros::Duration(10.0) );
			listener.lookupTransform("base_link", lastPointCloud.header.frame_id, ros::Time(0), transform);
		}catch(tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
		
		tf::Vector3 point(pointInCloud.x,pointInCloud.y,pointInCloud.z);
		
		tf::Vector3 point_tf = transform * point;

		float angleFromBase = capAngle(atan2(point_tf.x(), point_tf.y())-angleCenter);
		float distance = sqrt(pow(point_tf.x(),2)+pow(point_tf.y(),2));

		if(angleFromBase < angleWidth && angleFromBase > -angleWidth){
				if(distance < 0.35*std::abs(std::cos(angleFromBase))){
					ROS_INFO("Angle: %f",angleFromBase);
					return 1;
				}
		}	 
	}
	return 0;
}

void moveTowardsPose(const nav_msgs::Odometry& currentOdom, const geometry_msgs::PoseStamped& targetPose){
	geometry_msgs::Twist signal = calculateTwist(currentOdom, targetPose);
	if(checkCollisionCourse(signal, lastPointCloud, lastOdom)){
		ROS_INFO("Collision detected!");
		signal.linear.x = 0;
		signal.angular.z = 0;
	}	

	sendMotorSignal(signal);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_linear_navigator");

    ros::NodeHandle n;

    motorSignal_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist",1);
    ros::Subscriber targetPose_sub = n.subscribe("/move_base_simple/goal", 1, targetPoseCallback);
    ros::Subscriber currentPose_sub = n.subscribe("/odom", 1, currentPoseCallback);
    ros::Subscriber obstacle_sub = n.subscribe("/my_cloud", 1, obstacleDisntacesCallback);

	sensor_msgs::PointCloud lastPointCloud_loc;
	geometry_msgs::PoseStamped targetPose_loc;
	nav_msgs::Odometry lastOdom_loc;

	lastPointCloud = lastPointCloud_loc;
	targetPose = targetPose_loc;
	lastOdom = lastOdom_loc;

    ros::Rate loop_rate(10);

	lastOdom = nav_msgs::Odometry();

    while(ros::ok()){
		if(gotOdom && gotTarget){
			moveTowardsPose(lastOdom, targetPose);
		}
        ros::spinOnce();
        loop_rate.sleep();
    }
}
