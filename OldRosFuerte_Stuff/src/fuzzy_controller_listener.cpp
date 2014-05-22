//ros generic includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float64.h"
#include <unistd.h>
#include "ros/callback_queue.h"
#include <sstream>
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "visualization_msgs/Marker.h"
#include "stdio.h"

// ardrone include for pub/sub stuff
#include "ardrone_autonomy/Navdata.h"
#include "ar_track_alvar/AlvarMarker.h"


// Fuzzy Controller Includes
#include "fl/Headers.h"

#include <typeinfo>
#include <iomanip>

#include <signal.h>



/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}


void navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
	ROS_INFO("navdata/altd: %ld", (long)navdataPtr->altd);
/*
	if(navdataCount%10==0)
	{
		char buf[200];
		snprintf(buf,200,"Motors: %f %f %f %f",
				(float)navdataPtr->motor1,
				(float)navdataPtr->motor2,
				(float)navdataPtr->motor3,
				(float)navdataPtr->motor4);
		//gui->setMotorSpeeds(std::string(buf));
	}
	navdataCount++;
	*/
}



void ar_track_pose(const visualization_msgs::MarkerConstPtr alvarmarkerPtr)//ar_track_alvar::AlvarMarkerConstPtr alvarmarkerPtr)
{
	ROS_INFO("Entering ar_track thingy");
	ROS_INFO("id: %d", (int)alvarmarkerPtr->id);
	ROS_INFO("pose/position/x: %f", (double)alvarmarkerPtr->pose.position.x);
	ROS_INFO("pose/position/y: %f", (double)alvarmarkerPtr->pose.position.y);
	ROS_INFO("pose/position/z: %f", (double)alvarmarkerPtr->pose.position.z);
	ROS_INFO("pose/orientation/w: %f", (double)alvarmarkerPtr->pose.orientation.w);
	ROS_INFO("pose/orientation/x: %f", (double)alvarmarkerPtr->pose.orientation.x);
	ROS_INFO("pose/orientation/y: %f", (double)alvarmarkerPtr->pose.orientation.y);
	ROS_INFO("pose/orientation/z: %f", (double)alvarmarkerPtr->pose.orientation.z);
	
	
	//pose/orientation/w
}


 void landCb(std_msgs::EmptyConstPtr)
{
	ROS_INFO("command 'LAND' dectected");
}
void toggleStateCb(std_msgs::EmptyConstPtr)
{
	ROS_INFO("command 'RESET' detected");
}
void takeoffCb(std_msgs::EmptyConstPtr)
{
	ROS_INFO("command 'Takeoff' detected");
}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "ardrone_fuzzy_listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber temp_sub = n.subscribe(n.resolveName("ardrone/navdata/tnt"), 100, chatterCallback);
  ros::Subscriber  navdata_sub	   = n.subscribe(n.resolveName("ardrone/navdata"),50, navdataCb);
  ros::Subscriber  ar_track_alvar_pose	   = n.subscribe(n.resolveName("visualization_marker"),50, ar_track_pose);
  
  
  ros::Subscriber takeoff_sub	   = n.subscribe(n.resolveName("ardrone/takeoff"),1, takeoffCb);
  ros::Subscriber land_sub	   = n.subscribe(n.resolveName("ardrone/land"),1, landCb);
  ros::Subscriber toggleState_sub	   = n.subscribe(n.resolveName("ardrone/reset"),1, toggleStateCb);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
