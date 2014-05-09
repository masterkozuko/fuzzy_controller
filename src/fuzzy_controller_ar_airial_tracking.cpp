//ros generic includes
#include <stdarg.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float64.h"
#include <unistd.h>
#include "ros/callback_queue.h"
#include "ros/time.h"

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

#define DEFAULT_CTRL_ORIENTATION 1.0
#define DEFAULT_CTRL_HEIGHT 700.0

 //mah variables XD
const int tagid_Sonar_Elevation_Controller = 555;
int num_tags = 0;
long sonar_reading = 0;
int incoming_tag = 0;
 
double orientation_x = 0;
double orientation_y = 0;
double tag_pose_x = 0;
double tag_pose_y = 0;
double tag_pose_z = 0;

bool tag_detected = false;
const double timeOut_betweenControlStates = 3.0;

double LastTagTimeSeconds = 0.0;
double TimeHereSeconds = 0.0;

class  TagInfoAce
{
		
	private:
	int tag_id;
	fl::Engine* engine;
	fl::InputVariable* inputVariable[10];
	fl::OutputVariable* outputVariable[10];
	double outputs[10];
	
	public:
	TagInfoAce()
	{
		this->engine = NULL;
		for(int i = 0; i < 10; i++)
		{
			this->inputVariable[i] =  NULL;
			this->outputVariable[i] = NULL;
		}
	}
	TagInfoAce(int tag_id, fl::Engine* engine = NULL, fl::InputVariable* inputVariable = NULL, fl::OutputVariable* outputVariable = NULL)
	{
		this->tag_id = tag_id;
		this->engine = engine;
		this->inputVariable[0] = inputVariable;
		this->outputVariable[0] = outputVariable;
	}
	void setEngine(fl::Engine* engine = NULL, fl::InputVariable* inputVariable = NULL, fl::OutputVariable* outputVariable= NULL)
	{
		this->engine = engine;
		this->inputVariable[0] = inputVariable;
		this->outputVariable[0] = outputVariable;
	}
	void setEngine(fl::Engine* engine = NULL, int num = 1, ...)
	{
		this->engine = engine;
		va_list arguments;                     // A place to store the list of arguments
		
		va_start ( arguments, num );           // Initializing arguments to store all values after num
		for ( int x = 0; x < num; x++ )
		{        // Loop until all numbers are added
			this->inputVariable[x] = va_arg ( arguments, fl::InputVariable*); 
			this->outputVariable[x] = va_arg ( arguments, fl::OutputVariable*); 
		}
		va_end ( arguments );		
	}
	double processInput(double error_input)
	{
		fl::scalar fuzzy_input = error_input;
		inputVariable[0]->setInputValue(fuzzy_input);
		engine->process();
		return outputVariable[0]->defuzzify();
	}
	/*
	float* outputs = processInput(3, error_x, error_y, error_z);
	outputs[0]
	outputs[1]
	outputs[2]
	*/
	double* processInput(int num, ...)
	{
		ROS_INFO("In process inputs");
		va_list arguments;                     // A place to store the list of arguments
		va_start ( arguments, num );           // Initializing arguments to store all values after num
		ROS_INFO("Iterating through varrags");
		for ( int x = 0; x < num; x++ )        // Loop until all numbers are added
		{
			fl::scalar fuzzy_input = (double)(va_arg (arguments, double));
			ROS_INFO("Setting input %d", x);
			inputVariable[x]->setInputValue(fuzzy_input);
			// Crashes before this point up
			engine->process();
			ROS_INFO("Setting output %d", x);
			outputs[x] = outputVariable[x]->defuzzify();
		}
		va_end ( arguments );
		ROS_INFO("RETURNING OUTPUTS");
		return outputs;
	}
	int getTagId()
	{
		return tag_id;
	}
	~TagInfoAce()
	{
		if(engine)
		{
			delete engine;
			engine = NULL;
		}
	}
	
};
TagInfoAce tag_infos[19];
 

float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ar_track_pose(const visualization_msgs::MarkerConstPtr alvarmarkerPtr)//ar_track_alvar::AlvarMarkerConstPtr alvarmarkerPtr)
{

	tag_pose_x = (double)alvarmarkerPtr->pose.position.x;
	tag_pose_y = (double)alvarmarkerPtr->pose.position.y;
	tag_pose_z = (double)alvarmarkerPtr->pose.position.z;
	
	
	incoming_tag = (int)alvarmarkerPtr->id;
	ros::Time LastTagTime = ros::Time::now();
	LastTagTimeSeconds  = LastTagTime.toSec();
	//LastTagTime = ros::Time::now().toSec();
	//ROS_INFO("Last Tagged detected at %f", LastTagTimeSeconds);
}
 
double Large_Tag_FuzzyController_Orientation_Rules(double current_position, double target_position = DEFAULT_CTRL_ORIENTATION)
{
	double error = current_position - target_position;
	double output = 0.0;
	bool found = false;
	
	
	for(int i = 0; i < num_tags; i++)
	{
		if((incoming_tag  == tag_infos[i].getTagId()) & ((TimeHereSeconds - LastTagTimeSeconds) < (timeOut_betweenControlStates)))
		{
			found = true;
			output = tag_infos[i].processInput(error);
			ROS_INFO("Found tag %d, processing input",incoming_tag);
			break;
		}
	}
	if(!found)
	{
		//ROS_INFO("Incorrect tag found, not processing input");
	}
	return output;
}
double* FuzzyController_ArTracking_Main(double* current_position, double* target_position)
{
	//double error = current_position -target_position;
	double* output;
	bool found = false;
	
	
	for(int i = 0; i < num_tags; i++)
	{
		if((incoming_tag  == tag_infos[i].getTagId()) & ((TimeHereSeconds - LastTagTimeSeconds) < (timeOut_betweenControlStates)))
		{
			found = true;
			output = tag_infos[i].processInput(3, current_position[0] - target_position[0], 
												current_position[1] - target_position[1], 
												current_position[2] - target_position[2]);
			ROS_INFO("Found tag %d, processing input",incoming_tag);
			break;
		}
	}
	if(!found)
	{
		//ROS_INFO("Incorrect tag found, not processing input");
	}
	return output;
}

 double FuzzyController_5Rules_Sonar_Elevation(double current_position, double target_position)
{
	double error = current_position -target_position;
	double output = 0.0;
	bool found = false;
	for(int i = 0; i < num_tags; i++)
	{
		if(tagid_Sonar_Elevation_Controller == tag_infos[i].getTagId())
		{
			found = true;
			output = tag_infos[i].processInput(error);
			//ROS_INFO("Last Tag:%d lost, hovering with sonar",incoming_tag);
			//tag_detected = true;
			break;
		}
	}
	if(!found)
	{
		//ROS_INFO("Incorrect tag found, not processing input");
		//tag_detected = false;
	}
	return output;
}

 
 bool keep_running = false;
 bool drone_takeoff = false;	
 
void landCb(std_msgs::EmptyConstPtr)
{
	drone_takeoff =false;
	ROS_INFO("command 'LAND' dectected");
	keep_running = drone_takeoff;
	
}
void toggleStateCb(std_msgs::EmptyConstPtr)
{
	ROS_INFO("command 'RESET' detected");
	
}
void takeoffCb(std_msgs::EmptyConstPtr)
{
	ROS_INFO("command 'Takeoff' detected");
	drone_takeoff = true;
	keep_running = drone_takeoff;
	
}



void velCb(const geometry_msgs::TwistConstPtr vel)
{
}

void navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
	//ROS_INFO("navdata/altd: %ld", (long)navdataPtr->altd);
	sonar_reading = (long)navdataPtr->altd;
}
struct ControlCommand
{
	inline ControlCommand() {roll = pitch = yaw = gaz = 0;}
	inline ControlCommand(double roll, double pitch, double yaw, double gaz)
	{
		this->roll = roll;
		this->pitch = pitch;
		this->yaw = yaw;
		this->gaz = gaz;
	}
	double yaw, roll, pitch, gaz;
};

void setEngineArTracking_main_control(TagInfoAce& ts)
{
	fl::Engine* engine = new fl::Engine;
	engine->setName("Hoover");

	fl::InputVariable* inputVariable1 = new fl::InputVariable;
inputVariable1->setEnabled(true);
inputVariable1->setName("ar_pose_x");
inputVariable1->setRange(-1.000, 1.000);
inputVariable1->addTerm(new fl::Trapezoid("Far_Left", -30.000, -0.800, -0.740, -0.510));
inputVariable1->addTerm(new fl::Triangle("Left", -0.660, -0.360, 0.000));
inputVariable1->addTerm(new fl::Triangle("Center", -0.222, 0.000, 0.222));
inputVariable1->addTerm(new fl::Triangle("Right", 0.000, 0.366, 0.674));
inputVariable1->addTerm(new fl::Trapezoid("Far_Right", 0.538, 0.730, 10.000, 30.000));
engine->addInputVariable(inputVariable1);

fl::InputVariable* inputVariable2 = new fl::InputVariable;
inputVariable2->setEnabled(true);
inputVariable2->setName("ar_pose_y");
inputVariable2->setRange(-2.000, 2.000);
inputVariable2->addTerm(new fl::Ramp("too_close", -0.600, -1.576));
inputVariable2->addTerm(new fl::Triangle("far", 0.000, 0.660, 1.268));
inputVariable2->addTerm(new fl::Triangle("centered", -0.600, 0.000, 0.600));
inputVariable2->addTerm(new fl::Triangle("close", -1.172, -0.600, 0.000));
inputVariable2->addTerm(new fl::Ramp("too_far", 0.600, 1.576));
engine->addInputVariable(inputVariable2);

fl::InputVariable* inputVariable3 = new fl::InputVariable;
inputVariable3->setEnabled(true);
inputVariable3->setName("ar_pose_z");
inputVariable3->setRange(-2.000, 2.000);
inputVariable3->addTerm(new fl::Ramp("too_low", 0.600, 1.444));
inputVariable3->addTerm(new fl::Triangle("high", -1.232, -0.668, 0.000));
inputVariable3->addTerm(new fl::Triangle("centered", -0.360, 0.000, 0.360));
inputVariable3->addTerm(new fl::Triangle("low", 0.000, 0.672, 1.424));
inputVariable3->addTerm(new fl::Ramp("too_high", -0.600, -1.424));
engine->addInputVariable(inputVariable3);

fl::OutputVariable* outputVariable1 = new fl::OutputVariable;
outputVariable1->setEnabled(true);
outputVariable1->setName("cmd_x_vel");
outputVariable1->setRange(-1.000, 1.000);
outputVariable1->fuzzyOutput()->setAccumulation(new fl::Maximum);
outputVariable1->setDefuzzifier(new fl::Centroid(200));
outputVariable1->setDefaultValue(fl::nan);
outputVariable1->setLockValidOutput(false);
outputVariable1->setLockOutputRange(false);
outputVariable1->addTerm(new fl::Gaussian("stay", 0.000, 0.070));
outputVariable1->addTerm(new fl::ZShape("go_far_left", -1.000, 0.010));
outputVariable1->addTerm(new fl::Gaussian("go_left", -0.280, 0.080));
outputVariable1->addTerm(new fl::Gaussian("go_right", 0.280, 0.080));
outputVariable1->addTerm(new fl::SShape("go_far_right", 0.010, 1.000));
engine->addOutputVariable(outputVariable1);

fl::OutputVariable* outputVariable2 = new fl::OutputVariable;
outputVariable2->setEnabled(true);
outputVariable2->setName("cmd_z_vel");
outputVariable2->setRange(-2.000, 2.000);
outputVariable2->fuzzyOutput()->setAccumulation(new fl::Maximum);
outputVariable2->setDefuzzifier(new fl::Centroid(200));
outputVariable2->setDefaultValue(fl::nan);
outputVariable2->setLockValidOutput(false);
outputVariable2->setLockOutputRange(false);
outputVariable2->addTerm(new fl::ZShape("fast_backward", -2.000, -0.120));
outputVariable2->addTerm(new fl::Gaussian("backward", -0.800, 0.250));
outputVariable2->addTerm(new fl::Gaussian("stay", 0.000, 0.180));
outputVariable2->addTerm(new fl::Gaussian("forward", 0.830, 0.250));
outputVariable2->addTerm(new fl::SShape("fast_forward", 0.120, 2.000));
engine->addOutputVariable(outputVariable2);

fl::OutputVariable* outputVariable3 = new fl::OutputVariable;
outputVariable3->setEnabled(true);
outputVariable3->setName("cmd_y_vel");
outputVariable3->setRange(-2.000, 2.000);
outputVariable3->fuzzyOutput()->setAccumulation(new fl::Maximum);
outputVariable3->setDefuzzifier(new fl::Centroid(200));
outputVariable3->setDefaultValue(fl::nan);
outputVariable3->setLockValidOutput(false);
outputVariable3->setLockOutputRange(false);
outputVariable3->addTerm(new fl::SShape("go_higher", 0.120, 2.000));
outputVariable3->addTerm(new fl::Gaussian("go_up", 0.760, 0.210));
outputVariable3->addTerm(new fl::Gaussian("stay", 0.000, 0.150));
outputVariable3->addTerm(new fl::Gaussian("go_down", -0.760, 0.210));
outputVariable3->addTerm(new fl::ZShape("go_lower", -2.000, -0.120));
engine->addOutputVariable(outputVariable3);

fl::RuleBlock* ruleBlock = new fl::RuleBlock;
ruleBlock->setEnabled(true);
ruleBlock->setName("");
ruleBlock->setConjunction(new fl::Minimum);
ruleBlock->setDisjunction(new fl::Maximum);
ruleBlock->setActivation(new fl::Minimum);
ruleBlock->addRule(fl::Rule::parse("if ar_pose_x is Far_Left then cmd_x_vel is go_far_right", engine));
ruleBlock->addRule(fl::Rule::parse("if ar_pose_x is Center then cmd_x_vel is stay", engine));
ruleBlock->addRule(fl::Rule::parse("if ar_pose_x is Left then cmd_x_vel is go_right", engine));
ruleBlock->addRule(fl::Rule::parse("if ar_pose_x is Right then cmd_x_vel is go_left", engine));
ruleBlock->addRule(fl::Rule::parse("if ar_pose_x is Far_Right then cmd_x_vel is go_far_left", engine));
ruleBlock->addRule(fl::Rule::parse("if ar_pose_y is too_close then cmd_z_vel is fast_backward", engine));
ruleBlock->addRule(fl::Rule::parse("if ar_pose_y is close then cmd_z_vel is backward", engine));
ruleBlock->addRule(fl::Rule::parse("if ar_pose_y is centered then cmd_z_vel is stay", engine));
ruleBlock->addRule(fl::Rule::parse("if ar_pose_y is too_far then cmd_z_vel is fast_forward", engine));
ruleBlock->addRule(fl::Rule::parse("if ar_pose_y is far then cmd_z_vel is forward", engine));
ruleBlock->addRule(fl::Rule::parse("if ar_pose_z is too_high then cmd_y_vel is go_lower", engine));
ruleBlock->addRule(fl::Rule::parse("if ar_pose_z is high then cmd_y_vel is go_down", engine));
ruleBlock->addRule(fl::Rule::parse("if ar_pose_z is centered then cmd_y_vel is stay", engine));
ruleBlock->addRule(fl::Rule::parse("if ar_pose_z is low then cmd_y_vel is go_up", engine));
ruleBlock->addRule(fl::Rule::parse("if ar_pose_z is too_low then cmd_y_vel is go_higher", engine));
engine->addRuleBlock(ruleBlock);

	
	
	/*ts.setEngine(engine, 3, inputVariable1, outputVariable1,
	inputVariable2, outputVariable2, 
	inputVariable3, outputVariable3;
	ROS_INFO("Set up fuzzy system for Tracking ArTag id:10");
	*/
}
	
	
void setEngineSonarElevation(TagInfoAce& ts)
{
	//CONTROLLER GOES BELOW THIS COMMENT
	fl::Engine* engine = new fl::Engine;
	engine->setName("qtfuzzylite");
	engine->addHedge(new fl::Any);
	engine->addHedge(new fl::Extremely);
	engine->addHedge(new fl::Not);
	engine->addHedge(new fl::Seldom);
	engine->addHedge(new fl::Somewhat);
	engine->addHedge(new fl::Very);
	
	fl::InputVariable* inputVariable1 = new fl::InputVariable;
	inputVariable1->setName("sonar_reading");
	inputVariable1->setRange(-1500.000, 1500.000);
	
	inputVariable1->addTerm(new fl::ZShape("too_lows", -1500.000,-300.000));
	inputVariable1->addTerm(new fl::Gaussian("a_little_low", -300.000,100.000));
	inputVariable1->addTerm(new fl::Bell("On_Target", 0.000,50.000,3.000));
	inputVariable1->addTerm(new fl::Gaussian("a_little_high", 300.000,100.000));
	inputVariable1->addTerm(new fl::SShape("too_high", 300.000,1500.000));
	engine->addInputVariable(inputVariable1);
	
	fl::OutputVariable* outputVariable1 = new fl::OutputVariable;
	outputVariable1->setName("cmd_vel_gaz");
	outputVariable1->setRange(-1.000, 1.000);
	outputVariable1->setDefaultValue(std::numeric_limits<fl::scalar>::quiet_NaN());
	outputVariable1->setDefuzzifier(new fl::Centroid(500));
	outputVariable1->fuzzyOutput()->setAccumulation(new fl::Maximum);
	
	outputVariable1->addTerm(new fl::ZShape("large_decrease_velocity", -0.750,-0.150));
	outputVariable1->addTerm(new fl::Gaussian("small_decrease_velocity", -0.150,0.050));
	outputVariable1->addTerm(new fl::Bell("no_change", 0.000,0.010,1.000));
	outputVariable1->addTerm(new fl::Gaussian("small_increase_velocity", 0.150,0.050));
	outputVariable1->addTerm(new fl::SShape("large_increase_velocity", 0.150,0.750));
	engine->addOutputVariable(outputVariable1);
	
	fl::RuleBlock* ruleblock1 = new fl::RuleBlock;
	ruleblock1->setName("");
	ruleblock1->setConjunction(new fl::Minimum);
	ruleblock1->setDisjunction(new fl::Maximum);
	ruleblock1->setActivation(new fl::Minimum);
	
	ruleblock1->addRule(fl::Rule::parse("if sonar_reading is too_lows then cmd_vel_gaz is large_increase_velocity", engine));
	ruleblock1->addRule(fl::Rule::parse("if sonar_reading is a_little_high then cmd_vel_gaz is small_decrease_velocity", engine));
	ruleblock1->addRule(fl::Rule::parse("if sonar_reading is a_little_low then cmd_vel_gaz is small_increase_velocity", engine));
	ruleblock1->addRule(fl::Rule::parse("if sonar_reading is On_Target then cmd_vel_gaz is no_change", engine));
	ruleblock1->addRule(fl::Rule::parse("if sonar_reading is too_high then cmd_vel_gaz is large_decrease_velocity", engine));
	engine->addRuleBlock(ruleblock1);
	//END OF CONTROLLER
	
	ts.setEngine(engine, inputVariable1, outputVariable1);
	ROS_INFO("Set up fuzzy system for sonar elevation");
}

int main(int argc, char **argv)
{
	ROS_INFO("Main now Running!!");
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
  ros::init(argc, argv, "fuzzy_lander");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows yousetUnparsedRule to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>(n.resolveName("ardrone/navdata/tnt"), 1000);
  ros::Publisher vel_pub	   = n.advertise<geometry_msgs::Twist>(n.resolveName("cmd_vel"),1);
  ros::Subscriber vel_sub	   = n.subscribe(n.resolveName("cmd_vel"),50, velCb);
  ros::Publisher tum_ardrone_pub	   = n.advertise<std_msgs::String>(n.resolveName("tum_ardrone/com"),50);
  //ros::Publisher dronepose_sub	   = n.subscribe(n.resolveName("ardrone/predictedPose"),50, droneposeCb);
  ros::Subscriber  navdata_sub	   = n.subscribe(n.resolveName("ardrone/navdata"),50, navdataCb);
  ros::Publisher takeoff_pu1b	   = n.advertise<std_msgs::Empty>(n.resolveName("ardrone/takeoff"),1);
  ros::Publisher land_pub	   = n.advertise<std_msgs::Empty>(n.resolveName("ardrone/land"),1);
  ros::Publisher toggleState_pub	   = n.advertise<std_msgs::Empty>(n.resolveName("ardrone/reset"),1);
  ros::Subscriber takeoff_sub	   = n.subscribe(n.resolveName("ardrone/takeoff"),1, takeoffCb);
  ros::Subscriber land_sub	   = n.subscribe(n.resolveName("ardrone/land"),1, landCb);
  ros::Subscriber toggleState_sub	   = n.subscribe(n.resolveName("ardrone/reset"),1, toggleStateCb);
  ros::ServiceClient toggleCam_srv        = n.serviceClient<std_srvs::Empty>(n.resolveName("ardrone/togglecam"),1);
  ros::ServiceClient flattrim_srv         = n.serviceClient<std_srvs::Empty>(n.resolveName("ardrone/flattrim"),1);
  
  //ar_Track_alvar stuff
  ros::Subscriber  ar_track_alvar_pose	   = n.subscribe(n.resolveName("visualization_marker"),50, ar_track_pose);
  //tag_infos[0] = TagInfoAce(0, engine, input , output);

  
  tag_infos[0] = TagInfoAce(10);
  tag_infos[1] = TagInfoAce(170);
  tag_infos[2] = TagInfoAce(10);
  tag_infos[3] = TagInfoAce(tagid_Sonar_Elevation_Controller);  
  num_tags = 4;
  
  //setEngineLargeTag_z_rot(tag_infos[2]);
  //setEngineLargeTag_X_controller(tag_infos[2]);
  setEngineSonarElevation(tag_infos[3]);
  setEngineArTracking_main_control(tag_infos[0]);
 
  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  keep_running = false;
  while (ros::ok())
  {
	  
	  if ((sonar_reading<0) & (keep_running == true))
	  {
		  ROS_INFO("Oh noes, the sonar reading is: %ld, which is means we are going to crash!", sonar_reading);
		  ROS_INFO("Attempting to send land command and stopping controller");
		  land_pub.publish(std_msgs::Empty());
		  ROS_INFO("Done sending command... I hoped it landed safely XD");
		  keep_running = false;
		  
		 //ROS_INFO("Mah Count: %d, mah altd: %ld", count, sonar_reading);
		  
	  }
	  if (keep_running == true)
	  {
		  double output_velocity_x = 0;
		  double output_velocity_y = 0;
		  double output_velocity_z = 0;
		  
		  ros::Time TimeHere = ros::Time::now();
		  TimeHereSeconds = TimeHere.toSec();		  
		  
		  if ((!tag_detected) & ((TimeHereSeconds - LastTagTimeSeconds) > (timeOut_betweenControlStates)))
		  {
			  //ROS_INFO("runing FuzzyController_5Rules_Sonar_Elevation");
			  output_velocity_z = FuzzyController_5Rules_Sonar_Elevation(sonar_reading,700);
		  }
		  
		  if ((incoming_tag == 10) & ((TimeHereSeconds - LastTagTimeSeconds) < (timeOut_betweenControlStates/5)))
		  {
			  double inputs[3] = {tag_pose_x,tag_pose_y,tag_pose_z};
				  
			  double desired_inputs[3] = {0.0,2.0,0.0};

			  ROS_INFO("Calling main control scheme");
			  double* outputs = FuzzyController_ArTracking_Main(inputs, desired_inputs);
			  ROS_INFO("Extracting outputs");
			  ROS_INFO("They are: X=%f Y=%f Z=%f", outputs[1], outputs[2], outputs[3]);
			  output_velocity_x = outputs[1];
			  output_velocity_z = outputs[2];
			  output_velocity_y = outputs[3];
			  
		  }
		  if ((incoming_tag > 0) & ((TimeHereSeconds - LastTagTimeSeconds) < (timeOut_betweenControlStates)))
		  {
			  //ROS_INFO("runing Large_Tag_FuzzyController_Orientation_Rules");
			  //output_velocity_z_rot = Large_Tag_FuzzyController_Orientation_Rules(orientation_x);
		  }
		  //ROS_INFO("Velocity out CMD for Z: %f with input %ld", output_velocity_z, sonar_reading);
		  //ROS_INFO("Sending command to drone now!");
		  /*
		  inputVariable1->setName("orientation_x");
		  inputVariable2->setName("displacement_x");
		  inputVariable3->setName("displacement_z");
		  inputVariable4->setName("displacement_y");
		  
		  outputVariable1->setName("cmd_vel_z_rot");
		  outputVariable2->setName("cmd_vel_y_linear");
		  outputVariable3->setName("cmd_vel_x_linear");
		  outputVariable4->setName("cmd_vel_z_linear");
		  */
		  
		  
		  //inline 				         ControlCommand(double roll, double pitch, double yaw, double gaz)
		  ControlCommand cmd =   ControlCommand(output_velocity_y,output_velocity_x,0,output_velocity_z);
		  geometry_msgs::Twist cmdT;
		  cmdT.linear.z = cmd.gaz;
		  cmdT.linear.x = -cmd.pitch;
		  cmdT.linear.y = -cmd.roll;
		  cmdT.angular.x = cmdT.angular.y =0.1;
		  vel_pub.publish(cmdT);
	  }
	  else
	  {
		  if (drone_takeoff == true)
		  {
			  land_pub.publish(std_msgs::Empty());
		  }
	  }
	  

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

