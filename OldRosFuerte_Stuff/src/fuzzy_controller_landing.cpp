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
	float outputs[10];
	
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
	float processInput(float error_input)
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
	float* processInput(int num, ...)
	{
		ROS_INFO("In process inputs");
		va_list arguments;                     // A place to store the list of arguments
		va_start ( arguments, num );           // Initializing arguments to store all values after num
		ROS_INFO("Iterating through varrags");
		for ( int x = 0; x < num; x++ )        // Loop until all numbers are added
		{
			
			fl::scalar fuzzy_input = (float)(va_arg (arguments, double));
			ROS_INFO("Setting input %d", x);
			inputVariable[x]->setInputValue(fuzzy_input);
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
	//ROS_INFO("Entering ar_track thingy");
	/*
	ROS_INFO("id: %d", (int)alvarmarkerPtr->id);
	ROS_INFO("pose/position/x: %f", (double)alvarmarkerPtr->pose.position.x);
	ROS_INFO("pose/position/y: %f", (double)alvarmarkerPtr->pose.position.y);
	ROS_INFO("pose/position/z: %f", (double)alvarmarkerPtr->pose.position.z);
	ROS_INFO("pose/orientation/w: %f", (double)alvarmarkerPtr->pose.orientation.w);
	ROS_INFO("pose/orientation/x: %f", (double)alvarmarkerPtr->pose.orientation.x);
	ROS_INFO("pose/orientation/y: %f", (double)alvarmarkerPtr->pose.orientation.y);
	ROS_INFO("pose/orientation/z: %f", (double)alvarmarkerPtr->pose.orientation.z);
	*/
	orientation_x = (double)alvarmarkerPtr->pose.orientation.x;
	tag_pose_x = (double)alvarmarkerPtr->pose.position.x;
	tag_pose_y = (double)alvarmarkerPtr->pose.position.y;
	tag_pose_z = (double)alvarmarkerPtr->pose.position.z;
	
	
	incoming_tag = (int)alvarmarkerPtr->id;
	ros::Time LastTagTime = ros::Time::now();
	LastTagTimeSeconds  = LastTagTime.toSec();
	//LastTagTime = ros::Time::now().toSec();
	//ROS_INFO("Last Tagged detected at %f", LastTagTimeSeconds);
}
 
float Large_Tag_FuzzyController_Orientation_Rules(double current_position, double target_position = DEFAULT_CTRL_ORIENTATION)
{
	float error = current_position -target_position;
	float output = 0.0;
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
float* Large_Tag_FuzzyController_Orientation_Main(float* current_position, float* target_position)
{
	//float error = current_position -target_position;
	float* output;
	bool found = false;
	
	
	for(int i = 0; i < num_tags; i++)
	{
		if((incoming_tag  == tag_infos[i].getTagId()) & ((TimeHereSeconds - LastTagTimeSeconds) < (timeOut_betweenControlStates)))
		{
			found = true;
			output = tag_infos[i].processInput(4, current_position[0] - target_position[0], 
												current_position[1] - target_position[1], 
												current_position[2] - target_position[2], 
												current_position[3] - target_position[3]);
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

 float FuzzyController_5Rules_Sonar_Elevation(double current_position, double target_position = DEFAULT_CTRL_HEIGHT)
{
	float error = current_position -target_position;
	float output = 0.0;
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

void setEngineLargeTag_main_control(TagInfoAce& ts)
{
	fl::Engine* engine = new fl::Engine;
	engine->setName("qtfuzzylite");
	
	fl::InputVariable* inputVariable1 = new fl::InputVariable;
	inputVariable1->setEnabled(true);
	inputVariable1->setName("orientation_x");
	inputVariable1->setRange(-1.000, 1.000);
	inputVariable1->addTerm(new fl::ZShape("way_left", -1.000, -0.180));
	inputVariable1->addTerm(new fl::Gaussian("a_little_left", -0.320, 0.110));
	inputVariable1->addTerm(new fl::Bell("On_Target", 0.000, 0.030, 3.000));
	inputVariable1->addTerm(new fl::Gaussian("a_little_right", 0.320, 0.110));
	inputVariable1->addTerm(new fl::SShape("way_right", 0.180, 1.000));
	engine->addInputVariable(inputVariable1);
	
	fl::InputVariable* inputVariable2 = new fl::InputVariable;
	inputVariable2->setEnabled(true);
	inputVariable2->setName("displacement_x");
	inputVariable2->setRange(-0.060, 0.060);
	inputVariable2->addTerm(new fl::ZShape("way_left", -0.060, -0.020));
	inputVariable2->addTerm(new fl::Gaussian("too_left", -0.020, 0.006));
	inputVariable2->addTerm(new fl::Gaussian("centered", 0.000, 0.004));
	inputVariable2->addTerm(new fl::Gaussian("too_right", 0.020, 0.006));
	inputVariable2->addTerm(new fl::SShape("way_right", 0.020, 0.060));
	engine->addInputVariable(inputVariable2);
	
	fl::InputVariable* inputVariable3 = new fl::InputVariable;
	inputVariable3->setEnabled(true);
	inputVariable3->setName("displacement_z");
	inputVariable3->setRange(0.000, 0.040);
	inputVariable3->addTerm(new fl::Gaussian("too_low", -0.008, 0.003));
	inputVariable3->addTerm(new fl::Gaussian("centered", 0.000, 0.004));
	inputVariable3->addTerm(new fl::Gaussian("too_high", 0.014, 0.004));
	inputVariable3->addTerm(new fl::SShape("way_high", 0.011, 0.040));
	engine->addInputVariable(inputVariable3);

	fl::InputVariable* inputVariable4 = new fl::InputVariable;
	inputVariable4->setEnabled(true);
	inputVariable4->setName("displacement_y");
	inputVariable4->setRange(-0.020, 0.020);
	inputVariable4->addTerm(new fl::ZShape("way_high", -0.020, -0.006));
	inputVariable4->addTerm(new fl::Gaussian("too_high", -0.009, 0.004));
	inputVariable4->addTerm(new fl::Gaussian("centered", 0.000, 0.002));
	inputVariable4->addTerm(new fl::Gaussian("too_low", 0.010, 0.004));
	inputVariable4->addTerm(new fl::SShape("way_low", 0.008, 0.020));
	engine->addInputVariable(inputVariable4);

	fl::OutputVariable* outputVariable1 = new fl::OutputVariable;
	outputVariable1->setEnabled(true);
	outputVariable1->setName("cmd_vel_z_rot");
	outputVariable1->setRange(-0.400, 0.400);
	outputVariable1->fuzzyOutput()->setAccumulation(new fl::Maximum);
	outputVariable1->setDefuzzifier(new fl::Centroid(200));
	outputVariable1->setDefaultValue(0.000);
	outputVariable1->setLockValidOutput(false);
	outputVariable1->setLockOutputRange(false);
	outputVariable1->addTerm(new fl::ZShape("large_turn_left", -0.750, -0.100));
	outputVariable1->addTerm(new fl::Gaussian("small_turn_left", -0.130, 0.030));
	outputVariable1->addTerm(new fl::Bell("hold_direction", 0.000, 0.010, 1.000));
	outputVariable1->addTerm(new fl::Gaussian("small_turn_right", 0.130, 0.030));
	outputVariable1->addTerm(new fl::SShape("large_turn_right", 0.150, 0.750));
	engine->addOutputVariable(outputVariable1);
	fl::OutputVariable* outputVariable2 = new fl::OutputVariable;
	outputVariable2->setEnabled(true);
	outputVariable2->setName("cmd_vel_y_linear");
	outputVariable2->setRange(-0.750, 0.750);
	outputVariable2->fuzzyOutput()->setAccumulation(new fl::Maximum);
	outputVariable2->setDefuzzifier(new fl::Centroid(200));
	outputVariable2->setDefaultValue(0.000);
	outputVariable2->setLockValidOutput(false);
	outputVariable2->setLockOutputRange(false);
	outputVariable2->addTerm(new fl::ZShape("large_move_right", -0.750, -0.115));
	outputVariable2->addTerm(new fl::Gaussian("small_move_right", -0.125, 0.035));
	outputVariable2->addTerm(new fl::Gaussian("do_nothing", 0.000, 0.020));
	outputVariable2->addTerm(new fl::Gaussian("small_move_left", 0.125, 0.035));
	outputVariable2->addTerm(new fl::SShape("large_move_left", 0.120, 0.750));
	engine->addOutputVariable(outputVariable2);
	
	fl::OutputVariable* outputVariable3 = new fl::OutputVariable;
	outputVariable3->setEnabled(true);
	outputVariable3->setName("cmd_vel_x_linear");
	outputVariable3->setRange(-0.750, 0.750);
	outputVariable3->fuzzyOutput()->setAccumulation(new fl::Maximum);
	outputVariable3->setDefuzzifier(new fl::Centroid(200));
	outputVariable3->setDefaultValue(0.000);
	outputVariable3->setLockValidOutput(false);
	outputVariable3->setLockOutputRange(false);
	outputVariable3->addTerm(new fl::ZShape("large_move_backward", -0.750, -0.120));
	outputVariable3->addTerm(new fl::Gaussian("small_move_backward", -0.125, 0.035));
	outputVariable3->addTerm(new fl::Gaussian("do_nothing", 0.000, 0.035));
	outputVariable3->addTerm(new fl::Gaussian("small_move_foward", 0.125, 0.035));
	outputVariable3->addTerm(new fl::SShape("large_move_foward", 0.125, 0.750));
	engine->addOutputVariable(outputVariable3);
	
	fl::OutputVariable* outputVariable4 = new fl::OutputVariable;
	outputVariable4->setEnabled(true);
	outputVariable4->setName("cmd_vel_z_linear");
	outputVariable4->setRange(-0.450, 0.750);
	outputVariable4->fuzzyOutput()->setAccumulation(new fl::Maximum);
	outputVariable4->setDefuzzifier(new fl::Centroid(200));
	outputVariable4->setDefaultValue(0.000);
	outputVariable4->setLockValidOutput(false);
	outputVariable4->setLockOutputRange(false);
	outputVariable4->addTerm(new fl::ZShape("large_move_down", -1.000, -0.200));
	outputVariable4->addTerm(new fl::Gaussian("small_move_down", -0.200, 0.080));
	outputVariable4->addTerm(new fl::Gaussian("do_nothing", 0.000, 0.080));
	outputVariable4->addTerm(new fl::Gaussian("small_move_up", 0.200, 0.080));
	outputVariable4->addTerm(new fl::SShape("large_move_up", 0.200, 1.000));
	engine->addOutputVariable(outputVariable4);
	
	fl::RuleBlock* ruleBlock = new fl::RuleBlock;
	ruleBlock->setEnabled(true);
	ruleBlock->setName("");
	ruleBlock->setConjunction(new fl::Minimum);
	ruleBlock->setDisjunction(new fl::Maximum);
	ruleBlock->setActivation(new fl::Minimum);
	ruleBlock->addRule(fl::Rule::parse("if orientation_x is way_left then cmd_vel_z_rot is large_turn_right", engine));
	ruleBlock->addRule(fl::Rule::parse("if orientation_x is a_little_left then cmd_vel_z_rot is small_turn_right", engine));
	ruleBlock->addRule(fl::Rule::parse("if orientation_x is On_Target then cmd_vel_z_rot is hold_direction", engine));
	ruleBlock->addRule(fl::Rule::parse("if orientation_x is a_little_right then cmd_vel_z_rot is small_turn_left", engine));
	ruleBlock->addRule(fl::Rule::parse("if orientation_x is way_right then cmd_vel_z_rot is large_turn_left", engine));
	ruleBlock->addRule(fl::Rule::parse("if displacement_x is way_left then cmd_vel_y_linear is large_move_right", engine));
	ruleBlock->addRule(fl::Rule::parse("if displacement_x is too_left then cmd_vel_y_linear is small_move_right", engine));
	ruleBlock->addRule(fl::Rule::parse("if displacement_x is centered then cmd_vel_y_linear is do_nothing", engine));
	ruleBlock->addRule(fl::Rule::parse("if displacement_x is too_right then cmd_vel_y_linear is small_move_left", engine));
	ruleBlock->addRule(fl::Rule::parse("if displacement_x is way_right then cmd_vel_y_linear is large_move_left", engine));
	ruleBlock->addRule(fl::Rule::parse("if displacement_y is way_low then cmd_vel_x_linear is large_move_backward", engine));
	ruleBlock->addRule(fl::Rule::parse("if displacement_y is too_low then cmd_vel_x_linear is small_move_backward", engine));
	ruleBlock->addRule(fl::Rule::parse("if displacement_y is centered then cmd_vel_x_linear is do_nothing", engine));
	ruleBlock->addRule(fl::Rule::parse("if displacement_y is too_high then cmd_vel_x_linear is small_move_foward", engine));
	ruleBlock->addRule(fl::Rule::parse("if displacement_y is way_high then cmd_vel_x_linear is large_move_foward", engine));
	ruleBlock->addRule(fl::Rule::parse("if displacement_z is way_high then cmd_vel_z_linear is large_move_down", engine));
	ruleBlock->addRule(fl::Rule::parse("if displacement_z is too_high then cmd_vel_z_linear is small_move_down", engine));
	ruleBlock->addRule(fl::Rule::parse("if displacement_z is centered then cmd_vel_z_linear is do_nothing", engine));
	ruleBlock->addRule(fl::Rule::parse("if displacement_z is too_low then cmd_vel_z_linear is small_move_up", engine));
	engine->addRuleBlock(ruleBlock);
	
	ts.setEngine(engine, 4, inputVariable1, outputVariable1,
	inputVariable2, outputVariable2, 
	inputVariable3, outputVariable3,
	inputVariable4, outputVariable4);
	ROS_INFO("Set up fuzzy system for Large Tag");
}
	
void setEngineLargeTag_z_rot(TagInfoAce& ts)
{
	
	// Controler goes below this comment
	fl::Engine* engine = new fl::Engine;
	engine->setName("qtfuzzylite");
	engine->addHedge(new fl::Any);
	engine->addHedge(new fl::Extremely);
	engine->addHedge(new fl::Not);
	engine->addHedge(new fl::Seldom);
	engine->addHedge(new fl::Somewhat);
	engine->addHedge(new fl::Very);
	
	fl::InputVariable* inputVariable1 = new fl::InputVariable;
	inputVariable1->setName("orientation_w");
	inputVariable1->setRange(-1.000, 1.000);
	
	inputVariable1->addTerm(new fl::ZShape("way_left", -1.000,-0.180));
	inputVariable1->addTerm(new fl::Gaussian("a_little_left", -0.320,0.110));
	inputVariable1->addTerm(new fl::Bell("On_Target", 0.000,0.030,3.000));
	inputVariable1->addTerm(new fl::Gaussian("a_little_right", 0.320,0.110));
	inputVariable1->addTerm(new fl::SShape("way_right", 0.180,1.000));
	engine->addInputVariable(inputVariable1);
	
	fl::OutputVariable* outputVariable1 = new fl::OutputVariable;
	outputVariable1->setName("cmd_vel_z_rot");
	outputVariable1->setRange(-1.000, 1.000);
	outputVariable1->setDefaultValue(std::numeric_limits<fl::scalar>::quiet_NaN());
	outputVariable1->setDefuzzifier(new fl::Centroid(500));
	outputVariable1->fuzzyOutput()->setAccumulation(new fl::Maximum);
	
	outputVariable1->addTerm(new fl::ZShape("large_turn_left", -0.750,-0.100));
	outputVariable1->addTerm(new fl::Gaussian("small_turn_left", -0.130,0.030));
	outputVariable1->addTerm(new fl::Bell("hold_direction", 0.000,0.010,1.000));
	outputVariable1->addTerm(new fl::Gaussian("small_turn_right", 0.130,0.030));
	outputVariable1->addTerm(new fl::SShape("large_turn_right", 0.150,0.750));
	engine->addOutputVariable(outputVariable1);
	
	fl::RuleBlock* ruleblock1 = new fl::RuleBlock;
	ruleblock1->setName("");
	ruleblock1->setConjunction(new fl::Minimum);
	ruleblock1->setDisjunction(new fl::Maximum);
	ruleblock1->setActivation(new fl::Minimum);
	
	ruleblock1->addRule(fl::Rule::parse("if orientation_w is way_left then cmd_vel_z_rot is large_turn_right", engine));
	ruleblock1->addRule(fl::Rule::parse("if orientation_w is a_little_left then cmd_vel_z_rot is small_turn_right", engine));
	ruleblock1->addRule(fl::Rule::parse("if orientation_w is On_Target then cmd_vel_z_rot is hold_direction", engine));
	ruleblock1->addRule(fl::Rule::parse("if orientation_w is a_little_right then cmd_vel_z_rot is small_turn_left", engine));
	ruleblock1->addRule(fl::Rule::parse("if orientation_w is way_right then cmd_vel_z_rot is large_turn_left", engine));
	engine->addRuleBlock(ruleblock1);
	// END OF CONTROLLER
	
	
	
	ts.setEngine(engine, inputVariable1, outputVariable1);
	ROS_INFO("Set up fuzzy system for Large Tag");
}

void setEngineLargeTag_X_controller(TagInfoAce& ts)
{
	//CONTROLLER HERE
	
	fl::Engine* engine = new fl::Engine;
	engine->setName("qtfuzzylite");
	
	fl::InputVariable* inputVariable1 = new fl::InputVariable;
	inputVariable1->setName("large_pose_position_x");
	inputVariable1->setRange(-0.050, 0.050);
	
	inputVariable1->addTerm(new fl::ZShape("way_left", -0.050,-0.010));
	inputVariable1->addTerm(new fl::Gaussian("tracker_slightly_left", -0.010,0.003));
	inputVariable1->addTerm(new fl::Gaussian("tracker_centered", 0.000,0.004));
	inputVariable1->addTerm(new fl::Gaussian("tracker_slightly_right", 0.010,0.003));
	inputVariable1->addTerm(new fl::SShape("way_right", 0.010,0.050));
	engine->addInputVariable(inputVariable1);
	
	fl::OutputVariable* outputVariable1 = new fl::OutputVariable;
	outputVariable1->setName("cmd_vel_linear_y");
	outputVariable1->setRange(-1.000, 1.000);
	outputVariable1->setDefaultValue(std::numeric_limits<fl::scalar>::quiet_NaN());
	outputVariable1->setDefuzzifier(new fl::Centroid(500));
	outputVariable1->fuzzyOutput()->setAccumulation(new fl::Maximum);
	
	outputVariable1->addTerm(new fl::ZShape("large_move_right", -1.000,-0.100));
	outputVariable1->addTerm(new fl::Gaussian("small_move_right", -0.120,0.040));
	outputVariable1->addTerm(new fl::Gaussian("do_nothing", 0.000,0.020));
	outputVariable1->addTerm(new fl::Gaussian("small_move_left", 0.120,0.040));
	outputVariable1->addTerm(new fl::SShape("large_move_left", 0.100,1.000));
	engine->addOutputVariable(outputVariable1);
	
	fl::RuleBlock* ruleblock1 = new fl::RuleBlock;
	ruleblock1->setName("");
	ruleblock1->setConjunction(new fl::Minimum);
	ruleblock1->setDisjunction(new fl::Maximum);
	ruleblock1->setActivation(new fl::Minimum);
	
	ruleblock1->addRule(fl::Rule::parse("if large_pose_position_x is way_left then cmd_vel_linear_y is large_move_right", engine));
	ruleblock1->addRule(fl::Rule::parse("if large_pose_position_x is tracker_slightly_left then cmd_vel_linear_y is small_move_right", engine));
	ruleblock1->addRule(fl::Rule::parse("if large_pose_position_x is tracker_centered then cmd_vel_linear_y is do_nothing", engine));
	ruleblock1->addRule(fl::Rule::parse("if large_pose_position_x is tracker_slightly_right then cmd_vel_linear_y is small_move_left", engine));
	ruleblock1->addRule(fl::Rule::parse("if large_pose_position_x is way_right then cmd_vel_linear_y is large_move_left", engine));
	engine->addRuleBlock(ruleblock1);
	//END OF CONTROLLER
	
	
	ts.setEngine(engine, inputVariable1, outputVariable1);
	ROS_INFO("Set up fuzzy system for Large Tag");
}
	
	
void setEngineSmallTagLeft(TagInfoAce& ts)
{
	fl::Engine* engine = new fl::Engine;
	engine->setName("qtfuzzylite");
	engine->addHedge(new fl::Any);
	engine->addHedge(new fl::Extremely);
	engine->addHedge(new fl::Not);
	engine->addHedge(new fl::Seldom);
	engine->addHedge(new fl::Somewhat);
	engine->addHedge(new fl::Very);
	
	fl::InputVariable* inputVariable1 = new fl::InputVariable;
	inputVariable1->setName("orientation_w");
	inputVariable1->setRange(-1.000, 1.000);
	
	inputVariable1->addTerm(new fl::ZShape("way_left", -1.000,-0.180));
	inputVariable1->addTerm(new fl::Gaussian("a_little_left", -0.320,0.110));
	inputVariable1->addTerm(new fl::Bell("On_Target", 0.000,0.030,3.000));
	inputVariable1->addTerm(new fl::Gaussian("a_little_right", 0.320,0.110));
	inputVariable1->addTerm(new fl::SShape("way_right", 0.180,1.000));
	engine->addInputVariable(inputVariable1);
	
	fl::OutputVariable* outputVariable1 = new fl::OutputVariable;
	outputVariable1->setName("cmd_vel_z_rot");
	outputVariable1->setRange(-0.400, 0.400);
	outputVariable1->setDefaultValue(std::numeric_limits<fl::scalar>::quiet_NaN());
	outputVariable1->setDefuzzifier(new fl::Centroid(500));
	outputVariable1->fuzzyOutput()->setAccumulation(new fl::Maximum);
	
	outputVariable1->addTerm(new fl::ZShape("large_turn_left", -0.750,-0.100));
	outputVariable1->addTerm(new fl::Gaussian("small_turn_left", -0.130,0.030));
	outputVariable1->addTerm(new fl::Bell("hold_direction", 0.000,0.010,1.000));
	outputVariable1->addTerm(new fl::Gaussian("small_turn_right", 0.130,0.030));
	outputVariable1->addTerm(new fl::SShape("large_turn_right", 0.150,0.750));
	engine->addOutputVariable(outputVariable1);
	
	fl::RuleBlock* ruleblock1 = new fl::RuleBlock;
	ruleblock1->setName("");
	ruleblock1->setConjunction(new fl::Minimum);
	ruleblock1->setDisjunction(new fl::Maximum);
	ruleblock1->setActivation(new fl::Minimum);
	
	ruleblock1->addRule(fl::Rule::parse("if orientation_w is way_left then cmd_vel_z_rot is large_turn_right", engine));
	ruleblock1->addRule(fl::Rule::parse("if orientation_w is a_little_left then cmd_vel_z_rot is small_turn_right", engine));
	ruleblock1->addRule(fl::Rule::parse("if orientation_w is On_Target then cmd_vel_z_rot is hold_direction", engine));
	ruleblock1->addRule(fl::Rule::parse("if orientation_w is a_little_right then cmd_vel_z_rot is small_turn_left", engine));
	ruleblock1->addRule(fl::Rule::parse("if orientation_w is way_right then cmd_vel_z_rot is large_turn_left", engine));
	engine->addRuleBlock(ruleblock1);
	// END OF CONTROLLER
	
	
	
	ts.setEngine(engine, inputVariable1, outputVariable1);
	ROS_INFO("Set up fuzzy system for small left");
}
void setEngineSmallTagRight(TagInfoAce& ts)
{
	fl::Engine* engine = new fl::Engine;
	engine->setName("qtfuzzylite");
	engine->addHedge(new fl::Any);
	engine->addHedge(new fl::Extremely);
	engine->addHedge(new fl::Not);
	engine->addHedge(new fl::Seldom);
	engine->addHedge(new fl::Somewhat);
	engine->addHedge(new fl::Very);
	
	fl::InputVariable* inputVariable1 = new fl::InputVariable;
	inputVariable1->setName("orientation_w");
	inputVariable1->setRange(-1.000, 1.000);
	
	inputVariable1->addTerm(new fl::ZShape("way_left", -1.000,-0.180));
	inputVariable1->addTerm(new fl::Gaussian("a_little_left", -0.320,0.110));
	inputVariable1->addTerm(new fl::Bell("On_Target", 0.000,0.030,3.000));
	inputVariable1->addTerm(new fl::Gaussian("a_little_right", 0.320,0.110));
	inputVariable1->addTerm(new fl::SShape("way_right", 0.180,1.000));
	engine->addInputVariable(inputVariable1);
	
	fl::OutputVariable* outputVariable1 = new fl::OutputVariable;
	outputVariable1->setName("cmd_vel_z_rot");
	outputVariable1->setRange(-0.400, 0.400);
	outputVariable1->setDefaultValue(std::numeric_limits<fl::scalar>::quiet_NaN());
	outputVariable1->setDefuzzifier(new fl::Centroid(500));
	outputVariable1->fuzzyOutput()->setAccumulation(new fl::Maximum);
	
	outputVariable1->addTerm(new fl::ZShape("large_turn_left", -0.750,-0.100));
	outputVariable1->addTerm(new fl::Gaussian("small_turn_left", -0.130,0.030));
	outputVariable1->addTerm(new fl::Bell("hold_direction", 0.000,0.010,1.000));
	outputVariable1->addTerm(new fl::Gaussian("small_turn_right", 0.130,0.030));
	outputVariable1->addTerm(new fl::SShape("large_turn_right", 0.150,0.750));
	engine->addOutputVariable(outputVariable1);
	
	fl::RuleBlock* ruleblock1 = new fl::RuleBlock;
	ruleblock1->setName("");
	ruleblock1->setConjunction(new fl::Minimum);
	ruleblock1->setDisjunction(new fl::Maximum);
	ruleblock1->setActivation(new fl::Minimum);
	
	ruleblock1->addRule(fl::Rule::parse("if orientation_w is way_left then cmd_vel_z_rot large is_turn_right", engine));
	ruleblock1->addRule(fl::Rule::parse("if orientation_w is a_little_left then cmd_vel_z_rot is small_turn_right", engine));
	ruleblock1->addRule(fl::Rule::parse("if orientation_w is On_Target then cmd_vel_z_rot is hold_direction", engine));
	ruleblock1->addRule(fl::Rule::parse("if orientation_w is a_little_right then cmd_vel_z_rot is small_turn_left", engine));
	ruleblock1->addRule(fl::Rule::parse("if orientation_w is way_right then cmd_vel_z_rot is large_turn_left", engine));
	engine->addRuleBlock(ruleblock1);
	// END OF CONTROLLER
	
	
	
	ts.setEngine(engine, inputVariable1, outputVariable1);
	ROS_INFO("Set up fuzzy system for small right");
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

  
  tag_infos[0] = TagInfoAce(7);
  tag_infos[1] = TagInfoAce(170);
  tag_infos[2] = TagInfoAce(0);
  tag_infos[3] = TagInfoAce(tagid_Sonar_Elevation_Controller);  
  num_tags = 4;
  
  setEngineSmallTagLeft(tag_infos[0]);
  setEngineSmallTagRight(tag_infos[1]);
  //setEngineLargeTag_z_rot(tag_infos[2]);
  //setEngineLargeTag_X_controller(tag_infos[2]);
  setEngineSonarElevation(tag_infos[3]);
  setEngineLargeTag_main_control(tag_infos[2]);
 
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
		  //ROS_INFO("Running Fuzzy Controler now using the sonar reading");
		  float output_velocity_x = 0;
		  float output_velocity_y = 0;
		  float output_velocity_z = 0;
		  float output_velocity_z_rot = 0;
		  //output_velocity_z = FuzzyController3Rules(sonar_reading);
		  //ros::Time TimeNow = ros::Time::now();
		  //ROS_INFO("%f-%f) > %d", StartTime, LastTagTime, timeOut_betweenControlStates) ;
		  ros::Time TimeHere = ros::Time::now();
		  TimeHereSeconds = TimeHere.toSec();
		  
		  //ROS_INFO("TimeHereSeconds: %f \t LastTagTimeSeconds: %f \t > timeOut_betweenControlStates: %f",TimeHereSeconds,LastTagTimeSeconds,timeOut_betweenControlStates);
		  
		  
		  if ((!tag_detected) & ((TimeHereSeconds - LastTagTimeSeconds) > (timeOut_betweenControlStates)))
		  {
			  //ROS_INFO("runing FuzzyController_5Rules_Sonar_Elevation");
			  output_velocity_z = FuzzyController_5Rules_Sonar_Elevation(sonar_reading,700);
		  }
		  
		  if ((incoming_tag == 0) & ((TimeHereSeconds - LastTagTimeSeconds) < (timeOut_betweenControlStates/5)))
		  {
			  //ROS_INFO("runing Large_Tag_FuzzyController_Orientation_Rules");
			  float inputs[4] = {orientation_x,tag_pose_x,tag_pose_z,tag_pose_y};
				  
			  float desired_inputs[4] = {DEFAULT_CTRL_ORIENTATION,0.0,0.0,0.0};

			  ROS_INFO("Calling main orientation scheme");
			  float* outputs = Large_Tag_FuzzyController_Orientation_Main(inputs, desired_inputs);
			  ROS_INFO("Extracting outputs");
			  ROS_INFO("They are: %f %f %f %f", outputs[0], outputs[1], outputs[2], outputs[3]);
			  output_velocity_z_rot = outputs[0];  
			  output_velocity_y = outputs[1];
			  output_velocity_x = outputs[2];
			  output_velocity_z = outputs[3];
			  
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
		  cmdT.angular.z = output_velocity_z_rot;//-cmd.yaw;
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

