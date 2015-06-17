#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <last_letter_msgs/SimPWM.h>
#include <last_letter_msgs/SimStates.h>

ros::Publisher pub;
ros::Subscriber sub;

ros::Time simTime(0.0);
ros::WallTime realTimePrev = ros::WallTime::now(), realTimeNow;
ros::Duration dt;
ros::WallDuration wallCounter;
rosgraph_msgs::Clock simClock;
unsigned long frameCounter = 0;

////////////////////////////////////
// Control input trigger callback //
////////////////////////////////////
void controlsCallback(const last_letter_msgs::SimPWM pwm)
{
	simTime = simTime + dt;
	simClock.clock = simTime;
	pub.publish(simClock);
	frameCounter++;
}

///////////////////////////////////////
// Simulation state trigger callback //
///////////////////////////////////////
void stateCallback(const last_letter_msgs::SimStates state)
{
	simTime = simTime + dt;
	simClock.clock = simTime;
	pub.publish(simClock);
	frameCounter++;
}

///////////////
//Main function
///////////////
int main(int argc, char **argv)
{
	ROS_INFO("Initializing clock node");
	ros::init(argc, argv, "simClockNode");
	ros::NodeHandle n;

	bool status;

	//Check if clock_active exists
	if ( !ros::param::get("/clock_active", status) || !status){

		ROS_INFO("STARTING CLOCK");
		//If no clock is active, start a clock topic
		pub = n.advertise<rosgraph_msgs::Clock>("/clock",1000);
		ros::param::set("/clock_active", true);
	} else {

		ROS_INFO("CLOCK ALREADY EXISTS");
		ros::param::set("nodeStatus/clock", 1);
		return 0;
	}


	sub = n.subscribe("ctrlPWM", 100, controlsCallback);

	int statusModel=0;
	ros::param::set("nodeStatus/clock", 1);
	while (statusModel!=1) {
		ros::param::get("nodeStatus/model",statusModel);
	}

	// ros::WallDuration(5).sleep(); //wait for other nodes to get raised


	double simRate;
	ros::param::get("/world/simRate",simRate); //frame rate in Hz
	double deltaT;
	ros::param::get("/world/deltaT",deltaT); //simulation time step in seconds
	int timeControls;
	ros::param::get("/world/timeControls",timeControls); //frame rate in Hz

	ros::WallRate spinner(simRate);
	dt = ros::Duration(deltaT);

	ROS_INFO("simClockNode up");

	if (timeControls==0) // Default real-time simulation
	{
		ROS_INFO("Using default real-time simulation clock");
		sub = n.subscribe("states", 100, stateCallback);
		ros::WallDuration(3).sleep(); //wait for other nodes to get raised
		simClock.clock = simTime;
		pub.publish(simClock);
		while (ros::ok())
		{
			realTimeNow = ros::WallTime::now();
			wallCounter = realTimeNow - realTimePrev;
			if (wallCounter.toSec() > 5) {
				ROS_INFO("Simulation rate: %lu Hz", frameCounter/5);
				realTimePrev = realTimeNow;
				frameCounter=0;
			}
			ros::spinOnce();
			spinner.sleep();

		}
	}
	else if (timeControls==1) // Simulation waits for controls message
	{
		ROS_INFO("Using controls-triggered simulation clock");
		sub = n.subscribe("ctrlPWM", 100, controlsCallback);
		simClock.clock = simTime;
		pub.publish(simClock);
		while (ros::ok())
		{
			realTimeNow = ros::WallTime::now();
			wallCounter = realTimeNow - realTimePrev;
			if (wallCounter.toSec() > 5) {
				ROS_INFO("Simulation rate: %lu Hz", frameCounter/5);
				realTimePrev = realTimeNow;
				frameCounter=0;
			}
			ros::spinOnce();

		}
	}
	else if (timeControls==2) // Simulation runs as fast as possible
	{
		ROS_INFO("Using free-spinning simulation clock");
		sub = n.subscribe("states", 100, stateCallback);
		ros::WallDuration(3).sleep(); //wait for other nodes to get raised
		simClock.clock = simTime;
		pub.publish(simClock);
		while (ros::ok())
		{
			realTimeNow = ros::WallTime::now();
			wallCounter = realTimeNow - realTimePrev;
			if (wallCounter.toSec() > 5) {
				ROS_INFO("Simulation rate: %lu Hz", frameCounter/5);
				realTimePrev = realTimeNow;
				frameCounter=0;
			}
			ros::spinOnce();
		}
	}
	else {
		ROS_ERROR("Invalid timeControls value!");
		ros::shutdown();
	}

	return 0;

}
