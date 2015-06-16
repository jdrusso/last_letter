#include "model.hpp"

ModelPlane * uav;

//////////////////////////////
// Simulation step callback //
//////////////////////////////
void stepCallback(const rosgraph_msgs::Clock time)
{
	uav->step();
	if (isnan(uav->states.velocity.linear.x))
	{
		ROS_FATAL("State NAN detected on main!");
		uav->init();
	}
}

///////////////
//Main function
///////////////
int main(int argc, char **argv)
{

	ros::init(argc, argv, "simNode");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("clock",1000, stepCallback);
	uav = new ModelPlane(n); //Create a ModelPlane passing the nodehandle for subscriptions & publicatons

	int statusClock=0, statusEnv=0, statusArg=0;
	while (statusClock!=1) {
		ros::param::get("nodeStatus/clock", statusClock);
	}
	while (statusEnv!=1) {
		ros::param::get("nodeStatus/environment", statusEnv);
	}
	while (statusArg!=1) {
		ros::param::get("nodeStatus/argumentHandler", statusArg);
	}

	ros::param::set("nodeStatus/model", 1);

	// ros::WallDuration(3).sleep(); //wait for other nodes to get raised

	ROS_INFO("simNode up");

	uav->init();

	while (ros::ok())
	{
		ros::spin();
	}

	delete uav;

	return 0;
}
