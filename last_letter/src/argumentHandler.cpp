#include <ros/ros.h>

///////////////
//Main function
///////////////
int main(int argc, char **argv)
{

	ros::init(argc, argv, "argumentHandler");
	ros::NodeHandle n;



	ROS_INFO("argumentHandler up");

	std::string s;
	int i;
	double f;
	bool b;
	XmlRpc::XmlRpcValue list;

	// // Read initial NED coordinates
	// if(!ros::param::getCached("init/position", list)) {ROS_FATAL("Invalid parameters for -init/position- in param server!"); ros::shutdown();}
	// for (i = 0; i < list.size(); ++i) {
	// 	ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	// 	temp[i]=list[i];
	// }
	// states.pose.position.x=temp[0];
	// states.pose.position.y=temp[1];
	// states.pose.position.z=temp[2];

	if(!ros::param::get("argumentHandler/simRate",s)) {
		ROS_INFO("simRate is not (string)nan");
		if (!ros::param::get("argumentHandler/simRate",f)) {
			ROS_ERROR("simRate is not a double either");
		}
		else {
			ros::param::set("/world/simRate", f);
			ROS_INFO("Overwrote /world/simRate");
		}
	}
	else if (s.compare("nan") == 0) {
		ROS_INFO("simRate value is not set by an argument");
	}
	else {
		ROS_ERROR("simRate is string but not nan");
	}


	if(!ros::param::get("argumentHandler/home",s)) {
		ROS_INFO("home is not (string)nan");
		if (!ros::param::get("argumentHandler/home",list)) {
			ROS_ERROR("home is not a list either!!!");
		}
		else {
			ros::param::set("init/coordinates", list);
			ROS_INFO("Overwrote init/coordinates");
		}
	}
	else if (s.compare("nan") == 0) {
		ROS_INFO("home value is not set by an argument");
	}
	else {
		ROS_ERROR("home is string but not nan");
	}



	if(!ros::param::get("argumentHandler/deltaT",s)) {
		ROS_INFO("deltaT is not (string)nan");
		if (!ros::param::get("argumentHandler/deltaT",f)) {
			ROS_ERROR("deltaT is not a double either!!!");
		}
		else {
			ros::param::set("/world/deltaT", f);
			ROS_INFO("Overwrote /world/deltaT");
		}
	}
	else if (s.compare("nan") == 0) {
		ROS_INFO("deltaT value is not set by an argument");
	}
	else {
		ROS_ERROR("deltaT is string but not nan");
	}


	if(ros::param::get("argumentHandler/ArduPlane",b)) {
		if (b==true) {
			ros::param::set("/world/timeControls", 1);
			ROS_INFO("Overwrote /world/timeControls to 1");
		}
		else {
			ROS_INFO("ArduPlane argument was set to false");
		}
	}
	else {
		ROS_ERROR("ArduPlane parameter is not available");
	}


	if(!ros::param::get("argumentHandler/instance",s)) {
		ROS_INFO("instance is not (string)nan");
		if (!ros::param::get("argumentHandler/instance",i)) {
			ROS_INFO("instance is not an integer either");
		}
		else {
			ros::param::set("instance", i);
			ROS_INFO("Overwrote instance to %d", i);
		}
	}
	else if (s.compare("nan") == 0) {
		ROS_INFO("instance value is not set by an argument. Defaulting to 0.");
		ros::param::set("instance", 0);
	}
	else {
		ROS_ERROR("instance is string but not nan");
	}

	ros::param::set("nodeStatus/argumentHandler", 1);

	ROS_INFO("Arguments loaded. argumentHandler node closing.");

	return 0;
}
