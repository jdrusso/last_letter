//////////////////////////
// Define Gravity class
//////////////////////////

// Class constructor
Gravity::Gravity(ModelPlane * parent)
{
	parentObj = parent;
	wrenchGrav.torque.x = 0;
	wrenchGrav.torque.y = 0;
	wrenchGrav.torque.z = 0;
}

// Class destructor
Gravity::~Gravity()
{
	delete parentObj;
}

// Force calculation function
geometry_msgs::Vector3 Gravity::getForce()
{
	double Reb[9];
	geometry_msgs::Quaternion tempQuat;
	tempQuat = parentObj->states.pose.orientation;
	if (isnan(tempQuat)) {ROS_FATAL("gravityLib.cpp: NaN member in orientation quaternion"); ros::shutdown();}
	quat2rotmtx(tempQuat,Reb);
	if (isnan_mtx(Reb, 9)) {ROS_FATAL("gravityLib.cpp: NaN member in rotation matrix"); ros::shutdown();}
	g = parentObj->environment.gravity;

	geometry_msgs::Vector3 gravVect;
	gravVect.x = 0;
	gravVect.y = 0;
	gravVect.z = parentObj->kinematics.mass*g;

	wrenchGrav.force = Reb/gravVect;
	if (isnan(wrenchGrav.force)) {ROS_FATAL("gravityLib.cpp: NaN member in force vector"); ros::shutdown();}
	// Printouts
	// std::cout << wrenchGrav.force.x << " ";
	// std::cout << g << " ";
	// std::cout << gravVect.z << " ";
	// std::cout << std::endl;

	return wrenchGrav.force;
}

// Torque calculation function
geometry_msgs::Vector3 Gravity::getTorque()
{
	// Gravity does not generate torque around the CG
	return wrenchGrav.torque;
}