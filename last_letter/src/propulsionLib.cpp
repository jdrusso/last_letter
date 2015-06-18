//////////////////////////////
// Propulsion interfrace class
//////////////////////////////

// Constructor
Propulsion::Propulsion(ModelPlane * parent, int ID)
{
	parentObj = parent;
	id = ID;
	XmlRpc::XmlRpcValue list;
	int i;
	char paramMsg[50];
	sprintf(paramMsg, "motor%i/CGOffset", id);
	if(!ros::param::getCached(paramMsg, list)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	for (i = 0; i < list.size(); ++i) {
		ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	}
	CGOffset.x = list[0];
	CGOffset.y = list[1];
	CGOffset.z = list[2];

	sprintf(paramMsg, "motor%i/mountOrientation", id);
	if(!ros::param::getCached(paramMsg, list)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	for (i = 0; i < list.size(); ++i) {
		ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	}
	// !!! Order mixed because tf::Quaternion::setEuler seems to work with PRY, instead of YPR
	mountOrientation.y = list[0];
	mountOrientation.z = list[1];
	mountOrientation.x = list[2];

	theta = 0; // Initialize propeller angle

	sprintf(paramMsg, "motor%i/chanMotor", id);
	if(!ros::param::getCached(paramMsg, chanMotor)) {ROS_INFO("No MOTOR%i channel selected", id); chanMotor=-1;}
	sprintf(paramMsg, "motor%i/chanGimbal", id);
	if(!ros::param::getCached(paramMsg, chanGimbal)) {ROS_INFO("No GIMBAL%i channel selected", id); chanGimbal=-1;}
	sprintf(paramMsg, "motor%i/gimbalAngle_max", id);
	if(!ros::param::getCached(paramMsg, gimbalAngle_max)) {ROS_INFO("No GIMBALANGLE_MAX%i value selected", id); gimbalAngle_max=0.0;}

	inputMotor = 0.0;
	inputGimbal = 0.0;

	sprintf(paramMsg, "motor%i/rotationDir", id);
	if(!ros::param::getCached(paramMsg, rotationDir)) {ROS_INFO("No ROTATION_DIR%i value selected", id); rotationDir=1.0;}

}

// Destructor
Propulsion::~Propulsion()
{
	delete parentObj;
}

void Propulsion::getInput()
{
	char paramMsg[50];
	sprintf(paramMsg, "motor%i/gimbalAngle_max", id);
	ros::param::getCached(paramMsg, gimbalAngle_max);
	if (chanMotor>-1) {inputMotor = (double)(parentObj->input.value[chanMotor]-1000)/1000; }
	if (chanGimbal>-1) {inputGimbal = gimbalAngle_max * (double)(parentObj->input.value[chanGimbal]-1500)/500; }
}

// Engine physics step, container for the generic class
void Propulsion::stepEngine()
{
	rotateWind();
	updateRadPS();
	rotateProp();
	rotateForce();
	rotateTorque();
}


// Convert the relateive wind from body axes to propeller axes
void Propulsion::rotateWind()
{

	tf::Quaternion tempQuat;
	// Construct transformation from body axes to mount frame
	tempQuat.setEuler(mountOrientation.z, mountOrientation.y, mountOrientation.x);
	body_to_mount.setOrigin(tf::Vector3(CGOffset.x, CGOffset.y, CGOffset.z));
	body_to_mount.setRotation(tempQuat);

	// Construct transformation to apply gimbal movement. Gimbal rotation MUST be aligned with the resulting z-axis!
	// !!! Order mixed because tf::Quaternion::setEuler seems to work with PRY, instead of YPR
	tempQuat.setEuler(0.0, 0.0, inputGimbal);
	mount_to_gimbal.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	mount_to_gimbal.setRotation(tempQuat);

	// Transform the relative wind from body axes to propeller axes
	tf::Vector3 bodyWind(parentObj->airdata.u_r, parentObj->airdata.v_r, parentObj->airdata.w_r);
	tf::Vector3 tempVect;
	tempVect = mount_to_gimbal * (body_to_mount * bodyWind);
	// tempVect = mount_to_gimbal * (body_to_mount * bodyWind);

	relativeWind.x = tempVect.getX();
	relativeWind.y = tempVect.getY();
	relativeWind.z = tempVect.getZ();

	normalWind = relativeWind.x;

}

void Propulsion::rotateProp() // Update propeller angle
{
	theta += omega*parentObj->dt;
	if (theta > 2.0*M_PI) theta -= 2*M_PI;
	if (theta < 0.0) theta += 2*M_PI;

	tf::Quaternion tempQuat;
	// !!! Order mixed because tf::Quaternion::setEuler seems to work with PRY, instead of YPR
	tempQuat.setEuler(0.0, theta, 0.0);

	gimbal_to_prop.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
	gimbal_to_prop.setRotation(tempQuat);

	body_to_prop = (body_to_mount * mount_to_gimbal) * gimbal_to_prop;
	char prop_frame[50];
	sprintf(prop_frame, "propeller_%i", id);
	broadcaster.sendTransform(tf::StampedTransform(body_to_prop, ros::Time::now(), "base_link", prop_frame));

}

 // Convert the resulting force to the body axes
void Propulsion::rotateForce()
{

	tf::Vector3 tempVect(wrenchProp.force.x, wrenchProp.force.y, wrenchProp.force.z);
	tempVect = body_to_prop * tempVect; // I'm not sure why this works and not inverted

	wrenchProp.force.x = tempVect.getX();
	wrenchProp.force.y = tempVect.getY();
	wrenchProp.force.z = tempVect.getZ();

}

// Convert the resulting torque to the body axes
void Propulsion::rotateTorque()
{

	tf::Vector3 tempVect(wrenchProp.torque.x, wrenchProp.torque.y, wrenchProp.torque.z);
	tempVect = body_to_prop * tempVect;

	wrenchProp.torque.x = tempVect.getX();
	wrenchProp.torque.y = tempVect.getY();
	wrenchProp.torque.z = tempVect.getZ();

	// Convert the torque from the motor frame to the body frame
	double ratio = parentObj->kinematics.J[0] / (parentObj->kinematics.J[0] + parentObj->kinematics.mass * CGOffset.x*CGOffset.x);
	wrenchProp.torque.x =  ratio * wrenchProp.torque.x;
	ratio = parentObj->kinematics.J[4] / (parentObj->kinematics.J[4] + parentObj->kinematics.mass * CGOffset.y*CGOffset.y);
	wrenchProp.torque.y =  ratio * wrenchProp.torque.y;
	ratio = parentObj->kinematics.J[8] / (parentObj->kinematics.J[8] + parentObj->kinematics.mass * CGOffset.z*CGOffset.z);
	wrenchProp.torque.z =  ratio * wrenchProp.torque.z;

	// Add torque to to force misalignment with CG
	// r x F, where r is the distance from CoG to CoL
	// Will potentially add the following code in the future, to support shift of CoG mid-flight
	// XmlRpc::XmlRpcValue list;
	// int i;
	// if(!ros::param::getCached("motor/CGOffset", list)) {ROS_FATAL("Invalid parameters for -/motor/CGOffset- in param server!"); ros::shutdown();}
	// for (i = 0; i < list.size(); ++i) {
	// 	ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	// 	CGOffset[i]=list[i];
	// }
	wrenchProp.torque.x = wrenchProp.torque.x + CGOffset.y*wrenchProp.force.z - CGOffset.z*wrenchProp.force.y;
	wrenchProp.torque.y = wrenchProp.torque.y - CGOffset.x*wrenchProp.force.z + CGOffset.z*wrenchProp.force.x;
	wrenchProp.torque.z = wrenchProp.torque.z - CGOffset.y*wrenchProp.force.x + CGOffset.x*wrenchProp.force.y;

}

//////////////////
// No Engine Model
//////////////////

// Constructor
NoEngine::NoEngine(ModelPlane * parent, int id) : Propulsion(parent, id)
{
	wrenchProp.force.x = 0.0;
	wrenchProp.force.y = 0.0;
	wrenchProp.force.z = 0.0;
	wrenchProp.torque.x = 0.0;
	wrenchProp.torque.y = 0.0;
	wrenchProp.torque.z = 0.0;
	omega = 0.0;
}

// Destructor
NoEngine::~NoEngine()
{
}

void NoEngine::updateRadPS()
{
}

// Force calculation function
geometry_msgs::Vector3 NoEngine::getForce()
{
	return wrenchProp.force;
}

// Torque calculation function
geometry_msgs::Vector3 NoEngine::getTorque()
{
	return wrenchProp.torque;
}

////////////////////////////////////////
// Engine model found in R. Beard's book
////////////////////////////////////////

// Constructor
EngBeard::EngBeard(ModelPlane * parent, int ID):Propulsion(parent, ID)
{
	std::cout << "reading parameters for new Beard engine" << std::endl;
	omega = 0; // Initialize engine rotational speed
	// Read engine data from parameter server
	char paramMsg[50];
	sprintf(paramMsg, "motor%i/s_prop", id);
	if(!ros::param::getCached(paramMsg, s_prop)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "motor%i/c_prop", id);
	if(!ros::param::getCached(paramMsg, c_prop)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "motor%i/k_motor", id);
	if(!ros::param::getCached(paramMsg, k_motor)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "motor%i/k_t_p", id);
	if(!ros::param::getCached(paramMsg, k_t_p)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "motor%i/k_omega", id);
	if(!ros::param::getCached(paramMsg, k_omega)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
}

// Destructor
EngBeard::~EngBeard()
{
}

// Update motor rotational speed and other states for each timestep
void EngBeard::updateRadPS()
{
	rho = parentObj->environment.density;
	airspeed = normalWind; // Read vehicle airspeed
	// Propagate rotational speed with a first order response
	omega = rotationDir * 1 / (0.5 + parentObj->dt) * (0.5 * omega + parentObj->dt * inputMotor * k_omega);
	parentObj->states.rotorspeed[0]=std::fabs(omega); // Write engine speed to states message
}

// Calculate propulsion forces
geometry_msgs::Vector3 EngBeard::getForce()
{
	wrenchProp.force.x = 1.0/2.0*rho*s_prop*c_prop*(pow(omega * k_motor,2)-pow(airspeed,2));
	wrenchProp.force.y = 0;
	wrenchProp.force.z = 0;

	return wrenchProp.force;
}

// Calculate propulsion torques
geometry_msgs::Vector3 EngBeard::getTorque()
{
	wrenchProp.torque.x = -k_t_p*pow(omega,2);
	wrenchProp.torque.y = 0;
	wrenchProp.torque.z = 0;

	return wrenchProp.torque;
}

///////////////////////////////////////////////////
// Piston engine model (based upon Zanzoterra 305i)
///////////////////////////////////////////////////

// Constructor
PistonEng::PistonEng(ModelPlane * parent, int ID) : Propulsion(parent, ID)
{
	XmlRpc::XmlRpcValue list;
	int i, length;
	char s[100];
	char paramMsg[50];
	sprintf(paramMsg, "motor%i/propDiam", id);
	if(!ros::param::getCached(paramMsg, propDiam)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "motor%i/engInertia", id);
	if(!ros::param::getCached(paramMsg, engInertia)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	// Initialize RadPS limits
	sprintf(paramMsg, "motor%i/RadPSLimits", id);
	if(!ros::param::getCached(paramMsg, list)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	ROS_ASSERT(list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	omegaMin = list[0];
	omegaMax = list[1];

	Factory factory;
	// Create engine power polynomial
	sprintf(s,"%s%i/%s","motor", id, "engPowerPoly");
	engPowerPoly =  factory.buildPolynomial(s);
	// Create propeller efficiency polynomial
	sprintf(s,"%s%i/%s","motor", id , "nCoeffPoly");
	npPoly =  factory.buildPolynomial(s);
	// Create propeller power polynomial
	sprintf(s,"%s%i/%s","motor", id, "propPowerPoly");
	propPowerPoly =  factory.buildPolynomial(s);

	omega = omegaMin; // Initialize engine rotational speed

	wrenchProp.force.x = 0.0;
	wrenchProp.force.y = 0.0;
	wrenchProp.force.z = 0.0;
	wrenchProp.torque.x = 0.0;
	wrenchProp.torque.y = 0.0;
	wrenchProp.torque.z = 0.0;

}

// Destructor
PistonEng::~PistonEng()
{
	delete npPoly;
	// delete engPowerPoly; //@TODO examine if I need to uncomment this
	delete propPowerPoly;
}

// Update motor rotational speed and calculate thrust
void PistonEng::updateRadPS()
{
	rho = parentObj->environment.density;

	double powerHP = engPowerPoly->evaluate(omega/2.0/M_PI*60);
	double engPower = inputMotor * powerHP * 745.7; // Calculate current engine power

	double advRatio = normalWind/ (omega/2.0/M_PI) /propDiam; // Convert advance ratio to dimensionless units, not 1/rad
	advRatio = std::max(advRatio, 0.0); // Force advance ratio above zero, in lack of a better propeller model
	double propPower = propPowerPoly->evaluate(advRatio) * parentObj->environment.density * pow(omega/2.0/M_PI,3) * pow(propDiam,5);
	double npCoeff = npPoly->evaluate(advRatio);
	// wrenchProp.force.x = propPower*std::fabs(npCoeff)/(parentObj->states.velocity.linear.x+1.0e-10); // Added epsilon for numerical stability
	wrenchProp.force.x = propPower*std::fabs(npCoeff/(normalWind+1.0e-10)); // Added epsilon for numerical stability

	double fadeFactor = (exp(-normalWind*3/12));
	double staticThrust = 0.9*fadeFactor*pow(M_PI/2.0*propDiam*propDiam*rho*engPower*engPower,1.0/3); //static thrust fades at 5% at 12m/s
	// ROS_INFO("engPower: %g, staticThrust: %g, regForce: %g", engPower, staticThrust, wrenchProp.force.x);
	wrenchProp.force.x = wrenchProp.force.x + staticThrust;

	// Constrain propeller force to +-2 times the aircraft weight
	wrenchProp.force.x = std::max(std::min(wrenchProp.force.x, 2.0*parentObj->kinematics.mass*9.81), -2.0*parentObj->kinematics.mass*9.81);
	wrenchProp.torque.x = propPower / omega;
	if (inputMotor < 0.01) {
		wrenchProp.force.x = 0;
		wrenchProp.torque.x = 0;
	} // To avoid aircraft rolling and turning on the ground while throttle is off
	wrenchProp.torque.y = 0.0;
	wrenchProp.torque.z = 0.0;
	// double deltaP = parentObj->kinematics.forceInput.x * parentObj->states.velocity.linear.x / npCoeff;
	double deltaT = (engPower - propPower)/omega;
	double omegaDot = 1/engInertia*deltaT;
	omega += rotationDir * omegaDot * parentObj->dt;
	omega = rotationDir * std::max(std::min(std::fabs(omega), omegaMax), omegaMin); // Constrain omega to working range

	parentObj->states.rotorspeed[0]=std::fabs(omega); // Write engine speed to states message

	// Printouts
	// std::cout << deltat << " ";
	// std::cout << powerHP << " ";
	// std::cout << engPower << " ";
	// std::cout << propPower << " ";
	// std:: cout << advRatio << " ";
	// std::cout << npCoeff << " ";
	// std::cout << wrenchProp.force.x << " ";
	// std::cout << wrenchProp.torque.x << " ";
	// std::cout << parentObj->kinematics.forceInput.x << " ";
	// std::cout << omegaDot << " ";
	// std::cout << omega;
	// std::cout << std::endl;

}

geometry_msgs::Vector3 PistonEng::getForce()
{
	if (isnan(wrenchProp.force.x) || isnan(wrenchProp.force.y) || isnan(wrenchProp.force.z)) {
		ROS_FATAL("State NaN in wrenchProp.force");
		ros::shutdown();
	}
	return wrenchProp.force;
}

geometry_msgs::Vector3 PistonEng::getTorque()
{
	if (isnan(wrenchProp.torque.x) || isnan(wrenchProp.torque.y) || isnan(wrenchProp.torque.z)) {
		ROS_FATAL("State NaN in wrenchProp.torque");
		ros::shutdown();
	}

	return wrenchProp.torque;
}


////////////////////////
// Electric engine model
////////////////////////

// Constructor
ElectricEng::ElectricEng(ModelPlane * parent, int ID) : Propulsion(parent, ID)
{
	XmlRpc::XmlRpcValue list;
	int i, length;
	char s[100];
	char paramMsg[50];
	sprintf(paramMsg, "prop%i/propDiam", id);
	if(!ros::param::getCached(paramMsg, propDiam)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "motor%i/engInertia", id);
	if(!ros::param::getCached(paramMsg, engInertia)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "motor%i/Kv", id);
	if(!ros::param::getCached(paramMsg, Kv)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	sprintf(paramMsg, "motor%i/Rm", id);
	if(!ros::param::getCached(paramMsg, Rm)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	if(!ros::param::getCached("battery/Rs", Rs)) {ROS_FATAL("Invalid parameters for -Cells- in param server!"); ros::shutdown();}
	if(!ros::param::getCached("battery/Cells", Cells)) {ROS_FATAL("Invalid parameters for -Cells- in param server!"); ros::shutdown();}
	sprintf(paramMsg, "motor%i/I0", id);
	if(!ros::param::getCached(paramMsg, I0)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	// Initialize RadPS limits
	sprintf(paramMsg, "motor%i/RadPSLimits", id);
	if(!ros::param::getCached(paramMsg, list)) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown();}
	ROS_ASSERT(list[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
	omegaMin = list[0];
	omegaMax = list[1];

	Factory factory;
	// Create propeller efficiency polynomial
	sprintf(s,"%s%i/%s","prop", id , "nCoeffPoly");
	npPoly =  factory.buildPolynomial(s);
	// Create propeller power polynomial
	sprintf(s,"%s%i/%s","prop", id, "powerPoly");
	propPowerPoly =  factory.buildPolynomial(s);

	omega = omegaMin; // Initialize engine rotational speed

	wrenchProp.force.x = 0.0;
	wrenchProp.force.y = 0.0;
	wrenchProp.force.z = 0.0;
	wrenchProp.torque.x = 0.0;
	wrenchProp.torque.y = 0.0;
	wrenchProp.torque.z = 0.0;

	sprintf(paramMsg, "propulsion%i", id);
	ros::NodeHandle n;
	pub = n.advertise<last_letter_msgs::ElectricEng>(paramMsg, 1000); //propulsion data publisher
}

// Destructor
ElectricEng::~ElectricEng()
{
	delete npPoly;
	delete propPowerPoly;
}

// Update motor rotational speed and calculate thrust
void ElectricEng::updateRadPS()
{
	rho = parentObj->environment.density;

	double Ei = std::fabs(omega)/2/M_PI/Kv;
	// double Ei = rotationDir * omega/2/M_PI/Kv;
	double Im = (Cells*4.0*inputMotor - Ei)/(Rs*inputMotor + Rm);
	// Im = std::max(Im,0.0); // Current cannot return to the ESC
	// Im = std::max(Im,-1.0); // Allow limited current back to the ESC
	double engPower = Ei*(Im - I0);

	double advRatio = normalWind / (std::fabs(omega)/2.0/M_PI) /propDiam; // Convert advance ratio to dimensionless units, not 1/rad
	// advRatio = std::max(advRatio, 0.0); // Force advance ratio above zero, in lack of a better propeller model
	double propPower = propPowerPoly->evaluate(advRatio) * parentObj->environment.density * pow(std::fabs(omega)/2.0/M_PI,3) * pow(propDiam,5);
	double npCoeff = npPoly->evaluate(advRatio);

	wrenchProp.force.x = propPower*std::fabs(npCoeff/(normalWind+1.0e-10)); // Added epsilon for numerical stability
	wrenchProp.force.y = 0.0;
	wrenchProp.force.z = 0.0;

	double fadeFactor = (exp(-normalWind*3/12));
	double staticThrust = 0.9*fadeFactor*pow(M_PI/2.0*propDiam*propDiam*rho*engPower*engPower,1.0/3); //static thrust fades at 5% at 12m/s
	wrenchProp.force.x = wrenchProp.force.x + staticThrust;

	// Constrain propeller force to [0,+5] times the aircraft weight
	wrenchProp.force.x = std::max(std::min(wrenchProp.force.x, 5.0*parentObj->kinematics.mass*9.81), 0.0*parentObj->kinematics.mass*9.81);
	wrenchProp.torque.x = propPower / omega;

	if (inputMotor < 0.01) {
		wrenchProp.force.x = 0;
		wrenchProp.torque.x = 0;
	} // To avoid aircraft rolling and turning on the ground while throttle is off
	wrenchProp.torque.y = 0.0;
	wrenchProp.torque.z = 0.0;
	// double deltaP = parentObj->kinematics.forceInput.x * parentObj->states.velocity.linear.x / npCoeff;
	double deltaT = (engPower - propPower)/std::fabs(omega);
	double omegaDot = 1/engInertia*deltaT;
	omega += rotationDir * omegaDot * parentObj->dt;
	omega = rotationDir * std::max(std::min(std::fabs(omega), omegaMax), omegaMin); // Constrain omega to working range

	parentObj->states.rotorspeed[0]=std::fabs(omega); // Write engine speed to states message

	message.header.stamp = ros::Time::now();
	message.powerEng = propPower;
	message.omega = omega;
	message.throttle = inputMotor*100.0;
	message.powerProp = propPower;
	message.thrust = wrenchProp.force.x;
	message.torque = wrenchProp.torque.x;
	message.advRatio = advRatio;
	message.airspeed = normalWind;
	message.ncoeff = npCoeff;
	pub.publish(message);

	// Printouts
	// std::cout << deltat << " ";
	// std::cout << powerHP << " ";
	// std::cout << engPower << " ";
	// std::cout << propPower << " ";
	// std:: cout << advRatio << " ";
	// std::cout << npCoeff << " ";
	// std::cout << wrenchProp.force.x << " ";
	// std::cout << wrenchProp.force.z << " ";
	// std::cout << wrenchProp.torque.x << " ";
	// std::cout << parentObj->kinematics.forceInput.x << " ";
	// std::cout << omegaDot << " ";
	// std::cout << omega;
	// std::cout << tempVect.getX() << " ";
	// std::cout << tempVect.getZ() << " ";
	// std::cout << parentObj->input[3] << " ";
	// std::cout << std::endl;

}

geometry_msgs::Vector3 ElectricEng::getForce()
{
	if (isnan(wrenchProp.force.x) || isnan(wrenchProp.force.y) || isnan(wrenchProp.force.z)) {
		ROS_FATAL("State NaN in wrenchProp.force");
		ros::shutdown();
	}
	return wrenchProp.force;
}

geometry_msgs::Vector3 ElectricEng::getTorque()
{
	if (isnan(wrenchProp.torque.x) || isnan(wrenchProp.torque.y) || isnan(wrenchProp.torque.z)) {
		ROS_FATAL("State NaN in wrenchProp.torque");
		ros::shutdown();
	}

	return wrenchProp.torque;
}