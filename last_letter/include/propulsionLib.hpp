///////////////////////////////////////////
// Propulsion class related declarations //
///////////////////////////////////////////

#include <tf/transform_broadcaster.h>
#include <last_letter_msgs/ElectricEng.h>

////////////////////////////////////////////
// Propulsion interface class declaration //
////////////////////////////////////////////
class Propulsion
{
	public:
	///////////
	//Variables
	ModelPlane * parentObj; // pointer to parent ModelPlane class
	int id; // The current motor ID
	geometry_msgs::Vector3 CGOffset; // vector from CG to engine coordinates
	geometry_msgs::Vector3 mountOrientation;
	geometry_msgs::Vector3 relativeWind; // relative wind vector in the propeller frame
	double inputMotor, inputGimbal; // control input (0-1)
	double gimbalAngle_max;
	int chanMotor, chanGimbal;
	double omega; // motor angular speed in rad/s
	double rotationDir; // motor direction of rotation
	double theta; // propeller angle in rads
	double normalWind; // scalar wind normal to propeller disc
	geometry_msgs::Wrench wrenchProp;

	tf::TransformBroadcaster broadcaster; // Transformations broadcaster object
	tf::Transform body_to_mount, mount_to_gimbal, gimbal_to_prop, body_to_prop; // Transformations in the propeller assembly

	///////////
	//Functions
	Propulsion(ModelPlane *, int);
	~Propulsion();

	void getInput(); // store control input
	void stepEngine(); // engine physics step, container for the generic class
	void rotateWind(); // convert the wind to the propeller axes
	virtual void updateRadPS() =0; // Step the angular speed
	void rotateProp(); // Update the propeller angle
	void rotateForce(); // convert the resulting force to the body axes
	void rotateTorque(); // convert the resulting torque to the body axes
	virtual geometry_msgs::Vector3 getForce() =0; // Calculate Forces
	virtual geometry_msgs::Vector3 getTorque() =0; //Calculate Torques
};

////////////////////////////
// No engine, dummy class //
////////////////////////////
class NoEngine: public Propulsion
{
public:
	NoEngine(ModelPlane *, int);
	~NoEngine();

	void updateRadPS();
	geometry_msgs::Vector3 getForce();
	geometry_msgs::Vector3 getTorque();
};

////////////////////////////////////////////////////
// Electric engine model found in R. Beard's book //
////////////////////////////////////////////////////
class EngBeard: public Propulsion
{
	public:
	///////////
	//Variables
	double s_prop, c_prop, k_motor, k_t_p, k_omega;
	double airspeed, rho;

	///////////
	//Functions
	EngBeard(ModelPlane *, int);
	~EngBeard();

	void updateRadPS(); //Step the angular speed
	geometry_msgs::Vector3 getForce(); //Calculate Forces
	geometry_msgs::Vector3 getTorque(); //Calculate Torques
};

///////////////////
// Piston engine //
///////////////////
class PistonEng : public Propulsion
{
public:
	////////////
	// Variables
	double omegaMin, omegaMax;
	double propDiam, engInertia, rho;

	//////////
	// Members
	Polynomial * engPowerPoly;
	Polynomial * npPoly;
	Polynomial * propPowerPoly;
	// Polynomial1D * npPoly;
	// Polynomial1D * propPowerPoly;
	// Polynomial2D * engPowerPoly;

	////////////
	// Functions
	PistonEng(ModelPlane *, int);
	~PistonEng();

	void updateRadPS();
	geometry_msgs::Vector3 getForce();
	geometry_msgs::Vector3 getTorque();
};

///////////////////////////
// Electric hobby engine //
///////////////////////////
class ElectricEng : public Propulsion
{
public:
	////////////////
	// Variables //
	////////////////
	double omegaMin, omegaMax;
	double propDiam, engInertia, rho;
	double Kv, Rm, Rs, I0, Cells;
	last_letter_msgs::ElectricEng message;
	ros::Publisher pub;

	//////////////
	// Members //
	//////////////
	Polynomial * engPowerPoly, * npPoly, * propPowerPoly;

	////////////////
	// Functions //
	////////////////
	ElectricEng(ModelPlane *, int);
	~ElectricEng();

	void updateRadPS();
	geometry_msgs::Vector3 getForce();
	geometry_msgs::Vector3 getTorque();
};