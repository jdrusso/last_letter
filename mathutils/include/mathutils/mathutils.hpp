#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <cstdio>
#include <cmath>

#define R_earth 6378137.0
#define f_earth 1.0/298.257223563
#define e_earth sqrt(2.0*f_earth - f_earth*f_earth)

extern "C" {
    // LU decomoposition of a general matrix
    void dgetrf_(int* M, int *N, double* A, int* lda, int* IPIV, int* INFO);

    // generate inverse of a matrix given its LU decomposition
    void dgetri_(int* N, double* A, int* lda, int* IPIV, double* WORK, int* lwork, int* INFO);
}

////////////
// Functions

void quat_normalize (geometry_msgs::Quaternion *q);
void quat_normal_wcomplete (geometry_msgs::Quaternion *q);
void quat_equiv_wpos_get (geometry_msgs::Quaternion *q);
void quat_equiv_wneg_get (geometry_msgs::Quaternion *q);
void quat_inverse (const geometry_msgs::Quaternion q, geometry_msgs::Quaternion *q_inv);
void quat_product (const geometry_msgs::Quaternion a, const geometry_msgs::Quaternion b, geometry_msgs::Quaternion *c);

geometry_msgs::Quaternion euler2quat (const geometry_msgs::Vector3 euler);
void quat2rotmtx (const geometry_msgs::Quaternion q, double *rotmtx);
geometry_msgs::Vector3 quat2euler (const geometry_msgs::Quaternion q);
void euler2rotmtx (const geometry_msgs::Vector3 euler, double *rotmtx);

void quat_vector3_rotate (geometry_msgs::Quaternion q, geometry_msgs::Vector3 v, geometry_msgs::Vector3 *res);

void multi_mtx_mtx_3X3 (double *a, double *b, double *res);
void multi_mtx_mtx_3Xn(double *a, double *b, double *res,int n);
void multi_mtxT_mtx_3X3 (double *a, double *b, double *res);
void multi_mtxT_mtx_3Xn(double *a, double *b, double *res,int n);

void vector3_cross(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b, geometry_msgs::Vector3 *c);

geometry_msgs::Vector3 operator+(const geometry_msgs::Vector3& a, geometry_msgs::Vector3& b);
geometry_msgs::Vector3 operator-(geometry_msgs::Vector3& vec);
geometry_msgs::Vector3 operator-(geometry_msgs::Vector3& a,geometry_msgs::Vector3& b);
geometry_msgs::Vector3 operator*(const double *mtx, geometry_msgs::Vector3& vec);
geometry_msgs::Vector3 operator*(const double a, geometry_msgs::Vector3& vec);
geometry_msgs::Vector3 operator/(const double *mtx, geometry_msgs::Vector3& vec);

int inverse(double* A, double* Ainv, int N);


//////////
// Classes

class Polynomial
{
public:
	Polynomial();
	~Polynomial();
	virtual double evaluate() {return 0;}
	virtual double evaluate(double x) {return 0;}
	virtual double evaluate(double x, double y) {return 0;}
};

class Polynomial1D : public Polynomial
{
public:
	double coeffNo;
	double * coeffs;

	Polynomial1D(int maxOrder, double * coeffArray);
	~Polynomial1D();
	double evaluate(double x);
};

class Polynomial2D : public Polynomial
{
public:
	double coeffNo1, coeffNo2;
	double * coeffs;

	Polynomial2D(int maxOrder1, int maxOrder2, double * coeffArray);
	~Polynomial2D();
	double evaluate(double x, double y);
	// Important notes: maxOrder of variable y must be greater or equal to maxOrder of variable x
	// Pass the coefficient array with the following ordering (eg for maxOrder1=1, maxOrder2=3):
	// [00 01 02 03 10 11 12]
};

class Spline3 : public Polynomial
{
public:
	int breaksNo;
	double * breaks, * coeffs;

	Spline3(int breaksNoIn, double * breaksIn, double * coeffsIn);
	~Spline3();
	double evaluate(double x);
};


// Discrete transfer function implementation
// for strictly proper TFs
class discrTF
{
public:
	///////////
	//Variables
	double * alpha, * beta;
	double * outputHist, * inputHist;
	int alphaOrder, betaOrder;

	///////////
	//Functions

	// Constructor
	discrTF (double * alphaIn, int alphaOrderIn, double * betaIn, int betaOrderIn);

	// Destructor
	~discrTF ();

	// matrix reset
	void init(double restInp, double restOut);

	// main step
	double step(double input);
};

geometry_msgs::Vector3 getAirData (geometry_msgs::Vector3 speeds);

void WGS84_NM(double lat,double *NE, double *ME);


/////////////////////////////////////////
// Check for NaN in various structures //
/////////////////////////////////////////
bool isnan(geometry_msgs::Vector3 vec);
bool isnan(geometry_msgs::Quaternion q);
bool isnan_mtx(double * R, int n);

bool myisfinite(const geometry_msgs::Vector3 vec);
bool myisfinite(const geometry_msgs::Quaternion q);
bool myisfinite_mtx(double * R, int n);