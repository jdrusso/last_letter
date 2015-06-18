#include "mathutils/mathutils.hpp"
#include <cmath>

void quat_normalize (geometry_msgs::Quaternion *q)
{
  double w = q->w, x = q->x, y = q->y, z = q->z;
  double norm = sqrt (w*w + x*x + y*y + z*z);
  q -> w /= norm;
  q -> x /= norm;
  q -> y /= norm;
  q -> z /= norm;
}

void quat_normal_wcomplete (geometry_msgs::Quaternion *q)
{
  double x = q->x, y = q->y, z = q->z;
  q -> w = sqrt (1.0 - (x*x + y*y + z*z));
}

void quat_equiv_wpos_get (geometry_msgs::Quaternion *q)
{
  if (q -> w < 0.0)
    {
      q->w = -(q->w);
      q->x = -(q->x); q->y = -(q->y); q->z = -(q->z);
    }
}

void quat_equiv_wneg_get (geometry_msgs::Quaternion *q)
{
  if (q -> w > 0.0)
    {
      q->w = -(q->w);
      q->x = -(q->x); q->y = -(q->y); q->z = -(q->z);
    }
}

void quat_inverse (const geometry_msgs::Quaternion q, geometry_msgs::Quaternion *q_inv)
{
  double w = q.w, x = q.x, y = q.y, z = q.z;
  double norm;
  norm = sqrt (w*w + x*x + y*y + z*z);
  q_inv -> w = +w / norm;
  q_inv -> x = -x / norm;
  q_inv -> y = -y / norm;
  q_inv -> z = -z / norm;
}

void quat_product (const geometry_msgs::Quaternion a, const geometry_msgs::Quaternion b, geometry_msgs::Quaternion *c)
{
  c -> w = a.w*b.w - (a.x*b.x + a.y*b.y + a.z*b.z);
  c -> x = (a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y);
  c -> y = (a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x);
  c -> z = (a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w);
}

geometry_msgs::Quaternion euler2quat (const geometry_msgs::Vector3 euler)
{
  double cpsi, spsi, ctheta, stheta, cphi, sphi;
  geometry_msgs::Quaternion q;
  cpsi = cos (0.5 * euler.z); spsi = sin (0.5 * euler.z);
  ctheta = cos (0.5 * euler.y); stheta = sin (0.5 * euler.y);
  cphi = cos (0.5 * euler.x); sphi = sin (0.5 * euler.x);
  q.w = cphi*ctheta*cpsi + sphi*stheta*spsi;
  q.x = sphi*ctheta*cpsi - cphi*stheta*spsi;
  q.y = cphi*stheta*cpsi + sphi*ctheta*spsi;
  q.z = cphi*ctheta*spsi - sphi*stheta*cpsi;

  return q;
}

void quat2rotmtx (const geometry_msgs::Quaternion q, double *rotmtx)
{
  double q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
  double q00 = q0*q0, q11 = q1*q1, q22 = q2*q2, q33 = q3*q3;
  double q01 = q0*q1, q02 = q0*q2, q03 = q0*q3;
  double q12 = q1*q2, q13 = q1*q3;
  double q23 = q2*q3;
  rotmtx [0] = q00 + q11 - q22 - q33;
  rotmtx [1] = 2.0*(q12 - q03);
  rotmtx [2] = 2.0*(q02 + q13);
  rotmtx [3] = 2.0*(q12 + q03);
  rotmtx [4] = q00 - q11 + q22 - q33;
  rotmtx [5] = 2.0*(q23 - q01);
  rotmtx [6] = 2.0*(q13 - q02);
  rotmtx [7] = 2.0*(q01 + q23);
  rotmtx [8] = q00 - q11 - q22 + q33;
}

geometry_msgs::Vector3 quat2euler (const geometry_msgs::Quaternion q)
// Based on Stevens & Lewis p. 42
{
//	double rotmtx[9];
	geometry_msgs::Vector3 euler;
//	quat2rotmtx(q, rotmtx);
//	euler.x = atan2(rotmtx[5],rotmtx[8]);
//	euler.y = -asin(rotmtx[2]);
//	euler.z = atan2(rotmtx[1],rotmtx[0]);
//
//	return euler;
	const double tol = 0.499;
	const double q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
	const double q00 = q0*q0, q11 = q1*q1, q22 = q2*q2, q33 = q3*q3;
	const double q01 = q0*q1, q23 = q2*q3;
	const double q12 = q1*q2, q03 = q0*q3;
	double test = q1*q3 - q0*q2;
	if (test < -tol)
	  {
	    euler.x = 0.0;
	    euler.y = 0.5 * M_PI;
	    euler.z = 2.0 * atan2 (q0, q3);
	    return euler;
	  }
	else if (test > +tol)
	  {
	    euler.x = 0.0;
	    euler.y = -0.5 * M_PI;
	    euler.z = 2.0 * atan2 (q0, q3);
	    return euler;
	  }
	else
	  {
	    euler.x = atan2 (2.0*(q01 + q23), q00 - q11 - q22 + q33);
	    euler.y = asin (-2.0*test);
	    euler.z = atan2 (2.0*(q12 + q03), q00 + q11 - q22 - q33);
	    return euler;
	  }
}

void euler2rotmtx (const geometry_msgs::Vector3 euler, double *rotmtx)
{
  double cpsi, spsi, ctheta, stheta, cphi, sphi;
  /**** Calculate trigonometric values. ****/
  cpsi = cos (euler.z); spsi = sin (euler.z);
  ctheta = cos (euler.y); stheta = sin (euler.y);
  cphi = cos (euler.x); sphi = sin (euler.x);
  /**** Calculate rotation matrix. ****/
  rotmtx [0] = cpsi * ctheta;
  rotmtx [1] = sphi * cpsi * stheta - cphi * spsi;
  rotmtx [2] = cphi * cpsi * stheta + sphi * spsi;
  rotmtx [3] = spsi * ctheta;
  rotmtx [4] = sphi * spsi * stheta + cphi * cpsi;
  rotmtx [5] = cphi * spsi * stheta - sphi * cpsi;
  rotmtx [6] = -stheta;
  rotmtx [7] = sphi * ctheta;
  rotmtx [8] = cphi * ctheta;
}

void quat_vector3_rotate (geometry_msgs::Quaternion q, geometry_msgs::Vector3 v, geometry_msgs::Vector3 *res)
{
  geometry_msgs::Quaternion vq, q_inv, qa, qb;
  vq.w = 0.0; vq.x = v.x; vq.y = v.y; vq.z = v.z;
  quat_inverse (q, & q_inv);
  quat_product (q, vq, & qa);
  quat_product (qa, q_inv, & qb);
  res -> x = qb.x;
  res -> y = qb.y;
  res -> z = qb.z;
}

void multi_mtx_mtx_3X3(double *a, double *b, double *res)
{
  int iii, jjj, kkk;
  for (iii = 0; iii < 3; ++ iii) {
    for (jjj = 0; jjj < 3; ++ jjj) {
      res [iii * 3 + jjj] = 0.0;
      for (kkk = 0; kkk < 3; ++ kkk)
	res [iii * 3 + jjj] += a [iii * 3 + kkk] * b [kkk * 3 + jjj];
    }
  }
}

void multi_mtx_mtx_3Xn(double *a, double *b, double *res,int n)
{
  int iii, jjj, kkk;
  for (iii = 0; iii < 3; ++ iii) {
    for (jjj = 0; jjj < n; ++ jjj) {
      res [iii * n + jjj] = 0.0;
      for (kkk = 0; kkk < 3; ++ kkk)
	res [iii * n + jjj] += a [iii * 3 + kkk] * b [kkk*n + jjj];
    }
  }
}


void multi_mtxT_mtx_3X3 (double *a, double *b, double *res)
{
  int iii, jjj, kkk;
  for (iii = 0; iii < 3; ++ iii) {
    for (jjj = 0; jjj < 3; ++ jjj) {
      res [iii * 3 + jjj] = 0.0;
      for (kkk = 0; kkk < 3; ++ kkk)
	res [iii * 3 + jjj] += a [kkk * 3 + iii] * b [kkk* 3 + jjj];
    }
  }
}

void multi_mtxT_mtx_3Xn(double *a, double *b, double *res,int n)
{
  int iii, jjj, kkk;
  for (iii = 0; iii < 3; ++ iii) {
    for (jjj = 0; jjj < n; ++ jjj) {
      res [iii * n + jjj] = 0.0;
      for (kkk = 0; kkk < 3; ++ kkk)
	res [iii * n + jjj] += a [kkk * 3 + iii] * b [kkk*n + jjj];
    }
  }
}


void vector3_cross(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b, geometry_msgs::Vector3 *c)
{
	c->x = a.y*b.z - a.z*b.y;
	c->y = a.z*b.x - a.x*b.z;
	c->z = a.x*b.y - a.y*b.x;
}

geometry_msgs::Vector3 operator+(const geometry_msgs::Vector3& a, geometry_msgs::Vector3& b)
{
    geometry_msgs::Vector3 c;
    c.x = a.x+b.x;
    c.y = a.y+b.y;
    c.z = a.z+b.z;

    return c;
}

//geometry_msgs::Vector3 operator+(geometry_msgs::Vector3& a, geometry_msgs::Vector3& b)
//{
//    geometry_msgs::Vector3 c;
//    c.x = a.x+b.x;
//    c.y = a.y+b.y;
//    c.z = a.z+b.z;

//    return c;
//}

geometry_msgs::Vector3 operator-(geometry_msgs::Vector3& vec)
{
    geometry_msgs::Vector3 c;
    c.x = -vec.x;
    c.y = -vec.y;
    c.z = -vec.z;

    return c;
}

geometry_msgs::Vector3 operator-(geometry_msgs::Vector3& a,geometry_msgs::Vector3& b)
{
    geometry_msgs::Vector3 c;
    c.x = a.x-b.x;
    c.y = a.y-b.y;
    c.z = a.z-b.z;

    return c;
}

geometry_msgs::Vector3 operator*(const double *mtx, geometry_msgs::Vector3& vec)
{
    geometry_msgs::Vector3 c;
    c.x = mtx[0]*vec.x+mtx[1]*vec.y+mtx[2]*vec.z;
    c.y = mtx[3]*vec.x+mtx[4]*vec.y+mtx[5]*vec.z;
    c.z = mtx[6]*vec.x+mtx[7]*vec.y+mtx[8]*vec.z;

    return c;
}

geometry_msgs::Vector3 operator*(const double a, geometry_msgs::Vector3& vec)
{
    geometry_msgs::Vector3 c;
    c.x = a*vec.x;
    c.y = a*vec.y;
    c.z = a*vec.z;

    return c;
}

geometry_msgs::Vector3 operator/(const double *mtx, geometry_msgs::Vector3& vec)
{
    geometry_msgs::Vector3 c;
    c.x = mtx[0]*vec.x+mtx[3]*vec.y+mtx[6]*vec.z;
    c.y = mtx[1]*vec.x+mtx[4]*vec.y+mtx[7]*vec.z;
    c.z = mtx[2]*vec.x+mtx[5]*vec.y+mtx[8]*vec.z;

    return c;
}

int inverse(double* A, double* Ainv, int N)
{
    int *IPIV = new int[N+1];
    int LWORK = N*N;
    double *WORK = new double[LWORK];
    int INFO;

    memcpy(Ainv,A,LWORK*sizeof(double));


    dgetrf_(&N,&N,Ainv,&N,IPIV,&INFO);
    dgetri_(&N,Ainv,&N,IPIV,WORK,&LWORK,&INFO);

    delete IPIV;
    delete WORK;

    return INFO;
}

////////////////////
// Class Definitions
////////////////////

////////////////////
// Define Polynomial

Polynomial::Polynomial ()
{
};

Polynomial::~Polynomial ()
{
};


//////////////////////
// Define Polynomial1D

// class constructor
Polynomial1D::Polynomial1D (int maxOrder, double * coeffArray) : Polynomial()
{
    int i;
    coeffNo = maxOrder;
    // Create and initialize polynomial coefficients container
    coeffs = (double*)malloc(sizeof(double) * (coeffNo+1));
    for (i=0; i<=coeffNo; i++) {
        coeffs[i] = coeffArray[i];
    }
}

// class destructor
Polynomial1D::~Polynomial1D ()
{
    free(coeffs);
}

// polynomial evaluation
double Polynomial1D::evaluate (double x) {
    int i;
    double sum=0;
    for (i=0; i<=coeffNo; i++) {
        sum += coeffs[i]*pow(x,i);
    }
    return sum;
}


//////////////////////
// Define Polynomial2D

// class constructor
Polynomial2D::Polynomial2D (int maxOrder1, int maxOrder2, double * coeffArray) : Polynomial()
{
    // Attention! maxOrder2 > maxOrder1. If not, swap the variables!
    int i;
    coeffNo1 = maxOrder1;
    coeffNo2 = maxOrder2;
    // Create and initialize polynomial coefficients container
    int arrayLen = (2*maxOrder2 + 2*maxOrder1*maxOrder2 + maxOrder1 - maxOrder1*maxOrder1 + 2)/2;
    coeffs = (double*)malloc(sizeof(double) * arrayLen);
    for (i=0; i<arrayLen; i++) {
        coeffs[i] = coeffArray[i];
    }
}

// class destructor
Polynomial2D::~Polynomial2D ()
{
    free(coeffs);
}

// polynomial evaluation
double Polynomial2D::evaluate (double x, double y) {
    int i, j, k=0;
    double sum=0;
    for (i=0; i<=coeffNo1; i++) {
        for (j=0; j<=coeffNo2; j++) {
            if (i+j<=coeffNo2) {
                sum += coeffs[k]*pow(x,i)*pow(y,j);
                k++;
            }
        }
    }
    // std::cout << "2DPoly: " << x << " " << y << " " << sum << std::endl; // Sanity check output
    return sum;
}


/////////////////
// Define Spline3
// Cubic spline, 4 parameters per variable interval

// class constructor
Spline3::Spline3(int breaksNoIn, double * breaksIn, double * coeffsIn) : Polynomial()
{
    int i;
    breaksNo = breaksNoIn;
    // Create and initialize breaks container
    breaks = (double*)malloc(sizeof(double) * (breaksNo+1));
    for (i=0; i<=breaksNo; i++) {
        breaks[i] = breaksIn[i];
    }
    // Create and initialize polynomial coefficients container
    coeffs = (double*)malloc(sizeof(double) * (breaksNo*4));
    for (i=0; i<(breaksNo*4); i++) {
        coeffs[i] = coeffsIn[i];
    }
}

// class destructor
Spline3::~Spline3()
{
    free(breaks);
    free(coeffs);
}

// polynomial evaluation
double Spline3::evaluate(double x)
{
    int i;
    for (i=0;i<breaksNo;i++) {
    if (x<=breaks[i+1])
      break;
    }
    if (i==breaksNo) i--;
    double delta = x-breaks[i];
    double value = coeffs[4*i]*pow(delta,3) + coeffs[4*i+1]*pow(delta,2) + coeffs[4*i+2]*delta + coeffs[4*i+3];
    return value;
}

///////////////////////
// Define discrTF class
///////////////////////

  // Constructor
  discrTF::discrTF (double * alphaIn, int alphaOrderIn, double * betaIn, int betaOrderIn)
  {
    int i;
    alphaOrder = alphaOrderIn;
    betaOrder = betaOrderIn;

    outputHist = (double*)malloc(sizeof(double) * alphaOrder);
    inputHist = (double*)malloc(sizeof(double) * alphaOrder);

    alpha = (double*)malloc(sizeof(double) * alphaOrder);
    beta = (double*)malloc(sizeof(double) * (betaOrder+1));
    for (i=0; i<alphaOrder; i++){
      alpha[i] = alphaIn[i];
    }
    for (i=0; i<=betaOrder; i++){
      beta[i] = betaIn[i];
    }

    init(0, 0);
  }

  // Destructor
  discrTF::~discrTF ()
  {
    free(alpha);
    free(beta);
    free(outputHist);
    free(inputHist);
  }

  // matrix reset
  void discrTF::init(double restInp, double restOut)
  {
    int i;
    for (i=0; i<alphaOrder; i++){
      outputHist[i]=restOut;
      inputHist[i]=restInp;
    }
  }

  // main step
  double discrTF::step(double input)
  {
    int i;
    double sum = 0;
    for (i=0; i<alphaOrder; i++){ // evaluate past output contribution
      sum -= alpha[i]*outputHist[i];
    }
    for (i=0; i<=betaOrder; i++){ // evaluate input contribution
      sum += beta[i]*inputHist[i];
    }
    for (i=0; i<(alphaOrder-1); i++){ // Slide history vectors back
      outputHist[i] = outputHist[i+1];
      inputHist[i] = inputHist[i+1];
    }
    outputHist[alphaOrder-1] = sum;
    inputHist[alphaOrder-1]  = input;

    return sum;
  }

/////////////////////////////////////////
// Check for NaN in various structures //
/////////////////////////////////////////

bool isnan(geometry_msgs::Vector3 vec)
{
  if (isnan(vec.x)) {
    return true;
  }
  if (isnan(vec.y)) {
    return true;
  }
  if (isnan(vec.z)) {
    return true;
  }
  return false;
}

bool isnan(geometry_msgs::Quaternion q)
{
  if (isnan(q.x)) {
    return true;
  }
  if (isnan(q.y)) {
    return true;
  }
  if (isnan(q.z)) {
    return true;
  }
  if (isnan(q.w)) {
    return true;
  }
  return false;
}