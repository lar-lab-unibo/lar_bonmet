
#ifndef _ANTHROP_KINEMATIC_H
#define _ANTHROP_KINEMATIC_H

#include "AxesGroupParams.h"
#include "AxesGroup.h"
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <Eigen/Core>
#include <Eigen/LU>
#include <math.h>

#include "../Utils/Log.h"

using namespace KDL;
using namespace Eigen;
using namespace std;

class AnthropKinematic{
public:
	// Kinematic model: function definition --> Header

	//-------------------------------------------------------------
	// - Direct (Forward) Kinematic Problem
	//-------------------------------------------------------------

	//-------------------------------------------------------------
	// Position FORWARD kinematic function: computes the Work Space
	// position and orientation once given the pose in the Joint
	// Space ref. system in terms of motors angular position:
	// --- INPUT:
	//		q	->	JnTArray 1x6 : joints position vector.
	// --- OUTPUT:
	//		T	->	Matrix 4x4   : rototranslation matrix, with
	//                             orientation matrix R and pose
	//                             vector P ref. to End Effector.
	//-------------------------------------------------------------
	void FPK(JntArray &q, Frame &T);

	//-------------------------------------------------------------
	// Velocity FORWARD kinematic function: computes the speed of the
	// tool in the Work Space once known the speed of the joints in
	// the Joint Space:
	// --- INPUT:
	//		q_dot	->	JntArray 1x6 : joints speed vector.
	//		J       ->  MatrixXd 6x6 : Jacobian matrix of the robot.
	// --- OUTPUT:
	//		s_dot	->  Vector 1x6   : velocityvector, composed by a
	//                                 rotation speed vector and a speed
	//								   array of the End Effector.
	//-------------------------------------------------------------
	void FVK(JntArray &q_dot, Twist &s_dot);

	//-------------------------------------------------------------
	// Acceleration FORWARD kinematic that computes the accelerations
	// in the Work Space once given an acceleration vector in the
	// Joint Space:
	// --- INPUT:
	//		q_dot		->	JntArray 1x6 : joints speed vector.
	//      q_dot_dot	->  JntArray 1x6 : joints acceleration vector.
	//		J           ->  MatrixXd 6x6 : Jacobian matrix of the robot.
	//		J_dot       ->  MatrixXd 6x6 : Time derivative of the Jacobian
	//                                      matrix of the robot.
	// --- OUTPUT:
	//		s_acc	    ->  Vector 1x6   : acceleration vector, composed by a
	//                                 rotation acceler. vector and an
	//								   accel. vector of the End Effector.
	//-------------------------------------------------------------
	void FAK(JntArray &q_dot, JntArray &q_dot_dot, Twist &s_dot_dot);

	//-------------------------------------------------------------
	// - Inverse Kinematic Problem
	//-------------------------------------------------------------

	//-------------------------------------------------------------
	// Function that compute the inversion of the kinematic model
	// mapping a given Work Space position into the Joint Space,
	// giving a vector of (n_axis x 1) dimension, after the
	// discrimination among all the possible solution that can be
	// obtained from the anlytical solution of the problem.
	// An error code if Work Space and/or Joint Space limitsare
	// exceeded is reported in output as return value.
	// --- INPUT:
	//     q_prec     ->  JntArray 6x1 : precedent (t-1) instant of
	//                                   computation time Joint Space
	//                                   angles configuration.
	//		    T	  ->  Frame    4x4 : rototranslation matrix of the
	//                                   End Effector.
	// --- OUTPUT:
	//          q     ->  JntArray 6x1 : angles position correspondant
	//                                   to the position T of the E.E.
	// --- CODE ERROR (integer value):
	//          0 : Normal condition, no errors encountered.
	//          1 : Software error,
	//          2 : Joint Space error, joints value exceed limits.
	//          3 : Work Space error, point/pose outside admissible space
	//-------------------------------------------------------------
	int IPK(JntArray &q, JntArray &q_old, Frame &T);

	//-------------------------------------------------------------
	// Function that computes the speed in the Joint Space once a
	// Work Space vector containing speed and angular speed of the
	// End Effector is given. Needs the inverse Jacobian matrix.
	// --- INPUT:
	//		s_dot	->	Vector   1x6 : Work Space speed of the E.E.
	// --- OUTPUT:
	//      q_dot	->  JntArray 1x6 : Joint Space velocity.
	//-------------------------------------------------------------
	void IVK(JntArray &q_dot, Twist &s_dot);

	//-------------------------------------------------------------
	// Function that computes the acceleration in the Joint Space
	// once given the corrispective acceleration in Work Space and
	// the speed in the Joint Space.
	// Needs the inverse of the Jacobian and its time derivative.
	// --- INPUT:
	//		 q_dot	->	JntArray 1x6 : Joint Space speed vector.
	//   s_dot_dot	->	Vector   1x6 : Work Space acceleration.
	// --- OUTPUT:
	//   q_dot_dot  ->  JntArray 1x6 : Joint Space acceleration.
	//-------------------------------------------------------------
	void IAK(JntArray & q_dot, JntArray &q_dot_dot, Twist &s_dot_dot);

	//-------------------------------------------------------------
	// - Utility functions: methods that are needed in the computation
	//   of the Kinematic model:
	//-------------------------------------------------------------

	//-------------------------------------------------------------
	// Function that computes the Jacobi's matrix of robot: it takes as input
	// the position in both Joint Space and Work Space and create the matrix
	// representing a mapping object between the two spaces.
	// --- INPUT:
	//		q	->	JntArray 1x6 : Joint Space array of motor's angles.
	//      T   ->  Frame    4x4 : Work Space pose needed in some kinematics.
	// --- OUTPUT (stored in a class's variable):
	//      J   ->  MatrixXd 6x6 : Jacobi's matrix computed.
	//  J_inv   ->  MatrixXd 6x6 : Jacobian inverse matrix.
	//-------------------------------------------------------------
	void Jacobian(JntArray &q, Frame &T);

	//-------------------------------------------------------------
	// Time derivative of the Jacobian matrix, needed in the computation
	// of the acceleration, both passing from Joint Space to Work Space
	// and viceversa.
	// --- INPUT:
	//		q_dot	->	JntArray 1x6 : speed joint array in Joint Space.
	//          J   ->  MatrixXd 6x6 : Jacobian matrix.
	// --- OUTPUT (stored in a class's variable):
	//      J_dot   ->  MatrixXd 6x6 : time derivative of the Jacobian.
	//-------------------------------------------------------------
	void JacobianDerivative(JntArray &q_dot);

	//-------------------------------------------------------------
	// Function that initialize the geometric parameters needed to
	// computes the Kinematic model.
	// --- INPUT:
	//		group	->	Pointer to the group structure : reference
	//						to access all the AxesGroup parameters
	//                      and data.
	//-------------------------------------------------------------
	void SetKinematicParameters(AxesGroup *group);

	//-------------------------------------------------------------
	// Function that, once given an angle between [-pi, pi] from the
	// atan2 function, mapped it into a [-4pi, 4pi] reference syst.
	// accordignly to an offset vector defined previously.
	// The computed output is the closest to the previous angle
	// values passed to the function:
	// --- INPUT:
	//		qAtan	->	double : angle computed as output of the
	//                           atan2 function, [-pi, pi] range.
	//      qPrec   ->  double : previous angle value [-kpi, kpi].
	// --- OUTPUT (returned value):
	//          q   ->  double : actual angle with extended range.
	//-------------------------------------------------------------
	double GetMultiTurnAngle(double qAtan, double qPrec);

	//-------------------------------------------------------------
	// Function that check if a computed joint angle is outside the
	// boundaries of the customer's Joint Space specified in the XML.
	// Tolerance with epsilon value computed previously is supported.
	// --- INPUT:
	//		       q    ->    double : angle to be checked.
	//    indexAngle    ->    int    : integer index value that tells
	//                                 which angle is considered (1,2,..)
	// --- OUTPUT (returned value):
	//     codeError    ->    int : 0. normal condition, angle INSIDE
	//                                 the joint space (no error).
	//                              2. error cond. angle outside limits.
	//-------------------------------------------------------------
	int GetJointLimitChecked(double q, int indexAngle);

	//-------------------------------------------------------------
	// Function that computes the angular solution of a second order
	// parametrized equation (parametrization in terms of tan(a/2)
	// instead using cos and sin terms. Computes angles and give back
	// also a check error flag if computation is not possible.
	// --- INPUT:
	//    K1, K2, K3    ->   double  : computational constants of the
	//                                 second order equation.
	// --- OUTPUT (returned value):
	//     codeError    ->    int : 0. normal condition, angle INSIDE
	//                                 the work space (no error).
	//                              1. error cond. angle outside limits.
	//            qA    ->   double  : first angular solution.
	//            qB    ->   double  : second angular solution.
	//-------------------------------------------------------------
	int GetAngleSolutions(double K1, double K2, double K3, double qA, double qB);

	//-------------------------------------------------------------
	// Parameters definition for the Kinematic model:
	//-------------------------------------------------------------

	/*
		DEBUG VARIABLE: USED ONLY DURING TEST
	*/
	bool debugAnthrop;

	// number of joints:
	int n_axis;

	// geometrical lenght of the links:
	double L1;
	double L2;
	double L2a;
	double L3;
	double L4;
	double L5;
	double L6;
	double L7;

	// geometrical distance of the joints:
	double L20;		// distance from joint 2 to 3;
	double L30;		// distance from joint 3 to 4;

	// geometrical angular diplacement due to joint distance:
	double q20;		// rotation from joint 2 to 3;
	double q30;		// rotation from joint 3 to 4;

	// Denavit-Hartenberg parameters vectors:
	VectorXd a;	     // X axis translation;
	VectorXd d;      // Z axis translation;
	VectorXd alpha;  // X axis rotation;

	// Vectors containing offset and sign changing to passes from
	// Bonmet's robotic convention to standard Denavit-Hartenberg
	// one: the Kinematic model is developed following the standard
	// one but the angles given as output for the controller are in
	// the Bonmet convention.
	JntArray dh_offset;   // offset to the angles;
	JntArray kin_offset;  // kinematic offset (q20,q30);

	// Joint Space LIMITS for the motors position: given by the
	// user, they define boundary limits for the robot in Jnt Space
	JntArray q_max;
	JntArray q_min;

	// espilon value needed in the Joint Space position diagnosis
	// working as a tolerance quantity on the defined limit:
	double epsRangeJoint;

	// value of percentage that indicates how much the Joint Space
	// range must be streched:
	double percJoint;

	// computational tolerance for zero-equalities:
	double epsZero;

	// definition of a vector containing the extention of the atan2 result
	// on one circle or more (2pi, 4pi, ...) clockwise and counterclockwise:
	VectorXd offsetRound;

	// size of the multi-turn vector:
	int n_turn;

	// Jacobi's matrices (direct, inverse, time derivative):
	MatrixXd J;
	MatrixXd J_inv;
	MatrixXd J_dot;

	// choosen solution from IPK
	int solutionIPK;
	//-------------------------------------------------------------

private:

};

#endif

AnthropKinematic.h
Visualizzazione di AnthropKinematic.h.
