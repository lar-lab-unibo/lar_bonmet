#include "AnthropKinematic.h"

#define DEG_TO_RAD(x)	((x)*(M_PI/180.0))
#define pi M_PI

#define SingNull 2.0e10
#define SingNullCheck 1.0e10

using namespace KDL;
using namespace Eigen;

//-------------------------------------------------------------
// Utility functions: methods that are needed in the computation
// of the Kinematic model:
//-------------------------------------------------------------

//-------------------------------------------------------------
// Function that initialize the geometric parameters needed to
// computes the Kinematic model.
// --- INPUT:
//		group	->	Pointer to the group structure : reference
//						to access all the AxesGroup parameters
//                      and data.
//-------------------------------------------------------------
void AnthropKinematic::SetKinematicParameters(AxesGroup *group){

	/*
	DEBUG VARIABLE: USED ONLY DURING TEST
	*/
	debugAnthrop = true;

	// definition of the number of axis:
	n_axis = 6;

	//definition of the size of the vectors initializied in .h file:
	// - Denavit Hartenberg vectors:
	a     = VectorXd(n_axis);
	alpha = VectorXd(n_axis);
	d     = VectorXd(n_axis);

	// DH->Bonmet convention vectors:
	dh_offset  = JntArray(n_axis);
	kin_offset = JntArray(n_axis);

	// Joint Space limits quantities:
	q_max = JntArray(n_axis);
	q_min = JntArray(n_axis);

	// acquisition of the geometric parameters from the structure
	// of the XML file's parameters:
	L1  = group->Params.Kin.Anthrop.L1;
	L2  = group->Params.Kin.Anthrop.L2;
	L2a = group->Params.Kin.Anthrop.L2b;
	L3  = group->Params.Kin.Anthrop.L3;
	L4  = group->Params.Kin.Anthrop.L4;
	L5  = group->Params.Kin.Anthrop.L5;
	L6  = group->Params.Kin.Anthrop.L6;
	L7  = group->Params.Kin.Anthrop.L7;

	// computation of geometrical distance between links:
	L20 = sqrt(L2*L2 + L2a*L2a);
	L30 = sqrt(L3*L3 + L4*L4);

	// computation of angular offset due to geometrical distances:
	q20 = atan2(L2a, L2);
	q30 = atan2(L3, L4);

	// definition of the Denavit-Hartenberg convention vectors:
	a     << L6, L20, L3, 0.0, 0.0, 0.0;
	d     << L1, L7, 0.0, L4, 0.0, L5;
	alpha << pi / 2, 0.0, -pi / 2, pi / 2, -pi / 2, 0.0;

	// Denavit-Hartenberg offset and sign management definition:
	dh_offset(0) = 0.0;
	dh_offset(1) = pi / 2;
	dh_offset(2) = -pi;
	dh_offset(3) = pi;
	dh_offset(4) = 0.0;
	dh_offset(5) = 0.0;

	kin_offset(0) = 0.0;
	kin_offset(1) = - q20;
	kin_offset(2) = q30;
	kin_offset(3) = 0.0;
	kin_offset(4) = 0.0;
	kin_offset(5) = 0.0;

	for (int i = 0; i < n_axis; i++){
		q_max(i) = DEG_TO_RAD(group->AxList[i]->Params.PosUpperLimit);
		q_min(i) = DEG_TO_RAD(group->AxList[i]->Params.PosLowerLimit);
	}

	// definition of the Jacobian matrix, its inverse and derivative with
	// respect to time:
	J     = MatrixXd(n_axis, n_axis);
	J_inv = MatrixXd(n_axis, n_axis);
	J_dot = MatrixXd(n_axis, n_axis);

	// computation of the tolerance (epsilon) value for the Joint
	// Space boundaries diagnosis:
	percJoint = 0.5;	// percentage value [%]
	// report in percentage the reference stretching value:
	double percJoint_100 = percJoint / 100.0;
	// definition of joint range vector:
	VectorXd rangeJointSpace = VectorXd(n_axis);

	// computation of the range per each joint:
	for (int i = 0; i < n_axis; i++){
		rangeJointSpace(i) = (abs(q_max(i)) + abs(q_min(i)))*percJoint_100;
	}

	// research of the smallest joint range and use it as threshold:
	double smallRangeJoint = rangeJointSpace(0);
	int indexRangeJoint = 0;

	// iterative reasearch among all the values:
	for (int i = 1; i < n_axis; i++){
		if (rangeJointSpace(i) < smallRangeJoint){
			indexRangeJoint = i;
			smallRangeJoint = rangeJointSpace(i);
		}
	}

	// definition of the epsilon value for the threshold joints check:
	epsRangeJoint = rangeJointSpace(indexRangeJoint);

	// multi circle for the solution of the atan2 extended to -360/360 and
	// more on the base of the offset wanted:
	// - initialize the number of entries in the multi-turn vector:
	n_turn = 5;
	// - initialization of the offset circle vector: support the computation of
	//   two complete circle, until 720 deg both clockwise and counterclockwise:
	offsetRound = VectorXd(n_turn);
	// - assignment of values to the offset circle vector:
	offsetRound(0) = -4 * pi;
	offsetRound(1) = -2 * pi;
	offsetRound(2) = 0.0;
	offsetRound(3) =  2 * pi;
	offsetRound(4) =  4 * pi;

	// tolerance value for zero-equalities check:
	epsZero = 1.0e-6;
	//-------------------------------------------------------------
}

//-------------------------------------------------------------
// - Direct (Forward) Kinematic model computation:
//-------------------------------------------------------------

//-------------------------------------------------------------
// Position FORWARD kinematic function: computes the Work Space
// position and orientation once given the pose in the Joint
// Space ref. system in terms of motors angular position:
// --- INPUT:
//		q	->	JnTArray 1x6 : joints position vector.
// --- OUTPUT:
//		T	->	Matrix 4x4   : rototranslation matrix, with
//          svn rever0----------------------------------------
void AnthropKinematic::FPK(JntArray &q, Frame &T){

	// the angles given as input are converted from Bonmet
	// convention to Denavit-Hartenberg one:
	for (int i = 0; i < n_axis; i++){
		q(i) += kin_offset(i) + dh_offset(i);
	}
	//-------------------------------------------------------------
	// - LINK 1, JOINT 0:
	Frame A_01;
	A_01 = Frame::DH(a(0), alpha(0), d(0), q(0));
	//-------------------------------------------------------------
	// - LINK 2, JOINT 1:
	Frame A_12;
	A_12 = Frame::DH(a(1), alpha(1), d(1), q(1));
	//-------------------------------------------------------------
	// - LINK 3, JOINT 2:
	Frame A_23;
	A_23 = Frame::DH(a(2), alpha(2), d(2), q(2));
	//-------------------------------------------------------------
	// - LINK 4, JOINT 3:
	Frame A_34;
	A_34 = Frame::DH(a(3), alpha(3), d(3), q(3));
	//-------------------------------------------------------------
	// - LINK 5, JOINT 4:
	Frame A_45;
	A_45 = Frame::DH(a(4), alpha(4), d(4), q(4));
	//-------------------------------------------------------------
	// - LINK 6, JOINT 5:
	Frame A_56;
	A_56 = Frame::DH(a(5), alpha(5), d(5), q(5));
	//-------------------------------------------------------------
	// computation of the rototranslation matrix from 0 to 6 ref. system:
	T = A_01*A_12*A_23*A_34*A_45*A_56;
	//-------------------------------------------------------------
}

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
void AnthropKinematic::FVK(JntArray &q_dot, Twist &s_dot){

	// two computation-needed vectors (internal):
	VectorXd qDot = VectorXd(n_axis);
	VectorXd sDot = VectorXd(n_axis);
	// assignment of the values of input q_dot to Eigen Class vector:
	for (int i = 0; i < n_axis; i++){
		qDot(i) = q_dot(i);
	}
	// computation of the Work Space speed with Jacobian matrix:
	sDot = J*qDot;

	// assignement in output of the values computed as 1x6 speed vector:
	for (int i = 0; i < 3; i++){
		s_dot.vel(i) = sDot(i);
		s_dot.rot(i) = sDot(i + 3);
	}
	//-------------------------------------------------------------
}

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
void AnthropKinematic::FAK(JntArray &q_dot, JntArray &q_dot_dot, Twist &s_dot_dot){

	// three computation-needed vectors (internal):
	VectorXd qDot    = VectorXd(n_axis);
	VectorXd qDotDot = VectorXd(n_axis);
	VectorXd sDotDot = VectorXd(n_axis);
	// assignment of the values of input q_dot to Eigen Class vector:
	for (int i = 0; i < n_axis; i++){
		qDot(i)    = q_dot(i);
		qDotDot(i) = q_dot_dot(i);
	}
	// computation of the Work Space speed using Jacobian and its derivative:
	sDotDot = J*qDotDot + J_dot*qDot;

	// assignement in output of the values computed as 1x6 accel. vector:
	for (int i = 0; i < 3; i++){
		s_dot_dot.vel(i) = sDotDot(i);
		s_dot_dot.rot(i) = sDotDot(i + 3);
	}
	//-------------------------------------------------------------
}

//-------------------------------------------------------------
// - Inverse Kinematic model computation:
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
int AnthropKinematic::IPK(JntArray &q, JntArray &q_old, Frame &T){

	//-------------------------------------------------------------
	// definition of all the quantities and variables needed in the
	// inverse kinematic computation problem:

	// number of solutions for arm and wrist:
	int n_solutions = 8;

	// matrices:-----------------------------------------------------
	Matrix3d R_06, R_03, R_46;
	MatrixXd qOut = MatrixXd(n_solutions, n_axis);
	MatrixXd qArm = MatrixXd(n_solutions, n_axis);
	MatrixXd qWrist = MatrixXd(n_solutions, (n_axis/2));
	MatrixXd qWristSecSol = MatrixXd(n_solutions, (n_axis/2));
	// errors diagnosis:
	MatrixXd errorArm = MatrixXd(n_solutions, n_axis);
	MatrixXd errorJoint = MatrixXd(n_solutions, n_axis);

	// vectors:------------------------------------------------------
	Vector3d P, Pc, selectionColumn;
	Vector3d N, S, A;
	int SolArm[n_solutions];

	// distance from prev. configuration:
	JntArray q_prec = JntArray(n_axis);
	VectorXd tempVector = VectorXd(n_axis);
	double distance[n_axis];

	// errors code: Joint Space
	int errorJoint_1[2];	// Joint space errors: JOINT 1
	int errorJoint_2[4];	// Joint space errors: JOINT 2
	int errorJoint_3[8];	// Joint space errors: JOINT 3
	int errorJoint_4[2];    // Joint space errors: JOINT 4
	int errorJoint_5[2];    // Joint space errors: JOINT 5
	int errorJoint_6[2];    // Joint space errors: JOINT 6

	// errors code: Work Space
	int errorWork[7];       // Work space errors
	int errorWorkJoint_3[4];// Joint 3 Work Space errors

	// errors flag:
	VectorXd errorArmFlag = VectorXd(n_solutions);
	VectorXd errorJointFlag = VectorXd(n_solutions);

	// angles multi-solutions temporary storage:---------------------
	double q1_1, q1_2;
	double q2_1, q2_2, q2_3, q2_4;
	double q3_1, q3_2, q3_3, q3_4, q3_5, q3_6, q3_7, q3_8;
	double q4_1, q4_2;
	double q5_1, q5_2;
	double q6_1, q6_2;
	// temporary variable for (q4+q6):
	double q46_1, q46_2;

	// variables needed in the development of the calculation:-------
	// constant for all the joint:
	double k1, k2, k3;

	// singularity test variable for the arm:
	int checkArm;

	// search of the closest solution to prec. one:
	double smallDistance;
	int indexDistance;

	// spherical wrist singularity flags:
	bool singularity_1, singularity_2;

	// adapting the previous joint config. to DH offset:
	for (int i = 0; i < n_axis; i++){
		q_prec(i) = q_old(i) + kin_offset(i) + dh_offset(i);
	}
	//-------------------------------------------------------------
	// Inverse kinematic solution computed in closed form: decoupling
	// inverse POSITION problem for the arm and inverse ORIENTATION
	// problem for the spherical wrist the solution can be computed
	// as two decoupled problems.

	//-------------------------------------------------------------
	// INVERSE POSITION PROBLEM SOLUTION
	//-------------------------------------------------------------
	// computation of the position of the center of the spherical
	// wrist with respect to the base reference frame:

	// rotation matrix extraction from the rototranslation one:
	for (int i = 0; i < 3; i++){
		for (int k = 0; k < 3; k++){
			R_06(i, k) = T(i, k);
		}
	}

	// extraction of the End Effector position from the matrix T:
	P << T(0, 3), T(1, 3), T(2, 3);

	// definition of a vector that select only the last column of matrix R:
	selectionColumn << 0.0, 0.0, 1.0;

	// computation of the position of the center of the spherical wrist:
	Pc = P - d(5)*R_06*selectionColumn;

	// the three angles of the arm (q1, q2, q3) passes throught the solution
	// of the equations describing in plane XY and ZX the position of the
	// center of the wrist wrt base ref. system, using parametrization
	// trigonometric fomula:
	//		-. sin = (2tan(a/2))/(1+tan(a/2)^2)
	//      -. cos = (1-tan(a/2)^2/(1+tan(a/2)^2)

	//-------------------------------------------------------------
	// ANGLE 1 - Sol. 1,2
	//-------------------------------------------------------------
	// computation of the constants needed in the angle 1 solution:
	// --- CONSTANT K1:
	k1 = -Pc(0);
	// --- CONSTANT K2:
	k2 = Pc(1);
	// --- CONSTANT K3:
	k3 = d(1);
	// computation of the solutions for q1:
	errorWork[0] = GetAngleSolutions(k1, k2, k3, q1_1, q1_2);
	// rounding to zero smaller values:
	if (abs(q1_1) < epsZero){ q1_1 = 0.0; }
	if (abs(q1_2) < epsZero){ q1_2 = 0.0; }

	// extension of the range of the angles computed:
	q1_1 = GetMultiTurnAngle(q1_1, q_prec(0));
	q1_2 = GetMultiTurnAngle(q1_2, q_prec(0));
	// joint limits check:
	errorJoint_1[0] = GetJointLimitChecked(q1_1, 0);
	errorJoint_1[1] = GetJointLimitChecked(q1_2, 0);

	//-------------------------------------------------------------
	// ANGLE 2 - Sol. 1,2 - (computation with Angle 1 - Sol. 1)
	//-------------------------------------------------------------
	if (q1_1 >= SingNullCheck){
		// singularity in joint 1 - sol.1 -> do not compute q2 sol.1,2
		q2_1 = SingNull;
		q2_2 = SingNull;
	}
	else{
		// joint 1 not in singularity: computes the joint 2 - sol.1,2
		// assignment of the constant value needed in the second order equation
		// solution to obtain the values of q2_1 and q2_2:
		// --- CONSTANT K1
		k1 = 2.0*d(0)*a(1) - 2.0*Pc(2)*a(1);
		// --- CONSTANT k2
		k2 = 2.0*a(0)*a(1) - 2.0*a(1)*(Pc(0)*cos(q1_1) + Pc(1)*sin(q1_1));
		// --- CONSTANT K3
		k3 = Pc(0)*Pc(0) + Pc(1)*Pc(1) + Pc(2)*Pc(2) + d(0)*d(0) + a(1)*a(1) - L30*L30 + a(0)*a(0) + d(1)*d(1) - 2.0*Pc(2)*d(0) + 2.0*d(1)*(Pc(1)*cos(q1_1) - Pc(0)*sin(q1_1)) - 2.0*a(0)*(Pc(0)*cos(q1_1) + Pc(1)*sin(q1_1));
		//-------------------------------------------------------------
		// computes solutions for angle 2 - sol 1,2:
		errorWork[1] = GetAngleSolutions(k1, k2, k3, q2_1, q2_2);
		// rounding to zero smaller values:
		if (abs(q2_1) < epsZero){ q2_1 = 0.0; }
		if (abs(q2_2) < epsZero){ q2_2 = 0.0; }
	}

	// extend the solution range:
	q2_1 = GetMultiTurnAngle(q2_1, q_prec(1));
	q2_2 = GetMultiTurnAngle(q2_2, q_prec(1));
	// joint limits check:
	errorJoint_2[0] = GetJointLimitChecked(q2_1, 1);
	errorJoint_2[1] = GetJointLimitChecked(q2_2, 1);

	//-------------------------------------------------------------
	// ANGLE 2 - Sol. 3,4 - (computation with Angle 1 - Sol. 2)
	//-------------------------------------------------------------
	if (q1_2 >= SingNullCheck){
		// singularity in joint 1 - sol.1 -> do not compute q2 sol.1,2
		q2_3 = SingNull;
		q2_4 = SingNull;
	}
	else{
		// joint 1 not in singularity: computes the joint 2 - sol.3,4
		// assignment of the constant value needed in the second order equation
		// solution to obtain the values of q2_3 and q2_4:
		// --- CONSTANT K1
		k1 = 2.0*d(0)*a(1) - 2.0*Pc(2)*a(1);
		// --- CONSTANT k2
		k2 = 2.0*a(0)*a(1) - 2.0*a(1)*(Pc(0)*cos(q1_2) + Pc(1)*sin(q1_2));
		// --- CONSTANT K3
		k3 = Pc(0)*Pc(0) + Pc(1)*Pc(1) + Pc(2)*Pc(2) + d(0)*d(0) + a(1)*a(1) - L30*L30 + a(0)*a(0) + d(1)*d(1) - 2.0*Pc(2)*d(0) + 2.0*d(1)*(Pc(1)*cos(q1_2) - Pc(0)*sin(q1_2)) - 2.0*a(0)*(Pc(0)*cos(q1_2) + Pc(1)*sin(q1_2));
		//-------------------------------------------------------------
		// computes solutions for angle 2 - sol 3,4:
		errorWork[2] = GetAngleSolutions(k1, k2, k3, q2_3, q2_4);
		// rounding to zero smaller values:
		if (abs(q2_3) < epsZero){ q2_3 = 0.0; }
		if (abs(q2_4) < epsZero){ q2_4 = 0.0; }
	}

	// extend the solution range:
	q2_3 = GetMultiTurnAngle(q2_3, q_prec(1));
	q2_4 = GetMultiTurnAngle(q2_4, q_prec(1));
	// joint limits check:
	errorJoint_2[2] = GetJointLimitChecked(q2_3, 1);
	errorJoint_2[3] = GetJointLimitChecked(q2_4, 1);

	//-------------------------------------------------------------
	// ANGLE 3 - Sol. 1,2 - (computation with Angle 2 - Sol. 1)
	//-------------------------------------------------------------
	if (q2_1 >= SingNullCheck){
		// singularity in joint 2 - sol.1 -> do not compute q3 sol.1,2
		q3_1 = SingNull;
		q3_2 = SingNull;
	}
	else{
		// joint 2 sol.1 not singular so it's possible to compute a set
		// of solutions for angle 3 (Sol. 1,2).
		// Assignment of constant values needed in the computation:
		// --- CONSTANT KA1
		k1 = -L30*sin(q2_1);
		// --- CONSTANT KA2
		k2 = L30*cos(q2_1);
		// --- CONSTANT KA3
		k3 = d(0) + a(1)*sin(q2_1) - Pc(2);
		//-------------------------------------------------------------
		// computes solutions for angle 3 - sol 1,2:
		errorWork[3] = GetAngleSolutions(k1, k2, k3, q3_1, q3_2);
		// rounding to zero smaller values:
		if (abs(q3_1) < epsZero){ q3_1 = 0.0; }
		if (abs(q3_2) < epsZero){ q3_2 = 0.0; }
	}

	// extend the solution range:
	q3_1 = GetMultiTurnAngle(q3_1, q_prec(2));
	q3_2 = GetMultiTurnAngle(q3_2, q_prec(2));
	// joint limits check:
	errorJoint_3[0] = GetJointLimitChecked(q3_1, 2);
	errorJoint_3[1] = GetJointLimitChecked(q3_2, 2);

	//-------------------------------------------------------------
	// ANGLE 3 - Sol. 3,4 - (computation with Angle 2 - Sol. 2)
	//-------------------------------------------------------------
	if (q2_2 >= SingNullCheck){
		// singularity in joint 2 - sol.2 -> do not compute q3 sol.1,2
		q3_3 = SingNull;
		q3_4 = SingNull;
	}
	else{
		// joint 2 sol.2 not singular so it's possible to compute a set
		// of solutions for angle 3 (Sol. 3,4).
		// Assignment of constant values needed in the computation:
		// --- CONSTANT KB1
		k1 = -L30*sin(q2_2);
		// --- CONSTANT KB2
		k2 = L30*cos(q2_2);
		// --- CONSTANT KB3
		k3 = d(0) + a(1)*sin(q2_2) - Pc(2);
		//-------------------------------------------------------------
		// computes solutions for angle 3 - sol 3,4:
		errorWork[4] = GetAngleSolutions(k1, k2, k3, q3_3, q3_4);
		// rounding to zero smaller values:
		if (abs(q3_3) < epsZero){ q3_3 = 0.0; }
		if (abs(q3_4) < epsZero){ q3_4 = 0.0; }
	}

	// extend the solution range:
	q3_3 = GetMultiTurnAngle(q3_3, q_prec(2));
	q3_4 = GetMultiTurnAngle(q3_4, q_prec(2));
	// joint limits check:
	errorJoint_3[2] = GetJointLimitChecked(q3_3, 2);
	errorJoint_3[3] = GetJointLimitChecked(q3_4, 2);

	//-------------------------------------------------------------
	// ANGLE 3 - Sol. 5,6 - (computation with Angle 2 - Sol. 3)
	//-------------------------------------------------------------
	if (q2_3 >= SingNullCheck){
		// singularity in joint 2 - sol.3 -> do not compute q3 sol.5,6
		q3_5 = SingNull;
		q3_6 = SingNull;
	}
	else{
		// joint 2 sol.3 not singular so it's possible to compute a set
		// of solutions for angle 3 (Sol. 5,6).
		// Assignment of constant values needed in the computation:
		// --- CONSTANT KC1
		k1 = -L30*sin(q2_3);
		// --- CONSTANT KC2
		k2 = L30*cos(q2_3);
		// --- CONSTANT KC3
		k3 = d(0) + a(1)*sin(q2_3) - Pc(2);
		//-------------------------------------------------------------
		// computes solutions for angle 3 - sol 5,6:
		errorWork[5] = GetAngleSolutions(k1, k2, k3, q3_5, q3_6);
		// rounding to zero smaller values:
		if (abs(q3_5) < epsZero){ q3_5 = 0.0; }
		if (abs(q3_6) < epsZero){ q3_6 = 0.0; }
	}

	// extend the solution range:
	q3_5 = GetMultiTurnAngle(q3_5, q_prec(2));
	q3_6 = GetMultiTurnAngle(q3_6, q_prec(2));
	// joint limits check:
	errorJoint_1[4] = GetJointLimitChecked(q3_5, 2);
	errorJoint_1[5] = GetJointLimitChecked(q3_6, 2);

	//-------------------------------------------------------------
	// ANGLE 3 - Sol. 7,8 - (computation with Angle 2 - Sol. 4)
	//-------------------------------------------------------------
	if (q2_4 >= SingNullCheck){
		// singularity in joint 2 - sol.4 -> do not compute q3 sol.7,8
		q3_7 = SingNull;
		q3_8 = SingNull;
	}
	else{
		// joint 2 sol.4 not singular so it's possible to compute a set
		// of solutions for angle 3 (Sol. 7,8).
		// Assignment of constant values needed in the computation:
		// --- CONSTANT KD1
		k1 = -L30*sin(q2_4);
		// --- CONSTANT KD2
		k2 = L30*cos(q2_4);
		// --- CONSTANT KD3
		k3 = d(0) + a(1)*sin(q2_4) - Pc(2);
		//-------------------------------------------------------------
		// computes solutions for angle 3 - sol 7,8:
		errorWork[6] = GetAngleSolutions(k1, k2, k3, q3_7, q3_8);
		// rounding to zero smaller values:
		if (abs(q3_7) < epsZero){ q3_7 = 0.0; }
		if (abs(q3_8) < epsZero){ q3_8 = 0.0; }
	}

	// extend the solution range:
	q3_7 = GetMultiTurnAngle(q3_7, q_prec(2));
	q3_8 = GetMultiTurnAngle(q3_8, q_prec(2));
	// joint limits check:
	errorJoint_1[6] = GetJointLimitChecked(q3_7, 2);
	errorJoint_1[7] = GetJointLimitChecked(q3_8, 2);

	//-------------------------------------------------------------
	// composition of all the possible solution derived composing the
	// values obtained in the solution of the inverse kinematic for
	// the arm of the robot:
	// For the composition of the matrix of the solution, the offset
	// Kinematic Offset (q20, q30) are added to the angles:
	// --- q arm - Sol.1
	qArm(0, 0) = q1_1;
	qArm(0, 1) = q2_1;
	qArm(0, 2) = q3_1;
	// --- q arm - Sol.2
	qArm(1, 0) = q1_1;
	qArm(1, 1) = q2_1;
	qArm(1, 2) = q3_2;
	// --- q arm - Sol.3
	qArm(2, 0) = q1_1;
	qArm(2, 1) = q2_2;
	qArm(2, 2) = q3_3;
	// --- q arm - Sol.4
	qArm(3, 0) = q1_1;
	qArm(3, 1) = q2_2;
	qArm(3, 2) = q3_4;
	// --- q arm - Sol.5
	qArm(4, 0) = q1_2;
	qArm(4, 1) = q2_3;
	qArm(4, 2) = q3_5;
	// --- q arm - Sol.6
	qArm(5, 0) = q1_2;
	qArm(5, 1) = q2_3;
	qArm(5, 2) = q3_6;
	// --- q arm - Sol.7
	qArm(6, 0) = q1_2;
	qArm(6, 1) = q2_4;
	qArm(6, 2) = q3_7;
	// --- q arm - Sol.8
	qArm(7, 0) = q1_2;
	qArm(7, 1) = q2_4;
	qArm(7, 2) = q3_8;
	//-------------------------------------------------------------
	// research among all the set of solution for the arm, those with
	// represent singularity condition (angles equal to SingNull):
	checkArm = 0;
	for (int k = 0; k < n_solutions; k++){
		for (int i = 0; i < 3; i++){
			// check if an angle is in singular configuration:
			if (qArm(k, i) >= SingNullCheck){ checkArm += 1; }
		}
		// creates a vector telling if a solution is feasible or not:
		if (checkArm > 0){
			SolArm[k] = 1;
			checkArm = 0;
		}
		else SolArm[k] = 0;
	}
	//-------------------------------------------------------------
	// in a cycle, ruled if the matrix SolArm(i)!=1 so the arm configuration
	// is not singular, the problem for the wrist will be solved.
	//-------------------------------------------------------------
	// INVERSE ORIENTATION PROBLEM SOLUTION (SPHERICAL WRIST)
	//-------------------------------------------------------------
	// computation of the set of angles (q4,q5,q6) if the arm set of
	// angles is not in singular configuration, otherwise the wrist
	// solution will no tbe computed:
	for (int k = 0; k < n_solutions; k++){
		// search among all the solutions:
		if (SolArm[k] == 0){
			//-------------------------------------------------------------
			// ROTO-TRANSLATION MATRIX OF THE ARM COMPUTATION (i-th sol.)
			//-------------------------------------------------------------
			// To compute the rototranslation matrix of the arm, the three
			// joints DH matrix are needed to be computed:
			//-------------------------------------------------------------
			// - LINK 1 -> A_01(q1)
			Frame A_01 = Frame::DH(a(0), alpha(0), d(0), qArm(k, 0));
			// - LINK 2 -> A_12(q2)
			Frame A_12 = Frame::DH(a(1), alpha(1), d(1), qArm(k, 1));
			// - LINK 3 -> A_23(q3)
			Frame A_23 = Frame::DH(a(2), alpha(2), d(2), qArm(k, 2));
			// rototranslation matrix computation of the arm -> T_03:
			Frame T_03 = A_01*A_12*A_23;
			// extraction of the rotation matrix from the T_06 and T_03:
			for (int i = 0; i < 3; i++){
				for (int j = 0; j < 3; j++){
					// rotation matrix of the arm T matrix (0->3):
					R_03(i, j) = T_03(i, j);
				}
			}
			// computation of the rotation matrix of the spherical wrist:
			R_46 = R_03.transpose()*R_06;
			// extraction of the vectors composing the rotation matrix:
			for (int i = 0; i < 3; i++){
				N(i) = R_46(i, 0);
				S(i) = R_46(i, 1);
				A(i) = R_46(i, 2);
			}
			//-------------------------------------------------------------
			// angle 5 multiple solutions computation and others angles (2 sets)
			// --- SOLUTION 1:
			q4_1 = atan2(-A(1), -A(0));
			q5_1 = atan2(sqrt(A(0)*A(0) + A(1)*A(1)), A(2));
			q6_1 = atan2(-S(2), N(2));
			// --- SOLUTION 2:
			q4_2 = atan2(A(1), A(0));
			q5_2 = atan2(-sqrt(A(0)*A(0) + A(1)*A(1)), A(2));
			q6_2 = atan2(S(2), -N(2));
			//-------------------------------------------------------------
			// check spherical wrist singularities (on the base angle 5)
			// initial setting for singularities flag:
			singularity_1 = false;
			singularity_2 = false;
			// --- ANGLE 5 - SOLUTION 1:
			// angle q5 equal to zero: singular condition happened
			if (abs(q5_1) < epsZero){
				if (A(2) > 0){
					// positive solution computation:
					q46_1 = atan2(N(1), N(0));
					// extension of angle (q4+q6) to multi turn case:
					q46_1 = GetMultiTurnAngle(q46_1, (q_prec(3) + q_prec(5)));
					// subtraction of angle q4 (equal in singularity to previous):
					q4_1 = q_prec(3);
					q6_1 = q46_1 - q4_1;
					// rounding to zero smaller values:
					if (abs(q4_1) < epsZero){ q4_1 = 0.0; }
					if (abs(q6_1) < epsZero){ q6_1 = 0.0; }
					// singularity flag setting:
					singularity_1 = true;
				}
				else{
					// negative solution computation:
					q46_1 = atan2(-N(1), -N(0));
					// extension of angle (q4+q6) to multi turn case:
					q46_1 = GetMultiTurnAngle(q46_1, (q_prec(3) + q_prec(5)));
					// subtraction of angle q4 (equal in singularity to previous):
					q4_1 = q_prec(3);
					q6_1 = -q46_1 + q4_1;
					// rounding to zero smaller values:
					if (abs(q4_1) < epsZero){ q4_1 = 0.0; }
					if (abs(q6_1) < epsZero){ q6_1 = 0.0; }
					// singularity flag setting:
					singularity_1 = true;
				}
			}
			// --- ANGLE 5 - SOLUTION 2:
			// angle q5 equal to zero: singular condition happened
			if (abs(q5_2) < epsZero){
				if (A(2) > 0){
					// positive solution computation:
					q46_2 = atan2(N(1), N(0));
					// extension of angle (q4+q6) to multi turn case:
					q46_2 = GetMultiTurnAngle(q46_2, (q_prec(3) + q_prec(5)));
					// subtraction of angle q4 (equal in singularity to previous):
					q4_2 = q_prec(3);
					q6_2 = q46_2 - q4_2;
					// rounding to zero smaller values:
					if (abs(q4_2) < epsZero){ q4_2 = 0.0; }
					if (abs(q6_2) < epsZero){ q6_2 = 0.0; }
					// singularity flag setting:
					singularity_2 = true;
				}
				else{
					// negative solution computation:
					q46_2 = atan2(-N(1), -N(0));
					// extension of angle (q4+q6) to multi turn case:
					q46_2 = GetMultiTurnAngle(q46_2, (q_prec(3) + q_prec(5)));
					// subtraction of angle q4 (equal in singularity to previous):
					q4_2 = q_prec(3);
					q6_2 = -q46_2 + q4_2;
					// rounding to zero smaller values:
					if (abs(q4_2) < epsZero){ q4_2 = 0.0; }
					if (abs(q6_2) < epsZero){ q6_2 = 0.0; }
					// singularity flag setting:
					singularity_2 = true;
				}
			}
			//-------------------------------------------------------------
			// choose the right solution on the base of angle 5 (discriminant):
			if (singularity_1 == true){
				// singular cond. with extension to multi turn just computed
				// positive precedent angle 5 -> singularity on q5:
				qWrist(k, 0) = q4_1;
				qWrist(k, 1) = q5_1;
				qWrist(k, 2) = q6_1;
				// the unsued solution is stored in a diagnosis structure:
				qWristSecSol(k, 0) = q4_2;
				qWristSecSol(k, 1) = q5_2;
				qWristSecSol(k, 2) = q6_2;
			}
			else if (singularity_2 == true){
				// singular cond. with extension to multi turn just computed
				// negative precedent angle 5 -> consider angle 5 negative:
				qWrist(k, 0) = q4_2;
				qWrist(k, 1) = q5_2;
				qWrist(k, 2) = q6_2;
				// the unsued solution is stored in a diagnosis structure:
				qWristSecSol(k, 0) = q4_1;
				qWristSecSol(k, 1) = q5_1;
				qWristSecSol(k, 2) = q6_1;
			}
			else{
				if (q_prec(4) >= epsZero){
					// range extension for angle 4 and 6:
					q4_1 = GetMultiTurnAngle(q4_1, q_prec(3));
					q5_1 = GetMultiTurnAngle(q5_1, q_prec(4));
					q6_1 = GetMultiTurnAngle(q6_1, q_prec(5));
					// positive precedent angle 5 -> consider angle 5 positive:
					qWrist(k, 0) = q4_1;
					qWrist(k, 1) = q5_1;
					qWrist(k, 2) = q6_1;
					// the unsued solution is stored in a diagnosis structure:
					qWristSecSol(k, 0) = q4_2;
					qWristSecSol(k, 1) = q5_2;
					qWristSecSol(k, 2) = q6_2;
				}
				else{
					// range extension for angle 4 and 6:
					q4_2 = GetMultiTurnAngle(q4_2, q_prec(3));
					q5_2 = GetMultiTurnAngle(q5_2, q_prec(4));
					q6_2 = GetMultiTurnAngle(q6_2, q_prec(5));
					// negative precedent angle 5 -> consider angle 5 negative:
					qWrist(k, 0) = q4_2;
					qWrist(k, 1) = q5_2;
					qWrist(k, 2) = q6_2;
					// the unsued solution is stored in a diagnosis structure:
					qWristSecSol(k, 0) = q4_1;
					qWristSecSol(k, 1) = q5_1;
					qWristSecSol(k, 2) = q6_1;
				}
			}
			//-------------------------------------------------------------
			// check if the joint angles are inside limits Joint Space:
			// --- ANGLE 4:
			errorJoint_4[0] = GetJointLimitChecked(q4_1, 3);
			errorJoint_4[1] = GetJointLimitChecked(q4_2, 3);
			// --- ANGLE 5:
			errorJoint_5[0] = GetJointLimitChecked(q5_1, 4);
			errorJoint_5[1] = GetJointLimitChecked(q5_2, 4);
			// --- ANGLE 6:
			errorJoint_6[0] = GetJointLimitChecked(q6_1, 5);
			errorJoint_6[1] = GetJointLimitChecked(q6_2, 5);
		}
		else{
			// singularity detected in the arm configuration: no solution to
			// the spherical wrist will be computed since the overall solution
			// is not feasible:
			qWrist(k, 0) = SingNull;
			qWrist(k, 1) = SingNull;
			qWrist(k, 2) = SingNull;
			// also the second set of solution for the spherical wrist is setted
			// to singularity value:
			qWristSecSol(k, 0) = SingNull;
			qWristSecSol(k, 1) = SingNull;
			qWristSecSol(k, 2) = SingNull;
		}
	}
	//-------------------------------------------------------------
	// SELECTION OF THE MINIMAL DISTANCE SOLUTION FROM THE PREVIOUS
	//-------------------------------------------------------------
	// in this section of the function, the matrix composed by all the solutions
	// feasible and singular for the robot, is analyzed and the minimal distance
	// solution with respect to the previous joint configuration (q_prec) is
	// choosen and given as result in output.
	//-------------------------------------------------------------
	// construction of the solution matrix:
	for (int i = 0; i < n_solutions; i++){
		for (int k = 0; k < 3; k++){
			qOut(i, k) = qArm(i, k);
			qOut(i, k + 3) = qWrist(i, k);
		}
	}
	// adding to the multi solutions matrix the dh and kin offset:
	for (int k = 0; k < n_solutions; k++){
		for (int i = 0; i < n_axis; i++){
			qArm(k, i) -= (kin_offset(i) + dh_offset(i));
		}
	}
	//-------------------------------------------------------------
	// construction of a matrix collecting all the errors for the Work
	// Space and Joint Space revealed during computation:
	// - WORK SPACE DIAGNOSIS:

	//-------------------------------------------------------------
	// - JOINT SPACE DIAGNOSIS:
	// assignment of the joint 1 - sol.1,2 errors for the Joint Space:
	for (int i = 0; i < (n_solutions / 2); i++){
		// joint 1 Joint Space error assignment:
		errorJoint(i, 0) = errorJoint_1[0];
		errorJoint((i + 4), 0) = errorJoint_1[1];
	}
	// assignment of the joint 2 errors:
	// --- Sol.2 - 1:
	errorJoint(0, 1) = errorJoint_2[0];
	errorJoint(1, 1) = errorJoint_2[0];
	// --- Sol.2 - 2:
	errorJoint(2, 1) = errorJoint_2[1];
	errorJoint(3, 1) = errorJoint_2[1];
	// --- Sol.2 - 3:
	errorJoint(4, 1) = errorJoint_2[2];
	errorJoint(5, 1) = errorJoint_2[2];
	// --- Sol.2 - 4:
	errorJoint(6, 1) = errorJoint_2[3];
	errorJoint(7, 1) = errorJoint_2[3];
	// joint 3 errors:
	for (int i = 0; i < n_solutions; i++){
		// joint 3 assignment:
		errorJoint(i, 2) = errorJoint_3[i];
	}
	//-------------------------------------------------------------
	// for every set of angles the distance norm (Euclidean, norm2) is
	// computed with respect to the previous configuration of the angles,
	// checking also if a singularity is reported in the set of angles:
	checkArm = 0;
	for (int i = 0; i < n_solutions; i++){
		errorArmFlag(i) = 0;
		errorJointFlag(i) = 0;
	}
	// check and computation of distances:
	for (int k = 0; k < n_solutions; k++){
		for (int i = 0; i < n_axis; i++){
			// check if an angle is in singular configuration:
			if (qOut(k, i) >= SingNullCheck){ checkArm += 1; }
			// check if Work Space and/or Joint Space error are active:
			if (errorArm(k, i) > 0){ errorArmFlag(k) = errorArmFlag(k) + 1; }
			if (errorJoint(k, i) > 0){ errorJointFlag(k) = errorJointFlag(k) + 1; }
		}
		if ((checkArm > 0) || (errorArmFlag(k) > 0) || (errorJointFlag(k) > 0)){
			distance[k] = SingNull;
			checkArm = 0;
		}
		else{
			// the solution doesn't present singularities so it is
			// possible to compute the norm:
			for (int i = 0; i < n_axis; i++){
				tempVector(i) = qOut(k, i) - q_old(i);
			}
			// distance computed for k-th solution:
			distance[k] = tempVector.norm();
		}
	}
	// search, if the distance is not singular, for the smallest distance vector:
	smallDistance = distance[0];
	indexDistance = 0;
	for (int i = 1; i < n_solutions; i++){
		if (distance[i] < smallDistance){
			smallDistance = distance[i];
			indexDistance = i;
		}
	}
	//-------------------------------------------------------------
	// once the adjacent solution is computed, it is passed as output:
	// and adding kinematic offset and dh_offset to the function:
	for (int i = 0; i < n_axis; i++){
		q(i) = qOut(indexDistance, i);
	}
	// assignment to class-variable of the index choosen:
	solutionIPK = indexDistance;
	//-------------------------------------------------------------
	// generation of the return value on the base of selected minimal
	// distance solution and error flags computed:
	if (errorArmFlag(indexDistance) > 0){ return 0; }			// Work Space error
	else if (errorJointFlag(indexDistance) > 0){ return 2; }	// Joint Space error
	else return 0;                                              // No errors matched.
	//-------------------------------------------------------------
}

//-------------------------------------------------------------
// Function that computes the speed in the Joint Space once a
// Work Space vector containing speed and angular speed of the
// End Effector is given. Needs the inverse Jacobian matrix.
// --- INPUT:
//		s_dot	->	Twist    1x6 : Work Space speed of the E.E.
// --- OUTPUT:
//      q_dot	->  JntArray 1x6 : Joint Space velocity.
//-------------------------------------------------------------
void AnthropKinematic::IVK(JntArray &q_dot, Twist &s_dot){

	// temporary storage vector defined in Eigen class:
	VectorXd qDot = VectorXd(n_axis);
	VectorXd sDot = VectorXd(n_axis);
	// acquisition of the values from the given vector:
	for (int i = 0; i < 3; i++){
		sDot(i) = s_dot.vel(i);
		sDot(i + 3) = s_dot.rot(i);
	}

	// computation of the Joint Space speed:
	qDot = J_inv*sDot;

	// passing the computed values in output:
	for (int i = 0; i < n_axis; i++){
		q_dot(i) = qDot(i);
	}
	//-------------------------------------------------------------
}

//-------------------------------------------------------------
// Function that computes the acceleration in the Joint Space
// once given the corrispective acceleration in Work Space and
// the speed in the Joint Space.
// Needs the inverse of the Jacobian and its time derivative.
// --- INPUT:
//		 q_dot	->	JntArray 1x6 : Joint Space speed vector.
//   s_dot_dot	->	Twist    1x6 : Work Space acceleration.
// --- OUTPUT:
//   q_dot_dot  ->  JntArray 1x6 : Joint Space acceleration.
//-------------------------------------------------------------
void AnthropKinematic::IAK(JntArray & q_dot, JntArray &q_dot_dot, Twist &s_dot_dot){

	// definition of vectors in Eigen convention to compute
	// the acceleration in Joint Space:
	VectorXd qDot    = VectorXd(n_axis);
	VectorXd qDotDot = VectorXd(n_axis);
	VectorXd sDotDot = VectorXd(n_axis);

	// assignment of the values to the vector from the given ones:
	for (int i = 0; i < n_axis; i++){
		qDot(i)    = q_dot(i);
	}
	for (int i = 0; i < 3; i++){
		sDotDot(i) = s_dot_dot.vel(i);
		sDotDot(i + 3) = s_dot_dot.rot(i);
	}

	// computation of the acceleration in Joint Space:
	qDotDot = J_inv*(sDotDot - J_dot*qDot);

	// passing the computed values in output:
	for (int i = 0; i < n_axis; i++){
		q_dot_dot(i) = qDotDot(i);
	}
	//-------------------------------------------------------------
}

//-------------------------------------------------------------
// Utilities function needed in the definition and computation of the
// kinematic model of the robot:
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
void AnthropKinematic::Jacobian(JntArray &q, Frame &T){

	// computation of the Joint Space variables in DH convention:
	double q1 = (q(0) + dh_offset(0)) + kin_offset(0);
	double q2 = (q(1) + dh_offset(1)) + kin_offset(1);
	double q3 = (q(2) + dh_offset(2)) + kin_offset(2);
	double q4 = (q(3) + dh_offset(3)) + kin_offset(3);
	double q5 = (q(4) + dh_offset(4)) + kin_offset(4);
	double q6 = (q(5) + dh_offset(5)) + kin_offset(5);

	// definition of the Jacobian matrix element by element:
	// Row 1:
	J(0, 0) = cos(q1)*(d(1) + d(5)*sin(q4)*sin(q5)) + sin(q1)*(-a(0) - a(1)*cos(q2) + (d(3) + d(5)*cos(q5))*sin(q2 + q3) + cos(q2 + q3)*(-a(2) + d(5)*cos(q4) + sin(q5)));
	J(0, 1) = -cos(q1)*(cos(q2 + q3)*(d(3) + d(5)*cos(q5)) + a(1)*sin(q2) + sin(q2 + q3)*(a(2) - d(5)*cos(q4)*sin(q5)));
	J(0, 2) = -cos(q1)*(cos(q2 + q3)*(d(3) + d(5)*cos(q5)) + sin(q2 + q3)*(a(2) - d(5)*cos(q4)*sin(q5)));
	J(0, 3) = d(5)*(cos(q4)*sin(q1) + cos(q1)*cos(q2 + q3)*sin(q4))*sin(q5);
	J(0, 4) = d(5)*(cos(q5)*sin(q1)*sin(q4) + cos(q1)*(-cos(q2 + q3)*cos(q4)*cos(q5) + sin(q2 + q3)*sin(q5)));
	J(0, 5) = 0.0;
	// Row 2:
	J(1, 0) = sin(q1)*(d(1) + d(5)*sin(q4)*sin(q5) + cos(q1)*(a(0) + a(1)*cos(q2)) - (d(3) + d(5)*cos(q5))*sin(q2 + q3) + cos(q2 + q3)*(a(2) - d(5)*cos(q4)*sin(q5)));
	J(1, 1) = -sin(q1)*(cos(q2 + q3)*(d(3) + d(5)*cos(q5)) + a(1)*sin(q2) + sin(q2 + q3)*(a(2) - d(5)*cos(q4)*sin(q5)));
	J(1, 2) = -sin(q1)*(cos(q2 + q3)*(d(3) + d(5)*cos(q5)) + sin(q2 + q3)*(a(2) - d(5)*cos(q4)*sin(q5)));
	J(1, 3) = d(5)*(-cos(q1)*cos(q4) + cos(q2 + q3)*sin(q1)*sin(q4))*sin(q5);
	J(1, 4) = -d(5)*cos(q5)*(cos(q2 + q3)*cos(q4)*sin(q1) + cos(q1)*sin(q4)) + d(5)*sin(q1)*sin(q2 + q3)*sin(q5);
	J(1, 5) = 0.0;
	// Row 3:
	J(2, 0) = 0.0;
	J(2, 1) = a(1)*cos(q2) - (d(3) + d(5)*cos(q5))*sin(q2 + q3) + cos(q2 + q3)*(a(2) - d(5)*cos(q4)*sin(q5));
	J(2, 2) = -(d(3) + d(5)*cos(q5))*sin(q2 + q3) + cos(q2 + q3)*(a(2) - d(5)*cos(q4) + sin(q5));
	J(2, 3) = d(5)*sin(q2 + q3)*sin(q5)*sin(q4);
	J(2, 4) = -d(5)*(cos(q4)*cos(q5)*sin(q2 + q3) + cos(q2 + q3)*sin(q5));
	J(2, 5) = 0.0;
	// Row 4:
	J(3, 0) = 0, 0;
	J(3, 1) = sin(q1);
	J(3, 2) = sin(q1);
	J(3, 3) = -cos(q1)*sin(q2 + q3);
	J(3, 4) = cos(q4)*sin(q1) + cos(q1)*cos(q2 + q3)*sin(q4);
	J(3, 5) = sin(q1)*sin(q4)*sin(q5) - cos(q1)*(cos(q5)*sin(q2 + q3) + cos(q2 + q3)*cos(q4)*sin(q5));
	// Row 5:
	J(4, 0) = 0.0;
	J(4, 1) = -cos(q1);
	J(4, 2) = -cos(q1);
	J(4, 3) = -sin(q1)*sin(q2 + q3);
	J(4, 4) = -cos(q1)*cos(q4) + cos(q2 + q3)*sin(q1)*sin(q4);
	J(4, 5) = -cos(q5)*sin(q1)*sin(q2 + q3) - (cos(q2 + q3)*cos(q4)*sin(q1) + cos(q1)*sin(q4))*sin(q5);
	// Row 6:
	J(5, 0) = 1.0;
	J(5, 1) = 0.0;
	J(5, 2) = 0.0;
	J(5, 3) = cos(q2 + q3);
	J(5, 4) = sin(q2 + q3);
	J(5, 5) = cos(q2 + q3)*cos(q5) - cos(q4)*sin(q2 + q3)*sin(q5);
	//-------------------------------------------------------------
	// computation of the inverse of the Jacobian Matrix:
	J_inv = J.inverse();
	//-------------------------------------------------------------
}

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
void AnthropKinematic::JacobianDerivative(JntArray &q_dot){

	// definition of temporary useful matrices:
	Matrix3d rot, vel;
	MatrixXd temporary = MatrixXd::Zero(n_axis, n_axis);

	// temporary vector collecting the column of the Jacobian:
	VectorXd jac_column = VectorXd(n_axis);

	for (int k = 0; k < n_axis; k++){

		// acquisition of the k-th Jacobian's column:
		for (int i = 0; i < n_axis; i++){
			jac_column(i) = J(i, k);
		}

		// composition of the rotation and velocity matrices:
		rot <<           0.0, -jac_column(5),  jac_column(4),
			   jac_column(5),            0.0, -jac_column(3),
			  -jac_column(4),  jac_column(3),            0.0;

		vel <<           0.0, -jac_column(2),  jac_column(1),
			   jac_column(2),            0.0, -jac_column(0),
			  -jac_column(1),  jac_column(0),            0.0;

		// computation of the top left/right corners of the
		// matrices rot and vel:
		temporary.topLeftCorner(n_axis / 2.0, n_axis / 2.0)     += rot*q_dot(k);
		temporary.topRightCorner(n_axis / 2.0, n_axis / 2.0)    += vel*q_dot(k);
		temporary.bottomRightCorner(n_axis / 2.0, n_axis / 2.0) += rot*q_dot(k);
	}
	// computation of the derivative of the Jacobi's matrix:
	J_dot = temporary*J;
	//-------------------------------------------------------------
}

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
double AnthropKinematic::GetMultiTurnAngle(double qAtan, double qPrec){

	// if the angle is in singularity, don't enter in the check loop;
	if (abs(qAtan) >= SingNullCheck) return qAtan;

	// returned value: correct multi-turn angle [rad] and its
	// temporary value:
	double q, q_t;
	double qDistPrec[n_turn];

	// set the vector of possible angular solutions (multi-turn):
	for (int i = 0; i < n_turn; i++){
		qDistPrec[i] = qAtan - qPrec + offsetRound(i);
	}
	// computation of the distance from the previous given value:
	double smallDistance = abs(qDistPrec[0]);
	q_t = qAtan + offsetRound[0];

	// research cicle and assignement:
	for (int i = 1; i < n_turn; i++){
		if (abs(qDistPrec[i]) < smallDistance){
			smallDistance = abs(qDistPrec[i]);
			q_t = qAtan + offsetRound[i];
		}
	}
	// opdate output angle with multi-turn tolerance:
	q = q_t;
	return q;
	//-------------------------------------------------------------
}

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
//                              1. error cond. angle outside limits.
//-------------------------------------------------------------
int AnthropKinematic::GetJointLimitChecked(double q, int indexAngle){

	// if the angle is in singularity, don't enter in the check loop;
	if (abs(q) >= SingNullCheck) return 2;

	// check if the angle is inside the related boundaries:
	if ((q > q_max(indexAngle)) && (q < q_max(indexAngle))){
		// error condition: limits exceeded, set the angle to SingNull
		// value and return error code:
		q = SingNull;
		return 2;
	}
	return 0;
	//-------------------------------------------------------------
}

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
int AnthropKinematic::GetAngleSolutions(double K1, double K2, double K3, double qA, double qB){

	// temporary computational variables:
	double numA, denA;
	double numB, denB;
	// temporary Work Space error flag:
	int codeWorkSpace;
	//-------------------------------------------------------------
	// check if the solution exist among the discriminant of the function:
	if ((K1*K1 + K2*K2) < (K3*K3)){
		// no solution exist since the discriminant is negative:
		qA = SingNull;
		qB = SingNull;
		// assignment Work Space error check's variable:
		codeWorkSpace = 1;
	}
	else{
		if ((abs(K3 - K2) < epsZero) && (abs(K1) < epsZero)){
			// no solution exist since coeff. are zero:
			qA = SingNull;
			qB = SingNull;
			// assignment Work Space error check's variable:
			codeWorkSpace = 1;
		}
		else if (abs(K3 - K2) < epsZero){
			// exist only a single solution:
			qA = -(K2 + K3) / (2 * K1);
			qB = SingNull;
			// assignment Work Space error check's variable:
			codeWorkSpace = 0;
		}
		else{
			// computation using normal second order formula (reduced):
			// - x12 = -b/2 +- sqrt(b/2^2 - a*c) / a;
			// --- solution angle A ---
			numA = -K1 + sqrt(K1*K1 + K2*K2 - K3*K3);
			denA = K3 - K2;
			qA = 2.0*atan2(numA, denA);
			// --- solution angle B ---
			numB = -K1 - sqrt(K1*K1 + K2*K2 - K3*K3);
			denB = K3 - K2;
			qB = 2.0*atan2(numB, denB);
			// assignment Work Space error check's variable:
			codeWorkSpace = 0;
		}
	}
	return codeWorkSpace;
	//-------------------------------------------------------------
}

AnthropKinematic.cpp
Visualizzazione di AnthropKinematic.h.
