#include <iostream>
#include "vrep_bridge/vrep_bridge.h"

//for controller 
#include "controller/Inverse-dynamics.h"

// for tasks
#include "tasks/task-com.h"
#include "tasks/task-operational.h"
#include "tasks/task-joint-posture.h"
#include "tasks/task-joint-bounds.h"
//#include "tasks/task-mobile.h"
#include "tasks/task-singularity.h"

// for trajectories 
#include "trajectories/trajectory-operationalspace.h"
#include "trajectories/trajectory-jointspace.h"

// for solver
#include "solvers/solver-HQP-factory.hxx"
#include "solvers/solver-utils.h"
//#include "solvers/solver-HQP-eiquadprog.h"
#include "solvers/solver-HQP-qpoases.h"
//#include <tsid/math/utils.hpp>

// for contact point

//#include "contacts/contact-3d.h"
#include "utils/container.h"

#include <string>
#include <vector>
//#include <conio.h> // for keyboard hit

// for vrep keyboard event
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
bool _kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
}

HQP::robot::RobotModel * robot_;
HQP::InverseDynamics * invdyn_;
HQP::tasks::TaskJointPosture * jointTask;
HQP::tasks::TaskOperationalSpace * moveTask, * move2Task;
HQP::tasks::TaskJointLimit * jointLimitTask;
HQP::contact::Contact3dPoint * contactTask;
HQP::tasks::TaskSingularityAvoidance * singularTask;

HQP::trajectories::TrajectoryJointCubic * trajPosture;
HQP::trajectories::TrajectoryJointConstant * trajPostureConstant;
HQP::trajectories::TrajectoryOperationCubic * trajEECubic;
HQP::trajectories::TrajectoryOperationConstant * trajEEConstant;

VectorXd q(dof);
VectorXd qdot(dof);
VectorXd qdes(dof);
VectorXd q_lb(dof); // mobile 2 + robot 7
VectorXd q_ub(dof); // mobile 2 + robot 7
double vrep_time = 0.0;
double Hz = 1000.0;
int na;
int nv;
int nq;

using namespace HQP;
using namespace std;

int main()
{
   robot_ = new HQP::robot::RobotModel(0); // 0: Manipulator, 1: Mobile Manipulaotr, 2: humanoid
   na = robot_->na();	
   nv = robot_->nv();

   invdyn_ = new HQP::InverseDynamics(*robot_);
   q.setZero();
   qdot.setZero();	

   solver::SolverHQPBase * solver = solver::SolverHQPFactory::createNewSolver(solver::SOLVER_HQP_QPOASES, "solver-eiquadprog");
	
	// Level 0 : Joint Velocity Limit for Mobile + Manipulator 
	q_lb = -180.0 / 180.0 * M_PI * VectorXd(dof).setOnes();
	q_ub = -1.0*q_lb;

	double kp_jointlimit = 100.0, w_jointlimit = 1.00;
	jointLimitTask = new tasks::TaskJointLimit("joint_limit_task", *robot_);
	jointLimitTask->Kp(kp_jointlimit*VectorXd::Ones(robot_->nv()));
	jointLimitTask->Kd(2.0*jointLimitTask->Kp().cwiseSqrt());
	jointLimitTask->setJointLimit(q_lb, q_ub);

	
	jointTask = new tasks::TaskJointPosture("joint_control_task", *robot_);
	double kp_posture = 30000.0, w_posture = 1.00;
	jointTask->Kp(kp_posture*VectorXd::Ones(robot_->nv()));
	jointTask->Kd(2.0*jointTask->Kp().cwiseSqrt());

	moveTask = new tasks::TaskOperationalSpace("end_effector_task", *robot_, 7);
	double kp_move = 3000.0, w_move = 1.0;
	VectorXd a = VectorXd::Ones(6);
	a.tail(3) *= 10.0;
	moveTask->Kp(kp_move*a);
	moveTask->Kd(2.0*moveTask->Kp().cwiseSqrt());
	moveTask->setSingular(false);

	move2Task = new tasks::TaskOperationalSpace("end_effector_task2", *robot_, 7);
	move2Task->Kp(kp_move*a);
	move2Task->Kd(2.0*move2Task->Kp().cwiseSqrt());
	move2Task->setSingular(true);


	//trajectories::TrajectoryBase *trajPosture = new trajectories::TrajectoryJointConstant("joint_traj", qdes);
	trajectories::TrajectorySample sampleJoint(robot_->nv());
	trajectories::TrajectorySample s(12, 6);
	
	//trajectories::TrajectoryOperationConstant *trajEE = new trajectories::TrajectoryOperationConstant("operational_traj", T_endeffector);
	//

	// Transform3d T_mobile;
	// trajectories::TrajectorySample s_mobile(12, 6);
	//trajectories::TrajectoryOperationConstant *trajmobile = new trajectories::TrajectoryOperationConstant("mobile_traj", T_mobile);

	solver->resize(invdyn_->nVar(), invdyn_->nEq(), invdyn_->nIn(), invdyn_->nBound());
	// for v-rep
	VRepBridge vb;
	vb.isSimulationRun = false;
	vb.exitFlag = false;

	int ctrl_mode = 0;
	bool mode_change = true;
	bool HQP_flag = false;
	bool flag = false;

	double start_time = 0.0;
	VectorXd q_init(dof);

	while (vb.simConnectionCheck() && !vb.exitFlag)
	{
		if (_kbhit()) {
			int key;
			key = getchar();
			switch (key)
			{
			case '\t':
				if (vb.isSimulationRun) {
					cout << "Simulation Pause" << endl;
					vb.isSimulationRun = false;
				}
				else {
					cout << "Simulation Run" << endl;
				//	vb._cntt = 0;
					vb.isSimulationRun = true;
				}
				break;
			case 'g': // for gravity compensation
				ctrl_mode = 0; 
				mode_change = true;
				HQP_flag = true;
				break;
			case 'h': // for home position joint ctrl
				ctrl_mode = 1;
				mode_change = true;
				HQP_flag = true;
				break;
			case 'i': // for init position joint ctrl
				ctrl_mode = 2;
				mode_change = true;
				HQP_flag = true;
				break;
			case 't': // for init position joint ctrl
				ctrl_mode = 3;
				mode_change = true;
				HQP_flag = true;
				break;	
			case 'q':
				vb.isSimulationRun = false;
				vb.exitFlag = true;
				simxEndDialog(vb.getClientID(), vb.dialog_handle, simx_opmode_oneshot);
				break;

			default:
				break;
			}
		}
		if (vb.isSimulationRun)
		{
			vb.read();
			
			if (vb._cntt > 0)
			{
				vrep_time = vb._cntt / Hz;

				VectorXd q_current(robot_->na()), qdot_current(robot_->na());			
				q_current = vb.current_q_;
				qdot_current = vb.current_qdot_;				
				robot_->getUpdateKinematics(q_current, qdot_current);

				if (ctrl_mode == 0 ){
					if (mode_change){
						cout << "Gravity compensation" << endl;
						mode_change = false;
					}

					const VectorXd & tau = robot_->getNLEtorque();	
					vb.desired_torque_ = tau;
				}
				else if (ctrl_mode == 1){
					if (mode_change){
						cout << "Home position (joint control using HQP)" << endl;
						cout << "This mode has joint limit avoidance" << endl;
					
						// Level 0 : Joint Velocity Limit for Mobile + Manipulator 
					//	invdyn_->resizeHqpData();
						invdyn_ = new HQP::InverseDynamics(*robot_);
						invdyn_->addJointLimitTask(*jointLimitTask, 1.0, 0, 0.0);
						invdyn_->addJointPostureTask(*jointTask, 1.0, 1, 0.0); //weight, level, duration
						
						q_init = q_current;
						start_time = vrep_time;

						qdes.setZero();
						qdes(1) = -M_PI/4.0;
						qdes(3) = -M_PI/2.0;
						qdes(5) = M_PI/4.0;

						trajPosture = new trajectories::TrajectoryJointCubic("joint_traj");
						trajPosture->setInitSample(q_init);
						trajPosture->setGoalSample(qdes);
						trajPosture->setDuration(0.1);
						trajPosture->setStartTime(start_time);
						trajPosture->setReference(qdes);
						mode_change = false;			
					}			
					trajPosture->setCurrentTime(vrep_time);
					sampleJoint = trajPosture->computeNext();
					jointTask->setReference(sampleJoint);

					const solver::HQPData & HQPData = invdyn_->computeProblemData(vrep_time, q_current, qdot_current);
					if (vb._cntt  % 100 == 0){
				 		cout << solver::HQPDataToString(HQPData, true) << endl;			
						HQP_flag = false;
					}
					const solver::HQPOutput & sol = solver->solve(HQPData);
					const VectorXd & tau = invdyn_->getActuatorForces(sol);
					vb.desired_torque_ = tau;
				}
				else if (ctrl_mode == 2){
					if (mode_change){
						cout << "Initial position (joint control using HQP)" << endl;
						cout << "This mode has joint limit avoidance" << endl;
					
						// Level 0 : Joint Velocity Limit for Mobile + Manipulator 
					//	invdyn_->resizeHqpData();
						invdyn_ = new HQP::InverseDynamics(*robot_);
						invdyn_->addJointLimitTask(*jointLimitTask, 1.0, 0, 0.0);
						invdyn_->addJointPostureTask(*jointTask, 1.0, 1, 0.0); //weight, level, duration
						
						q_init = q_current;
						start_time = vrep_time;

						qdes.setZero();
						trajPosture = new trajectories::TrajectoryJointCubic("joint_traj");
						trajPosture->setInitSample(q_init);
						trajPosture->setGoalSample(qdes);
						trajPosture->setDuration(1.0);
						trajPosture->setStartTime(start_time);
						trajPosture->setReference(qdes);
						mode_change = false;			
					}			
					trajPosture->setCurrentTime(vrep_time);
					sampleJoint = trajPosture->computeNext();
					jointTask->setReference(sampleJoint);

					const solver::HQPData & HQPData = invdyn_->computeProblemData(vrep_time, q_current, qdot_current);
					if (vb._cntt  % 100 == 0){
				 		cout << solver::HQPDataToString(HQPData, true) << endl;			
						HQP_flag = false;
					}
					const solver::HQPOutput & sol = solver->solve(HQPData);
					const VectorXd & tau = invdyn_->getActuatorForces(sol);
					vb.desired_torque_ = tau;
				}
				else if (ctrl_mode == 3){
					if (mode_change){
						cout << "x + 20 cm for 1 sec" << endl;
						cout << "This mode has joint limit avoidance" << endl;
						cout << "This mode has joint posture control" << endl;

						start_time = vrep_time;
					
						// Level 0 : Joint Velocity Limit for Mobile + Manipulator 
						invdyn_ = new HQP::InverseDynamics(*robot_);
					//	invdyn_->addJointLimitTask(*jointLimitTask, 1.0, 0, 0.0);

						// Level 1: Operational Task Control
						invdyn_->addOperationalTask(*moveTask, w_move, 1, 0.0);
						invdyn_->addOperationalTask(*move2Task, w_move, 1, 0.0);

						Transform3d init_T = robot_->getTransformation(7);
						Transform3d goal_T = init_T;
						//goal_T.linear() = rot_temp * goal_T.linear(); 
						goal_T.translation()(2) += 0.3;
						//trajEE->setReference(goal_T);
						
						trajEECubic = new trajectories::TrajectoryOperationCubic("op_traj");
						trajEECubic->setInitSample(init_T);
						trajEECubic->setGoalSample(goal_T);
						trajEECubic->setDuration(3.0);
						trajEECubic->setStartTime(start_time);
						trajEECubic->setReference(goal_T);						
						
						trajEEConstant = new trajectories::TrajectoryOperationConstant("op_const");
						trajEEConstant->setReference(goal_T);
						// Level 2: Joint Task Control with low gain
						kp_posture = 300.0;
						jointTask->Kp(kp_posture*VectorXd::Ones(robot_->nv()));
						jointTask->Kd(2.0*jointTask->Kp().cwiseSqrt());

						invdyn_->addJointPostureTask(*jointTask, 1.0, 2, 0.0); //weight, level, duration

						qdes.setZero();
						// qdes(1) = -M_PI/4.0;
						// qdes(3) = -M_PI/2.0;
						// qdes(5) = M_PI/4.0;

						trajPostureConstant = new trajectories::TrajectoryJointConstant("joint_traj_const"); 
						trajPostureConstant->setReference(qdes);
						mode_change = false;			
					}

					sampleJoint = trajPostureConstant->computeNext();
					jointTask->setReference(sampleJoint);

					trajEECubic->setCurrentTime(vrep_time);
					s = trajEECubic->computeNext();
					//cout << s.pos.head(3) <<endl;
					moveTask->setReference(s);
					move2Task->setReference(s);

					const solver::HQPData & HQPData = invdyn_->computeProblemData(vrep_time, q_current, qdot_current);
					if (vb._cntt  % 100 == 0){
				 		cout << solver::HQPDataToString(HQPData, true) << endl;			
						HQP_flag = false;
					}
					const solver::HQPOutput & sol = solver->solve(HQPData);
					const VectorXd & tau = invdyn_->getActuatorForces(sol);
					vb.desired_torque_ = tau;


				}
				

				// if (vrep_time == 1.0 / Hz) {
				// 	init_T = robot_->getTransformation(7);
				// 	goal_T = init_T;
				// 	goal_T.translation()(0) += 0.33;
				// 	T_endeffector = robot_->getTransformation(7);
				// 	T_endeffector.translation()(0) += 0.33;
				// 	//T_endeffector.translation()(1) -= 0.13;
		
				// 	trajEE->setReference(T_endeffector);		
				// 	invdyn_->addOperationalTask(*moveTask, w_move, 1, 0.0);
				// 	invdyn_->addOperationalTask(*move2Task, w_move, 1, 0.0);
				// }

				// s = trajEE->computeNext();
				// moveTask->setReference(s);
				// move2Task->setReference(s);

				// MatrixXd J_EE_left(6, 7); 
				// J_EE_left = robot_->getJacobian(7).block(0, 2, 6, 7);

				// s = trajEE->computeNext();
				// moveTask->setReference(s);
				// move2Task->setReference(s);
				
				// samplePosture = trajPosture->computeNext();
				// postureTask->setReference(samplePosture);
				// const solver::HQPData & HQPData = invdyn_->computeProblemData(vrep_time, q_current, qdot_current);

				// if (vrep_time == 1.0/Hz)
				// 	cout << solver::HQPDataToString(HQPData, true) << endl;				
								
				// const solver::HQPOutput & sol = solver->solve(HQPData);
				// const VectorXd & tau = invdyn_->getActuatorForces(sol);
				// VectorXd dv = invdyn_->getAccelerations(sol);


				// vb.desired_base_vel_(0) = dv(0);
				// vb.desired_base_vel_(1) = dv(1);
				// vb.desired_base_vel_(2) = dv(1);
				// vb.desired_base_vel_(3) = dv(0);

				
				vb.write();

			}		
			vb.simLoop();
			vb._cntt++;

		}
	}

	return 0;
}
