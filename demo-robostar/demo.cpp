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

#include <signal.h>
#include <sys/mman.h>
#include <sys/resource.h>
// Xenomai
#include <native/task.h>
#include <native/timer.h>
#include <native/pipe.h>
#include <native/mutex.h>

#include "ecrt.h"
#include "hardware/ethercat_elmo.h"
#include "hardware/robostar_7_dof_robot_hw_config.h"

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
double Hz = 2000.0;
int na;
int nv;
int nq;

using namespace HQP;
using namespace std;
using namespace EtherCAT_Elmo;

//#define time_check
#ifdef time_check
#include <time.h>
#include <stdint.h>
#include <stdlib.h>

#define BILLION 1000000000L

int localpid(void) {
 static int a[9] = { 0 };
 return a[0];
}
#endif


RT_TASK ethercatTask;
bool quit_flag = false;
bool ethercat_task_flag = false;
bool mode_change = false;
bool value_changed = false;
int ctrl_mode = 0;
bool HQP_flag = false;

void cleanup_all(void)
{
    if(ethercat_task_flag)
        rt_task_delete(&ethercatTask);
}

void catch_signal(int sig)
{
    quit_flag = true;
    cleanup_all();
    cout << "exit" << endl;
    return;
}

void ethercat_task(void *arg)
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
    double kp_posture = 2000.0, w_posture = 1.00;
	jointTask->Kp(kp_posture*VectorXd::Ones(robot_->nv()));
	jointTask->Kd(2.0*jointTask->Kp().cwiseSqrt());

	moveTask = new tasks::TaskOperationalSpace("end_effector_task", *robot_, 7);
    double kp_move = 1000.0, w_move = 1.0;
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


	EthercatElmoBridge elmo(DOF);
	RTIME now, prev;

	VectorXd positionElmo(DOF);
	VectorXd velocityElmo(DOF);
	VectorXd torqueElmo(DOF);

	VectorXd positionDesiredElmo(DOF);
	VectorXd velocityDesiredElmo(DOF);
	VectorXd desired_torque(DOF);
	elmo.init();

    int err = rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(PERIOD_NS));
    if (err)
    {
        cout<< "error on set periodic" << strerror(-err);
        return;
    }
    rt_printf("Controller starts.\n");
    prev = rt_timer_read();

    double control_time = 0.0, start_time = 0.0;
	long tick = 0;
    VectorXd q_init(robot_->na()), q_current(robot_->na()), qdot_current(robot_->na());
    while(true)
    {
        rt_task_wait_period(NULL);
		tick++;
		if(elmo.isEnable())
		{
			elmo.read();
			control_time = tick / Hz;
		     for (size_t i = 0; i < DOF; i++)
            {
                ElmoGoldDevice::elmo_gold_tx & txPDO = elmo.device_[i].txPDO;
                q_current(i) = txPDO.positionActualValue * CNT2RAD[i] - OFFSET[i];
                qdot_current(i) = txPDO.velocityActualValue * CNT2RAD[i];
                //torqueElmo(i) = txPDO.torqueActualValue * RATEDTORQUE[i] / 1000.;
            }	
			robot_->getUpdateKinematics(q_current, qdot_current);

			if (ctrl_mode == 0 ){
				if (mode_change){
					cout << "Gravity compensation" << endl;
					mode_change = false;
				}

				const VectorXd & tau = robot_->getNLEtorque();	
				desired_torque = tau;
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
                    start_time = control_time;

					qdes.setZero();
					qdes(1) = -M_PI/4.0;
					qdes(3) = -M_PI/2.0;
					qdes(5) = M_PI/4.0;

					trajPosture = new trajectories::TrajectoryJointCubic("joint_traj");
					trajPosture->setInitSample(q_init);
					trajPosture->setGoalSample(qdes);
                    trajPosture->setDuration(5.0);
					trajPosture->setStartTime(start_time);
					trajPosture->setReference(qdes);
					mode_change = false;			
				}			
                trajPosture->setCurrentTime(control_time);
				sampleJoint = trajPosture->computeNext();
				jointTask->setReference(sampleJoint);

                const solver::HQPData & HQPData = invdyn_->computeProblemData(control_time, q_current, qdot_current);
                if (tick  % ((int)Hz / 10) == 0){
					cout << solver::HQPDataToString(HQPData, true) << endl;			
					HQP_flag = false;
				}
				const solver::HQPOutput & sol = solver->solve(HQPData);
				const VectorXd & tau = invdyn_->getActuatorForces(sol);
				desired_torque = tau;
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
                    start_time = control_time;

					qdes.setZero();
					trajPosture = new trajectories::TrajectoryJointCubic("joint_traj");
					trajPosture->setInitSample(q_init);
					trajPosture->setGoalSample(qdes);
                    trajPosture->setDuration(5.0);
					trajPosture->setStartTime(start_time);
					trajPosture->setReference(qdes);
					mode_change = false;			
				}		
#ifdef time_check
clock_gettime(CLOCK_MONOTONIC, &start); /* mark start time */
#endif	
                trajPosture->setCurrentTime(control_time);
				sampleJoint = trajPosture->computeNext();
				jointTask->setReference(sampleJoint);

                const solver::HQPData & HQPData = invdyn_->computeProblemData(control_time, q_current, qdot_current);
                if (tick  % ((int)Hz / 10) == 0){
					cout << solver::HQPDataToString(HQPData, true) << endl;			
					HQP_flag = false;
				}
				const solver::HQPOutput & sol = solver->solve(HQPData);
				const VectorXd & tau = invdyn_->getActuatorForces(sol);
				desired_torque = tau;

#ifdef time_check
clock_gettime(CLOCK_MONOTONIC, &end); /* mark start time */
diff = BILLION * (end.tv_sec - start.tv_sec) + end.tv_nsec - start.tv_nsec;
if (tick  % ((int)Hz / 10) == 1){
	cout << "milli time" << diff / 1000000.0 << endl;			
	HQP_flag = false;
}
#endif
			}
			else if (ctrl_mode == 3){
				if (mode_change){
					cout << "x + 20 cm for 1 sec" << endl;
					cout << "This mode has joint limit avoidance" << endl;
					cout << "This mode has joint posture control" << endl;

                    start_time = control_time;
				
					// Level 0 : Joint Velocity Limit for Mobile + Manipulator 
					invdyn_ = new HQP::InverseDynamics(*robot_);
					invdyn_->addJointLimitTask(*jointLimitTask, 1.0, 0, 0.0);

					// Level 1: Operational Task Control
					invdyn_->addOperationalTask(*moveTask, w_move, 1, 0.0);
                //	invdyn_->addOperationalTask(*move2Task, w_move, 1, 0.0);

					Transform3d init_T = robot_->getTransformation(7);
					Transform3d goal_T = init_T;
					//goal_T.linear() = rot_temp * goal_T.linear(); 
                    goal_T.translation()(2) += 0.2;
					//trajEE->setReference(goal_T);
					
					trajEECubic = new trajectories::TrajectoryOperationCubic("op_traj");
					trajEECubic->setInitSample(init_T);
					trajEECubic->setGoalSample(goal_T);
                    trajEECubic->setDuration(10.0);
					trajEECubic->setStartTime(start_time);
					trajEECubic->setReference(goal_T);						
					
					trajEEConstant = new trajectories::TrajectoryOperationConstant("op_const");
					trajEEConstant->setReference(goal_T);
					// Level 2: Joint Task Control with low gain
					kp_posture = 300.0;
					jointTask->Kp(kp_posture*VectorXd::Ones(robot_->nv()));
					jointTask->Kd(2.0*jointTask->Kp().cwiseSqrt());

                    //invdyn_->addJointPostureTask(*jointTask, 1.0, 2, 0.0); //weight, level, duration

					qdes.setZero();
					// qdes(1) = -M_PI/4.0;
					// qdes(3) = -M_PI/2.0;
					// qdes(5) = M_PI/4.0;

					trajPostureConstant = new trajectories::TrajectoryJointConstant("joint_traj_const"); 
					trajPostureConstant->setReference(qdes);
					mode_change = false;			
				}
#ifdef time_check
clock_gettime(CLOCK_MONOTONIC, &start); /* mark start time */
#endif
				sampleJoint = trajPostureConstant->computeNext();
				jointTask->setReference(sampleJoint);

                trajEECubic->setCurrentTime(control_time);
				s = trajEECubic->computeNext();
				//cout << s.pos.head(3) <<endl;
				moveTask->setReference(s);
				move2Task->setReference(s);

                const solver::HQPData & HQPData = invdyn_->computeProblemData(control_time, q_current, qdot_current);
				if (HQP_flag){
					cout << solver::HQPDataToString(HQPData, true) << endl;			
					HQP_flag = false;
				}
				const solver::HQPOutput & sol = solver->solve(HQPData);
				const VectorXd & tau = invdyn_->getActuatorForces(sol);
				desired_torque = tau;
#ifdef time_check
clock_gettime(CLOCK_MONOTONIC, &end); /* mark start time */
diff = BILLION * (end.tv_sec - start.tv_sec) + end.tv_nsec - start.tv_nsec;
if (tick  % ((int)Hz / 10) == 1){
	cout << "milli time" << diff / 1000000.0 << endl;			
	HQP_flag = false;
}

#endif

			}
            if ((tick % ((int)Hz / 10)) == 0)
            {
                cout << "des torque:" << endl;
                cout << desired_torque.transpose() << endl;
            }
			for (size_t i = 0; i < DOF; i++)
            {
                ElmoGoldDevice::elmo_gold_rx & rxPDO = elmo.device_[i].rxPDO;
                 rxPDO.modeOfOperation = CyclicSynchronousTorquemode;
                // rxPDO.modeOfOperation = CyclicSynchronousPositionmode;
                //rxPDO.targetPosition = (positionDesiredElmo(i) + OFFSET[i]) * RAD2CNT[i];
                //rxPDO.maxTorque = 0;
                //rxPDO.targetTorque = 0;
                rxPDO.maxTorque = 500;
                rxPDO.targetTorque = desired_torque(i) / RATEDTORQUE[i] * 1000.0;
            }
            elmo.write();

		}		
        else
        {
            elmo.servoOn();

            for (size_t i = 0; i < DOF; i++)
            {
                ElmoGoldDevice::elmo_gold_tx & txPDO = elmo.device_[i].txPDO;
                positionElmo(i) = txPDO.positionActualValue * CNT2RAD[i] - OFFSET[i];
                velocityElmo(i) = txPDO.velocityActualValue * CNT2RAD[i];
                torqueElmo(i) = txPDO.torqueActualValue * RATEDTORQUE[i] / 1000.;
            }
        }

	}
		
}
int main()
{
    // for v-rep
    signal(SIGTERM, catch_signal);
    signal(SIGINT, catch_signal);

    /* rt_print task*/
    rt_print_auto_init(1);

    if(mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        cout << "mlockall failed" << endl;
        return 0;
    }
	
    int err;
     err = rt_task_create(&ethercatTask, "ethc_task", 0, 80, 0);
    if (err)
    {
        cout << "failed to create Task : " << strerror(-err) << endl;
        return 0;
    }
    cout << "Successed to create Task" << endl;
    ethercat_task_flag = true;

    cout << "starting Task" << endl;
    err = rt_task_start(&ethercatTask, &ethercat_task, NULL);
    if (err)
    {
        cout << "failed to start Task : " << strerror(-err) << endl;
        return 0;
    }

    while(!quit_flag)
    {
		if (_kbhit()) {
			int key;
			key = getchar();
			switch (key)
			{
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
                catch_signal(0);
                break;
			default:
                break;

			}
		}
	}

	return 0;
}
