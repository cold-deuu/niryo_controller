#include "rci_qp_controller/math/util.hpp"
#include "rci_qp_controller/robot/robot_wrapper.hpp"
#include "rci_qp_controller/tasks/task_se3_equality.hpp"
#include "rci_qp_controller/tasks/task_joint_posture.hpp"
#include "rci_qp_controller/tasks/task_se3_equality.hpp"
#include "rci_qp_controller/tasks/task_joint_posture.hpp"
#include "rci_qp_controller/trajectory/trajectory_euclidian.hpp"
#include "rci_qp_controller/trajectory/trajectory_se3.hpp"
#include "rci_qp_controller/math/constraint_bound.hpp"
#include "rci_qp_controller/math/constraint_equality.hpp"
#include "rci_qp_controller/math/constraint_inequality.hpp"
#include "controller_manager_msgs/SwitchController.h"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <Eigen/QR>    
#include <Eigen/Core>
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include <stdio.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"
#include <ros/ros.h>
#include <sys/ioctl.h>
#include <termios.h>

//Variable
Eigen::VectorXd q_,v_,q_goal_,q_d_,v_d_,q_d_se_,v_d_se_;
Eigen::MatrixXd J_, Jinv_;
pinocchio::SE3 se3_goal_,a_,b_;
int nq_, mode_;
bool ctrl_init_;
double dt_ ,eps_;

//ros
ros::Publisher pub_;
ros::Subscriber sub_;
ros::ServiceClient srv_;


//void
void initialize();
void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

//Ctrler
void key_event();
void switch_ctrl(bool ref);

//KeyBoard Hit
bool kbhit()
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
};

void joint_publish(Eigen::VectorXd q_d);
