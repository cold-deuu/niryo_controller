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
#include <stdio.h>

#include <ros/ros.h>
#include <sys/ioctl.h>
#include <termios.h>

ros::Publisher pub_;
ros::Subscriber sub_;
ros::ServiceClient srv_;

Eigen::VectorXd q_, v_, q_d_, v_d_,q_goal_;
Eigen::MatrixXd Jinv_;

int mode_;
int nq_;
bool ctrl_init_;


void initialize();
void JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
void switch_ctrl(bool controllertype);
void key_event();
void joint_publish(Eigen::VectorXd q_d);

bool kbhit(){
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