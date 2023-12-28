#include "niryo_control/test.hpp"
#include <pinocchio/algorithm/compute-all-terms.hpp>

using namespace qp_controller::math;
using namespace qp_controller::robot;
using namespace qp_controller::trajectory;
using namespace qp_controller::tasks;

using namespace Eigen;
using namespace pinocchio;
using namespace std;

int main (int argc, char** argv)
{
    //Initial Setting
    ros::init(argc,argv,"niryo_control");
    ros::NodeHandle nh;
    ros::Rate r(100);

 
    //RobotWrapper
    const string ur_model_path = "/home/chan/niryo_ws/src/niryo_robot_description/";
    vector<string> package_dirs;
    package_dirs.push_back(ur_model_path);
    string urdfFileName = package_dirs[0] + "urdf/one/niryo_grip.urdf";
    RobotWrapper robot(urdfFileName, package_dirs, false);
    const Model &model = robot.model();
    Data data(robot.model());

    r.sleep();
    
    //Pub and Sub
    pub_ = nh.advertise<std_msgs::Float64MultiArray>("/position_controller/command",5);
    sub_ = nh.subscribe("joint_states",5,&JointStateCallback, ros::TransportHints().tcpNoDelay(true));
    srv_ = nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
    
    
    
    //Robot State
    nq_ = robot.nv();
    initialize();
    
    //Set Goal
    q_goal_ << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

    //Joint_Taks
    TaskJointPosture task("task-posture", robot);
    VectorXd Kp = 400. * VectorXd::Ones(nq_);
    VectorXd Kd = 40. * VectorXd::Ones(nq_);
    task.Kp(Kp);
    task.Kd(Kd);    
    TrajectoryEuclidianCubic *traj = new TrajectoryEuclidianCubic("traj_joint");
    TrajectorySample sample;

    //SE3 Task
    TaskSE3Equality task_se3("task-se3", robot, "hand_link");
    task_se3.Kp(Kp);
    task_se3.Kd(Kd);

    se3_goal_ = robot.framePosition(data, model.getFrameId("hand_link"));
    se3_goal_.translation()(2) -= 0.1;
    TrajectorySE3Cubic *traj_se3 = new TrajectorySE3Cubic("traj_SE3");


    while(ros::ok()){
        key_event();
        if (mode_ == 1){
            if(ctrl_init_){
                switch_ctrl(true);
                
                traj->setInitSample(q_);
                traj->setDuration(3.0);
                traj->setStartTime(ros::Time::now().toSec());
                traj->setGoalSample(q_goal_);

                ctrl_init_ = false;
            }
            ROS_WARN_STREAM(q_);
            ROS_INFO_STREAM(v_);
            pinocchio::computeAllTerms(model,data, q_, v_);
            cout<<ros::Time::now()<<endl;
            traj->setCurrentTime(ros::Time::now().toSec());

            sample = traj->computeNext();
            task.setReference(sample);
            auto &constraint = task.compute(ros::Time::now().toSec(), q_, v_, data);

            Jinv_ = constraint.matrix().completeOrthogonalDecomposition().pseudoInverse();
            cout<<"a"<<endl;

            ConstRefVector dv = Jinv_ * constraint.vector();
            v_d_ = dt_ * dv;
            q_d_ = pinocchio::integrate(robot.model(), q_, dt_ * v_d_);

            joint_publish(q_d_);

        }

        else if(mode_ == 2){
            if(ctrl_init_){
                switch_ctrl(true);
                traj_se3->setInitSample(robot.framePosition(data, model.getFrameId("hand_link")));
                traj_se3->setDuration(1.0);
                traj_se3->setStartTime(ros::Time::now().toSec());
                traj_se3->setGoalSample(se3_goal_);

                ctrl_init_ = false;
            }

            pinocchio::computeAllTerms(model,data, q_, v_);
            traj_se3->setCurrentTime(ros::Time::now().toSec());

            sample = traj_se3->computeNext();
            task_se3.setReference(sample);
            auto &constraint_se3 = task_se3.compute(ros::Time::now().toSec(), q_, v_, data);
            Jinv_ = constraint_se3.matrix().completeOrthogonalDecomposition().pseudoInverse();

            ConstRefVector dv = Jinv_ * constraint_se3.vector();
            v_d_se_ = dt_ * dv;
            q_d_se_= pinocchio::integrate(robot.model(), q_, dt_ * v_d_se_);
            joint_publish(q_d_se_);
        }
        else if(mode_ == 3){
            if(ctrl_init_){
                switch_ctrl(false);
                
                ctrl_init_ = false;
            }
        }

        r.sleep();
        ros::spinOnce();
    }
        
    
    return 0;
}
void initialize(){
    q_.resize(nq_);
    v_.resize(nq_);

    q_.setZero();
    v_.setZero();

    q_goal_ = q_;
    q_d_ = q_;
    q_d_se_ = q_;
    v_d_ = v_;
    v_d_se_ = v_;
    Jinv_.resize(nq_,nq_);
    
    eps_ = 1e-5;
    dt_ = 0.01;
    mode_ = 100;
    ctrl_init_ = false;
}
void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (int i; i<6; i++){
        q_(i) = msg->position[i];
        v_(i) = msg->velocity[i];
        }
    
}

void switch_ctrl(bool ref){
    controller_manager_msgs::SwitchController switch_msg;

    if(ref)
    {
        switch_msg.request.start_controllers.push_back("position_controller");
        switch_msg.request.stop_controllers.push_back("arm_controller");

    }
    else
    {
        switch_msg.request.start_controllers.push_back("arm_controller");
        switch_msg.request.stop_controllers.push_back("position_controller");

    }
    switch_msg.request.strictness = 1;
    switch_msg.request.start_asap = true;
    switch_msg.request.timeout = 2.;
    srv_.call(switch_msg);
}

void key_event(){
    if (kbhit()){
        int key = getchar();

        switch(key){
            case 'h':
                cout<<""<<endl;
                cout<<"Joint Posture"<<endl;
                mode_ = 1;
                ctrl_init_ = true;
                break;

            case 'g':
                cout<<""<<endl;
                cout<<"SE3 "<<endl;
                mode_ = 2;
                ctrl_init_ = true;            
                break;

            case 'm':
                cout<<""<<endl;
                cout<<"moveit"<<endl;
                mode_ = 3;
                ctrl_init_ = true;  
                break;
        };
    }
        
};

void joint_publish(Eigen::VectorXd q_d){
    std_msgs::Float64MultiArray msg;
    for (int i=0; i<nq_; i++){
        msg.data.push_back(q_d(i));
    }
    pub_.publish(msg);
}
