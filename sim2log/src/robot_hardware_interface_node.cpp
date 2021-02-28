#include "robot_hardware_interface.h"

MyRobot::MyRobot(ros::NodeHandle& nh) : nh_(nh) {
// Declare all JointHandles, JointInterfaces and JointLimitInterfaces of the robot.
    init();
    
// Create the controller manager
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    
//Set the frequency of the control loop.
    loop_hz_=10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    
//Run the control loop
    my_control_loop_ = nh_.createTimer(update_freq, &MyRobot::update, this);
}

MyRobot::~MyRobot() {
}

void MyRobot::init() {
        
// Create joint_state_interface for shoulder_pan_joint
    hardware_interface::JointStateHandle jointStateHandleA("shoulder_pan_joint", &joint_position_[0], &joint_velocity_[0], &joint_effort_[0]);
    joint_state_interface_.registerHandle(jointStateHandleA);
// Create effort joint interface as shoulder_pan_joint accepts effort command.
    hardware_interface::JointHandle jointEffortHandleA(jointStateHandleA, &joint_effort_command_[0]);
    effort_joint_interface_.registerHandle(jointEffortHandleA); 
/*// Create Joint Limit interface for shoulder_pan_joint
    joint_limits_interface::getJointLimits("shoulder_pan_joint", nh_, limits);
    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandleA(jointEffortHandleA, limits);
    effortJointSaturationInterface.registerHandle(jointLimitsHandleA); */

    
// Create joint_state_interface for shoulder_lift_joint
    hardware_interface::JointStateHandle jointStateHandleB("shoulder_lift_joint", &joint_position_[1], &joint_velocity_[1], &joint_effort_[1]);
    joint_state_interface_.registerHandle(jointStateHandleB);
// Create effort joint interface as shoulder_lift_joint accepts effort command..
    hardware_interface::JointHandle jointEffortHandleB(jointStateHandleB, &joint_effort_command_[1]);
    effort_joint_interface_.registerHandle(jointEffortHandleB);
/*// Create Joint Limit interface for shoulder_lift_joint
    joint_limits_interface::getJointLimits("shoulder_lift_joint", nh_, limits);
    joint_limits_interface::EffortJointSaturationHandle jointLimitsHandleB(jointEffortHandleB, limits);
    effortJointSaturationInterface.registerHandle(jointLimitsHandleB);  */  
    
// Create joint_state_interface for elbow_joint_position
    hardware_interface::JointStateHandle jointStateHandleC("elbow_joint_position", &joint_position_[2], &joint_velocity_[2], &joint_effort_[2]);
    joint_state_interface_.registerHandle(jointStateHandleC);
// Create position joint interface as elbow_joint_position accepts position command.
    hardware_interface::JointHandle jointEffortHandleC(jointStateHandleC, &joint_effort_command_[2]);
    effort_joint_interface_.registerHandle(jointEffortHandleC);
/*// Create Joint Limit interface for elbow_joint_position
    joint_limits_interface::getJointLimits("elbow_joint_position", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleC(jointPositionHandleC, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandleC);    */

// Create joint_state_interface for elbow_joint_position
    hardware_interface::JointStateHandle jointStateHandleD("wrist_1_joint", &joint_position_[3], &joint_velocity_[3], &joint_effort_[3]);
    joint_state_interface_.registerHandle(jointStateHandleD);
// Create position joint interface as elbow_joint_position accepts position command.
    hardware_interface::JointHandle jointEffortHandleD(jointStateHandleD, &joint_effort_command_[3]);
    effort_joint_interface_.registerHandle(jointEffortHandleD);
/*// Create Joint Limit interface for elbow_joint_position
    joint_limits_interface::getJointLimits("wrist_1_joint", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleD(jointPositionHandleD, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandleD);    */

// Create joint_state_interface for elbow_joint_position
    hardware_interface::JointStateHandle jointStateHandleE("wrist_2_joint", &joint_position_[4], &joint_velocity_[4], &joint_effort_[4]);
    joint_state_interface_.registerHandle(jointStateHandleE);
// Create position joint interface as elbow_joint_position accepts position command.
    hardware_interface::JointHandle jointEffortHandleE(jointStateHandleE, &joint_effort_command_[4]);
    effort_joint_interface_.registerHandle(jointEffortHandleE);
/*// Create Joint Limit interface for elbow_joint_position
    joint_limits_interface::getJointLimits("wrist_2_joint", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleE(jointPositionHandleE, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandleE);    */

// Create joint_state_interface for wrist_3_joint
    hardware_interface::JointStateHandle jointStateHandleG("wrist_3_joint", &joint_position_[5], &joint_velocity_[5], &joint_effort_[5]);
    joint_state_interface_.registerHandle(jointStateHandleG);
// Create position joint interface as wrist_3_joint accepts position command.
    hardware_interface::JointHandle jointPositionHandleG(jointStateHandleG, &joint_position_command_);
    position_joint_interface_.registerHandle(jointPositionHandleG);
/*// Create Joint Limit interface for wrist_3_joint
    joint_limits_interface::getJointLimits("wrist_3_joint", nh_, limits);
    joint_limits_interface::PositionJointSaturationHandle jointLimitsHandleG(jointPositionHandleG, limits);
    positionJointSaturationInterface.registerHandle(jointLimitsHandleG);   */

// Register all joints interfaces    
    registerInterface(&joint_state_interface_);
    registerInterface(&effort_joint_interface_);
    registerInterface(&position_joint_interface_);
    //registerInterface(&effortJointSaturationInterface);
    //registerInterface(&positionJointSaturationInterface);    
}

//This is the control loop
void MyRobot::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    //read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    //write(elapsed_time_);
}


/*
void MyRobot::read() {​

  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to get the current joint position and/or velocity and/or effort       

  //from robot.
  // and fill JointStateHandle variables joint_position_[i], joint_velocity_[i] and joint_effort_[i]

}
*/
/*
void MyRobot::write(ros::Duration elapsed_time) {
  // Safety
  //effortJointSaturationInterface.enforceLimits(elapsed_time);   // enforce limits for JointA and JointB
  //positionJointSaturationInterface.enforceLimits(elapsed_time); // enforce limits for JointC


  // Write the protocol (I2C/CAN/ros_serial/ros_industrial)used to send the commands to the robot's actuators.
  // the output commands need to send are joint_effort_command_[0] for JointA, joint_effort_command_[1] for JointB and 

  //joint_position_command_ for last Joint.

}
*/
int main(int argc, char** argv)
{

    //Initialze the ROS node.
    ros::init(argc, argv, "robot_hardware_interface_node");
    ros::NodeHandle nh;
    
    //Separate Sinner thread for the Non-Real time callbacks such as service callbacks to load controllers
    ros::MultiThreadedSpinner spinner(0); 
    
    // Create the object of the robot hardware_interface class and spin the thread. 
    MyRobot ROBOT(nh);
    spinner.spin();
    
    return 0;
}