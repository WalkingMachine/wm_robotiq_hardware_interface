//
// Created by philippe on 03/05/17.
//

#include "WMRobotiqHardwareInterface.h"
#include <nodelet/nodelet.h>

namespace wm_robotiq_hardware_interface {
    double *Pos;
    void StatusCB( robotiq_c_model_control::CModel_robot_inputConstPtr msg ){
        *Pos = msg->gPO;
    }

    hardware_interface::PositionJointInterface WMRobotiqHardwareInterface::joint_position_interface_;
    hardware_interface::JointStateInterface    WMRobotiqHardwareInterface::joint_state_interface_;

// << ---- H I G H   L E V E L   I N T E R F A C E ---- >>

    bool WMRobotiqHardwareInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
        using namespace hardware_interface;

        // Get parameters
        std::vector<std::string> Joints;
        if (!robot_hw_nh.getParam("joints", Joints)) { return false; }
        Name = Joints[0];
        if (!robot_hw_nh.getParam("port", port)) { return false; }

        // Initialise interface variables
        cmd = 0;
        pos = 0;
        vel = 0;
        eff = 0;


        // Register interfaces
        joint_state_interface_.registerHandle(JointStateHandle(Name, &pos, &vel, &eff));
        joint_position_interface_.registerHandle(JointHandle(joint_state_interface_.getHandle(Name), &cmd));
        registerInterface(&joint_state_interface_);
        registerInterface(&joint_position_interface_);

        // advertise publisher
        GripperCtrlPub = robot_hw_nh.advertise<robotiq_c_model_control::CModel_robot_output>( "CModelRobotOutput", 1 );
        //GripperStatSub.
        GripperStatSub = robot_hw_nh.subscribe( "CModelRobotInput", 1, StatusCB );
        Pos = &pos;

        return true;
    }

    void WMRobotiqHardwareInterface::read(const ros::Time &time, const ros::Duration &period) {
    }

    void WMRobotiqHardwareInterface::write(const ros::Time &time, const ros::Duration &period) {
        robotiq_c_model_control::CModel_robot_output msg;
        msg.rACT = 1;
        msg.rGTO = 1;
        msg.rSP = 200;
        msg.rFR = 0;
        msg.rPR = cmd;
        //pos = cmd;
        GripperCtrlPub.publish( msg );
    }

}


PLUGINLIB_EXPORT_CLASS( wm_robotiq_hardware_interface::WMRobotiqHardwareInterface, hardware_interface::RobotHW)