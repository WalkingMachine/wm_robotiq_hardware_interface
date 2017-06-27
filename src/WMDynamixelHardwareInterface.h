//
// Created by philippe on 03/05/17.
//

#ifndef PROJECT_WMDynamixelHardwareInterface_H
#define PROJECT_WMDynamixelHardwareInterface_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <string>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include "dynamixel_sdk.h"

namespace wm_dynamixel_hardware_interface
{
	class WMDynamixelHardwareInterface : public hardware_interface::RobotHW {
	public:
		// << ---- H I G H   L E V E L   I N T E R F A C E ---- >>
		// Functions
		virtual bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh);
		virtual void read(const ros::Time &time, const ros::Duration &period);
		virtual void write(const ros::Time &time, const ros::Duration &period);
		
		// Interface variables
		std::string Name;
		
		double cmd;
		double pos;
		double vel;
		double eff;
	
	private:
		// Variables
		static hardware_interface::VelocityJointInterface joint_velocity_interface_;
		static hardware_interface::JointStateInterface joint_state_interface_;
		
		// Access to the Dynamixel controller
		static dynamixel::PortHandler *portHandler;
		static dynamixel::PacketHandler *packetHandler;
		
		// Parameters
		int iId;
		int iOffset;
		
		bool InitPort(const char *PortName, int BaudRate);
		bool InitDynamixel();
		
		bool write1BDynamixel(int iAddress, int iValue);
		bool write2BDynamixel(int iAddress, int iValue);
		
		int read1BDynamixel(int iAddress);
		int read2BDynamixel(int iAddress);
		
		
		};
}
#endif //PROJECT_WMDynamixelHardwareInterface_H
