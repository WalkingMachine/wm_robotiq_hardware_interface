//
// Created by philippe on 03/05/17.
//

#include "WMDynamixelHardwareInterface.h"

namespace wm_dynamixel_hardware_interface {
// << ---- H I G H   L E V E L   I N T E R F A C E ---- >>
	
	bool WMDynamixelHardwareInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
		using namespace hardware_interface;
		
		// Get parameters
		std::vector<std::string> Joints;
		
		std::string Address;
		if (!robot_hw_nh.getParam("address", Address)) { return false; }
		
		std::string Baud;
		if (!robot_hw_nh.getParam("baudrate", Baud)) { return false; }
		
		std::string ID;
		if (!robot_hw_nh.getParam("id", ID)) { return false; }
		iId = atoi(ID.c_str());
		
		std::string Offset;
		if (!robot_hw_nh.getParam("offset", Offset)) { return false; }
		iOffset = atoi(ID.c_str());
		
		if (!robot_hw_nh.getParam("joints", Joints)) { return false; }
		Name = Joints[0];
		
		//initialise Dynamixel connection
		if(!InitPort(Address.c_str(), atoi(Baud.c_str()))) return false ;
		
		//initialise Dynamixel
		if(!InitDynamixel()) return false ;
		
		// Initialise interface variables
		cmd = 0;
		pos = 0;    //
		vel = 0;    //velocity
		eff = 0;    //effort
		
		// Register interfaces
		joint_state_interface_.registerHandle(JointStateHandle(Name, &pos, &vel, &eff));
		joint_velocity_interface_.registerHandle(JointHandle(joint_state_interface_.getHandle(Name), &cmd));
		registerInterface(&joint_state_interface_);
		registerInterface(&joint_velocity_interface_);
		
		return true;
	}
	
	void WMDynamixelHardwareInterface::read(const ros::Time &time, const ros::Duration &period) {
		//TODO: Match all physical units
		//pos = read2BDynamixel(ADDR_P1_PRESENT_POSITION_2BYTES);
		//vel = read2BDynamixel(ADDR_P1_PRESENT_SPEED_2BYTES);
		//eff = read2BDynamixel(ADDR_P1_PRESENT_LOAD_2BYTES);
	}
	
	void WMDynamixelHardwareInterface::write(const ros::Time &time, const ros::Duration &period) {
		//write2BDynamixel(ADDR_P1_MOVING_SPEED_2BYTES,cmd);
	}

	bool WMDynamixelHardwareInterface::InitPort(const char *PortName, int BaudRate) {
		if (portHandler == NULL || packetHandler == NULL) {
			// Link port
			portHandler = dynamixel::PortHandler::getPortHandler(PortName);

			// Link packet
			packetHandler = dynamixel::PacketHandler::getPacketHandler(1.0);

			// Open port
			if (!portHandler->openPort()) {
				return false;
			}

			// Set port baudrate
			if (!portHandler->setBaudRate(BaudRate)) {
				return false;
			}
		}
		return true;
	}

	bool WMDynamixelHardwareInterface::InitDynamixel() {
		//TODO:
		//set TORQUE to ON
		if(!write1BDynamixel(ADDR_P1_TORQUE_ENABLE,1)) return false;

		//set NORMAL mode
		if(!write2BDynamixel(ADDR_P1_CW_LIMIT_2BYTES,4096)) return false;
		if(!write2BDynamixel(ADDR_P1_CCW_LIMIT_2BYTES,0)) return false;

		//set SPEED to 100
		if(!write2BDynamixel(ADDR_P1_MOVING_SPEED_2BYTES,100)) return false;

		//go to offset position
		if(!write2BDynamixel(ADDR_P1_GOAL_POSITION_2BYTES,iOffset)) return false;

		//set SPEED back to 0
		if(!write2BDynamixel(ADDR_P1_MOVING_SPEED_2BYTES,0)) return false;

		//set WHEEL mode
		if(!write2BDynamixel(ADDR_P1_CW_LIMIT_2BYTES,0)) return false;
		if(!write2BDynamixel(ADDR_P1_CCW_LIMIT_2BYTES,0)) return false;
	}

	bool WMDynamixelHardwareInterface::write1BDynamixel(int iAddress, int iValue){
		int dxl_comm_result;
		uint8_t dxl_error = 0;

		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, iId, iAddress, iValue, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
			return false;
		}
		else if (dxl_error != 0)
		{
			packetHandler->printRxPacketError(dxl_error);
			return false;
		}

		return true;
	}

	bool WMDynamixelHardwareInterface::write2BDynamixel(int iAddress, int iValue){
		int dxl_comm_result;
		uint8_t dxl_error = 0;

		dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, iId, iAddress, iValue, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
			return false;
		}
		else if (dxl_error != 0)
		{
			packetHandler->printRxPacketError(dxl_error);
			return false;
		}
		return true;
	}

	int WMDynamixelHardwareInterface::read1BDynamixel(int iAddress) {
		int dxl_comm_result;
		uint8_t dxl_error = 0;

		uint16_t dxl_present_position;
		// Read present position
		dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, iId, iAddress, &dxl_present_position, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS)
		{
			packetHandler->printTxRxResult(dxl_comm_result);
			return false;
		}
		else if (dxl_error != 0)
		{
			packetHandler->printRxPacketError(dxl_error);
			return false;
		}
		return dxl_present_position;
	}
	
	int WMDynamixelHardwareInterface::read2BDynamixel(int iAddress) {
		int dxl_comm_result;
		uint8_t dxl_error = 0;
		
		uint16_t dxl_present_position;
		// Read present position
		dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, iId, iAddress, &dxl_present_position, &dxl_error);
		if (dxl_comm_result != COMM_SUCCESS) {
			packetHandler->printTxRxResult(dxl_comm_result);
			return false;
		} else if (dxl_error != 0) {
			packetHandler->printRxPacketError(dxl_error);
			return false;
		}
		return dxl_present_position;
	}
}


PLUGINLIB_EXPORT_CLASS(wm_dynamixel_hardware_interface::WMDynamixelHardwareInterface, hardware_interface::RobotHW)