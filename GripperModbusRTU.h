#ifndef GRIPPER_MODBUS_RTU_H_
#define GRIPPER_MODBUS_RTU_H_

#include<modbus.h>
#include<iostream>

struct GripperModbusRTU
{
	
	modbus_t *mb_;
	uint16_t tab_reg_[3];

	GripperModbusRTU();
	~GripperModbusRTU();
	int activate();
	int close();
	int open();
};

#endif // !GRIPPER_MODBUS_RTU_H_

