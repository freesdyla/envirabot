#include "GripperModbusRTU.h"

GripperModbusRTU::GripperModbusRTU()
{
	std::memset(tab_reg_, 0, 6 * sizeof(uint16_t));
}

GripperModbusRTU::~GripperModbusRTU()
{
	modbus_free(mb_);
}

int GripperModbusRTU::activate()
{
	mb_ = modbus_new_rtu("COM7", 115200, 'N', 8, 1);
	if (mb_ == NULL)
	{
		std::cout << "new rtu error\n";
		return -1;
	}

	int rc = modbus_set_slave(mb_, 9);

	if (rc == -1)
	{
		std::cout << "invalid slave id\n";
		modbus_free(mb_);
		return -1;
	}

	rc = modbus_connect(mb_);

	if (rc == -1)
	{
		std::cout << "connect fail\n";
		modbus_free(mb_);
		return -1;
	}

	//clear ACT
	std::memset(tab_reg_, 0, 6 * sizeof(uint16_t));

	int num_bytes = modbus_write_registers(mb_, 0x03E8, 3, tab_reg_);

	if (num_bytes != 3)
	{
		std::cout << "clear act fail\n";
		return -1;
	}

	//activate
	tab_reg_[0] = 0x0100;

	num_bytes = modbus_write_registers(mb_, 0x03E8, 3, tab_reg_);

	if (num_bytes != 3)
	{
		std::cout << "active fail\n";
		return -1;
	}

	uint16_t reg_val;

	while (true)
	{
		num_bytes = modbus_read_registers(mb_, 0x07D0, 1, &reg_val);

		if (reg_val == 0x3100) break;

		Sleep(100);
	}

	return 0;
}

int GripperModbusRTU::close()
{
	if (mb_ == NULL)
	{
		std::cout << "not activated\n";
		return -1;
	}

	tab_reg_[0] = 0x0900;
	tab_reg_[1] = 0x00FF;	//full close 255
	tab_reg_[2] = 0x1000;	//speed, force

	int num_bytes = modbus_write_registers(mb_, 0x03E8, 3, tab_reg_);

	uint16_t reg_val[3];

	while (true)
	{
		num_bytes = modbus_read_registers(mb_, 0x07D0, 3, reg_val);

		if (reg_val[0] == 0xF900 /*&& reg_val[1] == 0x00FF && reg_val[2] == 0xBD00*/) break;

		//std::cout << std::hex <<reg_val[0] << " " << reg_val[1] << " " << reg_val[2] << "\n";

		Sleep(100);
	}

	return 0;
}

int GripperModbusRTU::open()
{
	if (mb_ == NULL)
	{
		std::cout << "not activated\n";
		return -1;
	}

	tab_reg_[0] = 0x0900;
	tab_reg_[1] = 140;//0x00A0;	//ls byte is position
	tab_reg_[2] = 0x1000;	//speed,force

	int num_bytes = modbus_write_registers(mb_, 0x03E8, 3, tab_reg_);

	uint16_t reg_val[3];

	while (true)
	{
		num_bytes = modbus_read_registers(mb_, 0x07D0, 3, reg_val);

		if (reg_val[0] == 0xF900 /*&& reg_val[1] == 0x00FF && reg_val[2] == 0xBD00*/) break;

		//std::cout << std::hex << reg_val[0] << " " << reg_val[1] << " " << reg_val[2] << "\n";

		Sleep(100);
	}

	return 0;
}