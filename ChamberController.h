#ifndef CHAMBER_CONTROLLER_H
#define CHAMBER_CONTROLLER_H

#include <iostream>
#include "boost/asio.hpp"
#include "utilities.h"


enum ChamberCommand
{
	open_door, close_door, open_curtain, close_curtain, pause_light, resume_light
};



struct ChamberController 
{
	char recv_buffer[128];
	boost::asio::ip::udp::endpoint remote_endpoint;

	ChamberController()
	{
		remote_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string("10.25.215.211"), 10000);
		//socket.open();	
	}

	int chamberControl(int chamber_id, ChamberCommand action)
	{
		boost::asio::io_service io_service;
		boost::asio::ip::udp::socket socket(io_service);
		
		socket.open(boost::asio::ip::udp::v4());
		

		boost::system::error_code err;

		std::string command;

		if (action == ChamberCommand::open_door)
			command = "open door ";
		else if (action == ChamberCommand::close_door)
			command = "close door ";
/*		else if (action == OPEN_CURTAIN)
			command = "open curtain ";
		else if (action == CLOSE_CURTAIN)
			command = "close curtain ";
		else if (action == PAUSE_LIGHT)
			command = "pause lighting ";
		else if (action == RESUME_LIGHT)
			command = "resume lighting ";
*/
		command += std::to_string(chamber_id) + " rover";

/*		if (action == DIM_LIGHT)
			command = "dim-lighting " + std::to_string(chamber_id) + " 1 rover";
*/

		int send_success = 0;
		for (int try_id = 0; try_id < 3 && send_success == 0; try_id++)
		{
			socket.send_to(boost::asio::buffer(command, command.size()), remote_endpoint, 0, err);

			for (int i = 0; i < 6; i++)
			{
				int num_bytes = socket.available();

				if (num_bytes > 0)
				{
					int num_bytes = socket.receive_from(boost::asio::buffer(recv_buffer, 1024), remote_endpoint, 0, err);	//blocking read
					std::string result(recv_buffer, recv_buffer+num_bytes);
					std::cout << num_bytes<<"  "<<result << std::endl;
					send_success = 1;
					break;
				}
				else
				{
					//no response from server
					if (i == 5)
					{
						Utilities::to_log_file(command);
					}
				}

				Sleep(1000);
			}

			Sleep(3000);
		}
		

		socket.close();
		return send_success;
	}


};


#endif