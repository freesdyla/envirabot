#ifndef MIR_CLIENT_H
#define MIR_CLIENT_H

#include <curl/curl.h>
#include <string>
#include <iostream>
#include <memory>
#include <json/json.h>
#include <set>
#include <map>

struct MirClient 
{
	std::string ip = "192.168.12.20";	
	std::map<std::string, std::string> mission_map;
	
	struct curl_slist *slist_get, *slist_putpost;
	
	CURLcode ret;
	CURL *hnd;

	MirClient()
	{
		slist_get = NULL;
		slist_get = curl_slist_append(slist_get, "accept: application/json");
		slist_get = curl_slist_append(slist_get, "Authorization: Basic RGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA==");
		slist_get = curl_slist_append(slist_get, "Accept-Language: en_US");
		
		slist_putpost = NULL;
	
		slist_putpost = curl_slist_append(slist_putpost, "accept: application/json");
		slist_putpost = curl_slist_append(slist_putpost, "Authorization: Basic RGlzdHJpYnV0b3I6NjJmMmYwZjFlZmYxMGQzMTUyYzk1ZjZmMDU5NjU3NmU0ODJiYjhlNDQ4MDY0MzNmNGNmOTI5NzkyODM0YjAxNA==");
		slist_putpost = curl_slist_append(slist_putpost, "Accept-Language: en_US");
		slist_putpost = curl_slist_append(slist_putpost, "Content-Type: application/json");

		hnd = curl_easy_init();
		curl_easy_setopt(hnd, CURLOPT_NOPROGRESS, 1L);
		curl_easy_setopt(hnd, CURLOPT_USERAGENT, "curl/7.54.0");
		curl_easy_setopt(hnd, CURLOPT_MAXREDIRS, 50L);
		curl_easy_setopt(hnd, CURLOPT_HTTP_VERSION, (long)CURL_HTTP_VERSION_2TLS);
		curl_easy_setopt(hnd, CURLOPT_TCP_KEEPALIVE, 1L);
		curl_easy_setopt(hnd, CURLOPT_WRITEFUNCTION, callback);

		mission_map["0"] = "";	//home/charging station

		for(int i=1; i<=8; i++)
		{
			mission_map[std::to_string(i)] = "";
			
			std::string tmp = "VL" + std::to_string(i);
			mission_map[tmp] = "";

			tmp = std::to_string(i) + "1";
			mission_map[tmp] = "";

			tmp = std::to_string(i) + "2";
			mission_map[tmp] = "";
		}

		mission_map["step_forward"] = ""; //move forward by 0.5m in chamber
		mission_map["step_forward_1m"] = "";
		mission_map["step_back"] = ""; //move backward by 0.5m in chamber
		mission_map["step_back_75cm"] = "";
		mission_map["step_back_25cm"] = "";
		mission_map["topview"] = "";	//rotate counterclockwise by 90
		mission_map["topview_reverse"] = "";	//rotate clockwise by 90
		mission_map["180"] = "";	//rotate by 180

		getMissions();
		clearError();
		continueMissionQueue();
	};

	~MirClient()
	{
		curl_easy_cleanup(hnd);
		hnd = NULL;
		curl_slist_free_all(slist_get);
		curl_slist_free_all(slist_putpost);
		slist_get = NULL;
		slist_putpost = NULL;
	};

	static std::size_t callback(const char* in, std::size_t size, std::size_t num, std::string* out)
    {
        	const std::size_t totalBytes(size * num);
        	out->append(in, totalBytes);
        	return totalBytes;
    }

	int goToChargingStation()
	{
		std::string mission_text;
		getStatus(mission_text);

		// if robot is not charging, go to charger
		if (mission_text.find("Charging...") == std::string::npos)
		{
			executeMission(mission_map.at("0"));
			return waitTillReachDst();
		}

		return 0;
	}

	int goToDoor(int target_chamber_id)
	{
		if( target_chamber_id < 1 || target_chamber_id > 8 ) 
			return -1;

		executeMission( mission_map.at(std::to_string(target_chamber_id)) );
	
		return waitTillReachDst();
	};

	int goToChamberPosition(int target_chamber_id, int position_id)
	{
		executeMission( mission_map.at(std::to_string(target_chamber_id*10 + position_id)));
	
		return waitTillReachDst();
	};

	int goToChamebrVLMarker(int target_chamber_id)
	{
		//if(target_chamber_id == 2) 
			//goToChamberPosition(target_chamber_id, 1); // there is a problem detecting VL marker

		std::string mission_str = "VL" + std::to_string(target_chamber_id);
		executeMission(mission_map.at(mission_str));

		return waitTillReachDst();
	}

	int moveToTopView()
	{
		executeMission(mission_map.at("topview"));

		return waitTillReachDst();
	}

	int reverseTopView()
	{
		executeMission(mission_map.at("topview_reverse"));

		return waitTillReachDst();
	}

	int moveToSideView()
	{
		executeMission(mission_map.at("topview_reverse"));

		return waitTillReachDst();
	}

	int reverseSideView()
	{
		executeMission(mission_map.at("topview"));

		return waitTillReachDst();
	}

	int stepForwardInChamber()
	{
		executeMission(mission_map.at("step_forward"));

		return waitTillReachDst();
	}

	int stepForward1MInChamber()
	{
		executeMission(mission_map.at("step_forward_1m"));

		return waitTillReachDst();
	}

	int stepBackwardInChamber()
	{
		executeMission(mission_map.at("step_back"));

		return waitTillReachDst();
	}

	int stepBackward75CMInChamber()
	{
		executeMission(mission_map.at("step_back_75cm"));

		return waitTillReachDst();
	}

	int stepBackward25CMInChamber()
	{
		executeMission(mission_map.at("step_back_25cm"));

		return waitTillReachDst();
	}

	int turn180()
	{
		executeMission(mission_map.at("180"));

		return waitTillReachDst();
	}

	int waitTillReachDst()
	{
		bool not_reached_dst = true;

		int timeout_cnt = 0;
		
		while(not_reached_dst)
		{
			std::string mission_text;
			getStatus(mission_text);

			//std::cout << mission_text << "\n";

			if(mission_text.find("Moving to") != std::string::npos 
				|| mission_text.find("Relative Move") != std::string::npos
				|| mission_text.find("Docking") != std::string::npos
				|| mission_text.find("MoveAction") != std::string::npos
				)
				while(1)
				{
					getStatus(mission_text);

					//std::cout << mission_text <<"  "<< timeout_cnt << "\n";
					
					if(mission_text.find("Waiting for new mission") != std::string::npos)
					{
						not_reached_dst = false;
						break;
					}
					else if(mission_text.find("Failed to reach goal position") != std::string::npos)
					{
						Utilities::to_log_file("mir failed to reach goal position");
						return -1;
						exit(0);
					}

					Sleep(100);

					if (timeout_cnt++ > 10 * 60 * 1)
					{
						Utilities::to_log_file("lost connection to mir");
						return -1;
						exit(0);
					}
				}

			Sleep(10);

			if(timeout_cnt++ > 100*60*1)
			{
				getStatus(mission_text);
				
				Utilities::to_log_file(mission_text);
				return -1;
				exit(0);
			}
		}

		return 0;
	};

	int getMissions()
	{
		std::unique_ptr<std::string> httpData(new std::string());

		std::string url_str = "http://" + ip + "/api/v2.0.0/missions";
		curl_easy_setopt(hnd, CURLOPT_URL, url_str.c_str());
		curl_easy_setopt(hnd, CURLOPT_HTTPHEADER, slist_get);
		curl_easy_setopt(hnd, CURLOPT_CUSTOMREQUEST, "GET");
		curl_easy_setopt(hnd, CURLOPT_WRITEDATA, httpData.get());

		ret = curl_easy_perform(hnd);

		Json::Value jsonData;
		Json::Reader jsonReader;

		if (jsonReader.parse(*httpData.get(), jsonData))
		{
			for(int i=0; i<jsonData.size(); i++)
			{
				std::string mission_name = jsonData[i]["name"].asString();
				auto it = mission_map.find(mission_name);
				if(it != mission_map.end())
					it->second = jsonData[i]["guid"].asString();
			}
		}

		//for(auto &m : mission_map) std::cout<<m.first<<" "<<m.second<<std::endl;

		return (int)ret;
	};

	int executeMission(std::string mission_id)
	{
  		std::unique_ptr<std::string> httpData(new std::string());

		std::string url_str = "http://" + ip + "/api/v2.0.0/mission_queue";
		curl_easy_setopt(hnd, CURLOPT_URL, url_str.c_str());
		std::string post_fields_str =  "{ \"mission_id\": \"" + mission_id + "\"}";
		curl_easy_setopt(hnd, CURLOPT_POSTFIELDS, post_fields_str.c_str());
		curl_easy_setopt(hnd, CURLOPT_HTTPHEADER, slist_putpost);
		curl_easy_setopt(hnd, CURLOPT_CUSTOMREQUEST, "POST");
		curl_easy_setopt(hnd, CURLOPT_WRITEDATA, httpData.get());

		ret = curl_easy_perform(hnd);

		return (int)ret;
	};

	int getStatus(std::string & mission_text)
	{
		std::unique_ptr<std::string> httpData(new std::string());
		std::string url_str = "http://" + ip + "/api/v2.0.0/status";
		curl_easy_setopt(hnd, CURLOPT_URL, url_str.c_str());
		curl_easy_setopt(hnd, CURLOPT_HTTPHEADER, slist_get);
		curl_easy_setopt(hnd, CURLOPT_CUSTOMREQUEST, "GET");
		curl_easy_setopt(hnd, CURLOPT_WRITEDATA, httpData.get());

		ret = curl_easy_perform(hnd);

		Json::Value jsonData;
		Json::Reader jsonReader;

		if (jsonReader.parse(*httpData.get(), jsonData))
		{
		//	std::cout << jsonData.toStyledString() << std::endl;
			mission_text = jsonData["mission_text"].asString();
			//std::string x_str = jsonData["Position"]["x"].asString();
			//x = std::stod(x_str);
		//	std::cout<<jsonData["mission_text"].asString();
		}

		return (int)ret;
	};

	int continueMissionQueue()
	{
		std::unique_ptr<std::string> httpData(new std::string());

		std::string url_str = "http://" + ip + "/api/v2.0.0/status";
		curl_easy_setopt(hnd, CURLOPT_URL, url_str.c_str());
		std::string post_fields_str =  "{ \"state_id\": 3}";
		curl_easy_setopt(hnd, CURLOPT_POSTFIELDS, post_fields_str.c_str());
		curl_easy_setopt(hnd, CURLOPT_HTTPHEADER, slist_putpost);
		curl_easy_setopt(hnd, CURLOPT_CUSTOMREQUEST, "PUT");
		curl_easy_setopt(hnd, CURLOPT_WRITEDATA, httpData.get());

		ret = curl_easy_perform(hnd);

		return (int)ret;
	};

	int clearError()
	{
		std::unique_ptr<std::string> httpData(new std::string());

		std::string url_str = "http://" + ip + "/api/v2.0.0/status";
		curl_easy_setopt(hnd, CURLOPT_URL, url_str.c_str());
		std::string post_fields_str = "{ \"clear_error\": true}";
		curl_easy_setopt(hnd, CURLOPT_POSTFIELDS, post_fields_str.c_str());
		curl_easy_setopt(hnd, CURLOPT_HTTPHEADER, slist_putpost);
		curl_easy_setopt(hnd, CURLOPT_CUSTOMREQUEST, "PUT");
		curl_easy_setopt(hnd, CURLOPT_WRITEDATA, httpData.get());

		ret = curl_easy_perform(hnd);

		return (int)ret;
	};
};

#endif
