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
	std::string ip = "mir.com";	
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
			
			for(int j=0; j<=2; j++)
				mission_map[std::to_string(i*10+j)] = "";
		}

		mission_map["VL1"] = "";
		mission_map["step_forward"] = ""; //move forward 0.5m
		mission_map["topview"] = "";	//rotate counterclockwise by 90
		mission_map["topview_reverse"] = "";	//rotate clockwise by 90

		getMissions();
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

	int goToDoor(int target_chamber_id)
	{
		if( target_chamber_id < 1 || target_chamber_id > 8 ) 
			return -1;

		executeMission( mission_map.at(std::to_string(target_chamber_id)) );
	
		waitTillReachDst();

		return 0;
	};

	int goToChamberPosition(int target_chamber_id, int position_id)
	{
		executeMission( mission_map.at(std::to_string(target_chamber_id*10 + position_id)));
	
		waitTillReachDst();

		return 0;
	};

	int goToChamebrVLMarker(int target_chamber_id)
	{
		std::string mission_str = "VL" + std::to_string(target_chamber_id);
		executeMission(mission_map.at(mission_str));

		waitTillReachDst();

		return 0;
	}

	int moveToTopView()
	{
		executeMission(mission_map.at("topview"));

		waitTillReachDst();

		return 0;
	}

	int reverseTopView()
	{
		executeMission(mission_map.at("topview_reverse"));

		waitTillReachDst();

		return 0;
	}

	int moveToSideView()
	{
		executeMission(mission_map.at("topview_reverse"));

		waitTillReachDst();

		return 0;
	}

	int reverseSideView()
	{
		executeMission(mission_map.at("topview"));

		waitTillReachDst();

		return 0;
	}

	int stepForwardInChamber()
	{
		executeMission(mission_map.at("step_forward"));

		waitTillReachDst();

		return 0;
	}

	int waitTillReachDst()
	{
		bool not_reached_dst = true;
		while(not_reached_dst)
		{
			std::string mission_text;
			getStatus(mission_text);

			//std::cout << mission_text << "\n";

			//Sleep(1000);

			if(mission_text.find("Moving to") != std::string::npos 
				|| mission_text.find("Relative Move") != std::string::npos
				|| mission_text.find("Docking") != std::string::npos
				)
				while(1)
				{
					getStatus(mission_text);
					
					if(mission_text.find("Waiting for new missions") != std::string::npos)
					{
						not_reached_dst = false;
						break;
					}
				}
		}

		//std::cout<<"reached dst"<<std::endl;

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
};

#endif
