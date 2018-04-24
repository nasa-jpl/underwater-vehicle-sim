#include <string>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <experimental/filesystem>
#include <iomanip>

#include "ros/ros.h"

#include "data_server/DataServer.h"
#include "data_server/DataServerEntry.h"

namespace fs = std::experimental::filesystem;

DataServer::DataServer(std::string filename)
{
	std::string csvEnding = ".csv";
	if(filename.size() > csvEnding.size() && 
	std::equal(csvEnding.rbegin(), csvEnding.rend(), filename.rbegin()))
	{
		loadFromCSVFile(filename);
	}
	else
	{
		throw DataServer::SaveError("Invalid file type");
	}
}

void DataServer::putData(std::string sourceName, DataServerEntry entry)
{
	//Add a vector if one does not exist in the map for this data source
	if(!data.count(sourceName))
	{
		data.emplace(sourceName, std::vector<DataServerEntry>());
	}

	std::vector<DataServerEntry>& dataEntries = data[sourceName];

	//Add the entry if it goes at the end of the list
	if(dataEntries.size() == 0 || entry.time >= dataEntries[dataEntries.size() - 1].time)
	{
		dataEntries.push_back(entry);
	}
}

std::vector<DataServerEntry>::iterator DataServer::getStartTime(std::string sourceName, ros::Time time)
{
	//Check to see if sourceName is in the map
	if(!data.count(sourceName))
	{
		throw DataServer::MissingKey(sourceName);
	}
	std::vector<DataServerEntry>& dataList = data[sourceName];

	//Search for time
	DataServerEntry searchEntry;
	searchEntry.time = time;
	return std::lower_bound(dataList.begin(), dataList.end(), searchEntry);
}

std::vector<DataServerEntry>::iterator DataServer::getEndTime(std::string sourceName, ros::Time time)
{
	//Check to see if sourceName is in the map
	if(!data.count(sourceName))
	{
		throw DataServer::MissingKey(sourceName);
	}

	std::vector<DataServerEntry>& dataList = data[sourceName];

	//Search for time
	DataServerEntry searchEntry;
	searchEntry.time = time;
	return std::upper_bound(dataList.begin(), dataList.end(), searchEntry);
}

const DataServerEntry& DataServer::getLatestData(std::string sourceName)
{
	std::vector<DataServerEntry>& dataList = data[sourceName];

	return dataList[dataList.size() - 1];
}

unsigned int DataServer::size(std::string sourceName)
{
	return data[sourceName].size();
}

void DataServer::saveToFile(std::string filename)
{
	std::string csvEnding = ".csv";
	if(filename.size() > csvEnding.size() && 
	   std::equal(csvEnding.rbegin(), csvEnding.rend(), filename.rbegin()))
	{
		saveToCSVFile(filename);
	}
	else
	{
		throw DataServer::SaveError("Invalid file type");
	}
}


void DataServer::loadFromCSVFile(std::string filename) 
{
	data.clear();

	std::string line;
	std::ifstream file;
	file.open(filename);
	if (file.is_open())
  	{
  		std::string firstLine = "";
    	while(getline(file,line))
    	{
	    	if(firstLine == "")
	    	{
	    		firstLine = line;
	    	}
	    	else
	    	{
	    		std::vector<std::string> splitLine;

	    		//split file on commas
	    		size_t pos = 0;
	    		std::string delimiter = ",";
				std::string token;
				while ((pos = line.find(delimiter)) != std::string::npos) {
				    token = line.substr(0, pos);
				    splitLine.push_back(token);
				    line.erase(0, pos + delimiter.length());
				}
				splitLine.push_back(line);

				DataServerEntry newEntry;
				newEntry.x = std::stof(splitLine[1]);
				newEntry.y = std::stof(splitLine[2]);
				newEntry.h = std::stof(splitLine[3]);
                newEntry.sonarDepth = std::stof(splitLine[4]);
				newEntry.time = ros::Time(std::stof(splitLine[5]));

				newEntry.temp = std::stof(splitLine[6]);
				newEntry.salt = std::stof(splitLine[7]);
				newEntry.dye = std::stof(splitLine[8]);
				newEntry.plumeStrength = std::stof(splitLine[9]);
				putData(splitLine[0], newEntry);

	    	}
    	}
		file.close();
	}
	else
  	{
  		throw DataServer::SaveError("Could not open file: " + filename);
  	}
}


void DataServer::saveToCSVFile(std::string filename)
{
	//Create the directories if they do not exist yet
	fs::path p(filename);
	fs::create_directories(p.parent_path());

	std::ofstream file (filename);
  	if (file.is_open())
  	{
  		file << "source,x,y,h,sonarDepth,time,temp,salt,dye,plumeStrength\n";

  		for ( auto it = data.begin(); it != data.end(); ++it )
  		{
            unsigned long endIndex = it->second.size();
  			for(unsigned long i = 0; i < endIndex; i++)
  			{
  				file << it->first << ",";
  				file << std::setprecision(9) <<  it->second[i].x << "," << it->second[i].y << "," << it->second[i].h << "," << it->second[i].sonarDepth << "," << it->second[i].time.toSec() << ",";
  				file << std::setprecision(9) << it->second[i].temp << ",";
                file << std::setprecision(9) << it->second[i].salt << "," << it->second[i].dye << ",";
                file << std::setprecision(9) << it->second[i].plumeStrength << "\n";
  			}
  		}

    	file.close();
  	}
  	else
  	{
  		throw DataServer::SaveError("Could not open file: " + filename);
  	}
}