#ifndef DATA_SERVER_H
#define DATA_SERVER_H

#include <string>
#include <unordered_map>
#include <vector>
#include <exception>

#include "ros/ros.h"

class DataServer
{

public:

	struct MissingKey : public std::exception
	{
		std::string key;

		MissingKey(std::string key) :
		key(key)
		{}

		const char * what () const throw ()
    	{
    		return (key + " not found in DataServer.").c_str();
    	}
	};

	struct SaveError : public std::exception
	{
		std::string msg;

		SaveError(std::string msg) :
		msg(msg)
		{}

		const char * what () const throw ()
    	{
    		return msg.c_str();
    	}
	};

	struct DataServerEntry
	{
		float x;
		float y;
		float h;
		ros::Time time;

		float temp;
		float salt;
		float dye;

		bool operator<(const DataServerEntry& rhs) const { return time < rhs.time; }
		bool operator==(const DataServerEntry& rhs) const { return time == rhs.time; }
		bool operator>(const DataServerEntry& rhs) const { return time > rhs.time; }
	};

	/**
	 *Constructor that loads data from the datafile into the DataServer object
	 * @param filename File to load
	 **/
	DataServer(std::string filename);

	DataServer() {}
	~DataServer() {}

	/**
	* Gets an iterator for the first data entry in a list larger than or equal to time
	* @param sourceName List to get the iterator for
	* @param time
	* @return Iterator for the vector at the specified time
	*/
	std::vector<DataServerEntry>::iterator getStartTime(std::string sourceName, ros::Time time);

	/**
	* Gets an iterator for the first data entry in a list larger than time
	* @param sourceName List to get the iterator for
	* @param time
	* @return Iterator for the vector at the specified time
	*/
	std::vector<DataServerEntry>::iterator getEndTime(std::string sourceName, ros::Time time);

	/**
	*Puts data in a list in the data server
	*@param sourceName List to put the data in
	*@param entry Data to put in the list
	*/
	void putData(std::string sourceName, DataServerEntry entry);

	/**
	 *Saves the data to a file, file type depends on the provided file extension
	 *@param filename Filename for the saved file
	 */
	void saveToFile(std::string filename);


private:
	/**
	 *Saves the data to a csv file
	 *@param filename Filename for the saved file
	 */
	void saveToCSVFile(std::string filename);

	/**
	 *load the data from a csv file
	 *@param filename Filename to load the data from
	 */
	void loadFromCSVFile(std::string filename);

private:

	std::unordered_map<std::string, std::vector<DataServerEntry>> data;
};


#endif

