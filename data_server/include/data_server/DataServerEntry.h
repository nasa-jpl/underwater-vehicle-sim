#ifndef DATA_SERVER_ENTRY_H
#define DATA_SERVER_ENTRY_H

struct DataServerEntry
{
	float x;
	float y;
	float h;
	float sonarDepth;
	ros::Time time;

	float temp;
	float salt;
	float dye;
	float plumeStrength;

	bool operator<(const DataServerEntry& rhs) const { return time < rhs.time; }
	bool operator==(const DataServerEntry& rhs) const { return time == rhs.time; }
	bool operator>(const DataServerEntry& rhs) const { return time > rhs.time; }
};
	
#endif