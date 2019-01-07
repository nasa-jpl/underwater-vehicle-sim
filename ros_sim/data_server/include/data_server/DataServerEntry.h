#ifndef DATA_SERVER_ENTRY_H
#define DATA_SERVER_ENTRY_H

struct DataServerEntry
{
	double x;
	double y;
	double h;
	double sonarDepth;
	ros::Time time;

	double temp;
	double salt;
	double dye;
	double plumeStrength;

	bool operator<(const DataServerEntry& rhs) const { return time < rhs.time; }
	bool operator==(const DataServerEntry& rhs) const { return time == rhs.time; }
	bool operator>(const DataServerEntry& rhs) const { return time > rhs.time; }
};
	
#endif