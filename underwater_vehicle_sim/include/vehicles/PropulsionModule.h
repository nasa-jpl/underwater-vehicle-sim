#ifndef PROPULSION_MODULE_H
#define PROPULSION_MODULE_H

class PropulsionModule
{

public:
	PropulsionModule() {}
	virtual ~PropulsionModule() {}

	/**
	* Calculates the new frame of the vehicle from the old one
	* @param currentLocaion The old vehicle frame relative to the world frame
	* @return The new vehicle frame relative to the world frame
	*/
	virtual tf::StampedTransform move(tf::StampedTransform currentLocation)=0;
};


#endif