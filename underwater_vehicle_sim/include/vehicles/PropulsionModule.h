#ifndef PROPULSION_MODULE_H
#define PROPULSION_MODULE_H

class PropulsionModule
{

public:
	PropulsionModule() {}
	virtual ~PropulsionModule() {}

	virtual tf::StampedTransform move(tf::StampedTransform currentLocation)=0;
};


#endif