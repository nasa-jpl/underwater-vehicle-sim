#ifndef USBL_MODULE_H
#define USBL_MODULE_H

#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "ros/ros.h"

#include "vehicles/GeneralModule.h"

#include <random>

class USBLModule : public GeneralModule
{

public:
	USBLModule(std::string name);
	~USBLModule() {}

	void update(const ros::Time& lastTime, VehicleState& vehicleState, ocean_models::ModelData& modelData);

private:
	enum Type {Standard, Inverted};
private:
	ros::Publisher usbl;

	/**
	*The orientation of the usbl reciever and transducer
	*/
	Type usblType;

	/**
	*Error in bearing measurment
	*/
	double bearingRandomError;
	double bearingBiasError;
	
	/**
	*Error in range measurment
	*/
	double rangeRandomError;
	double rangeBiasError;

	/**
	 *The distance in which valid bearings can be calculated
	 */
	double validBearingDistance;

	/**
	 *The distance in which valid ranges can be calculated
	 */
	double validRangeDistance;

	/**
	 *Probability of a bad range reading happening.
	 *Simulated indirect acoustic paths
	 */
	double badRangeProbability;

	/**
	 *Potential error of a bad range.
	 */
	double badRangeError;

	double beaconX;
	double beaconY;
	double beaconZ;

	std::default_random_engine generator;
	std::normal_distribution<double> bearingDistribution;
	std::uniform_real_distribution<double> badRangeRandom;
	std::uniform_real_distribution<double> badRangeDistribution;



};

#endif
