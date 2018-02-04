#include <vector>
#include <memory>
#include <math.h>
#include <limits>

#include "tf/LinearMath/Vector3.h"

#include "vent_planner/VentPlanner.h"

VentPlanner::VentPlanner()
{}

std::shared_ptr<Plan> VentPlanner::plan()
{
	std::shared_ptr<Plan> plan;
	return plan;
}

std::vector<tf::Vector3> VentPlanner::makeSpiral(tf::Vector3 startLocation, double startDirection, double spacing, double size)
{
	std::vector<tf::Vector3> spiral;
	spiral.push_back(startLocation);

	tf::Vector3 location = startLocation;

	const std::vector<double> directions = {startDirection, 
						 	   	  			startDirection + (M_PI / 2), 
						 	   	  			startDirection + M_PI, 
						 	   	  			startDirection + (M_PI * 3 / 2)};

	unsigned int currentDirection = 0;
	unsigned int lengthIndex = 1;

	while(lengthIndex * spacing <= size)
	{
		//Transect1 at transectLength
		location.setX(location.getX() + (cos(directions[currentDirection]) * spacing * lengthIndex));
		location.setY(location.getY() + (sin(directions[currentDirection]) * spacing * lengthIndex));
		spiral.push_back(location);
		currentDirection = (currentDirection + 1) % directions.size();
		
		//Transect2 at transectLength
		location.setX(location.getX() + (cos(directions[currentDirection]) * spacing * lengthIndex));
		location.setY(location.getY() + (sin(directions[currentDirection]) * spacing * lengthIndex));
		spiral.push_back(location);
		currentDirection = (currentDirection + 1) % directions.size();
		
		lengthIndex++;
	}

	//Final transect to finish out the spiral, same transect length as the last segment
	location.setX(location.getX() + (cos(directions[currentDirection]) * spacing * (lengthIndex - 1)));
	location.setY(location.getY() + (sin(directions[currentDirection]) * spacing * (lengthIndex - 1)));
	spiral.push_back(location);

	return spiral;
}

std::vector<tf::Vector3> VentPlanner::makeLawnmower(const tf::Vector3& startLocation,
										   			double alongTrackDirection,
										   			double acrossTrackDirection,
										   			double alongTrackSize,
										   			double acrossTrackSize,
		 								   			double spacing)
{
	std::vector<tf::Vector3> lawnmower;
	lawnmower.push_back(startLocation);

	tf::Vector3 location = startLocation;
	const std::vector<double> directions = {alongTrackDirection, 
						 	   	  			acrossTrackDirection, 
						 	   	  			alongTrackDirection - M_PI, 
						 	   	  			acrossTrackDirection};

	const std::vector<double> distance = {alongTrackSize, 
						 	   	  		  spacing, 
						 	   	  		  alongTrackSize, 
						 	   	  		  spacing};				 	  

	

	unsigned legIndex = 0;
	unsigned trackIndex = 0;
	while(spacing * trackIndex <= acrossTrackSize)
	{
		//Transect1 at transectLength
		location.setX(location.getX() + (cos(directions[legIndex]) * distance[legIndex]));
		location.setY(location.getY() + (sin(directions[legIndex]) * distance[legIndex]));
		lawnmower.push_back(location);

		if(legIndex == 0 || legIndex == 2)
		{
			trackIndex++;
		}
		legIndex = (legIndex + 1) % directions.size();

		
	}

	return lawnmower;
}