#include "vent_planner/util/CreatePathUtil.h"

std::vector<tf::Vector3> create_path_util::makeSpiral(tf::Vector3 startLocation, double startDirection, double spacing, double size)
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
        location.setZ(startLocation.getZ());
        spiral.push_back(location);
        currentDirection = (currentDirection + 1) % directions.size();
        
        //Transect2 at transectLength
        location.setX(location.getX() + (cos(directions[currentDirection]) * spacing * lengthIndex));
        location.setY(location.getY() + (sin(directions[currentDirection]) * spacing * lengthIndex));
        location.setZ(startLocation.getZ());
        spiral.push_back(location);
        currentDirection = (currentDirection + 1) % directions.size();
        
        lengthIndex++;
    }

    //Final transect to finish out the spiral, same transect length as the last segment
    location.setX(location.getX() + (cos(directions[currentDirection]) * spacing * (lengthIndex - 1)));
    location.setY(location.getY() + (sin(directions[currentDirection]) * spacing * (lengthIndex - 1)));
    location.setZ(startLocation.getZ());
    spiral.push_back(location);

    return spiral;
}

std::vector<tf::Vector3> create_path_util::makeLawnmower(const tf::Vector3& startLocation,
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
        location.setZ(startLocation.getZ());
        lawnmower.push_back(location);

        if(legIndex == 0 || legIndex == 2)
        {
            trackIndex++;
        }
        legIndex = (legIndex + 1) % directions.size();

        
    }

    return lawnmower;
}

std::vector<tf::Vector3> create_path_util::makePolygon(const tf::Vector3& center,
                                                       const unsigned int sides,
                                                       const double radius,
                                                       const double initialPointHeading,
                                                       const bool clockwise)
{
    std::vector<tf::Vector3> polygon;

    double angleInterval = (2 * M_PI) / sides;
    double currentPointHeading = initialPointHeading;
    //Add 1 to sides so start and end points are the same
    for(unsigned int i = 0; i < sides + 1; i++)
    {
        //sin and cos are reveresed (x = sin(theta), y = cos(theta))
        //becuase this is a heading with north up (+y)
        double xOffset = radius * sin(currentPointHeading);
        double yOffset = radius * cos(currentPointHeading);

        tf::Vector3 point;
        point.setX(center.getX() + xOffset);
        point.setY(center.getY() + yOffset);
        point.setZ(center.getZ());
        polygon.push_back(point);

        if(clockwise)
        {
            currentPointHeading += angleInterval;
        }
        else
        {
            currentPointHeading -= angleInterval;
        }
        
    }

    return polygon;
}