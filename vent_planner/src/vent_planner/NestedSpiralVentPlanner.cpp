#include <vector>
#include <memory>
#include <math.h>
#include <limits>
#include <math.h>

#include "ros/ros.h"
#include "tf/LinearMath/Vector3.h"

#include "data_server/GetData.h"
#include "data_server/GetLatestData.h"

#include "vent_planner/VentActionFactory.h"
#include "vent_planner/NestedSpiralVentPlanner.h"

#include "data_server/DataServerEntry.h"

#include "plume_detector/PlumeData.h"
#include "plume_detector/GetPlumeData.h"

NestedSpiralVentPlanner::NestedSpiralVentPlanner(ros::NodeHandle& nh, std::unique_ptr<VentActionFactory> actionFactory, std::string vehicleName) :
    nh(nh),
    actionFactory(std::move(actionFactory)),
    lastPlan(ros::Time::now()),
    initalPlan(false),
    plumeHeight(0),
    vehicleName(vehicleName),
    dataClient(nh.serviceClient<data_server::GetData>("/data_server/get")),
    latestDataClient(nh.serviceClient<data_server::GetLatestData>("/data_server/get_latest")),
    plumeClient(nh.serviceClient<plume_detector::GetPlumeData>("/plume_detector/get"))
{
    ROS_INFO("Planner: Waiting for data server...");
    dataClient.waitForExistence();
    latestDataClient.waitForExistence();
    plumeClient.waitForExistence();

    if(!nh.hasParam("planner/inital_spacing"))
    {
        ROS_FATAL("Parameter \"planner/inital_spacing\" not present in the parameter server.");
        exit(1);
    }

    if(!nh.hasParam("planner/final_spacing"))
    {
        ROS_FATAL("Parameter \"planner/final_spacing\" not present in the parameter server.");
        exit(1);
    }

    if(!nh.hasParam("planner/trigger_sigma"))
    {
        ROS_FATAL("Parameter \"planner/trigger_sigma\" not present in the parameter server.");
        exit(1);
    }

    nh.getParam("planner/inital_spacing", initalSpacing);
    nh.getParam("planner/final_spacing", finalSpacing);
    nh.getParam("planner/trigger_sigma", triggerSigma);
}

std::shared_ptr<Plan> NestedSpiralVentPlanner::plan()
{
    ROS_INFO("Planner: Plan Start");
    //Pop the top plan if it has been completed
    if(plans.size() > 0 && isCompleted(plans.top()))
    {
        ROS_INFO("Planner: Finished Plan");
        plans.pop();
        currentPlumeData.pop();

        if(plans.size() > 0)
        {
            ROS_INFO("Planner: Restart previous plan");
            lastPlan = ros::Time::now();
            plans.top()->resetInterrupted();
            return plans.top();
        }
    }
    else
    {
        ROS_INFO("Planner: No Finished Plan");
    }

    //Create inital plan and push it onto the stack
    if(!initalPlan && plans.size() == 0)
    {   
        std::shared_ptr<Plan> plan(new Plan());

        DataServerEntry latestEntry;
        if(getLatestData(latestEntry))
        {
            ROS_INFO("Planner: Generate Inital Plan");
            tf::Vector3 vehicleLocation(latestEntry.x, latestEntry.y, latestEntry.h);
            std::vector<tf::Vector3> spiralPoints = makeSpiral(vehicleLocation, 0, initalSpacing, 100000);
            std::shared_ptr<Action> newAction = actionFactory->createPointPathAction(vehicleName,
                                                                                    1.0,
                                                                                    0.349066,
                                                                                    0.523599, //30 deg
                                                                                    -100,
                                                                                    -2000,
                                                                                    spiralPoints);
            plan->addAction(newAction);


            //Create vector of plume data and add a reference to the stack
            plumeData.emplace_back();
            currentPlumeData.push(plumeData.size() - 1);

            //Add new plan to stack of plans
            plans.push(plan);
           

            initalPlan = true;

            lastPlan = ros::Time::now();
            ROS_INFO("Planner: Plan Generated");
            return plan;
        }
    }
    else
    {
        ROS_INFO("Planner: Plan");
        plume_detector::GetPlumeData srv;
        srv.request.name = vehicleName;
        srv.request.start_time = lastPlan;
        srv.request.end_time = ros::Time::now();

        ROS_INFO("Planner: Get plume data");
        plumeClient.call(srv);

        unsigned long dataStart = plumeData[currentPlumeData.top()].size();

        ROS_INFO("Planner: Add plume data: %lu, Data Size: %lu, Vector Index: %lu", dataStart, srv.response.time.size(), currentPlumeData.top());
        for(unsigned int i = 0; i < srv.response.time.size(); i++)
        {
            plumeData[currentPlumeData.top()].emplace_back(srv.response.time[i],
                                                      srv.response.x[i],
                                                      srv.response.y[i],
                                                      srv.response.h[i],
                                                      srv.response.val[i]);
        }


        double plumeX;
        double plumeY;
        double plumeStrength;
        bool gotHeight;

        if(plans.size() == 1)
        {
            //Get height of the plume for this last yo
            ROS_INFO("Planner: Get plume height");
            gotHeight = getHeightOfPlume(plumeData[currentPlumeData.top()], dataStart, plumeX, plumeY, plumeHeight, plumeStrength);
        }
        else
        {
            gotHeight = true;
            getPlumeMax(plumeData[currentPlumeData.top()], dataStart, plumeX, plumeY, plumeStrength);
        }
        
        ROS_INFO("Planner: Finish get plume data");
        if(gotHeight && triggerNewSpiral(plumeStrength) && !std::isnan(plumeX) && !std::isnan(plumeY) && !std::isinf(plumeX) && !std::isinf(plumeY))
        {
            int devFactor = plans.size();
            double spacing = initalSpacing / (devFactor * 2);
            double size = initalSpacing / devFactor;

            if(spacing > finalSpacing)
            {
                ROS_INFO("Planner: Trigger new spiral; level: %lu, x: %f, y: %f, height: %f, spacing: %f, size: %f ", plans.size(), plumeX, plumeY, plumeHeight, spacing, size);
            

                std::shared_ptr<Plan> plan(new Plan());

                tf::Vector3 spiralLocation(plumeX, plumeY, plumeHeight);
                std::vector<tf::Vector3> spiralPoints = makeSpiral(spiralLocation, plumeHeight, spacing, size);
                std::shared_ptr<Action> newAction = actionFactory->createPointPathAction(vehicleName,
                                                                                        1.0,
                                                                                        0.349066,
                                                                                        0.523599, //30 deg
                                                                                        spiralPoints);

                plan->addAction(newAction);


                //Create vector of plume data and add a reference to the stack
                plumeData.emplace_back();
                currentPlumeData.push(plumeData.size() - 1);

                //Add new plan to stack of plans
                plans.push(plan);

                lastPlan = ros::Time::now();
                return plan;  
            }
        }
    }

    lastPlan = ros::Time::now();
    if(plans.size() > 0)
    {
        plans.top()->resetInterrupted();
        ROS_INFO("Planner: Top plan sent");
        return plans.top();
    }

     ROS_INFO("Planner: NUll plan sent");
    return nullptr;
}

bool NestedSpiralVentPlanner::isDone()
{
    return false;
}

bool NestedSpiralVentPlanner::triggerNewSpiral(const double plumeStrength)
{
    double plumeAverage;
    double plumeMax;
    double plumeStdDev;
    plumeDataSummary(plumeAverage, plumeMax, plumeStdDev);

    if(plumeStdDev == 0 && plumeStrength >= 0.01)
    {
        return true;
    }
    else if(plumeStdDev > 0 && plumeStrength > plumeAverage + plumeStdDev * triggerSigma)
    {
        return true;
    }

    return false;
}

void NestedSpiralVentPlanner::getPlumeMax(const std::vector<PlumeData>& data, const unsigned int dataStart, double& plumeX, double& plumeY, double& plumeStrength)
{
    plumeStrength = 0;
    plumeX = std::numeric_limits<double>::quiet_NaN();
    plumeY = std::numeric_limits<double>::quiet_NaN();

    for(unsigned int i = dataStart; i < data.size(); i++)
    {
        auto& d = data[i];
        if(d.val >= plumeStrength)
        {
            plumeStrength = d.val;
            plumeX = d.x;
            plumeY = d.y;
        }
    }
}

void NestedSpiralVentPlanner::plumeDataSummary(double& average, double& max, double& stdDev)
{
    double plumeMax = 0;
    double plumeAverage = 0;
    unsigned int plumeCount = 0;
    double plumeStdDev = 0;


    for(auto& plumeVec : plumeData)
    {
        for(auto& plumeDataPoint : plumeVec)
        {
            if(plumeDataPoint.val > 0.0)
            {
                if(plumeDataPoint.val > plumeMax)
                {
                    plumeMax = plumeDataPoint.val;
                }

                plumeAverage += plumeDataPoint.val;
                plumeCount++; 
            }
        }
    }

    plumeAverage /= plumeCount;

    for(auto& plumeVec : plumeData)
    {
        for(auto& plumeDataPoint : plumeVec)
        {
            if(plumeDataPoint.val > 0.0)
            {
                plumeStdDev += (plumeAverage - plumeDataPoint.val) * (plumeAverage - plumeDataPoint.val);
            }
        }
    }

    plumeStdDev /= plumeCount;

    if(plumeStdDev != 0)
    {
        plumeStdDev = sqrt(plumeStdDev);
    }

    max = plumeMax;
    average = plumeAverage;
    stdDev = plumeStdDev;
}

bool NestedSpiralVentPlanner::getHeightOfPlume(const std::vector<PlumeData>& data, const unsigned int dataStart, double& plumeX, double& plumeY, double& plumeHeight, double& plumeStrength)
{
    //bin data by depth return bin with largest average
    unsigned int binSize = 10;
    double minHeight = std::numeric_limits<double>::max();
    double maxHeight = -std::numeric_limits<double>::max();

    if(dataStart >= data.size())
    {
        return false;
    }
    //calculate min and max heights for bins
    for(unsigned int i = dataStart; i < data.size(); i++)
    {
        auto& d = data[i];
        if(minHeight > d.h)
        {
            minHeight = d.h;
        }

        if(maxHeight < d.h)
        {
            maxHeight = d.h;
        }
    }

    int numBins = ceil((maxHeight - minHeight) / binSize);

    if(numBins == 0)
    {
        return false;
    }

    std::vector<double> bins(numBins, 0);
    std::vector<double> binsX(numBins, 0);
    std::vector<double> binsY(numBins, 0);
    std::vector<int> binCount(numBins, 0);

    for(unsigned int i = dataStart; i < data.size(); i++)
    {
        auto& d = data[i];
        int bin = (d.h - minHeight) / binSize;
        bins[bin] += d.val;
        binsX[bin] += d.x;
        binsY[bin] += d.y;
        binCount[bin]++;
    }

    int maxBin = 0;
    double maxBinVal = -std::numeric_limits<double>::max();
    double maxBinX = 0;
    double maxBinY = 0;
    for(unsigned int i = 0; i < binSize; i++)
    {
        if(maxBinVal < bins[i] / binCount[i])
        {
            maxBinVal = bins[i] / binCount[i];
            maxBinX = binsX[i] / binCount[i];
            maxBinY = binsY[i] / binCount[i];
            maxBin = i;
        }
    }

    //calculate max bins depth
    plumeHeight = minHeight + (maxBin * binSize) + binSize / 2;
    plumeStrength = maxBinVal;
    plumeX = maxBinX;
    plumeY = maxBinY;
    return true;
}

bool NestedSpiralVentPlanner::isCompleted(std::shared_ptr<Plan> plan)
{
    ROS_INFO("Planner: Check for completed");
    for(auto action : plan->getActions())
    {
        if(!(action->getState() == Action::State::COMPLETED || 
             action->getState() == Action::State::FAILED))
        {
            ROS_INFO("Planner: Not Completed");
            return false;
        }
    }
    ROS_INFO("Planner: Completed");
    return true;
}

bool NestedSpiralVentPlanner::getLatestData(DataServerEntry& returnEntry)
{
    data_server::GetLatestData srv;
    srv.request.name = vehicleName;

    bool valid = latestDataClient.call(srv);
    if(valid)
    {
        returnEntry.x = srv.response.x;
        returnEntry.y = srv.response.y;
        returnEntry.h = srv.response.h;

        returnEntry.time = srv.response.time;
        returnEntry.temp = srv.response.temp;
        returnEntry.salt = srv.response.salt;
        returnEntry.dye = srv.response.dye;

        return true;
    }

    return false;
}

std::vector<tf::Vector3> NestedSpiralVentPlanner::makeSpiral(tf::Vector3 startLocation, double startDirection, double spacing, double size)
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

std::vector<tf::Vector3> NestedSpiralVentPlanner::makeLawnmower(const tf::Vector3& startLocation,
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