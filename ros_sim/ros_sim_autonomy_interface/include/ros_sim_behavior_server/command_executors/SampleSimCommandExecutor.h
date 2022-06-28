#ifndef SAMPLE_SIM_COMMAND_EXECUTOR_H
#define SAMPLE_SIM_COMMAND_EXECUTOR_H

#include <vector>
#include <unordered_map>

#include "ros/ros.h"

#include "underwater_autonomy/behaviors/CommandExecutor.h"
#include "underwater_autonomy/util/VehiclePose.h"
#include "underwater_autonomy/behaviors/commands/SampleCommand.h"

#include "underwater_vehicle_msgs/VehicleInfo.h"
#include "underwater_vehicle_msgs/RequestSample.h"

#include "ros_sim_behavior_server/command_executors/SimCommandExecutorFactoryMethod.h"

class SampleSimCommandExecutor : public underwater_autonomy::CommandExecutor<underwater_autonomy::SampleCommand>,
                                   public SimCommandExecutorFactoryMethod<SampleSimCommandExecutor, underwater_autonomy::SampleCommand>
{
public:
    SampleSimCommandExecutor(underwater_autonomy::SampleCommand& action, ros::NodeHandle& nh, VehicleInfo& info);
    SampleSimCommandExecutor(const SampleSimCommandExecutor&&) = delete;
    SampleSimCommandExecutor(const SampleSimCommandExecutor&) = delete;

    SampleSimCommandExecutor& operator=(SampleSimCommandExecutor&& ) = delete;
    SampleSimCommandExecutor& operator=(SampleSimCommandExecutor& ) = delete;

    ~SampleSimCommandExecutor() {}

    /**
    * Executes the yoyo action in the ros simulation with the given parameters
    */
    void execute() override;
    
    /**
    * Monitors and updates the state of the yoyo action in the ros simulation 
    * All monitoring is done with action callbacks so this method is not used here
    */
    void monitor() override;

    void stop() override;

private:
    VehicleInfo vehicleInfo;
    ros::ServiceClient requestSampleClient;
};

#endif