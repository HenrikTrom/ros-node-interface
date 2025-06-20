#pragma once

#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <string>
#include <memory>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"

namespace ros_node_interface {

class TestModule
{
public:
    TestModule(){};
    TestModule(ros::NodeHandle &nh, const std::string &topic_name)
    {
        this->pub = nh.advertise<std_msgs::String>(topic_name, 10);
    };
    ~TestModule(){};
    // implement functions
    bool start(){return true;};
    bool stop(){return true;};
    std::string get_type()
    {
        return this->type;
    }
    private:
    std::string type = "camera";

private:
    ros::Publisher pub;
    int i = 0;
};

class BaseRosInterfaceModule{
public:
    BaseRosInterfaceModule(){};
    ~BaseRosInterfaceModule(){};
    virtual bool start() = 0;
    virtual void Terminate() = 0;

    std::string get_type()
    {
        return this->type;
    }
    ros::NodeHandle nh;
    std::string type = "TypeNotImplemented";
    ros::Publisher pub;
    bool ShouldClose = false;
};

template <class InterfaceModule>
class BaseRosInterface
{
public:
    /**
     * Generates a BaseRosInterface
     */
    BaseRosInterface(
        std::unique_ptr<InterfaceModule> interface_module
    ) : interface_module(std::move(interface_module))
    {

        ros::NodeHandle nh("~");
        // TODO n callbacks
        ros::AsyncSpinner spinner(0);
        spinner.start();
        // Init ROS services
        this->start_server = nh.advertiseService("start", &BaseRosInterface::start_cb, this);
        // ros::Duration(2.0).sleep();
        this->stop_server = nh.advertiseService("stop", &BaseRosInterface::stop_cb, this);
        // ros::Duration(2.0).sleep();
        this->should_run=false;
        this->shutdown_server = nh.advertiseService("shutdown", &BaseRosInterface::shutdown_cb, this);
        // ros::Duration(2.0).sleep();
        // this->mainThread = std::thread(&BaseRosInterface::threadFunction, this);
        ros::waitForShutdown();
    };

    /**
     * Initializes and starts the main interface
     *
     * @param [in]  req service request
     * @param [out] res service response
     * @return true if interface could get started, false otherwise
     */
    bool start_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
        if (this->should_run)
        {
            res.message="Process is already running";
            res.success = false;
            return true;
        }
        else
        {
            ROS_INFO_STREAM("Starting BaseRosInterface " + this->interface_module->get_type());
            res.message="Starting";
            res.success = true;
            this->should_run = true;
            this->interface_module->ShouldClose = false;
            this->interface_module->start();
            return true;
        }
    };

    /**
     * Stops the interface
     *
     * @param [in]  req service request
     * @param [out] res service response
     * @return true if interface could get stopped, false otherwise
     */
    bool stop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
        if (this->should_run)
        {
            res.message="Stopping";
            res.success = true;
            this->should_run = false;
            ROS_INFO_STREAM("Stopping RosInterface " << this->interface_module->get_type());
            this->interface_module->ShouldClose = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            this->interface_module->Terminate();
            return true;
        }
        else
        {
            res.message="Process is not running";
            res.success = false;
            return true;
        }
    };

    /**
     * Calls the stop method
     *
     * @param [in]  req service request
     * @param [out] res service response
     * @return true if interface could get stopped, false otherwise
     */
    bool stop_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
        this->stop(req, res);
        return true;

    };

    /**
     * Shuts down the interface
     *
     * @param [in]  req service request
     * @param [out] res service response
     * @return true if interface could get shutdown, false otherwise
     */
    bool shutdown_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
        this->stop(req, res);
        ROS_INFO_STREAM("Shutting down BaseRosInterface " << this->interface_module->get_type());
        ros::shutdown();
        return true;
    };
    
    ~BaseRosInterface(){
        std_srvs::Trigger::Request req;
        std_srvs::Trigger::Response res;
        this->stop(req, res);
    };

private:
    ros::NodeHandle nh;
    std::atomic<bool> should_run;
    std::thread mainThread;
    ros::ServiceServer stop_server;
    ros::ServiceServer start_server;
    ros::ServiceServer shutdown_server;
    // InterfaceModule interface_module;
    std::unique_ptr<InterfaceModule> interface_module;
};

} // namespace ros_node_interface