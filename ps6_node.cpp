//Andrew Riemer EECS 373 PS6_node
//CODE WILL DO THE FOLLOWING
//-starts the competition under program control
//-starts the conveyor moving (a service call to /ariac/conveyor/control)
//-monitors logical camera 2 for object “shipping_box”
//-halts the conveyor when logical-camera 2 reports z-value of shipping box is close to zero (i.e.
//-box centered under camera)
//-pauses for 5 seconds with the box under camera 2
//-restarts the conveyor and runs until box0 slides down to the loading dock
//-calls the drone to fetch the (empty) box

#include <ros/ros.h>
#include <iostream>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/DroneControl.h>
#include <std_srvs/Trigger.h>
#include <string>
using namespace std;

bool g_take_new_snapshot = false;
osrf_gear::LogicalCameraImage g_cam2_data;

void cam2CB(const osrf_gear::LogicalCameraImage& message_holder) {
    if(g_take_new_snapshot) {
        g_cam2_data = message_holder;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ps6");
    ros::NodeHandle nh;
    ros::ServiceClient startup_client = nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    std_srvs::Trigger startup_srv;
    ros::ServiceClient conveyor_client = nh.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    osrf_gear::ConveyorBeltControl conveyor_srv;
    ros::ServiceClient drone_client = nh.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
    osrf_gear::DroneControl drone_srv;
    ros::Subscriber cam2_subscriber = nh.subscribe("/ariac/logical_camera_2", 1, cam2CB);

    startup_srv.response.success = false;
    while(!startup_srv.response.success) {
        ROS_WARN("Unsuccessful starting up..."); //message while startup service is not working
        startup_client.call(startup_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Successful response from startup service"); //when the startup service is successfully running, we get this message once

    conveyor_srv.request.power = 100.0;
    conveyor_srv.response.success = false;
    while(!conveyor_srv.response.success) {
        ROS_WARN("Unsuccessful starting conveyor..."); //message while conveyor is not moving
        conveyor_client.call(conveyor_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Conveyor successfully started"); //when the conveyor is successfully running, we get this message once

    g_take_new_snapshot = true;
    while(g_cam2_data.models.size()<1) {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("I see a box!");//message when the camera 2 sensor sees the box directly under it

    //when z=0 shut off conveyor, wait 5 seconds, turn it back on and call drone

    bool marker = false;
    while(!marker){
    	if(g_cam2_data.models[0].pose.position.z < -0.3){
        	ROS_INFO("Waiting for Box 0 to approach camera 2...");
        	ros::spinOnce();
        	ros::Duration(0.5).sleep();
      }else {
		ROS_INFO("Box 0 is directly under camera 2, pausing conveyor for 5 seconds!");
		conveyor_srv.request.power = 0.0;
		conveyor_srv.response.success = false;
		marker = true;
		conveyor_client.call(conveyor_srv);
		ros::Duration(5.0).sleep(); //5 second pause while box is directly under camera 2
    	}
    }

    ROS_INFO("Resuming conveyor movement...");
    conveyor_srv.request.power = 100.0; //moving the conveyor belt again
    conveyor_srv.response.success = false;
    conveyor_client.call(conveyor_srv);

    ros::Duration(15.0).sleep();

    drone_srv.request.shipment_type = "shipping_box_0"; //calling drone to pick up box 0
    drone_srv.response.success = false;
    while(!drone_srv.response.success) {
        ROS_WARN("drone call not yet successful...");
        drone_client.call(drone_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("Drone response successful, Drone is on its way");

    conveyor_srv.request.power = 0.0; //sets the conveyor back to a speed of 0, not moving
    conveyor_srv.response.success = false;
    conveyor_client.call(conveyor_srv);

    ROS_INFO("Simulation Ended");
    return 0;
}
