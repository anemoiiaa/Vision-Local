/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/robocup_localization23/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/


ros::Subscriber imuSub;
ros::Subscriber visionSub;
ros::Subscriber coordinateSub;
ros::Subscriber vision_feature_Sub;
ros::Subscriber gamecontrolSub;
ros::Subscriber udp1Sub;
ros::Subscriber udp2Sub;
ros::Subscriber udp3Sub;
ros::Subscriber masterSub;

ros::Publisher localizationPub;


namespace robocup_localization23 {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"robocup_localization23");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.

    imuSub = n.subscribe("imu", 100, &QNode::imuCallback, this);
    visionSub = n.subscribe("robocup_vision23", 100, &QNode::visionCallback, this);
    vision_feature_Sub = n.subscribe("robocup_vision23_feature", 100, &QNode::visionfeatureCallback, this);
    coordinateSub = n.subscribe("ikcoordinate", 100, &QNode::coordinateCallback, this);
    gamecontrolSub = n.subscribe("gamecontroller", 100, &QNode::gamecontrolCallback, this);
    udp1Sub = n.subscribe("udp_RB1", 100, &QNode::udpCallback, this);
    udp2Sub = n.subscribe("udp_RB2", 100, &QNode::udpCallback, this);
    udp3Sub = n.subscribe("udp_RB3", 100, &QNode::udpCallback, this);
    masterSub = n.subscribe("master2localization23", 100, &QNode::masterCallback, this);

    //Localization Topic
    localizationPub = n.advertise<msg_generate::robocup_localization23>("robocup_localization23", 100);

    start();
    return true;
}


void QNode::run() {
    ros::Rate loop_rate(33);
    while ( ros::ok() ) {

        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


}  // namespace robocup_localization23
