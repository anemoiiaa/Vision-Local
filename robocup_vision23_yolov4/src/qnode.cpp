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
#include "../include/robocup_vision23_yolov4/qnode.hpp"

ros::Publisher motor_dxl_Publisher;
ros::Publisher vision_feature_Pub;
ros::Publisher visionPub;
ros::Subscriber masterSub;

ros::Publisher p2tPub;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robocup_vision23_yolov4 {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{
    img_qnode = NULL;
}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"robocup_vision23_yolov4");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    //Image Topic
    image_transport::ImageTransport it(n);
    image_sub = it.subscribeCamera("camera/image_raw", 100, &robocup_vision23_yolov4::QNode::imageCallBack, this);

    //PAN_TILT Topic
    motor_dxl_Publisher = n.advertise<serial_mcu::Motor_msg>("Dynamixel", 100);

    //Vision Topic
    visionPub = n.advertise<msg_generate::robocup_vision23>("robocup_vision23", 100);
    vision_feature_Pub = n.advertise<msg_generate::robocup_vision23_feature>("robocup_vision23_feature", 100);

    //Master Topic
    masterSub = n.subscribe("master_2_robocup_vision23", 100, &QNode::masterCallback, this);

    p2tPub = n.advertise<std_msgs::String>("p2tPub", 1000);

    // Add your ros communications here.
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

void QNode::imageCallBack(const sensor_msgs::ImageConstPtr& msg_img, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
    if(img_qnode == NULL && !isRecv)
    {

        img_qnode = new cv::Mat(cv_bridge::toCvCopy(msg_img,sensor_msgs::image_encodings::BGR8)->image);
        infoColor = new cv::Mat(3,3,CV_64FC1,(void*)info_msg->K.data());
        if(img_qnode != NULL)
        {
            isRecv = true;
            Q_EMIT recvImg(*img_qnode,*infoColor);
            delete img_qnode; delete infoColor;
            img_qnode = NULL; infoColor = NULL;
        }
    }
}

}  // namespace robocup_vision23_yolov4
