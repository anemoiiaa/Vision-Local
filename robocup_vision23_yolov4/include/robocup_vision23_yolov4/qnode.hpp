/**
 * @file /include/robocup_vision23_yolov4/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robocup_vision23_yolov4_QNODE_HPP_
#define robocup_vision23_yolov4_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
//For Image
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "../pan_tilt/Motor_msg.h"

//MSG
#include <msg_generate/robocup_vision23.h>
#include <msg_generate/robocup_vision23_feature.h>
#include <msg_generate/master_2_robocup_vision23.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robocup_vision23_yolov4 {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
    bool init();
        void run();

    //For Image
    cv::Mat* img_qnode;
    cv::Mat *infoColor;
    image_transport::CameraSubscriber image_sub;

    bool isRecv = false;
    void imageCallBack(const sensor_msgs::ImageConstPtr& msg_img, const sensor_msgs::CameraInfoConstPtr& info_msg);
    void masterCallback(const msg_generate::master_2_robocup_vision23::ConstPtr&);


Q_SIGNALS:
    void rosShutdown();
    void recvImg(const cv::Mat&, const cv::Mat&);

private:
	int init_argc;
	char** init_argv;

};

}  // namespace robocup_vision23_yolov4

#endif /* robocup_vision23_yolov4_QNODE_HPP_ */
