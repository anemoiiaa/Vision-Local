/**
 * @file /include/robocup_localization23/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef robocup_localization23_QNODE_HPP_
#define robocup_localization23_QNODE_HPP_

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

#include <msg_generate/imu_msg.h>
#include <msg_generate/robocup_vision23.h>
#include <msg_generate/robocup_vision23_feature.h>
#include <msg_generate/ikcoordinate_msg.h>
#include <msg_generate/robocup_localization23.h>
#include <msg_generate/game_control_data.h>
#include <msg_generate/localv2_msg.h>
#include <msg_generate/master2localization23.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robocup_localization23 {

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


Q_SIGNALS:
    void rosShutdown();

private:
    void imuCallback(const msg_generate::imu_msg::ConstPtr&);
    void visionCallback(const msg_generate::robocup_vision23::ConstPtr&);
    void coordinateCallback(const msg_generate::ikcoordinate_msg::ConstPtr&);
    void visionfeatureCallback(const msg_generate::robocup_vision23_feature::ConstPtr &msg);
    void gamecontrolCallback(const msg_generate::game_control_data::ConstPtr &msg);
    void udpCallback(const msg_generate::localv2_msg::ConstPtr &msg);
    void masterCallback(const msg_generate::master2localization23::ConstPtr &msg);

private:
	int init_argc;
	char** init_argv;

};

}  // namespace robocup_localization23

#endif /* robocup_localization23_QNODE_HPP_ */
