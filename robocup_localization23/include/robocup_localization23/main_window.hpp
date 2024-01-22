/**
 * @file /include/robocup_localization23/main_window.hpp
 *
 * @brief Qt based gui for robocup_localization23.
 *
 * @date November 2010
 **/
#ifndef robocup_localization23_MAIN_WINDOW_H
#define robocup_localization23_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "../objects/robot.hpp"
#include "../objects/obstacle.hpp"
#include "../objects/ball.hpp"
#include "../objects/line.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace robocup_localization23 {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

    //For timer
    QTimer *m_Timer;

    //For Screen
    QPixmap buf;
    QGraphicsScene *scene;
    void Print_Screen();
    int cvt_Print_xy(float target);
    QPolygonF create_Print_robot(ROBOT robot);


    //For Ros Topic
    void publish_msg();

    //For UI
    int set_robot_flag = 0;
    int setting_flag = 0;

    void setting();

    //For udp
    void sel_ball();









public Q_SLOTS:
    void main();
    void mouseReleaseEvent(QMouseEvent * e);
    void mouseMoveEvent(QMouseEvent * e);
    void on_btn_free_set_clicked();
    void on_btn_objects_save_clicked();
    void on_btn_test_clicked();
    void on_btn_set_1_clicked();
    void on_btn_set_2_clicked();
    void on_btn_set_3_clicked();
    void on_btn_set_4_clicked();
    void on_btn_set_5_clicked();
    void on_btn_set_6_clicked();
    void on_btn_set_auto_clicked();
    void on_btn_ball_set_clicked();
    void on_btn_ball_del_clicked();



private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace robocup_localization23

struct VISIONMSG
{
    int Ball_cam_X;
    int Ball_cam_Y;
    float Ball_2d_X;
    float Ball_2d_Y;
    float Ball_D;
    float PAN;
    float TILT;
    float Ball_speed_X;
    float Ball_speed_Y;
    vector<float> ROBOT_VEC_X;
    vector<float> ROBOT_VEC_Y;
    int Ball_speed_level;
    int Scan_mode;
    int flag_pan;
};

struct GAMEMSG
{
    int mySide = 1;
    int penalty = 0;
    int position = 0;
    int robotNum = 0;
};

#endif // robocup_localization23_MAIN_WINDOW_H
