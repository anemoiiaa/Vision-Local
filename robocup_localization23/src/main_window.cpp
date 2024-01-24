/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <fstream>
#include "../include/robocup_localization23/main_window.hpp"
#include "/home/robit/catkin_ws/src/robot_config.h" //only define ROBIT_HUMANOID_ROBOT_NUMBER_1
#define DEG2RAD (M_PI / 180)
#define PARTICLE_NUM 10000

extern ros::Subscriber imuSub;
extern ros::Subscriber visionSub;
extern ros::Subscriber coordinateSub;
extern ros::Subscriber vision_feature_Sub;
extern ros::Subscriber gamecontrolSub;
extern ros::Subscriber udp1Sub;
extern ros::Subscriber udp2Sub;
extern ros::Subscriber udp3Sub;
extern ros::Subscriber masterSub;

extern ros::Publisher localizationPub;
msg_generate::robocup_localization23 localizationMsg;

//For Objects
VISIONMSG visionMSG;
ROBOT robot0 = ROBOT();
ROBOT robot1 = ROBOT();
ROBOT robot2 = ROBOT();
ROBOT robot3 = ROBOT();
ROBOT robot4 = ROBOT();
BALL ball = BALL();
BALL ball1 = BALL();
BALL ball2 = BALL();
BALL ball3 = BALL();
BALL ball4 = BALL();
LINE Likelihood = LINE();
OBSTACLE obstacle0 = OBSTACLE();
OBSTACLE obstacle1 = OBSTACLE();
OBSTACLE obstacle2 = OBSTACLE();
OBSTACLE obstacle3 = OBSTACLE();

//For gamecontrol
GAMEMSG gameMSG;

//For particle
ROBOT pt[PARTICLE_NUM];
float particle_range = 0.0; // is 100
struct MEASUREMENT
{
    //파티클의 가중치 값을 저장, NUM은 pt[i]의 인덱스 (i), WEIGHT는 pt[i]의 가중치값
    int NUM;
    double WEIGHT;
};
MEASUREMENT measurement;
vector<MEASUREMENT> particle_weight; //purpose : MEASUREMENT를 저장하는 벡터 컨테이너

double sort_return(MEASUREMENT x, MEASUREMENT y)
//PRE CONDITION : MEASUREMENT 구조체 두개
//POST CONDITION : 두 구조체 중 WEIGHT멤버가 더 큰 쪽이 리턴
//purpose : MEASUREMENT 구조체를 담는 particle_weight 벡터 컨테이너에서 sort함수를 사용하기 위한 매개함수
{
    return x.WEIGHT > y.WEIGHT;
}

int vision_data_cnt = 0;
int vision_data_size = 0;


//For print
int robot_sight_flag = 0;
int master_target_x = 0;
int master_target_y = 0;


//For UI
int set_ball_flag = 0;

//callback
int vision_callback_timer = 0;

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace robocup_localization23 {

using namespace Qt;
using namespace std;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    setWindowIcon(QIcon(":/images/icon.png"));

    qnode.init();

    m_Timer = new QTimer(this);
    connect(m_Timer, SIGNAL(timeout()), this, SLOT(main()));
    m_Timer->start(100);

    //맵 이미지의 절대주소를 저장하는 QString 변수
    QString img_path = "/home/robit/catkin_ws/src/Vision-Local/robocup_localization23/resources/Map/2022_RoboCup_Field.png";
    QImage img(img_path); //QString 변수를 이용한 QImage 변수
    buf = QPixmap::fromImage(img);//QPixmap을 저장하는 버퍼
    buf = buf.scaled(825, 600);//버퍼 리스케일링

    scene = new QGraphicsScene(this);
    robot0.x = 550; robot0.y = 400; //로봇 시작 포인트, 맵의 중앙

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Functions
*****************************************************************************/
void robocup_localization23::MainWindow::main()
{
    //INIT
    setting(); //공 선택 & 파라미터 불러오기 & UI에서 오도메트리 값 설정
    sel_ball(); //UDP 통신에서 받은 다른 로봇의 공데이터로 공 위치 설정 &  obstacle 위치 계산
    Print_Screen(); //UI에 로봇 공 obstacle 파티클 그리기
    publish_msg(); //다른 노드로 데이터 publish
}
void robocup_localization23::MainWindow::sel_ball()
{
    //PRE CONDITION : robotX.b, visionMSG, ball 데이터 - 해당 데이터는 다른 노드에서 통신받아 옴
    //POST CONDITION : obstacleX, ball 데이터
    //purpose : 다른 로봇에서 받아 온 공 데이터로 최종 공 위치 설정, 이때 최종 공 위치는 각 로봇의 공 데이터 중 가장 공과 가까운 로봇의 데이터로 설정 & obstacle 의 위치 설정

    //로봇의 공 데이터 초기화
    robot1.b -= 0.005; robot2.b -= 0.005; robot3.b -= 0.005; robot4.b -= 0.005;

    //다른 로봇에서 공 데이터를 받아오지 않으면, 해당 로봇의 데이터 비활성화
    if(robot1.b <= 0){robot1.x = 0; robot1.y = 0; robot1.z = 0; robot1.b = 0; robot1.state = 0; ball1.x = 0; ball1.y = 0; ball1.d = 999999;}
    if(robot2.b <= 0){robot2.x = 0; robot2.y = 0; robot2.z = 0; robot2.b = 0; robot2.state = 0; ball2.x = 0; ball2.y = 0; ball2.d = 999999;}
    if(robot3.b <= 0){robot3.x = 0; robot3.y = 0; robot3.z = 0; robot3.b = 0; robot3.state = 0; ball3.x = 0; ball3.y = 0; ball3.d = 999999;}
    if(robot4.b <= 0){robot4.x = 0; robot4.y = 0; robot4.z = 0; robot4.b = 0; robot4.state = 0; ball4.x = 0; ball4.y = 0; ball4.d = 999999;}


    //visionMSG.ROBOT_VEC_X_size 해당 백터 콘테이너의 크기를 판별하여 크기만큼 obstacle 계산 ex) 콘테이너 크기 2 -> obstacle0, obstacle1 계산
    if(visionMSG.ROBOT_VEC_X.size() > 0)
    {
        obstacle0.x = robot0.x + (-1)*(visionMSG.ROBOT_VEC_X[0] / 10 * cos((robot0.z) * M_PI / 180) + visionMSG.ROBOT_VEC_Y[0] / 10 * sin((robot0.z) * M_PI / 180));
        obstacle0.y = robot0.y + (visionMSG.ROBOT_VEC_X[0] / 10 * sin((robot0.z) * M_PI / 180) - visionMSG.ROBOT_VEC_Y[0] / 10 * cos((robot0.z) * M_PI / 180));
        obstacle0.nocnt = 0;
    }
    else
    {
        obstacle0.nocnt += 1;
        if(obstacle0.nocnt > 20){obstacle0.nocnt = 20;obstacle0.x = 0;obstacle0.y = 0;}
    }
    if(visionMSG.ROBOT_VEC_X.size() > 1)
    {
        obstacle1.x = robot0.x + (-1)*(visionMSG.ROBOT_VEC_X[1] / 10 * cos((robot0.z) * M_PI / 180) + visionMSG.ROBOT_VEC_Y[1] / 10 * sin((robot0.z) * M_PI / 180));
        obstacle1.y = robot0.y + (visionMSG.ROBOT_VEC_X[1] / 10 * sin((robot0.z) * M_PI / 180) - visionMSG.ROBOT_VEC_Y[1] / 10 * cos((robot0.z) * M_PI / 180));
        obstacle1.nocnt = 0;
    }
    else
    {

        obstacle1.nocnt += 1;
        if(obstacle1.nocnt > 20){obstacle1.nocnt = 20;obstacle1.x = 0;obstacle1.y = 0;}
    }
    if(visionMSG.ROBOT_VEC_X.size() > 2)
    {
        obstacle2.x = robot0.x + (-1)*(visionMSG.ROBOT_VEC_X[2] / 10 * cos((robot0.z) * M_PI / 180) + visionMSG.ROBOT_VEC_Y[2] / 10 * sin((robot0.z) * M_PI / 180));
        obstacle2.y = robot0.y + (visionMSG.ROBOT_VEC_X[2] / 10 * sin((robot0.z) * M_PI / 180) - visionMSG.ROBOT_VEC_Y[2] / 10 * cos((robot0.z) * M_PI / 180));
        obstacle2.nocnt = 0;
    }
    else
    {

        obstacle2.nocnt += 1;
        if(obstacle2.nocnt > 20){obstacle2.nocnt = 20;obstacle2.x = 0;obstacle2.y = 0;}
    }
    if(visionMSG.ROBOT_VEC_X.size() > 3)
    {
        obstacle3.x = robot0.x + (-1)*(visionMSG.ROBOT_VEC_X[3] / 10 * cos((robot0.z) * M_PI / 180) + visionMSG.ROBOT_VEC_Y[3] / 10 * sin((robot0.z) * M_PI / 180));
        obstacle3.y = robot0.y + (visionMSG.ROBOT_VEC_X[3] / 10 * sin((robot0.z) * M_PI / 180) - visionMSG.ROBOT_VEC_Y[3] / 10 * cos((robot0.z) * M_PI / 180));
        obstacle3.nocnt = 0;
    }
    else
    {
        obstacle3.nocnt += 1;
        if(obstacle3.nocnt > 20){obstacle3.nocnt = 20;obstacle3.x = 0;obstacle3.y = 0;}
    }

    //visionMSG.Ball_2d 의 데이터를 통해 비전에서 받아온 공 데이터를 로컬 좌표로 변환
    if(visionMSG.Ball_2d_X != 0 || visionMSG.Ball_2d_Y != 0)
    {
        ball.noballcnt = 0;
        ball.x = robot0.x + (-1)*(visionMSG.Ball_2d_X / 10 * cos((robot0.z) * M_PI / 180) + visionMSG.Ball_2d_Y / 10 * sin((robot0.z) * M_PI / 180));
        ball.y = robot0.y + (visionMSG.Ball_2d_X / 10 * sin((robot0.z) * M_PI / 180) - visionMSG.Ball_2d_Y / 10 * cos((robot0.z) * M_PI / 180));
        ball.speed_x = (visionMSG.Ball_speed_X) * cos((-1) * robot0.z * DEG2RAD) - (visionMSG.Ball_speed_Y) * sin((-1) * robot0.z * DEG2RAD);
        ball.speed_y = (visionMSG.Ball_speed_X) * sin((-1) * robot0.z * DEG2RAD) + (visionMSG.Ball_speed_Y) * cos((-1) * robot0.z * DEG2RAD);

    }
    //데이터를 받지 않으면 데이터 비활성화
    else
    {
        ball.noballcnt += 1;
        if(ball.noballcnt >= 100)
        {
            ball.x = 0;
            ball.y = 0;
            ball.noballcnt = 100;
        }
    }


    double min_d = visionMSG.Ball_D / 10; //공 최종위치 판별 시 공 위치를 판별할 예외처리 변수 선언
    if(min_d == 0){min_d = 999999;}
    int min_i = 0;

    //각 로봇에서 측정된 공 데이터를 비교하여 해당 공 데이터가 최소값일 경우(측정된 로봇과 가장 가까울 경우) 해당 값 적용
    if(min_d > ball1.d){min_d = ball1.d; min_i = 1;}
    if(min_d > ball2.d){min_d = ball2.d; min_i = 2;}
    if(min_d > ball3.d){min_d = ball3.d; min_i = 3;}
    if(min_d > ball4.d){min_d = ball4.d; min_i = 4;}

    //최소값인 공 데이터가 해당 로봇이 아닐때 최소값인 공 데이터 적용
    if(min_i != 0 && (ball.noballcnt == 0 || ball.noballcnt == 100))
    {
        if(min_i == 1)
        {
            ball.x = ball1.x;
            ball.y = ball1.y;
        }
        if(min_i == 2)
        {
            ball.x = ball2.x;
            ball.y = ball2.y;
        }
        if(min_i == 3)
        {
            ball.x = ball3.x;
            ball.y = ball3.y;
        }
        if(min_i == 4)
        {
            ball.x = ball4.x;
            ball.y = ball4.y;
        }
        ball.noballcnt = 25;
    }

    //공 데이터 예외처리
    if(ball.x < 100 && ball.x != 0){ball.x = 100;}
    if(ball.x > 1000){ball.x = 1000;}
    if(ball.y < 100 && ball.y != 0){ball.y = 100;}
    if(ball.y > 700){ball.y = 700;}

    if(obstacle0.x < 100 && obstacle0.x != 0){obstacle0.x = 100;}
    if(obstacle0.x > 1000){obstacle0.x = 1000;}
    if(obstacle0.y < 100 && obstacle0.y != 0){obstacle0.y = 100;}
    if(obstacle0.y > 700){obstacle0.y = 700;}

    if(obstacle1.x < 100 && obstacle1.x != 0){obstacle1.x = 100;}
    if(obstacle1.x > 1000){obstacle1.x = 1000;}
    if(obstacle1.y < 100 && obstacle1.y != 0){obstacle1.y = 100;}
    if(obstacle1.y > 700){obstacle1.y = 700;}

    if(obstacle2.x < 100 && obstacle2.x != 0){obstacle2.x = 100;}
    if(obstacle2.x > 1000){obstacle2.x = 1000;}
    if(obstacle2.y < 100 && obstacle2.y != 0){obstacle2.y = 100;}
    if(obstacle2.y > 700){obstacle2.y = 700;}

    if(obstacle3.x < 100 && obstacle3.x != 0){obstacle3.x = 100;}
    if(obstacle3.x > 1000){obstacle3.x = 1000;}
    if(obstacle3.y < 100 && obstacle3.y != 0){obstacle3.y = 100;}
    if(obstacle3.y > 700){obstacle3.y = 700;}
}

void robocup_localization23::MainWindow::setting()
{
    //PRE CONDITION : vision_callback_timer, set_ball_flag, ball_set_N, setting_flag, 파라미터 저장파일
    //POST CONDITION : ball_N, robot0.odom_Nn
    //purpose : 로봇의 초기 세팅 및 입출력스트림을 통해 저장되어있던 오도메트리 값 설정, 볼의 위치를 재설정 하는 경우 볼 위치 변경, ui슬라이더를 통해 오도메트리 값 적용

    vision_callback_timer += 1; //vision timer count +1
    if(set_ball_flag) //set_ball_flag가 1일때 공 위치 변경
    {
        ball.x = ball.set_x;
        ball.y = ball.set_y;
        ball.noballcnt = 0;
    }
    if(vision_callback_timer > 10 && set_ball_flag == 0) //set_ball_flag가 비활성화 상태이고 timer가 10 이상이면 공 위치 비활성화
    {
        ball.x = 0;
        ball.y = 0;
    }

    if(setting_flag == 0) //setting_flag가 0이면 입출력스트림을 통해 데이터 로드
    {
        String param = "";
        #ifdef ROBIT_HUMANOID_ROBOT_NUMBER_1
            param = "/home/robit/catkin_ws/src/robocup_localization23/resources/param/PARAM1.txt";
        #endif
        #ifdef ROBIT_HUMANOID_ROBOT_NUMBER_2
            param = "/home/robit/catkin_ws/src/robocup_localization23/resources/param/PARAM2.txt";
        #endif
        #ifdef ROBIT_HUMANOID_ROBOT_NUMBER_3
            param = "/home/robit/catkin_ws/src/robocup_localization23/resources/param/PARAM3.txt";
        #endif
        #ifdef ROBIT_HUMANOID_ROBOT_NUMBER_4
            param = "/home/robit/catkin_ws/src/robocup_localization23/resources/param/PARAM4.txt";
        #endif
        #ifdef ROBIT_HUMANOID_ROBOT_NUMBER_5
            param = "/home/robit/catkin_ws/src/robocup_localization23/resources/param/PARAM5.txt";
        #endif
        #ifdef ROBIT_HUMANOID_ROBOT_NUMBER_6
            param = "/home/robit/catkin_ws/src/robocup_localization23/resources/param/PARAM6.txt";
        #endif
        ifstream Last_Index_Num_IN(param);
        if (Last_Index_Num_IN.is_open()){
            Last_Index_Num_IN >> particle_range;
            Last_Index_Num_IN >> robot0.odom_fx;
            Last_Index_Num_IN >> robot0.odom_bx;
            Last_Index_Num_IN >> robot0.odom_ly;
            Last_Index_Num_IN >> robot0.odom_ry;
        }
        Last_Index_Num_IN.close(); //setting_flag가 0이면 로드 된 데이터를 ui슬라이더에 적용
        ui.particle_range_value->setValue(particle_range);
        ui.odom_fx_value->setValue(robot0.odom_fx);
        ui.odom_bx_value->setValue(robot0.odom_bx);
        ui.odom_ly_value->setValue(robot0.odom_ly);
        ui.odom_ry_value->setValue(robot0.odom_ry);
        setting_flag = 1; //setting_flag set 1
    }
    //ui의 슬라이더 값을 오도메트리 값에 적용
    particle_range = ui.particle_range_value->value();
    robot0.odom_fx = ui.odom_fx_value->value();
    robot0.odom_bx = ui.odom_bx_value->value();
    robot0.odom_ly = ui.odom_ly_value->value();
    robot0.odom_ry = ui.odom_ry_value->value();
}

void robocup_localization23::MainWindow::publish_msg()
{
    //PRE CONDITION : ball_n, ball_speed_n, visionMSG.Ball_speed_level, robot0.n, obstacleX.n
    //POST CONDITION : localizationMsg
    //purpose : 다른 노드로 로컬 데이터를 전송 & 로컬 데이터 ui에 표시

    localizationMsg.Ball_X = ball.x;
    localizationMsg.Ball_Y = ball.y;
    localizationMsg.Ball_speed_X = ball.x + ball.speed_x * visionMSG.Ball_speed_level;
    localizationMsg.Ball_speed_Y = ball.y + ball.speed_y * visionMSG.Ball_speed_level;
    localizationMsg.Robot_X = robot0.x;
    localizationMsg.Robot_Y = robot0.y;
    localizationMsg.Obstacle0_X = obstacle0.x;
    localizationMsg.Obstacle0_Y = obstacle0.y;
    localizationMsg.Obstacle1_X = obstacle1.x;
    localizationMsg.Obstacle1_Y = obstacle1.y;
    localizationMsg.Obstacle2_X = obstacle2.x;
    localizationMsg.Obstacle2_Y = obstacle2.y;
    localizationMsg.Obstacle3_X = obstacle3.x;
    localizationMsg.Obstacle3_Y = obstacle3.y;
    localizationPub.publish(localizationMsg);


    //로컬 데이터 ui에 표시
    char TEXT[256];
    sprintf(TEXT, "BALL_X : %d", (int)localizationMsg.Ball_X); ui.BALL_X_value->setText(TEXT);
    sprintf(TEXT, "BALL_Y : %d", (int)localizationMsg.Ball_Y); ui.BALL_Y_value->setText(TEXT);
    sprintf(TEXT, "ROBOT_X : %d", (int)localizationMsg.Robot_X); ui.ROBOT_X_value->setText(TEXT);
    sprintf(TEXT, "ROBOT_Y : %d", (int)localizationMsg.Robot_Y); ui.ROBOT_Y_value->setText(TEXT);
}

void robocup_localization23::MainWindow::Print_Screen() // ui 출력
{
    //PRE CONDITION : robotX.n, obstacleX.n, ball.n, Likelihood
    //POST CONDITION : UI
    //purpose : UI출력

    //INIT
    if(scene){delete scene;}

    char TEXT[256];
    sprintf(TEXT, "IK_X : %d", (int)robot0.now_ik_param_x); ui.IK_X_value->setText(TEXT);
    sprintf(TEXT, "IK_Y : %d", (int)robot0.now_ik_param_y); ui.IK_Y_value->setText(TEXT);

    scene = new QGraphicsScene(this);
    ui.screen->setScene(scene);
    scene->addPixmap(buf);

    QBrush blueBrush(Qt::blue);
    QBrush redBrush(Qt::red);
    QBrush greenBrush(Qt::green);
    QBrush yellowBrush(Qt::yellow);
    QBrush blackBrush(Qt::black);
    QBrush grayBrush(Qt::gray);
    QPen grayPen(Qt::gray); grayPen.setWidth(1);
    QPen redPen(Qt::red); redPen.setWidth(2);
    QPen redPen3(Qt::red); redPen3.setWidth(3);
    QPen greenPen(Qt::green); greenPen.setWidth(2);
    QPen bluePen(Qt::blue); bluePen.setWidth(2);
    QPen blackPen1(Qt::black); blackPen1.setWidth(1);
    QPen blackPen(Qt::black); blackPen.setWidth(2);
    QPen blackPen3(Qt::black); blackPen3.setWidth(3);
    QPen LocalPen1(Qt::blue); LocalPen1.setWidth(5);
    QPen LocalPen2(Qt::green); LocalPen2.setWidth(5);
    QPen LocalPen3(Qt::yellow); LocalPen3.setWidth(5);
    QPen LocalPen4(Qt::red); LocalPen4.setWidth(5);
    QPen yellowPen(Qt::yellow); yellowPen.setWidth(3);
    QPen targetPen(Qt::red); targetPen.setWidth(3);
    QPen targetlinePen(Qt::gray); targetlinePen.setWidth(2);

    //print obstacle
    if(obstacle0.x != 0 && obstacle0.y != 0)
    {
        scene->addEllipse(cvt_Print_xy(obstacle0.x) - 35, cvt_Print_xy(obstacle0.y) - 35, 70, 70, grayPen, grayBrush);
    }
    if(obstacle1.x != 0 && obstacle1.y != 0)
    {
        scene->addEllipse(cvt_Print_xy(obstacle1.x) - 35, cvt_Print_xy(obstacle1.y) - 35, 70, 70, grayPen, grayBrush);
    }
    if(obstacle2.x != 0 && obstacle2.y != 0)
    {
        scene->addEllipse(cvt_Print_xy(obstacle2.x) - 35, cvt_Print_xy(obstacle2.y) - 35, 70, 70, grayPen, grayBrush);
    }
    if(obstacle3.x != 0 && obstacle3.y != 0)
    {
        scene->addEllipse(cvt_Print_xy(obstacle3.x) - 35, cvt_Print_xy(obstacle3.y) - 35, 70, 70, grayPen, grayBrush);
    }



    //PRINT PARTICLE
    if(true)
    {
        for(int i = 0; i < PARTICLE_NUM; i++)
        {
            scene->addLine(cvt_Print_xy(pt[i].x), cvt_Print_xy(pt[i].y), cvt_Print_xy(pt[i].x), cvt_Print_xy(pt[i].y), grayPen);
        }


    }

    //PRINT TIMESTAMP
    if(robot0.TIME_STAMP.size() > 0)
    {
        scene->addLine(cvt_Print_xy(robot0.x), cvt_Print_xy(robot0.y), cvt_Print_xy(robot0.TIME_STAMP[robot0.TIME_STAMP.size() - 1].X), cvt_Print_xy(robot0.TIME_STAMP[robot0.TIME_STAMP.size() - 1].Y), blackPen3);
        for(int i = 1; i < robot0.TIME_STAMP.size(); i++)
        {
            scene->addLine(cvt_Print_xy(robot0.TIME_STAMP[i - 1].X), cvt_Print_xy(robot0.TIME_STAMP[i - 1].Y), cvt_Print_xy(robot0.TIME_STAMP[i].X), cvt_Print_xy(robot0.TIME_STAMP[i].Y), blackPen3);
        }
    }


    //PRINT SIGHT
    //if(visionMSG.Scan_mode != 3){robot_sight_flag = 0; Likelihood.vision_point_vect.clear();}

    //비전에서 측정된 특징점 표시
    for(int i = 0; i < 27; i++)
    {
        if(Likelihood.Local_point_on_off[i] == 1)
        {
            scene->addEllipse(cvt_Print_xy(Likelihood.Local_point_x[i]) - 4, cvt_Print_xy(Likelihood.Local_point_y[i]) - 4, 8, 8, blackPen1);
        }
    }
    //cout<<Likelihood.vision_point_vect.size()<<endl;
    for(int i = 0; i < Likelihood.vision_point_vect.size(); i++)
    {
        //파티클 중 신뢰도가 0.9이상인 파티클의 특징점 위치 출력
        if(Likelihood.vision_point_vect[i].CONFIDENCE > 0.9)
        {
            scene->addLine(cvt_Print_xy(Likelihood.vision_point_vect[i].STD_X + Likelihood.vision_point_vect[i].POINT_VEC_X), cvt_Print_xy(Likelihood.vision_point_vect[i].STD_Y + Likelihood.vision_point_vect[i].POINT_VEC_Y), cvt_Print_xy(Likelihood.vision_point_vect[i].STD_X + Likelihood.vision_point_vect[i].POINT_VEC_X), cvt_Print_xy(Likelihood.vision_point_vect[i].STD_Y + Likelihood.vision_point_vect[i].POINT_VEC_Y), LocalPen1);
        }
        //파티클 중 신뢰도가 0.7이상인 파티클의 특징점 위치 출력
        else if(Likelihood.vision_point_vect[i].CONFIDENCE > 0.7)
        {
            scene->addLine(cvt_Print_xy(Likelihood.vision_point_vect[i].STD_X + Likelihood.vision_point_vect[i].POINT_VEC_X), cvt_Print_xy(Likelihood.vision_point_vect[i].STD_Y + Likelihood.vision_point_vect[i].POINT_VEC_Y), cvt_Print_xy(Likelihood.vision_point_vect[i].STD_X + Likelihood.vision_point_vect[i].POINT_VEC_X), cvt_Print_xy(Likelihood.vision_point_vect[i].STD_Y + Likelihood.vision_point_vect[i].POINT_VEC_Y), LocalPen2);
        }
        //파티클 중 신뢰도가 0.5이상인 파티클의 특징점 위치 출력
        else if(Likelihood.vision_point_vect[i].CONFIDENCE > 0.5)
        {
            scene->addLine(cvt_Print_xy(Likelihood.vision_point_vect[i].STD_X + Likelihood.vision_point_vect[i].POINT_VEC_X), cvt_Print_xy(Likelihood.vision_point_vect[i].STD_Y + Likelihood.vision_point_vect[i].POINT_VEC_Y), cvt_Print_xy(Likelihood.vision_point_vect[i].STD_X + Likelihood.vision_point_vect[i].POINT_VEC_X), cvt_Print_xy(Likelihood.vision_point_vect[i].STD_Y + Likelihood.vision_point_vect[i].POINT_VEC_Y), LocalPen3);
        }
//        else
//        {
//            scene->addLine(cvt_Print_xy(Likelihood.vision_point_vect[i].STD_X + Likelihood.vision_point_vect[i].POINT_VEC_X), cvt_Print_xy(Likelihood.vision_point_vect[i].STD_Y + Likelihood.vision_point_vect[i].POINT_VEC_Y), cvt_Print_xy(Likelihood.vision_point_vect[i].STD_X + Likelihood.vision_point_vect[i].POINT_VEC_X), cvt_Print_xy(Likelihood.vision_point_vect[i].STD_Y + Likelihood.vision_point_vect[i].POINT_VEC_Y), LocalPen4);
//        }

    }
    if(Likelihood.vision_point_vect.size() != 0)
    {
        //cout<<Likelihood.CIRCLE_R<<endl;
        scene->addEllipse(cvt_Print_xy(Likelihood.CIRCLE_CENTER.x - Likelihood.CIRCLE_R), cvt_Print_xy(Likelihood.CIRCLE_CENTER.y - Likelihood.CIRCLE_R), cvt_Print_xy(Likelihood.CIRCLE_R * 2), cvt_Print_xy(Likelihood.CIRCLE_R * 2), blackPen);
        //scene->addLine(cvt_Print_xy(Likelihood.CIRCLE_CENTER.x), cvt_Print_xy(Likelihood.CIRCLE_CENTER.y), cvt_Print_xy(Likelihood.CIRCLE_CENTER.x), cvt_Print_xy(Likelihood.CIRCLE_CENTER.y), targetPen);
    }

    //PRINT MASTER TARGET
    if(master_target_x != 0 || master_target_y != 0)
    {
        scene->addLine(cvt_Print_xy(master_target_x) + 4, cvt_Print_xy(master_target_y), cvt_Print_xy(master_target_x) - 4, cvt_Print_xy(master_target_y), targetPen);
        scene->addLine(cvt_Print_xy(master_target_x), cvt_Print_xy(master_target_y) + 4, cvt_Print_xy(master_target_x), cvt_Print_xy(master_target_y) - 4, targetPen);
        scene->addLine(cvt_Print_xy(master_target_x) + 2, cvt_Print_xy(master_target_y) + 2, cvt_Print_xy(robot0.x), cvt_Print_xy(robot0.y), targetlinePen);
    }

    //PRINT ROBOT
    QPolygonF robot0_poly = create_Print_robot(robot0);
    if(robot1.state != 3 && robot2.state != 3 && robot3.state != 3 && robot4.state != 3 && (ball.x != 0 || ball.y != 0)){scene->addPolygon(robot0_poly, redPen3, blueBrush);}
    else {scene->addPolygon(robot0_poly, blackPen, blueBrush);}
    scene->addLine(cvt_Print_xy(robot0.x), cvt_Print_xy(robot0.y), cvt_Print_xy(robot0.x) + sin((-1)*robot0.z * DEG2RAD) * 10, cvt_Print_xy(robot0.y) - cos(robot0.z * DEG2RAD) * 10, blackPen3);

    if(robot1.x != 0 || robot1.y != 0)
    {
        QPolygonF robot1_poly = create_Print_robot(robot1);

        if(robot1.state == 3){scene->addPolygon(robot1_poly, redPen3, redBrush);}
        else {scene->addPolygon(robot1_poly, blackPen, redBrush);}
        scene->addLine(cvt_Print_xy(robot1.x), cvt_Print_xy(robot1.y), cvt_Print_xy(robot1.x) + sin((-1)*robot1.z * DEG2RAD) * 10, cvt_Print_xy(robot1.y) - cos(robot1.z * DEG2RAD) * 10, blackPen3);
    }
    if(robot2.x != 0 || robot2.y != 0)
    {
        QPolygonF robot2_poly = create_Print_robot(robot2);
        if(robot2.state == 3){scene->addPolygon(robot2_poly, redPen3, greenBrush);}
        else {scene->addPolygon(robot2_poly, blackPen, greenBrush);}
        scene->addLine(cvt_Print_xy(robot2.x), cvt_Print_xy(robot2.y), cvt_Print_xy(robot2.x) + sin((-1)*robot2.z * DEG2RAD) * 10, cvt_Print_xy(robot2.y) - cos(robot2.z * DEG2RAD) * 10, blackPen3);
    }
    if(robot3.x != 0 || robot3.y != 0)
    {
        QPolygonF robot3_poly = create_Print_robot(robot3);
        if(robot3.state == 3){scene->addPolygon(robot3_poly, redPen3, yellowBrush);}
        else {scene->addPolygon(robot3_poly, blackPen, yellowBrush);}
        scene->addLine(cvt_Print_xy(robot3.x), cvt_Print_xy(robot3.y), cvt_Print_xy(robot3.x) + sin((-1)*robot3.z * DEG2RAD) * 10, cvt_Print_xy(robot3.y) - cos(robot3.z * DEG2RAD) * 10, blackPen3);
    }
    if(robot4.x != 0 || robot4.y != 0)
    {
        QPolygonF robot4_poly = create_Print_robot(robot4);
        if(robot4.state == 3){scene->addPolygon(robot4_poly, redPen3, blackBrush);}
        else {scene->addPolygon(robot4_poly, blackPen, blackBrush);}
        scene->addLine(cvt_Print_xy(robot4.x), cvt_Print_xy(robot4.y), cvt_Print_xy(robot4.x) + sin((-1)*robot4.z * DEG2RAD) * 10, cvt_Print_xy(robot4.y) - cos(robot4.z * DEG2RAD) * 10, blackPen3);
    }



    //PRINT BALL
    if(visionMSG.Ball_2d_X != 0 || visionMSG.Ball_2d_Y != 0)
    {
        scene->addEllipse(cvt_Print_xy(ball.x) - 5, cvt_Print_xy(ball.y) - 5, 11, 11, blackPen, redBrush);
        scene->addLine(cvt_Print_xy(ball.x), cvt_Print_xy(ball.y), cvt_Print_xy(ball.x + ball.speed_x * visionMSG.Ball_speed_level), cvt_Print_xy(ball.y + ball.speed_y * visionMSG.Ball_speed_level), blackPen);
    }
    else
    {
        if(ball.x != 0 || ball.y != 0)
        {
            scene->addEllipse(cvt_Print_xy(ball.x) - 5, cvt_Print_xy(ball.y) - 5, 11, 11, blackPen, yellowBrush);
        }
    }

    if(set_ball_flag)
    {
        scene->addEllipse(cvt_Print_xy(ball.x) - 5, cvt_Print_xy(ball.y) - 5, 11, 11, blackPen, blueBrush);
    }




}
void robocup_localization23::MainWindow::mouseReleaseEvent(QMouseEvent * e)
{
    //PRE CONDITION : mouseEvent, set_robot_flag, set_ball_flag
    //POST CONDITION : robot0.n, ball.n master_target_n,
    //purpose : set_object_flag가 활성화 되있고, 마우스 클릭 이벤트가 적용 시 해당 object를 마우스 위치로 이동 & 이동한 위치에 파티클 생성

    QPointF point = mapToParent(e->pos());
    QPoint position = mapToGlobal(QPoint(21, 60));
    if(set_robot_flag)
    {
        robot0.x = (point.x() - position.x()) * 100 / 75;
        robot0.y = (point.y() - position.y()) * 100 / 75;
        for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, particle_range);}
        set_robot_flag = 0;
        robot0.TIME_STAMP.clear();
        master_target_x = 0; master_target_y = 0;
    }

    if(set_ball_flag)
    {
        ball.x = (point.x() - position.x()) * 100 / 75;
        ball.y = (point.y() - position.y()) * 100 / 75;
        ball.set_x = ball.x;
        ball.set_y = ball.y;
        ball.noballcnt = 0;
    }
}

void robocup_localization23::MainWindow::mouseMoveEvent(QMouseEvent * e)
{
    //PRE CONDITION : mouseEvent, set_robot_flag, set_ball_flag
    //POST CONDITION : robot0.n, ball.n master_target_n,
    //purpose : set_object_flag가 활성화 되있고, 마우스 이동 이벤트가 적용 시 해당 object를 마우스 위치로 이동 & 이동한 위치에 파티클 생성
  QPointF point = mapToParent(e->pos());
  QPoint position = mapToGlobal(QPoint(21, 60));
  if(set_robot_flag)
  {
      robot0.x = (point.x() - position.x()) * 100 / 75;
      robot0.y = (point.y() - position.y()) * 100 / 75;
      robot0.TIME_STAMP.clear();
      master_target_x = 0; master_target_y = 0;
  }
  if(set_ball_flag)
  {
      ball.x = (point.x() - position.x()) * 100 / 75;
      ball.y = (point.y() - position.y()) * 100 / 75;
      ball.set_x = ball.x;
      ball.set_y = ball.y;
      ball.noballcnt = 0;
  }
}

QPolygonF robocup_localization23::MainWindow::create_Print_robot(ROBOT robot)
{
    //PRE CONDITION : robot.n, zoom(0.75)
    //POST CONDITION : Robot_model_poly
    //purpose : 로봇의 x, y, z좌표에 zoom값을 곱한 후 해당 값에 의한 폴리곤 생성

    float zoom = 0.75;
    float Robot_model_x = robot.x * zoom;
    float Robot_model_y = robot.y * zoom;
    float Robot_model_z = robot.z + 90;

    float robot_shape_1_x = -12 * zoom, robot_shape_1_y = 9 * zoom, robot_shape_2_x = 12 * zoom, robot_shape_2_y = 9 * zoom, robot_shape_3_x = -12 * zoom, robot_shape_3_y = -9 * zoom, robot_shape_4_x = 12 * zoom, robot_shape_4_y = -9 * zoom;

    QPolygonF Robot_model_poly;
    Robot_model_poly << QPointF((robot_shape_1_x)*sin(Robot_model_z*M_PI/180)-(robot_shape_1_y)*cos(Robot_model_z*M_PI/180)+Robot_model_x, (robot_shape_1_x)*cos(Robot_model_z*M_PI/180)+(robot_shape_1_y)*sin(Robot_model_z*M_PI/180)+Robot_model_y)
         << QPointF((robot_shape_2_x)*sin(Robot_model_z*M_PI/180)   -   (robot_shape_2_y)*cos(Robot_model_z*M_PI/180)   +   Robot_model_x                   ,                   (robot_shape_2_x)*cos(Robot_model_z*M_PI/180)   +   (robot_shape_2_y)*sin(Robot_model_z*M_PI/180)   +   Robot_model_y)
         << QPointF((robot_shape_4_x)*sin(Robot_model_z*M_PI/180)-(robot_shape_4_y)*cos(Robot_model_z*M_PI/180)+Robot_model_x, (robot_shape_4_x)*cos(Robot_model_z*M_PI/180)+(robot_shape_4_y)*sin(Robot_model_z*M_PI/180)+Robot_model_y)
         << QPointF((robot_shape_3_x)*sin(Robot_model_z*M_PI/180)-(robot_shape_3_y)*cos(Robot_model_z*M_PI/180)+Robot_model_x, (robot_shape_3_x)*cos(Robot_model_z*M_PI/180)+(robot_shape_3_y)*sin(Robot_model_z*M_PI/180)+Robot_model_y);

    return Robot_model_poly;
}

int robocup_localization23::MainWindow::cvt_Print_xy(float target)
{
    //PRE CONDITION : target
    //POST CONDITION : tartet * 0.75
    //purpose : target에 0.75를 곱한 값을 리턴

    float zoom = 0.75;
    target *= zoom;
    return (int)target;
}
void robocup_localization23::QNode::visionfeatureCallback(const msg_generate::robocup_vision23_feature::ConstPtr &msg)
{
    //PRE CONDITION : msg(vision)
    //POST CONDITION : measurement, particle_weight, robot0.N,  Likelihood
    //purpose : 비전 데이터를 콜백 할 때, 비전에서 받은 특징점 데이터를 통해 파티클의 가중치를 계산하고 가장 가중치가 높은 파티클로 로봇의 위치를 보정

    if(Likelihood.vision_point_vect.size() > 100) //비전에서 충분한 양의 특징점 데이터를 찾을 시 실행
    {
        if(vision_data_size / vision_data_cnt <= 1){for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, 20);}} //비전에서 충분하지 못한 시간동안 데이터를 찾을 경우 파티클 재생성
        for(int i = 0; i < PARTICLE_NUM; i++)
        {
            Likelihood.set_circle(robot0.z, Likelihood.vision_point_vect); //파티클 위치의 제한 설정
            measurement.NUM = i; //파티클의 가중치를 저장하는 measurement의 NUM 값에 번호 부여
            measurement.WEIGHT = Likelihood.sence(pt[i].x, pt[i].y, robot0.x, robot0.y, Likelihood.vision_point_vect);//파티클의 가중치를 저장하는 measurement의 WEIGHT에 해당 파티클의 가중치를 계산 한 후 가중치 값 저장
            double dis = sqrt(pow(pt[i].x -robot0.x, 2) + pow(pt[i].y -robot0.y, 2));//파티클과 로봇의 위치 사이의 거리 계산
            dis /= particle_range;//해당 거리를 파티클 범위로 나눗셈
            if(dis < 1){dis = 1;} //나눗셈 결과 값이 1 이하일때 예외처리
            measurement.WEIGHT /= dis; //가중치에 거리 값 나눗셈
            particle_weight.push_back(measurement); //particle_weight 벡터 컨테이너에 해당 measurement값 저장
        }
        sort(particle_weight.begin(), particle_weight.end(), sort_return);//particle_weight 벡터 컨테이너 정렬
        if(particle_weight[0].WEIGHT > 30)//가장 가중치가 높은 값이 30 이상일 경우 실행
        {
            if(vision_data_size / vision_data_cnt <= 1){cout<<"SMALL MATCHING!! : "<<particle_weight[0].WEIGHT<<endl;}//비전에 충분한 데이터가 저장되지 않은 경우 SMALL MATCHING문구 와 함께 가중치 값 출력
            else{cout<<"MATCHING!! : "<<particle_weight[0].WEIGHT<<endl;}//비전에 충분한 데이터가 저장된 경우 MATCHING문구 와 함께 가중치 값 출력

            //robot0의 데이터를 가장 가중치가 높은 값으로 설정
            robot0.x = pt[particle_weight[0].NUM].x; //(pt[particle_weight[0].NUM].x + pt[particle_weight[1].NUM].x + pt[particle_weight[2].NUM].x) / 3;
            robot0.y = pt[particle_weight[0].NUM].y; //(pt[particle_weight[0].NUM].y + pt[particle_weight[1].NUM].y + pt[particle_weight[2].NUM].y) / 3;
        }
        else{cout<<"FAIL!! : "<<particle_weight[0].WEIGHT<<endl;}//그렇지 않을 경우 가중치 값 및 실패 메세지 출력
        for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, particle_range);}//파티클 재생성

        Likelihood.check_local_point(50, robot0.z, Likelihood.vision_point_vect);//특징점이 로봇 근처에 있는지에 따라 해당 특징점 활성화 또는 비활성화

        //robot_sight_flag = 0;
        particle_weight.clear(); //particle_weight 벡터 컨테이너 초기화
        Likelihood.vision_point_vect.clear(); //Likelihood.vision_point_vect 벡터 컨테이너 초기화

        //vision_data_cnt 및 vision_data_size 변수 초기화
        vision_data_cnt = 0;
        vision_data_size = 0;


        for(int i = 0; i < 27; i++)//모든 로컬 포인트 체크
        {
            if(Likelihood.Local_point_check[i] == 1)//특정 로컬 포인트가 활성화 된 상태일 시 실행
            {
                //Likelihood에 데이터 저장
                Likelihood.vision_point.CONFIDENCE = 0.1;
                Likelihood.vision_point.DISTANCE = 1;
                Likelihood.vision_point.POINT_VEC_X = Likelihood.Local_point_x[i] - robot0.x;
                Likelihood.vision_point.POINT_VEC_Y = Likelihood.Local_point_y[i] - robot0.y;
                Likelihood.vision_point.STD_X = robot0.x;
                Likelihood.vision_point.STD_Y = robot0.y;
                Likelihood.vision_point_vect.push_back(Likelihood.vision_point);

            }
        }
    }

    for(int i = 0; i < msg->CONFIDENCE.size(); i++)
    {
        vision_data_cnt += 1;
        vision_data_size += msg->CONFIDENCE.size();
        Likelihood.vision_point.CONFIDENCE = msg->CONFIDENCE[i];
        Likelihood.vision_point.DISTANCE = msg->DISTANCE[i];
        Likelihood.vision_point.POINT_VEC_X = (-1)*(msg->POINT_VEC_X[i] / 10 * cos((robot0.z) * M_PI / 180) + msg->POINT_VEC_Y[i] / 10 * sin((robot0.z) * M_PI / 180));
        Likelihood.vision_point.POINT_VEC_Y = (msg->POINT_VEC_X[i] / 10 * sin((robot0.z) * M_PI / 180) - msg->POINT_VEC_Y[i] / 10 * cos((robot0.z) * M_PI / 180));
        Likelihood.vision_point.STD_X = robot0.x;
        Likelihood.vision_point.STD_Y = robot0.y;
        Likelihood.vision_point_vect.push_back(Likelihood.vision_point);

        Likelihood.set_circle(robot0.z, Likelihood.vision_point_vect);
        Likelihood.on_local_point(robot0.x, robot0.y, robot0.x, robot0.y);
    }



}
void robocup_localization23::QNode::imuCallback(const msg_generate::imu_msg::ConstPtr &msg)
{
    //PRE CONDITION : msg(imu)
    //POST CONDITION : robot0.z
    //purpose : imu에서 로봇의 yaw값을 받아 온 후 해당 값을 robot0.z에 대입

    robot0.z = msg->yaw;
}
void robocup_localization23::QNode::visionCallback(const msg_generate::robocup_vision23::ConstPtr &msg)
{
    //PRE CONDITION : msg(vision)
    //POST CONDITION : Ball_cam_N, Ball_2d_N, Ball_D, PAN, TILT, Ball_speed_N, ROBOT_VEC_N, Ball_speed_level
    //purpose : 비전에서 받아온 데이터 콜백

    visionMSG.Ball_cam_X = msg->Ball_cam_X;
    visionMSG.Ball_cam_Y = msg->Ball_cam_Y;
    visionMSG.Ball_2d_X = msg->Ball_2d_X;
    visionMSG.Ball_2d_Y = msg->Ball_2d_Y;
    visionMSG.Ball_D = msg->Ball_D;
    visionMSG.PAN = msg->PAN;
    visionMSG.TILT = msg->TILT;
    visionMSG.Ball_speed_X = msg->Ball_speed_X;
    visionMSG.Ball_speed_Y = msg->Ball_speed_Y;
    visionMSG.ROBOT_VEC_X.clear();
    visionMSG.ROBOT_VEC_Y.clear();
    for(int i = 0; i < msg->ROBOT_VEC_X.size(); i++)
    {
        visionMSG.ROBOT_VEC_X.push_back(msg->ROBOT_VEC_X[i]);
        visionMSG.ROBOT_VEC_Y.push_back(msg->ROBOT_VEC_Y[i]);
    }
    visionMSG.Ball_speed_level = msg->Ball_speed_level;
    visionMSG.Scan_mode = msg->Scan_mode;
    if(visionMSG.Ball_speed_level > 150){visionMSG.Ball_speed_level = 150;}
    if(visionMSG.Ball_cam_X != 0 || visionMSG.Ball_cam_Y != 0){set_ball_flag = 0;}
    vision_callback_timer = 0;

}

void robocup_localization23::QNode::coordinateCallback(const msg_generate::ikcoordinate_msg::ConstPtr &msg)
{
    //PRE CONDITION : msg(IK)
    //POST CONDITION : Xmoved, Ymoved
    //purpose : IK에서 받아온 데이터 콜백 및 해당 데이터로 robot0의 위치 업데이트, 업데이트 된 위치에 파티클 가우시안 분포로 생성

    double Xmoved = msg->X;
    double Ymoved = msg->Y;
    robot0.move(Xmoved, Ymoved);
    for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, particle_range);}
}

void robocup_localization23::QNode::gamecontrolCallback(const msg_generate::game_control_data::ConstPtr &msg)
{
    //PRE CONDITION : msg(gamecontroller)
    //POST CONDITION : mySide, penalty, position, robotNum
    //purpose : gamecontroller에서 콜백 받아온 경기 데이터를 적용

    gameMSG.mySide = msg->mySide;
    gameMSG.penalty = msg->penalty;
    gameMSG.position = msg->position;
    gameMSG.robotNum = msg->robotNum;
    if(gameMSG.penalty != 0)
    {
        if(gameMSG.mySide)
        {
            if(gameMSG.position == 1){robot0.x = 900; robot0.y = 700;}
            else {robot0.x = 800; robot0.y = 700;}
            for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, particle_range);}
        }
        else
        {
            if(gameMSG.position == 1){robot0.x = 200; robot0.y = 700;}
            else {robot0.x = 300; robot0.y = 700;}
            for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, particle_range);}
        }
        robot0.TIME_STAMP.clear();
        master_target_x = 0; master_target_y = 0;
    }
}

void robocup_localization23::QNode::udpCallback(const msg_generate::localv2_msg::ConstPtr &msg)
{
    //PRE CONDITION : msg(udpcom)
    //POST CONDITION : robotX.n, ballX.n,
    //purpose : udp통신으로 받아온 다른 로봇 데이터를 콜백하여 내부 변수에 적용
    if(msg->robot_num == 1)
    {
        robot1.x = msg->local_x;
        robot1.y = msg->local_y;
        robot1.z = msg->local_yaw;
        robot1.b = 1;
        robot1.state = msg->robot_case;

        ball1.x = msg->ball_x;
        ball1.y = msg->ball_y;
        ball1.d = sqrt(pow(msg->local_x - msg->ball_x, 2) + pow(msg->local_y - msg->ball_y, 2));
    }
    if(msg->robot_num == 2)
    {
        robot2.x = msg->local_x;
        robot2.y = msg->local_y;
        robot2.z = msg->local_yaw;
        robot2.b = 1;
        robot2.state = msg->robot_case;

        ball2.x = msg->ball_x;
        ball2.y = msg->ball_y;
        ball2.d = sqrt(pow(msg->local_x - msg->ball_x, 2) + pow(msg->local_y - msg->ball_y, 2));
    }
    if(msg->robot_num == 3)
    {
        robot3.x = msg->local_x;
        robot3.y = msg->local_y;
        robot3.z = msg->local_yaw;
        robot3.b = 1;
        robot3.state = msg->robot_case;

        ball3.x = msg->ball_x;
        ball3.y = msg->ball_y;
        ball3.d = sqrt(pow(msg->local_x - msg->ball_x, 2) + pow(msg->local_y - msg->ball_y, 2));
    }
    if(msg->robot_num == 4)
    {
        robot4.x = msg->local_x;
        robot4.y = msg->local_y;
        robot4.z = msg->local_yaw;
        robot4.b = 1;
        robot4.state = msg->robot_case;

        ball4.x = msg->ball_x;
        ball4.y = msg->ball_y;
        ball4.d = sqrt(pow(msg->local_x - msg->ball_x, 2) + pow(msg->local_y - msg->ball_y, 2));
    }
}
void robocup_localization23::QNode::masterCallback(const msg_generate::master2localization23::ConstPtr &msg)
{
    //PRE CONDITION : msg(master)
    //POST CONDITION : master_target_N
    //purpose : 마스터에서 받은 타겟 좌표 데이터 적용

    master_target_x = msg->target_x;
    master_target_y = msg->target_y;
}

void robocup_localization23::MainWindow::on_btn_free_set_clicked()
{
    //PRE CONDITION : UI
    //POST CONDITION : set_robot_flag
    //purpose : set_robot_flag를 1로 설정

    set_robot_flag = 1;
}
void robocup_localization23::MainWindow::on_btn_objects_save_clicked()
{
    //PRE CONDITION : particle_range, robot0.odom_Nn
    //POST CONDITION : param file
    //purpose : 현재 적용되어있는 오도메트리 값을 입출력스트림을 통해 저장

    String param = "";
    #ifdef ROBIT_HUMANOID_ROBOT_NUMBER_1
        param = "/home/robit/catkin_ws/src/robocup_localization23/resources/param/PARAM1.txt";
    #endif
    #ifdef ROBIT_HUMANOID_ROBOT_NUMBER_2
        param = "/home/robit/catkin_ws/src/robocup_localization23/resources/param/PARAM2.txt";
    #endif
    #ifdef ROBIT_HUMANOID_ROBOT_NUMBER_3
        param = "/home/robit/catkin_ws/src/robocup_localization23/resources/param/PARAM3.txt";
    #endif
    #ifdef ROBIT_HUMANOID_ROBOT_NUMBER_4
        param = "/home/robit/catkin_ws/src/robocup_localization23/resources/param/PARAM4.txt";
    #endif
    #ifdef ROBIT_HUMANOID_ROBOT_NUMBER_5
        param = "/home/robit/catkin_ws/src/robocup_localization23/resources/param/PARAM5.txt";
    #endif
    #ifdef ROBIT_HUMANOID_ROBOT_NUMBER_6
        param = "/home/robit/catkin_ws/src/robocup_localization23/resources/param/PARAM6.txt";
    #endif
    ofstream Last_Index_Num_OUT(param);
    if(Last_Index_Num_OUT.is_open()){
        Last_Index_Num_OUT << particle_range << endl;
        Last_Index_Num_OUT << robot0.odom_fx << endl;
        Last_Index_Num_OUT << robot0.odom_bx << endl;
        Last_Index_Num_OUT << robot0.odom_ly << endl;
        Last_Index_Num_OUT << robot0.odom_ry << endl;
    }
    Last_Index_Num_OUT.close();
}
void robocup_localization23::MainWindow::on_btn_set_1_clicked()
{
    //PRE CONDITION : UI
    //POST CONDITION : robot0.N, master_target_x
    //purpose : get in시 위치 매크로

    if(gameMSG.mySide)
    {
        robot0.x = 630;
        robot0.y = 100;
        for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, particle_range);}
    }
    else
    {
        robot0.x = 200;
        robot0.y = 100;
        for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, particle_range);}
    }
    robot0.TIME_STAMP.clear();
    master_target_x = 0; master_target_y = 0;
}
void robocup_localization23::MainWindow::on_btn_set_2_clicked()
{
    //PRE CONDITION : UI
    //POST CONDITION : robot0.N, master_target_x
    //purpose : get in시 위치 매크로

    if(gameMSG.mySide)
    {
        robot0.x = 800;
        robot0.y = 100;
        for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, particle_range);}
    }
    else
    {
        robot0.x = 300;
        robot0.y = 100;
        for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, particle_range);}
    }
    robot0.TIME_STAMP.clear();
    master_target_x = 0; master_target_y = 0;
}
void robocup_localization23::MainWindow::on_btn_set_3_clicked()
{
    //PRE CONDITION : UI
    //POST CONDITION : robot0.N, master_target_x
    //purpose : get in시 위치 매크로

    if(gameMSG.mySide)
    {
        robot0.x = 900;
        robot0.y = 100;
        for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, particle_range);}
    }
    else
    {
        robot0.x = 470;
        robot0.y = 100;
        for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, particle_range);}
    }
    robot0.TIME_STAMP.clear();
    master_target_x = 0; master_target_y = 0;
}
void robocup_localization23::MainWindow::on_btn_set_4_clicked()
{
    //PRE CONDITION : UI
    //POST CONDITION : robot0.N, master_target_x
    //purpose : get in시 위치 매크로

    if(gameMSG.mySide)
    {
        robot0.x = 630;
        robot0.y = 700;
        for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, particle_range);}
    }
    else
    {
        robot0.x = 200;
        robot0.y = 700;
        for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, particle_range);}
    }
    robot0.TIME_STAMP.clear();
    master_target_x = 0; master_target_y = 0;
}
void robocup_localization23::MainWindow::on_btn_set_5_clicked()
{
    //PRE CONDITION : UI
    //POST CONDITION : robot0.N, master_target_x
    //purpose : get in시 위치 매크로

    if(gameMSG.mySide)
    {
        robot0.x = 800;
        robot0.y = 700;
        for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, particle_range);}
    }
    else
    {
        robot0.x = 300;
        robot0.y = 700;
        for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, particle_range);}
    }
    robot0.TIME_STAMP.clear();
    master_target_x = 0; master_target_y = 0;
}
void robocup_localization23::MainWindow::on_btn_set_6_clicked()
{
    //PRE CONDITION : UI
    //POST CONDITION : robot0.N, master_target_x
    //purpose : get in시 위치 매크로

    if(gameMSG.mySide)
    {
        robot0.x = 900;
        robot0.y = 700;
        for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, particle_range);}
    }
    else
    {
        robot0.x = 470;
        robot0.y = 700;
        for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, particle_range);}
    }
    robot0.TIME_STAMP.clear();
    master_target_x = 0; master_target_y = 0;
}
void robocup_localization23::MainWindow::on_btn_set_auto_clicked()
{
    //PRE CONDITION : UI
    //POST CONDITION : robot0.N, master_target_x
    //purpose : get in시 위치 자동 매크로

    if(gameMSG.mySide)
    {
        if(gameMSG.robotNum == 1)
        {
            robot0.x = 630;
            robot0.y = 100;
        }
        if(gameMSG.robotNum == 2)
        {
            robot0.x = 800;
            robot0.y = 100;
        }
        if(gameMSG.robotNum == 3)
        {
            robot0.x = 630;
            robot0.y = 700;
        }
        if(gameMSG.robotNum == 4)
        {
            robot0.x = 900;
            robot0.y = 700;
        }
        for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, particle_range);}
    }
    else
    {
        if(gameMSG.robotNum == 1)
        {
            robot0.x = 470;
            robot0.y = 700;
        }
        if(gameMSG.robotNum == 2)
        {
            robot0.x = 300;
            robot0.y = 700;
        }
        if(gameMSG.robotNum == 3)
        {
            robot0.x = 470;
            robot0.y = 100;
        }
        if(gameMSG.robotNum == 4)
        {
            robot0.x = 200;
            robot0.y = 100;
        }
        for(int i = 0; i < PARTICLE_NUM; i++){pt[i].random_point(robot0.x, robot0.y, particle_range);}
    }
    robot0.TIME_STAMP.clear();
    master_target_x = 0; master_target_y = 0;
}
void robocup_localization23::MainWindow::on_btn_ball_set_clicked()
{
    //PRE CONDITION : UI
    //POST CONDITION : set_ball_flag
    //purpose : 공 설정

    set_ball_flag = 1;
}
void robocup_localization23::MainWindow::on_btn_ball_del_clicked()
{
    //PRE CONDITION : UI
    //POST CONDITION : set_ball_flag, ball.N
    //purpose : 공 삭제 버튼

    set_ball_flag = 0;
    ball.x = 0;
    ball.y = 0;
}
void robocup_localization23::MainWindow::on_btn_test_clicked()
{
    robot1.x = 0;
    robot1.y = 0;
    robot1.z = 0;

    ball1.x = 0;
    ball1.y = 0;
    ball1.d = 999999;

    robot2.x = 0;
    robot2.y = 0;
    robot2.z = 0;

    ball2.x = 0;
    ball2.y = 0;
    ball2.d = 999999;

    robot3.x = 0;
    robot3.y = 0;
    robot3.z = 0;

    ball3.x = 0;
    ball3.y = 0;
    ball3.d = 999999;

    robot4.x = 0;
    robot4.y = 0;
    robot4.z = 0;

    ball4.x = 0;
    ball4.y = 0;
    ball4.d = 999999;

    Likelihood.vision_point_vect.clear();
}


}  // namespace robocup_localization23

