#ifndef PAN_TILT_H
#define PAN_TILT_H

#define YAW 0
#define PITCH 1

#include <random>

#include "/home/robit/catkin_ws/src/robot_config.h"
#include "Motor_msg.h"
#include "../pid_control_float/pid_control_float.h"

#define ROBOT_HEIGHT      600//490//475// 505//545// 755.0//544
#define TILT_L              20//65.0//75.0

//TILT_D
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_1
#define TILT_D              -49
#endif

#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_2
#define TILT_D              -48
#endif

#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_3
#define TILT_D              -49
#endif

#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_4
#define TILT_D              -49
#endif

#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_5
#define TILT_D              -47
#endif

#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_6
#define TILT_D              -48
#endif

//PAN_INIT
#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_1
#define PAN_INIT              2048
#endif

#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_2
#define PAN_INIT              2080
#endif

#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_3
#define PAN_INIT              1970
#endif

#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_4
#define PAN_INIT              2060
#endif

#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_5
#define PAN_INIT              2048
#endif

#ifdef ROBIT_HUMANOID_ROBOT_NUMBER_6
#define PAN_INIT              2048
#endif


#define DEG2RAD (M_PI / 180)
#define RAD2DEG (180 / M_PI)

extern ros::Publisher motor_dxl_Publisher;

using namespace std;

class PAN_TILT
{
public:
    //For Init

    //PRE CONDITION :  pan_init, tilt_init
    //POST CONDITION : PAN_TILT 객체
    //PURPOSE : PAN_TILT 객체 생성자, PRE CONDITION값이 없으면 초기 값으로 설정되고 있다면 해당 값으로 설정됨
    PAN_TILT()
    {
        Dynamixel_ID_INIT_Position[0] = PAN_INIT;
        Dynamixel_ID_INIT_Position[1] = 2048;
        Dynamixel_ID_INIT_Position[2] = PAN_INIT;
        Dynamixel_ID_INIT_Position[3] = 2048;

        PID_Control_init(&Tracking_pid_pan, kP_pan, kI_pan, kD_pan, 150, 90);
        PID_Control_init(&Tracking_pid_tilt, kP_tilt, kI_tilt, kD_tilt, 150, 90);
    }
    PAN_TILT(int pan_init, int tilt_init)
    {
        Dynamixel_ID_INIT_Position[0] = pan_init;
        Dynamixel_ID_INIT_Position[1] = tilt_init;
        Dynamixel_ID_INIT_Position[2] = pan_init;
        Dynamixel_ID_INIT_Position[3] = tilt_init;

        PID_Control_init(&Tracking_pid_pan, kP_pan, kI_pan, kD_pan, 150, 90);
        PID_Control_init(&Tracking_pid_tilt, kP_tilt, kI_tilt, kD_tilt, 150, 90);
    }

    //For MIN MAX
    int PAN_MAX = 150;
    int PAN_MIN = -150;
    int TILT_MAX = 0;
    int TILT_MIN = -90;
    struct POSITION
    {
        double PAN_POSITION = 0;
        double TILT_POSITION = 0;
    };
    POSITION ptpos;

    //For Dynamixel
    unsigned int lowpassfilter[2][2];
    const double ratio = 0.5;
    int waist_flag = 0;
    unsigned int Dynamixel_ID_Position[4] = {0,};
    unsigned int Dynamixel_ID_INIT_Position[4] = {2048, 2048, 2048, 2048};

    //For Scan
    int Scan_index = 1; // 0 ~ (size of Scan_level) - 1
    int Scan_level[4] ={-100, 0, 100, 0}; //카메라 각도
    int Scan_timer = 0;
    int Scan_stop_time = 15; //스캔속도 설정
    int Scan_nice_time = 5; //카메라가 움직이기 직전 몇장을 이미지처리 할건지
    int Scan_nice_weight = 0;

    //For PID
    PID Tracking_pid_pan;
    PID Tracking_pid_tilt;

    double kP_pan = 50;//10;//0.035
    double kI_pan = 0.05;//0.3;
    double kD_pan = 15;//15;

    double kP_tilt = 25;//25;
    double kI_tilt = 0.05;//1;
    double kD_tilt = 25;//35;

    //For Tracking
    double target_pan = 0, target_tilt = 0;
    double target_x = 0, target_y = 0;
    double target_absx = 0, target_absy = 0;

    int tracking_cnt = 0;
    int no_ball_cnt = 0;

    int error = -1;



private:
    void Tracking_what(double absx, double absy)
    {
        //PRE CONDITION : absx, absy
        //POST CONDITION : Tracking
        //PURPOSE : 스캔할 물체의 위치에 따라 어떤 각도로 팬을 조절할지 정함

        int dis = sqrt(pow(target_absx, 2) + pow(target_absy, 2)); //물체와 거리 계산
        //cout<<dis<<endl;
        if(dis > 500) //거리가 500이 넘지않으면 실행
        {
          int angle = atan2(target_absy, target_absx) * RAD2DEG - 90; //타겟 과의 각도 계산

          //각도 예외처리
          if(angle > 180){angle -= 360;}
          else if(angle < -180){angle += 360;}
          //cout<<angle<<endl;

          //+-50의 임계값에 따라 각도 설정
          if(angle < -50){Scan_index = 2;}
          else if(angle > 50){Scan_index = 0;}
          else{Scan_index = 1;}
        }
        else{Scan_index = 1;} //거리가 500이 넘으면 팬 각도 정면으로 설정
        Tracking(ptpos.PAN_POSITION, Scan_level[Scan_index]); //모터 제어를 위해 값 전송
        send_ptmsg(); //팬틸트 데이터 msg 전송
    }
    void Tracking(double now_x, double X_POINT_STANDARD)
    {
        //PRE CONDITION : now_x = ptpos.PAN_POSITION, X_POINT_STANDARD = Scan_level[Scan_index]
        //POST CONDITION : 팬 조절
        //PURPOSE : 팬 트래킹 값에 따라 모터 PID제어
        double Pan_temp_glass;

        PID_Control_Float(&Tracking_pid_pan, X_POINT_STANDARD, now_x);

        Pan_temp_glass = -Tracking_pid_pan.nowOutput;
        ptpos.PAN_POSITION += Pan_temp_glass;
    }

public:
    int mode(int value)
    {
        //PRE CONDITION : value
        //POST CONDITION : cam_nice_point
        //PURPOSE : 로봇의 상태에 따라 팬틸트 제어

        if(value != 2 && value != 3){no_ball_cnt = 100; tracking_cnt = 0;}
        switch (value)
        {
        case -2:
        {
            //뒤로 넘어졌을 시
            safe_mode_B();
            break;
        }
        case -1:
        {
            //앞으로 넘어졌을 시
            safe_mode_F();
            break;
        }
        case 0:
        {
            //기본자세
            init();
            break;
        }
        case 1:
        {
            //Oper_pan_tilt
            send_ptmsg();
            return 1;
            break;
        }
        case 2:
        {
            //case2!!!!!!!!!!!!!!!!!
            //Ball 스캔
            if(tracking_cnt > 0)//공이 잡혀서 트래킹
            {

                if(target_x != 0 || target_y != 0)
                {
                    Tracking_what(target_absx, target_absy); //공 위치로 트래킹
                    no_ball_cnt = 0;
                }
                else
                {
                    tracking_cnt = 0;
                }
                if(Scan_index == 1){Scan_nice_weight += 1;} //로봇이 정면을 보고 있는 경우 Scan_nice_weight +1
                else{Scan_nice_weight = 0;}
                if(Scan_nice_weight >= 5){return 1;} //로봇이 정면을 보고 있는 경우 1 반환
                return 2; //로봇이 정면을 보고 있는 경우 2 반환

            }
            else if(no_ball_cnt > 30)//공이 특정 시간동안 안잡힘
            {
                ptpos.PAN_POSITION = Scan_level[Scan_index];
                send_ptmsg();
                Scan_timer += 1;
                if(Scan_timer >= Scan_stop_time) //스캔한 시간이 임계값을 넘겼을 경우
                {
                    Scan_timer = 0;
                    Scan_index += 1;//스캔 각도 이동
                    if(Scan_index >= 4){Scan_index = 0;}
                }
                if(Scan_timer >= Scan_stop_time - Scan_nice_time){return 1;}
                //return 1;
            }
            else//공이 잡히다가 안잡힘
            {
                ptpos.PAN_POSITION = Scan_level[Scan_index];
                send_ptmsg();
                return 1;
            }
            break;

        }
        case 3:
        {
            ptpos.PAN_POSITION = 0;
            send_ptmsg();
            return 1;
            break;

        }
        case 4:
        {
            //Master_Callback
            send_ptmsg();
            Scan_index = 1;
            return 1;
            break;
        }
        case 99:
        {
            //ERROR
            error *= -1;
            ptpos.PAN_POSITION = error * 100;
            ptpos.TILT_POSITION = -45;
            send_ptmsg();
            break;
        }
        default:
        {
            init();
            break;
        }


        }
        return 0;
    }

private:
    void init()
    {
        ptpos.PAN_POSITION = 0;
        ptpos.TILT_POSITION = -45;
        for(int i = 0; i < 5; i++){send_ptmsg();}
    }
    void safe_mode_F()
    {
        ptpos.PAN_POSITION = 0;
        ptpos.TILT_POSITION = -45;
        send_ptmsg();
    }
    void safe_mode_B()
    {
        ptpos.PAN_POSITION = 0;
        ptpos.TILT_POSITION = 30;
        send_ptmsg();
    }

private:
    void send_ptmsg()
    {
        serial_mcu::Motor_msg dxMsg;
        bool fflag = true;
        Dynamixel_ID_Position[YAW + waist_flag] = Dynamixel_ID_INIT_Position[YAW + waist_flag] + AngleToPosition(ptpos.PAN_POSITION);
        Dynamixel_ID_Position[PITCH + waist_flag] = Dynamixel_ID_INIT_Position[PITCH + waist_flag] + AngleToPosition(ptpos.TILT_POSITION);
        for(int i = 0; i < 2; i++)
        {
            unsigned int pos = Dynamixel_ID_Position[i + waist_flag];
            if(i==1 && pos>4096) {int posMax = 4095;  pos = posMax;}
            else if(i==1 && pos<0) {int posMin =1; pos = posMin;}
            if(waist_flag == 2 && fflag == true) {dxMsg.id.push_back(22); fflag = false;}
            else {dxMsg.id.push_back(i + 30);}
            dxMsg.position.push_back(pos);
            dxMsg.speed.push_back(1023);
        }
        dxMsg.length = dxMsg.id.size();
        dxMsg.mode = 3;
        motor_dxl_Publisher.publish(dxMsg);
    }
    int AngleToPosition(double angle)
    {
        return (int)((angle * 4096) / 360);
    }

};




#endif // PAN_TILT_H
