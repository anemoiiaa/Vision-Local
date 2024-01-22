#ifndef LINE_H
#define LINE_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#define DEG2RAD (M_PI / 180)

using namespace std;
using namespace cv;

class LINE
{
public:
    //For Init
    LINE()
    {

    }

public:
    struct VISION_POINT
    {
        double CONFIDENCE;
        double DISTANCE;
        int POINT_VEC_X;
        int POINT_VEC_Y;
        int STD_X;
        int STD_Y;
    };
    VISION_POINT vision_point;
    vector<VISION_POINT> vision_point_vect;

    Point CIRCLE_CENTER = Point(0, 0);
    double CIRCLE_R = 0;

    int Local_point_x[27] = {100, 550, 1000, 100, 300, 800, 1000, 100, 200, 900, 1000, 550, 250, 550, 850, 550, 100, 200, 900, 1000, 100, 300, 800, 1000, 100, 550, 1000};
    int Local_point_y[27] = {100, 100, 100, 150, 150, 150, 150, 250, 250, 250, 250, 325, 400, 400, 400, 475, 550, 550, 550, 550, 650, 650, 650, 650, 700, 700, 700};
    int Local_point_on_off[27] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int Local_point_check[27] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

private:
    Mat Likelihood_mat;


public:
    int set_circle(double yaw, vector<VISION_POINT> &vect)
    {
        double X_MAX = -9999, X_MIN = 9999;
        double Y_MAX = -9999, Y_MIN = 9999;
        for(int i = 0; i < vect.size(); i++)
        {
            int transformation_x = (vect[i].POINT_VEC_X)*cos((-1)*yaw*DEG2RAD) - (vect[i].POINT_VEC_Y)*sin((-1)*yaw*DEG2RAD);
            int transformation_y = (vect[i].POINT_VEC_X)*sin((-1)*yaw*DEG2RAD) + (vect[i].POINT_VEC_Y)*cos((-1)*yaw*DEG2RAD);
            if(transformation_x > X_MAX){X_MAX = transformation_x;}
            if(transformation_x < X_MIN){X_MIN = transformation_x;}
            if(transformation_y > Y_MAX){Y_MAX = transformation_y;}
            if(transformation_y < Y_MIN){Y_MIN = transformation_y;}
        }

        CIRCLE_CENTER = Point((int)(vect[0].STD_X + ((X_MAX + X_MIN) / 2)*cos(yaw*DEG2RAD) - ((Y_MAX + Y_MIN) / 2)*sin(yaw*DEG2RAD)), (int)(vect[0].STD_Y + ((X_MAX + X_MIN) / 2)*sin(yaw*DEG2RAD) + ((Y_MAX + Y_MIN) / 2)*cos(yaw*DEG2RAD)));
        CIRCLE_R = 0;
        for(int i = 0; i < vect.size(); i++)
        {
            double dis = sqrt(pow(CIRCLE_CENTER.x - vect[i].POINT_VEC_X - vect[i].STD_X, 2) + pow(CIRCLE_CENTER.y - vect[i].POINT_VEC_Y - vect[i].STD_Y, 2));
            if(CIRCLE_R < dis){CIRCLE_R = dis;}
        }
        CIRCLE_R += 50;
    }
    int on_local_point(int PT_X, int PT_Y, int NOW_X, int NOW_Y)
    {
        for(int i = 0; i < 27; i++)
        {
            if(pow(CIRCLE_R, 2) > pow(PT_X -NOW_X + CIRCLE_CENTER.x - Local_point_x[i], 2) + pow(PT_Y -NOW_Y + CIRCLE_CENTER.y - Local_point_y[i], 2)){Local_point_on_off[i] = 1;}
            else{Local_point_on_off[i] = 0;}
        }
    }
    int check_local_point(int ago_point_cnt, double yaw, vector<VISION_POINT> &vect)
    {
        double X_MAX = -9999, X_MIN = 9999;
        double Y_MAX = -9999, Y_MIN = 9999;
        for(int i = ago_point_cnt; i < vect.size(); i++)
        {
            int transformation_x = (vect[i].POINT_VEC_X)*cos((-1)*yaw*DEG2RAD) - (vect[i].POINT_VEC_Y)*sin((-1)*yaw*DEG2RAD);
            int transformation_y = (vect[i].POINT_VEC_X)*sin((-1)*yaw*DEG2RAD) + (vect[i].POINT_VEC_Y)*cos((-1)*yaw*DEG2RAD);
            if(transformation_x > X_MAX){X_MAX = transformation_x;}
            if(transformation_x < X_MIN){X_MIN = transformation_x;}
            if(transformation_y > Y_MAX){Y_MAX = transformation_y;}
            if(transformation_y < Y_MIN){Y_MIN = transformation_y;}
        }

        CIRCLE_CENTER = Point((int)(vect[0].STD_X + ((X_MAX + X_MIN) / 2)*cos(yaw*DEG2RAD) - ((Y_MAX + Y_MIN) / 2)*sin(yaw*DEG2RAD)), (int)(vect[0].STD_Y + ((X_MAX + X_MIN) / 2)*sin(yaw*DEG2RAD) + ((Y_MAX + Y_MIN) / 2)*cos(yaw*DEG2RAD)));
        CIRCLE_R = 0;
        for(int i = ago_point_cnt; i < vect.size(); i++)
        {
            double dis = sqrt(pow(CIRCLE_CENTER.x - vect[i].POINT_VEC_X - vect[i].STD_X, 2) + pow(CIRCLE_CENTER.y - vect[i].POINT_VEC_Y - vect[i].STD_Y, 2));
            if(CIRCLE_R < dis){CIRCLE_R = dis;}
        }
        CIRCLE_R += 50;
        for(int i = 0; i < 27; i++)
        {
            if(pow(CIRCLE_R, 2) > pow(CIRCLE_CENTER.x - Local_point_x[i], 2) + pow(CIRCLE_CENTER.y - Local_point_y[i], 2)){Local_point_check[i] = 1;}
            else{Local_point_check[i] = 0;}
        }
    }
    double sence(int PT_X, int PT_Y, int NOW_X, int NOW_Y, vector<VISION_POINT> &vect)
    {
        on_local_point(PT_X, PT_Y, NOW_X, NOW_Y);
        double weight = 0.0;
        int std_R = 50;

        for(int i = 0; i < vect.size(); i++)
        {
            int ptx = PT_X + vect[i].STD_X - NOW_X + vect[i].POINT_VEC_X;
            int pty = PT_Y + vect[i].STD_Y - NOW_Y + vect[i].POINT_VEC_Y;
            double min_dis = 99999999;
            int min_idx = 0;
            for(int j = 0; j < 27; j++)
            {
                double dis = 99999999;
                if(Local_point_on_off[j] == 1)
                {
                    dis = sqrt(pow(Local_point_x[j] - ptx, 2) + pow(Local_point_y[j] - pty, 2));
                }
                if(min_dis > dis){min_dis = dis; min_idx = j;}
            }
            if(min_dis <= 10){min_dis = 10;}
//            else if(min_dis >= 50){min_dis = (-1)*min_dis;}
            weight += (10 / min_dis) * vect[i].CONFIDENCE * abs(1 - vect[i].DISTANCE / 10000);

        }
        return weight;

    }
//    double sence(int PT_X, int PT_Y, int NOW_X, int NOW_Y, vector<VISION_POINT> &vect)
//    {
//        on_local_point(PT_X, PT_Y, NOW_X, NOW_Y);
//        double weight = 0.0;
//        int std_R = 50;

//        for(int i = 0; i < vect.size(); i++)
//        {
//            int ptx = PT_X + vect[i].STD_X - NOW_X + vect[i].POINT_VEC_X;
//            int pty = PT_Y + vect[i].STD_Y - NOW_Y + vect[i].POINT_VEC_Y;
//            for(int j = 0; j < 27; j++)
//            {
//                double dis = pow(Local_point_x[j] - ptx, 2) + pow(Local_point_y[j] - pty, 2);

//                if(pow(std_R, 2) > dis)
//                {
//                    weight += ((-1) * double(sqrt(dis)) / double(std_R) + 1) * vect[i].CONFIDENCE * abs(1 - vect[i].DISTANCE / 10000);
//                }
//            }

//        }
//        return weight;

//    }

private:
    int possibility_matching(int NUM, int N_NUM, int X_NUM, int L_NUM, int T_NUM)
    {
        if(NUM == 1){return 0;}
        else if(NUM == 2){if(T_NUM > 0){return 1;}}
        else if(NUM == 3){return 0;}
        else if(NUM == 4){if(X_NUM > 0){return 1;}}
        else if(NUM == 5){if(X_NUM > 0 || L_NUM > 0){return 1;}}
        else if(NUM == 6){if(X_NUM > 0){return 1;}}
        else if(NUM == 7){return 0;}
        else if(NUM == 8){if(T_NUM > 0){return 1;}}
        else if(NUM == 9){return 0;}
        return 0;
    }

    Rect get_grid_size(int Num)
    {
        int x = 0, y = 0, w = 0, h = 0;
        if(Num == 1){x = 0; y = 0; w = 400; h = 300;}
        else if(Num == 2){x = 400; y = 0; w = 300; h = 150;}
        else if(Num == 3){x = 700; y = 0; w = 400; h = 300;}
        else if(Num == 4){x = 0; y = 300; w = 400; h = 200;}
        else if(Num == 5){x = 400; y = 150; w = 300; h = 500;}
        else if(Num == 6){x = 700; y = 300; w = 400; h = 200;}
        else if(Num == 7){x = 0; y = 500; w = 400; h = 300;}
        else if(Num == 8){x = 400; y = 650; w = 300; h = 150;}
        else if(Num == 9){x = 700; y = 500; w = 400; h = 300;}
        else{x = 0; y = 0; w = 0; h = 0;}

        Rect rect(x, y, w, h);
        return rect;
    }
    Mat get_grid_likelihood(int Num)
    {
        Rect bounds(0, 0, 1100, 800);
        Rect r = get_grid_size(Num);
        Mat roi = Likelihood_mat(r & bounds);
        return roi;
    }

    int get_grid_index(int X, int Y)
    {
        if(X > 0 && Y > 0 && X <= 400 && Y <= 300){return 1;}
        else if(X > 400 && Y > 0 && X <= 700 && Y <= 150){return 2;}
        else if(X > 700 && Y > 0 && X <= 1100 && Y <= 300){return 3;}
        else if(X > 0 && Y > 300 && X <= 400 && Y <= 500){return 4;}
        else if(X > 400 && Y > 150 && X <= 700 && Y <= 650){return 5;}
        else if(X > 700 && Y > 300 && X <= 1100 && Y <= 500){return 6;}
        else if(X > 0 && Y > 500 && X <= 400 && Y <= 800){return 7;}
        else if(X > 400 && Y > 650 && X <= 700 && Y <= 800){return 8;}
        else if(X > 700 && Y > 500 && X <= 1100 && Y <= 800){return 9;}
        else{return 0;}
    }


};




#endif // LINE_H
