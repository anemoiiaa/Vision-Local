#ifndef ROBOT_H
#define ROBOT_H

#define REAL_MAP_SIZE_X 1100
#define REAL_MAP_SIZE_Y 800

#define DEG2RAD (M_PI / 180)
#include <ctime>

using namespace std;

class ROBOT
{
public:
    //For Init
    ROBOT(){}
    ROBOT(float x, float y)
    {
        this->x = x;
        this->y = y;
    }
    int state = 0;

    float x = 0.0;//REAL_MAP_SIZE_X / 2.0;
    float y = 0.0;//REAL_MAP_SIZE_Y / 2.0;
    float z = 0.0;

    float b = 0.0;

    float odom_fx = 0.30;
    float odom_bx = 0.30;

    float odom_ly = 0.30;
    float odom_ry = 0.30;

    float now_ik_param_x = 0;
    float now_ik_param_y = 0;

    float foward_noise = 5.0;
    float turn_noise = 10.0;
    float sense_noise = 15.0;

    struct POINT
    {
        unsigned int TIME;
        int X;
        int Y;
    };
    POINT point;
    vector<POINT> TIME_STAMP;

    double weight;

private:


public:
    void create_time_stamp(double Xmoved, double Ymoved)
    {
        if(Xmoved != 0 || Ymoved != 0)
        {
            time_t timer;
            timer = time(NULL);
            point.TIME = timer; point.X = this->x; point.Y = this->y;
            TIME_STAMP.push_back(point);
            if(TIME_STAMP.size() > 10){TIME_STAMP.erase(TIME_STAMP.begin());}
        }
        else
        {
            TIME_STAMP.clear();
        }
    }

    void move(double Xmoved, double Ymoved)
    {
        //time_stamp
        create_time_stamp(Xmoved, Ymoved);

        //odom
        this->now_ik_param_x = Xmoved;
        this->now_ik_param_y = Ymoved;
        if(Xmoved > 0){Xmoved *= odom_fx;}
        else{Xmoved *= odom_bx;}

        if(Ymoved > 0){Ymoved *= odom_ly;}
        else{Ymoved *= odom_ry;}

        this->x += Xmoved * cos((this->z + 90) * DEG2RAD) - Ymoved * sin((this->z + 90) * DEG2RAD);
        this->y -= Xmoved * sin((this->z + 90) * DEG2RAD) + Ymoved * cos((this->z + 90) * DEG2RAD);

        if(this->x < 100){this->x = 100;}
        if(this->x > 1000){this->x = 1000;}
        if(this->y < 100){this->y = 100;}
        if(this->y > 700){this->y = 700;}
    }
    void random_point(int x, int y, float particle_range)
    {
        this->x = x + gaussianRandom(0, particle_range);//30.0);
        this->y = y + gaussianRandom(0, particle_range);//30.0);
    }




private:
    double gaussianRandom(double average, double stdev)
    {
        double v1, v2, s, temp;
        do{
            v1 = 2 * ((double) rand() / RAND_MAX) - 1;      // -1.0 ~ 1.0 까지의 값
            v2 = 2 * ((double) rand() / RAND_MAX) - 1;      // -1.0 ~ 1.0 까지의 값
            s = v1 * v1 + v2 * v2;
        }while (s >= 1 || s == 0);
        s = sqrt((-2 * log(s)) / s);
        temp = v1 * s;
        temp = (stdev * temp) + average;
        return temp;
    }

};


#endif // ROBOT_H
