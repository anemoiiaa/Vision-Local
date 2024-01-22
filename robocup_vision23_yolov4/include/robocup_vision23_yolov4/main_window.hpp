/**
 * @file /include/robocup_vision23_yolov4/main_window.hpp
 *
 * @brief Qt based gui for robocup_vision23_yolov4.
 *
 * @date November 2010
 **/
#ifndef robocup_vision23_yolov4_MAIN_WINDOW_H
#define robocup_vision23_yolov4_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "../pan_tilt/pan_tilt.hpp"
#include "../YoloV4/YoloV4.hpp"
#include "../robit_vision/robitvision.hpp"
#include "../robit_vision/robit_master_vision.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>

#define DEG2RAD (M_PI / 180)

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace robocup_vision23_yolov4 {

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

  vector<Point2f> pts, condis;
  vector<String> data;

    //For calibration
    cv::Point2d focalLen;
    cv::Point2d prncPt;
    void caminfo_open(const cv::Mat& info)
    {

        Mat camMat(info); CamIntrParameter camInfo;

        camInfo.fx = camMat.at<double>(0, 0); camInfo.fy = camMat.at<double>(1, 1);
        camInfo.cx = camMat.at<double>(0, 2); camInfo.cy = camMat.at<double>(1, 2);

        focalLen = cv::Point2d(camInfo.fx, camInfo.fy); prncPt = cv::Point2d(camInfo.cx, camInfo.cy);

    }



    int ball_filter_x[30] = {0, };
    int ball_filter_y[30] = {0, };
    int ball_filter_cnt = 0;
    int ball_filter_idx = 0;



    Mat K_M = Mat(3, 3, CV_64FC1);
    Mat D_M = Mat(1, 5, CV_64FC1);
    Mat R_M = Mat(3, 3, CV_64FC1);
    Mat P_M = Mat(3, 4, CV_64FC1);

    Mat NEW_K_M = Mat(3, 3, CV_64FC1);

    cv::Point2d new_focalLen;
    cv::Point2d new_prncPt;

    char str_list[6];
    string str_name;
    int save_img_count = 0;


    void calibration_info()
    {
        String temp;
        double dtemp;
        double mtemp;
        double asc46;
        double K[9] = {0,0,0,0,0,0,0,0,0};
        double D[5] = {0,0,0,0,0};
        double R[9] = {0,0,0,0,0,0,0,0,0};
        double P[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
        int cnt;
        ifstream Calibration_Info("/home/robit/catkin_ws/src/ocam/config/camera.yaml");
        if (Calibration_Info.is_open()){
          for(int i = 0; i < 12; i++){Calibration_Info >> temp;}
          for(int i = 0; i < 9; i++)
          {
            char TEXT[256]; dtemp = 0; asc46 = 1; mtemp = 1;
            Calibration_Info >> temp;
            strcpy(TEXT, temp.c_str());
            for(int j = 0; j < temp.length(); j++)
            {
                if(TEXT[j] == 45){mtemp = -1;}
                if((TEXT[j] >= 48 && TEXT[j] <= 57) || TEXT[j] == 46){if(TEXT[j] == 46 || asc46 != 1){if(TEXT[j] != 46){dtemp += (TEXT[j] - 48) / asc46;} asc46 *= 10;}else {dtemp = (dtemp * 10) + (TEXT[j] - 48);}}
            }
            K[i] = mtemp * dtemp;
          }
          cnt = 0;
          for(int i = 0; i < K_M.rows; i++)
          {
              for(int j = 0; j < K_M.cols; j++){K_M.at<double>(i, j) = K[cnt]; cnt += 1;}
          }
          //cout<<K_M<<endl;
          //cout<<K[0]<<" "<<K[1]<<" "<<K[2]<<" "<<K[3]<<" "<<K[4]<<" "<<K[5]<<" "<<K[6]<<" "<<K[7]<<" "<<K[8]<<endl;
          for(int i = 0; i < 8; i++){Calibration_Info >> temp;}
          for(int i = 0; i < 5; i++)
          {
              char TEXT[256]; dtemp = 0; asc46 = 1; mtemp = 1;
              Calibration_Info >> temp;
              strcpy(TEXT, temp.c_str());
              for(int j = 0; j < temp.length(); j++)
              {
                  if(TEXT[j] == 45){mtemp = -1;}
                  if((TEXT[j] >= 48 && TEXT[j] <= 57) || TEXT[j] == 46){if(TEXT[j] == 46 || asc46 != 1){if(TEXT[j] != 46){dtemp += (TEXT[j] - 48) / asc46;} asc46 *= 10;}else {dtemp = (dtemp * 10) + (TEXT[j] - 48);}}
              }
              D[i] = mtemp * dtemp;
          }
          cnt = 0;
          for(int i = 0; i < D_M.rows; i++)
          {
              for(int j = 0; j < D_M.cols; j++){D_M.at<double>(i, j) = D[cnt]; cnt += 1;}
          }
          //cout<<D_M<<endl;
          //cout<<D[0]<<" "<<D[1]<<" "<<D[2]<<" "<<D[3]<<" "<<D[4]<<endl;
          for(int i = 0; i < 6; i++){Calibration_Info >> temp;}
          for(int i = 0; i < 9; i++)
          {
              char TEXT[256]; dtemp = 0; asc46 = 1; mtemp = 1;
              Calibration_Info >> temp;
              strcpy(TEXT, temp.c_str());
              for(int j = 0; j < temp.length(); j++)
              {
                  if(TEXT[j] == 45){mtemp = -1;}
                  if((TEXT[j] >= 48 && TEXT[j] <= 57) || TEXT[j] == 46){if(TEXT[j] == 46 || asc46 != 1){if(TEXT[j] != 46){dtemp += (TEXT[j] - 48) / asc46;} asc46 *= 10;}else {dtemp = (dtemp * 10) + (TEXT[j] - 48);}}
              }
              R[i] = mtemp * dtemp;
          }
          cnt = 0;
          for(int i = 0; i < R_M.rows; i++)
          {
              for(int j = 0; j < R_M.cols; j++){R_M.at<double>(i, j) = R[cnt]; cnt += 1;}
          }
          //cout<<R_M<<endl;
          //cout<<R[0]<<" "<<R[1]<<" "<<R[2]<<" "<<R[3]<<" "<<R[4]<<" "<<R[5]<<" "<<R[6]<<" "<<R[7]<<" "<<R[8]<<endl;
          for(int i = 0; i < 6; i++){Calibration_Info >> temp;}
          for(int i = 0; i < 12; i++)
          {
              char TEXT[256]; dtemp = 0; asc46 = 1; mtemp = 1;
              Calibration_Info >> temp;
              strcpy(TEXT, temp.c_str());
              for(int j = 0; j < temp.length(); j++)
              {
                  if(TEXT[j] == 45){mtemp = -1;}
                  if((TEXT[j] >= 48 && TEXT[j] <= 57) || TEXT[j] == 46){if(TEXT[j] == 46 || asc46 != 1){if(TEXT[j] != 46){dtemp += (TEXT[j] - 48) / asc46;} asc46 *= 10;}else {dtemp = (dtemp * 10) + (TEXT[j] - 48);}}
              }
              P[i] = mtemp * dtemp;
          }
          cnt = 0;
          for(int i = 0; i < P_M.rows; i++)
          {
              for(int j = 0; j < P_M.cols; j++){P_M.at<double>(i, j) = P[cnt]; cnt += 1;}
          }
          //cout<<P_M<<endl;
          //cout<<P[0]<<" "<<P[1]<<" "<<P[2]<<" "<<P[3]<<" "<<P[4]<<" "<<P[5]<<" "<<P[6]<<" "<<P[7]<<" "<<P[8]<<" "<<P[9]<<" "<<P[10]<<" "<<P[11]<<endl;
        }
        Calibration_Info.close();
        NEW_K_M = getOptimalNewCameraMatrix(K_M, D_M, Size(640, 480), 1, Size(640, 480), 0);
        cout<<NEW_K_M<<endl;


        new_focalLen = cv::Point2d(NEW_K_M.at<double>(0, 0), NEW_K_M.at<double>(1, 1)); new_prncPt = cv::Point2d(NEW_K_M.at<double>(0, 2), NEW_K_M.at<double>(1, 2));

    }

    //For Screen
    QPixmap buf;
    QGraphicsScene *scene;
    void Print_Screen(const cv::Mat& img, const cv::Mat& result_img);

    //For Pan_Tilt
    int cam_nice_point = 0;

    //For UI
    int HSV_flag = 0;
    void save_hsv();
    void setting();

    //For Yolo
    YOLOV4 yolov4 = YOLOV4();

    //For Image_processing
    int setting_flag = -1;
    int ball_cam_X = 0;
    int ball_cam_Y = 0;
    ObjectPos ballPos;
    ObjectPos linePos;
    ObjectPos robotPos;
    Mat Image_processing(const cv::Mat& img);

    //For timer
    int filter_cnt = 0;
    int fps_cnt = 0;
    int nice_cnt = 0;
    QTimer *m_Timer;

    //For filter
    double fst_filter_x = 0, fst_filter_y = 0;
    int fst_filter_cnt = 0;
    double sec_filter_x = 0, sec_filter_y = 0;
    int sec_filter_cnt = 0;
    double final_filter_x = 0;
    double final_filter_y = 0;
    double ball_speed_vec_x = 0;
    double ball_speed_vec_y = 0;
    double ball_speed_level = 0;

    //For line
    int edgecount;
    struct VISION_POINT
    {
        int NUM;
        int POINT_VEC_X;
        int POINT_VEC_Y;
    };
    VISION_POINT vision_point;
    vector<VISION_POINT> vision_point_vect;
    vector<int> featkind;
    vector<Point2f> edgepoint;
    vector<Point2f> featPoint;
    vector<Point2f> nomalLinePoint;
    void edge_line_point_detect(const cv::Mat& warp_res,const cv::Mat& result_mat, const cv::Mat& warp_mat2);
    void FeaturePointEctraction(const cv::Mat& warp_res,const cv::Mat& result_img, const cv::Mat& warp_mat2, const cv::Mat& skel);
    void normalLine(const cv::Mat& line_mat, const cv::Mat& result_mat, const cv::Mat& warp_mat2,const cv::Mat& warp_mat);


    //For Ros Topic
    void publish_vision_msg();
    void publish_localization_msg();







public Q_SLOTS:
    void filter_100ms();
    void update(const cv::Mat& img, const cv::Mat& info);
    void on_radioori_clicked(bool checked);
    void on_radiofield_clicked(bool checked);
    void on_radioline_clicked(bool checked);
    void on_btn_oper_pan_tilt_clicked(bool checked);
    void on_btn_mode_0_clicked();
    void on_btn_mode_1_clicked();
    void on_btn_mode_2_clicked();
    void on_btn_mode_3_clicked();


private:
	Ui::MainWindowDesign ui;
    QNode qnode;


};
struct HSV
{
    int hmin;
    int hmax;
    int smin;
    int smax;
    int vmin;
    int vmax;
    int dilate;
    int erode;
};

}  // namespace robocup_vision23_yolov4

#endif // robocup_vision23_yolov4_MAIN_WINDOW_H
