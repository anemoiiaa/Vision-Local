#ifndef YOLOV4_H
#define YOLOV4_H

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


using namespace std;
using namespace cv;
using namespace dnn;

class YOLOV4
{
public:
    //For Init
    YOLOV4()
    {
        YOLO = cv::dnn::readNetFromDarknet(cfg_name, weight_name);
        YOLO.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        YOLO.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }

    //For Open Model
    string cfg_name =  "/home/robit/catkin_ws/src/robocup_vision23_yolov4/net/yolov4-tiny-1ch-cls3-v7.cfg"; //"/home/robit/catkin_ws/src/robocup_vision23_yolov4/net/yolov4-tiny-1ch-cls2-v4.cfg"; //"/home/robit/catkin_ws/src/robocup_vision23_yolov4/net/yolov4-tiny-1ch-cls2-v4.cfg";
    string weight_name =  "/home/robit/catkin_ws/src/robocup_vision23_yolov4/net/yolov4-tiny-1ch-cls3-v7_100000.weights"; //"/home/robit/catkin_ws/src/robocup_vision23_yolov4/net/yolov4-tiny-1ch-cls2-v4_final.weights"; //"/home/robit/catkin_ws/src/robocup_vision23_yolov4/net/yolov4-tiny-1ch-cls2-v4_final.weights";
    //For Model Param
    int NUM_CLASSES = 3;
    double CONFIDENCE_THRESHOLD = 0.1;
    double SCORE_THRESHOLD_B = 0.8;
    double NMS_THRESHOLD_B = 0.8;
    double SCORE_THRESHOLD_L = 0.5;
    double NMS_THRESHOLD_L = 0.5;
    double SCORE_THRESHOLD_R = 0.8;
    double NMS_THRESHOLD_R = 0.8;
    //For save
    //Rect ago_rect(0,0,0,0);

    //For Run Yolo
    struct YOLO_RESULT
    {
        int NUM;
        double SCORE;
        int X;
        int Y;
        int W;
        int H;
    };
    YOLO_RESULT yolo_result;
    vector<YOLO_RESULT> vector_yolo;

    vector<YOLO_RESULT> YoloRun(const cv::Mat& img)
    {
        Mat image = img.clone();
        cvtColor(image, image, COLOR_BGR2GRAY);

        Ptr<CLAHE> clahe = createCLAHE();
        clahe->setClipLimit(2);
        clahe->apply(image,image);

        //cout<<image.channels()<<endl;
        Mat blob;

        vector<cv::Mat> detections;
        Mat inputBlob = blobFromImage(image, 1.0/225.0, Size(256, 192));
        YOLO.setInput(inputBlob);
        YOLO.forward(detections, YOLO.getUnconnectedOutLayersNames());

        vector<int> indices[NUM_CLASSES];
        vector<cv::Rect> boxes[NUM_CLASSES];
        vector<float> scores[NUM_CLASSES];
        for (auto& output : detections)
        {
            const auto num_boxes = output.rows;
            for (int i = 0; i < num_boxes; i++)
            {
                auto x = output.at<float>(i, 0) * image.cols;
                auto y = output.at<float>(i, 1) * image.rows;
                auto width = output.at<float>(i, 2) * image.cols;
                auto height = output.at<float>(i, 3) * image.rows;
                cv::Rect rect(x - width/2, y - height/2, width, height);

                for (int c = 0; c < NUM_CLASSES; c++)
                {
                    auto confidence = *output.ptr<float>(i, 5 + c);
                    if (confidence >= CONFIDENCE_THRESHOLD)
                    {
                        boxes[c].push_back(rect);
                        scores[c].push_back(confidence);
                    }
                }
            }
        }
        vector<YOLO_RESULT> result;
        for (int c = 0; c < NUM_CLASSES; c++)
        {
            if(c == 0)
            {
                cv::dnn::NMSBoxes(boxes[c], scores[c], SCORE_THRESHOLD_B, NMS_THRESHOLD_B, indices[c]);
            }
            else if(c == 1)
            {
                cv::dnn::NMSBoxes(boxes[c], scores[c], SCORE_THRESHOLD_L, NMS_THRESHOLD_L, indices[c]);
            }
            else if(c == 2)
            {
                cv::dnn::NMSBoxes(boxes[c], scores[c], SCORE_THRESHOLD_R, NMS_THRESHOLD_R, indices[c]);
            }
        }
        for (int c = 0; c < NUM_CLASSES; c++)
        {
            for (size_t i = 0; i < indices[c].size(); ++i)
            {
                auto idx = indices[c][i];
                const auto& rect = boxes[c][idx];
                yolo_result.NUM = c;
                yolo_result.SCORE = scores[c][idx];
                yolo_result.X = rect.x;
                yolo_result.Y = rect.y;
                yolo_result.W = rect.width;
                yolo_result.H = rect.height;
                result.push_back(yolo_result);
            }
        }
        return result;
    }



private:
    Net YOLO;

};




#endif // YOLOV4_H
