#include "robit_master_vision.h"

RobitLabeling::RobitLabeling()
{
    if(!m_Image.empty())           m_Image.release();

    if(!m_recBlobs.empty())        m_recBlobs.clear();

    m_nThreshold        = 0;
    m_nNeighbor         = 0;
    m_nBlobs            = 0;
    m_height            = 0;
    m_width             = 0;
}

RobitLabeling::RobitLabeling(const cv::Mat& Img,
                             const unsigned int& nThreshold,
                             const unsigned int& nNeighbor)
{
    if(!m_recBlobs.empty())        m_recBlobs.clear();

    m_Image             = Img.clone();
    m_Image_Obj         = Img;

    m_nThreshold        = nThreshold;
    m_nNeighbor         = nNeighbor;
    m_nBlobs            = 0;
    m_height            = Img.rows;
    m_width             = Img.cols;
}

void RobitLabeling::doLabeling()
{
    if(m_Image.channels() != 1)
    {
        std::cout << "Image type is not collect" << std::endl;
        return ;
    }

    switch (m_nNeighbor)
    {
    case 4:
        _labeling_four_neighbor();
        break;

    case 8:
        _labeling_eight_neighbor();
        break;

    default:
        std::cout << "Check nNeighbor" << std::endl;
        break;
    }
}

vector<cv::Rect> RobitLabeling::getRecBlobs()
{
    return m_recBlobs;
}

unsigned int RobitLabeling::getNumOfBlobs()
{
    return m_nBlobs;
}

void RobitLabeling::sortingRecBlobs()
{
    if(m_nBlobs < 2)
        return;

    _quick_sort_rec_blobs(0, m_nBlobs - 1);
}

void RobitLabeling::_labeling_four_neighbor()
{
    __dynamicAllocation();

    for(int nRow = 0, nowY = 0 ; nRow < m_height ; nRow ++, nowY += m_width)
        for(int nCol = 0 ; nCol < m_width ; nCol ++)
        {
            if(m_isVisited[nowY + nCol]   == true)     continue;
            if(m_Image.data[nowY + nCol]  == 0)        continue;

            m_Image.data[nowY + nCol]     = ++m_nBlobs;
            m_isVisited[nowY + nCol]      = true;

            cv::Point start_pt   = cv::Point(nCol, nRow);
            cv::Point end_pt     = cv::Point(nCol, nRow);


            if(__check_four_neighbor(start_pt, end_pt) > m_nThreshold)
            {
                m_recBlobs.push_back(cv::Rect(start_pt, end_pt));
                m_vecArea.push_back(m_area);
            }
            else
            {
                for(int nRow = start_pt.y, nowY = start_pt.y * m_width ; nRow <= end_pt.y ; nRow ++, nowY += m_width)
                    for(int nCol = start_pt.x ; nCol <= end_pt.x ; nCol ++)
                        if(m_Image.data[nowY + nCol] == m_nBlobs)
                            m_Image.data[nowY + nCol] = 0;

                m_nBlobs--;
            }
        }
    __freeAllocation();
}

void RobitLabeling::_labeling_eight_neighbor()
{
    __dynamicAllocation();

    for(int nRow = 0, nowY = 0 ; nRow < m_height ; nRow ++, nowY += m_width)
        for(int nCol = 0 ; nCol < m_width ; nCol ++)
        {
            if(m_isVisited[nowY + nCol]   == true)     continue;
            if(m_Image.data[nowY + nCol]  == 0)        continue;

            m_Image.data[nowY + nCol]     = ++m_nBlobs;
            m_isVisited[nowY + nCol]      = true;

            cv::Point start_pt   = cv::Point(nCol, nRow);
            cv::Point end_pt     = cv::Point(nCol, nRow);


            if(__check_eight_neighbor(start_pt, end_pt) > m_nThreshold)
            {
                m_recBlobs.push_back(cv::Rect(start_pt, end_pt));
                m_vecArea.push_back(m_area);
            }
            else
            {
                for(int nRow = start_pt.y, nowY = start_pt.y * m_width ; nRow <= end_pt.y ; nRow ++, nowY += m_width)
                    for(int nCol = start_pt.x ; nCol <= end_pt.x ; nCol ++)
                        if(m_Image.data[nowY + nCol] == m_nBlobs)
                            m_Image.data[nowY + nCol] = 0;

                m_nBlobs--;
            }
        }
    __freeAllocation();
}

void RobitLabeling::_quick_sort_rec_blobs(int left, int right)
{
    int L = left;
    int R = right;
    unsigned int pivot = m_vecArea[(left + right) / 2];

    while(L <= R)
    {
        while(m_vecArea[L] > pivot)     L++;
        while(m_vecArea[R] < pivot)     R--;
        if( L <= R)
        {
            if(L != R)
            {
                __swap(m_vecArea[L] , m_vecArea[R]);
                __swap(m_recBlobs[L], m_recBlobs[R]);
            }
            L++;
            R--;
        }
    }
    if(left < R)        _quick_sort_rec_blobs(left, R);
    if(L < right)       _quick_sort_rec_blobs(L, right);
}

void RobitLabeling::__dynamicAllocation()
{
    m_isVisited          = new bool [m_Image.size().area()];
    m_visited_pt         = new cv::Point [m_Image.size().area()];

    memset(m_isVisited,  false, m_Image.size().area() * sizeof(bool));
    memset(m_visited_pt, false, m_Image.size().area() * sizeof(cv::Point));
}

void RobitLabeling::__freeAllocation()
{
    delete[]          m_isVisited;
    delete[]          m_visited_pt;

    m_isVisited       = NULL;
    m_visited_pt      = NULL;
}

const unsigned int RobitLabeling::__check_four_neighbor(cv::Point &start_pt, cv::Point &end_pt)
{
    cv::Point present_pt = start_pt;
    m_area=1;
    const int Visit[4] = {-1, 1, -m_width, m_width};            //왼,오,상,하

    for(;;)
    {
        unsigned int nowIdx=(present_pt.y * m_width) +  present_pt.x;

        if(present_pt.x != 0 &&
                m_isVisited[nowIdx + Visit[LEFT]] == false &&
                m_Image.data[nowIdx + Visit[LEFT]] != 0)
        {
            m_isVisited[nowIdx + Visit[LEFT]] = true;
            m_Image.data[nowIdx + Visit[LEFT]] = m_Image.data[nowIdx];
            m_visited_pt[nowIdx + Visit[LEFT]] = present_pt;
            present_pt.x --;
            if(start_pt.x > present_pt.x)
                start_pt.x = present_pt.x;
            m_area++;
            continue;
        }

        if(present_pt.x != m_width - 1 &&
                m_isVisited[nowIdx + Visit[RIGHT]] == false &&
                m_Image.data[nowIdx + Visit[RIGHT]] != 0)
        {
            m_isVisited[nowIdx + Visit[RIGHT]] = true;
            m_Image.data[nowIdx + Visit[RIGHT]] = m_Image.data[nowIdx];
            m_visited_pt[nowIdx + Visit[RIGHT]] = present_pt;
            present_pt.x ++;
            if(end_pt.x < present_pt.x)
                end_pt.x = present_pt.x;
            m_area++;
            continue;
        }

        if(present_pt.y != 0 &&
                m_isVisited[nowIdx + Visit[UP]] == false &&
                m_Image.data[nowIdx + Visit[UP]] != 0)
        {
            m_isVisited[nowIdx + Visit[UP]] = true;
            m_Image.data[nowIdx + Visit[UP]] = m_Image.data[nowIdx];
            m_visited_pt[nowIdx + Visit[UP]] = present_pt;
            present_pt.y --;
            if(start_pt.y > present_pt.y)
                start_pt.y = present_pt.y;
            m_area++;
            continue;
        }

        if(present_pt.y != m_height - 1 &&
                m_isVisited[nowIdx + Visit[DOWN]] == false &&
                m_Image.data[nowIdx + Visit[DOWN]] != 0)
        {
            m_isVisited[nowIdx + Visit[DOWN]] = true;
            m_Image.data[nowIdx + Visit[DOWN]] = m_Image.data[nowIdx];
            m_visited_pt[nowIdx + Visit[DOWN]] = present_pt;
            present_pt.y ++;
            if(end_pt.y < present_pt.y)
                end_pt.y = present_pt.y;
            m_area++;
            continue;
        }

        if(m_visited_pt[nowIdx] == present_pt)    break;
        present_pt = m_visited_pt[nowIdx];
    }
    return m_area;
}

const unsigned int RobitLabeling::__check_eight_neighbor(cv::Point &start_pt, cv::Point &end_pt)
{
    cv::Point present_pt = start_pt;
    m_area=1;
    const int Visit[8] = {-1, -m_width -1, -m_width, -m_width + 1,
                          1, m_width + 1,  m_width,  m_width - 1};

    for(;;)
    {
        unsigned int nowIdx=(present_pt.y * m_width) +  present_pt.x;

        if(present_pt.x != 0 &&
                m_isVisited[nowIdx + Visit[W]] == false &&
                m_Image.data[nowIdx + Visit[W]] != 0)
        {
            m_isVisited[nowIdx + Visit[W]] = true;
            m_Image.data[nowIdx + Visit[W]] = m_Image.data[nowIdx];
            m_visited_pt[nowIdx + Visit[W]] = present_pt;
            present_pt.x --;
            if(start_pt.x > present_pt.x)
                start_pt.x = present_pt.x;
            m_area++;
            continue;
        }

        if(present_pt.x != 0 &&
                present_pt.y != 0 &&
                m_isVisited[nowIdx + Visit[NW]] == false &&
                m_Image.data[nowIdx + Visit[NW]] != 0)
        {
            m_isVisited[nowIdx + Visit[NW]] = true;
            m_Image.data[nowIdx + Visit[NW]] = m_Image.data[nowIdx];
            m_visited_pt[nowIdx + Visit[NW]] = present_pt;
            present_pt.x --;
            present_pt.y --;

            if(start_pt.x > present_pt.x)
                start_pt.x = present_pt.x;

            if(start_pt.y > present_pt.y)
                start_pt.y = present_pt.y;
            m_area++;
            continue;
        }

        if(present_pt.y != 0 &&
                m_isVisited[nowIdx + Visit[N]] == false &&
                m_Image.data[nowIdx + Visit[N]] != 0)
        {
            m_isVisited[nowIdx + Visit[N]] = true;
            m_Image.data[nowIdx + Visit[N]] = m_Image.data[nowIdx];
            m_visited_pt[nowIdx + Visit[N]] = present_pt;
            present_pt.y --;
            if(start_pt.y > present_pt.y)
                start_pt.y = present_pt.y;
            m_area++;
            continue;
        }

        if(present_pt.x != m_width - 1 &&
                present_pt.y != 0 &&
                m_isVisited[nowIdx + Visit[NE]] == false &&
                m_Image.data[nowIdx + Visit[NE]] != 0)
        {
            m_isVisited[nowIdx + Visit[NE]] = true;
            m_Image.data[nowIdx + Visit[NE]] = m_Image.data[nowIdx];
            m_visited_pt[nowIdx + Visit[NE]] = present_pt;
            present_pt.x ++;
            present_pt.y --;

            if(end_pt.x < present_pt.x)
                end_pt.x = present_pt.x;

            if(start_pt.y > present_pt.y)
                start_pt.y = present_pt.y;
            m_area++;
            continue;
        }

        if(present_pt.x != m_width - 1 &&
                m_isVisited[nowIdx + Visit[E]] == false &&
                m_Image.data[nowIdx + Visit[E]] != 0)
        {
            m_isVisited[nowIdx + Visit[E]] = true;
            m_Image.data[nowIdx + Visit[E]] = m_Image.data[nowIdx];
            m_visited_pt[nowIdx + Visit[E]] = present_pt;
            present_pt.x ++;

            if(end_pt.x < present_pt.x)
                end_pt.x = present_pt.x;
            m_area++;
            continue;
        }

        if(present_pt.x != m_width - 1 &&
                present_pt.y != m_height - 1 &&
                m_isVisited[nowIdx + Visit[SE]] == false &&
                m_Image.data[nowIdx + Visit[SE]] != 0)
        {
            m_isVisited[nowIdx + Visit[SE]] = true;
            m_Image.data[nowIdx + Visit[SE]] = m_Image.data[nowIdx];
            m_visited_pt[nowIdx + Visit[SE]] = present_pt;
            present_pt.x ++;
            present_pt.y ++;

            if(end_pt.x < present_pt.x)
                end_pt.x = present_pt.x;

            if(end_pt.y < present_pt.y)
                end_pt.y = present_pt.y;
            m_area++;
            continue;
        }

        if(present_pt.y != m_height - 1 &&
                m_isVisited[nowIdx + Visit[S]] == false &&
                m_Image.data[nowIdx + Visit[S]] != 0)
        {
            m_isVisited[nowIdx + Visit[S]] = true;
            m_Image.data[nowIdx + Visit[S]] = m_Image.data[nowIdx];
            m_visited_pt[nowIdx + Visit[S]] = present_pt;
            present_pt.y ++;
            if(end_pt.y < present_pt.y)
                end_pt.y = present_pt.y;
            m_area++;
            continue;
        }

        if(present_pt.x != 0 &&
                present_pt.y != m_height - 1 &&
                m_isVisited[nowIdx + Visit[SW]] == false &&
                m_Image.data[nowIdx + Visit[SW]] != 0)
        {
            m_isVisited[nowIdx + Visit[SW]] = true;
            m_Image.data[nowIdx + Visit[SW]] = m_Image.data[nowIdx];
            m_visited_pt[nowIdx + Visit[SW]] = present_pt;
            present_pt.x --;
            present_pt.y ++;

            if(start_pt.x > present_pt.x)
                start_pt.x = present_pt.x;

            if(end_pt.y < present_pt.y)
                end_pt.y = present_pt.y;
            m_area++;
            continue;
        }

        if(m_visited_pt[nowIdx] == present_pt)    break;
        present_pt = m_visited_pt[nowIdx];
    }
    return m_area;
}

bool RobitLabeling::mergeArea(Mat &ImgOrg){

    const int IMGSIZE = ImgOrg.size().area();


    for(int i = 0; i<IMGSIZE; i++){
        if( !m_Image.data[i] )
            m_Image_Obj.data[i] = 0;
    }

    bool isField = 0;

    cv::Point pastCen(0,0);

    for(int i = 0; i< m_nBlobs; i++){
        const int sX = m_recBlobs[i].x;
        const int sY = m_recBlobs[i].y;

        const int eX = m_recBlobs[i].x + m_recBlobs[i].width;
        const int eY = m_recBlobs[i].y + m_recBlobs[i].height;

        if(m_recBlobs[i].size().area() < 10000){
            for(int y = sY, idxY = sY * m_width; y<eY;y++, idxY += m_width){
                for(int x = sX; x < eX; x++)
                {
                    if(m_Image.data[idxY + x] == i + 1)
                        m_Image_Obj.data[idxY + x] = 0;
                }
            }
            continue;
        }      //블랍중에 n보다 작은크기의 블랍을 제거

        isField = true;

        int CenX = 0, CenY = 0, nCount = 0;
        Point exceptionPt;

        for(int fieldY = sY, fieldIdxY = sY * m_width; fieldY < eY; fieldY++, fieldIdxY += m_width)// y고정하고 밑에 for문에서 x++ 밑에 for문 끝나면 가로 크기 만큼 더한다(여기서는 fieldIdxY += img_bg.cols 이 부분)
        {
            for(int fieldX = sX; fieldX < eX; fieldX++)
            {
                if(m_Image.data[fieldIdxY + fieldX] == i + 1)
                {
                    CenX += fieldX;
                    CenY += fieldY;
                    nCount++;
                    exceptionPt.x = fieldX;
                    exceptionPt.y = fieldY;
                }
            }
        }

        if(nCount != 0)
        {
            CenX /= nCount;
            CenY /= nCount;

            if(m_Image.data[CenY * m_width + CenX] != i + 1)
            {
                CenX = exceptionPt.x;
                CenY = exceptionPt.y;
            }
        }


        if(pastCen.x != 0 && pastCen.y != 0)
        {
            if(cv::norm(Point(CenX, CenY) - exceptionPt) < 1500)
            {
                cv::line(m_Image,
                         pastCen,
                         Point(CenX, CenY),
                         Scalar(16),
                         3
                         );
            }
        }

        pastCen.x = CenX;
        pastCen.y = CenY;
        //블랍들을 합쳐서 하나의 큰 에리어로 합치는 과정
    }

    if(isField){
        vector< vector< Point > >contours;
        vector< Vec4i >hierarchy;


        cv::Mat img_contour = m_Image;


        cv::findContours(img_contour, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
        vector< vector< Point > >hull(contours.size());

        for(size_t s = 0; s < contours.size(); s++) convexHull(Mat(contours[s]), hull[s], false);

        drawing = Mat::zeros(img_contour.size(), CV_8UC1);
        Mat drawingContours = Mat::zeros(img_contour.size(), CV_8UC1);

        int maxContours = -1;
        int maxSize = 0;

        for(size_t s = 0; s < contours.size(); s++)
        {
            if(hierarchy[s][3] != -1)    continue;

            int nowSize = contours[s].size();
            if(nowSize > maxSize)
            {
                maxSize = nowSize;
                maxContours = s;
            }

            cv::drawContours(drawing, hull, s, Scalar(69), 2, 8, hierarchy, 0, Point());

            cv::drawContours(drawingContours, contours, s, Scalar(70), 2, 8, hierarchy, 0, Point());

        }

        if(maxContours != -1)
        {
            cv::drawContours(ImgOrg, hull, (size_t)maxContours, Scalar(100, 255, 0), 3, 8, hierarchy, 0, Point());
        }

    }

    return isField;
}

void RobitLabeling::eraseOutField(Mat &Img,Mat& Img2, int nMore,Mat &imgOrg){

    /////////////////////////////////////////////////////////////////////////////////////////////////////////// x->y+->y-


    for(int x = 0; x < drawing.cols; x++)
    {
        for(int y = 0; y < drawing.rows; y++)
        {
            Img.data[y*drawing.cols + x] = 0;
//            Img2.data[y*drawing.cols + x] = 0;

              imgOrg.at<Vec3b>(y,x) = Vec3b(0,0,0);//배경제거
              Img2.at<Vec3b>(y,x) = Vec3b(0,0,0);//배경제거




            if(drawing.data[y*drawing.cols + x] == 69)  //69
            {
                for(int i = 1; i <= nMore; i++){
                    if(y+i >= drawing.rows) break;

                        Img.data[(y+i)*drawing.cols + x] = 0;
//                        Img2.data[(y+i)*drawing.cols + x] = 0;

                }
                break;
            }
        }

        for(int y = drawing.rows-1; y >= 0; y--)
        {
            Img.data[y*drawing.cols + x] = 0;
//            Img2.data[y*drawing.cols + x] = 0;

               imgOrg.at<Vec3b>(y,x) = Vec3b(0,0,0);//배경제거
               Img2.at<Vec3b>(y,x) = Vec3b(0,0,0);//배경제거

            if(drawing.data[y*drawing.cols + x] == 69)
            {
                for(int i = 1; i <= nMore; i++){
                    if(y- i < 0) break;

                        Img.data[(y-i)*drawing.cols + x] = 0;
//                        Img2.data[(y-i)*drawing.cols + x] = 0;

                }
                break;
            }
            //blackbottom();


        }
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////// y->x+->x-
//    for(int y = 0; y < drawing.rows; y++)
//    {
//        for(int x = 0; x < drawing.cols; x++)
//        {
//            Img.data[x*drawing.rows + y] = 0;


//            if(drawing.data[x*drawing.rows + y] == 69)  //69
//            {
//                for(int i = 1; i <= nMore; i++){
//                    if(x+i >= drawing.cols) break;

//                        Img.data[(x+i)*drawing.rows + y] = 0;

//                }
//                break;
//            }
//        }

//        for(int x = drawing.cols-1; x >= 0; x--)
//        {
//            Img.data[x*drawing.rows + y] = 0;


//            if(drawing.data[x*drawing.rows + y] == 69)
//            {
//                for(int i = 1; i <= nMore; i++){
//                    if(x-i < 0) break;

//                        Img.data[(x-i)*drawing.rows + y] = 0;

//                }
//                break;
//            }
//        }
//    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
//    for(unsigned int i = 0; i < IMGSIZE; i++)
//    {
//        if(m_Image.data[i] == 255)
//        {
//            Img.data[i] = 0;


//        }


//    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////


}

RobitLabeling::~RobitLabeling()
{
    m_Image.release();
}


RobitLineDetect::RobitLineDetect()
    : RobitLabeling()
{

}

RobitLineDetect::RobitLineDetect(const Mat &Img,
                                 const unsigned int &nThreshold,
                                 const unsigned int &nNeighbor)
    : RobitLabeling(Img, nThreshold, nNeighbor)
{

}

RobitLineDetect::RobitLineDetect(const RobitLabeling& labels)
{

}

void RobitLineDetect::findLineFeatures()
{

}

RobitLineDetect::~RobitLineDetect()
{

}
