#include "feature_tracker.h"

//FeatureTracker的static成员变量n_id初始化为0
int FeatureTracker::n_id = 0;
int FeatureTracker::n_line_id = 0;
int FeatureTracker::n_line_id_2 = 0;
int cnt_point=0;
float all_point = 0;
 int sample_point_num = 5;
 int p = 5;
 int flag=0;
 int line_track_cnt_1=0;

std::vector<cv::Point2f> connects;
void selectseedconnects(int p, std::vector<cv::Point2f> &connects)
{
    for(int i=0;i<=(p-1)/2;i++)
    {
        for(int j=-i;j<=i;j++)
        {
          if(j==-i||j==i)
          {
            for(int k=-i;k<=i;k++)
            {
              connects.push_back(cv::Point2f(k,j));
            }
          }
          else
          {
            connects.push_back(cv::Point2f(i,j));
            connects.push_back(cv::Point2f(-i,j));
          }
        }
    }
}



//判断跟踪的特征点是否在图像边界内
bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    //cvRound()：返回跟参数最接近的整数值，即四舍五入；
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

bool inBorder(const cv::Vec4f &line)
{
    const int BORDER_SIZE = 2;
    int start_x = cvRound(line[0]);
    int start_y = cvRound(line[1]);
    int end_x = cvRound(line[2]);
    int end_y = cvRound(line[3]);
    return (BORDER_SIZE <= start_x && start_x < COL - BORDER_SIZE && BORDER_SIZE <= start_y && start_y < ROW - BORDER_SIZE) ||
            (BORDER_SIZE <= end_x && end_x < COL - BORDER_SIZE && BORDER_SIZE <= end_y && end_y < ROW - BORDER_SIZE);
}

//去除无法跟踪的特征点
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

//去除无法追踪到的特征点
void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

 static bool judgeMidPoint(cv::Vec4f &cur_line, cv::Vec4f &forw_line)
{
    Point2d cur_mid_point((cur_line[0]+cur_line[2])/2, (cur_line[1]+cur_line[3])/2);
    Point2d forw_mid_point((forw_line[0]+forw_line[2])/2, (forw_line[1]+forw_line[3])/2);
    double dist = sqrt(pow((cur_mid_point.x-forw_mid_point.x),2)+pow((cur_mid_point.y-forw_mid_point.y),2));
    if(dist < sqrt(pow(cur_line[0] - cur_line[1],2)+pow(cur_line[2] - cur_line[3],2)) && dist<40)
        return true;
    return false;
}




 int binarySearch( vector<float> array, float target) 
{
    int left =0 ;
    int right = array.size()-1;
    while (left<=right)
    {
        int middle = left + ((right-left)>>1);
        if(array[middle] > target)
        {
            right = middle -1;
        }
        else if (array[middle] < target)
        {
            left = middle +1;
        }
        else
        {
            return middle;
        }
        
    }
    return -1;
    
}

  void quickSort(float left, float right, vector<float> &forw_mix_x,vector<int> &order)
{
	if(left >= right)
		return;
	int i, j,temp_order,base2;
  float base, temp;//,base2;
	i = left, j = right;
	base = forw_mix_x[left];  //取最左边的数为基准数
  base2 = order[left];
	while (i < j)
	{
		while (forw_mix_x[j] >= base && i < j)
			j--;
		while (forw_mix_x[i] <= base && i < j)
			i++;
		if(i < j)
		{
			temp = forw_mix_x[i];
			forw_mix_x[i] = forw_mix_x[j];
			forw_mix_x[j] = temp;

      temp_order=order[i];
      order[i] =order[j];
      order[j] = temp_order;
		}
	}
	//基准数归位
	forw_mix_x[left] = forw_mix_x[i];
  order[left] = order[i];

	forw_mix_x[i] = base;
  order[i] = base2;

	quickSort(left, i - 1, forw_mix_x,order);//递归左边
	quickSort(i + 1, right, forw_mix_x,order);//递归右边
}

float getThetaDiff(float angel1,float angel_line,float thread)
{
    float tmp = 5;
    tmp = abs(angel_line - angel1);
    if(tmp>63.75&&tmp<=127.50)
      tmp = 127.50 - tmp;
    else if(tmp>127.50 && tmp<=191.25)
      tmp = tmp - 127.50;
    else if(tmp>191.25 && tmp<=255)
      tmp = 255 - tmp;

    tmp = tmp -thread;
    return tmp;
}
void getmintemp(cv::Mat magnitude,cv::Point2f points1,cv::Point2f points2,float &min,float &temp)
{
        if(magnitude.at<uchar>(points1.y,points1.x) < magnitude.at<uchar>(points2.y,points2.x))
        {
          min = magnitude.at<uchar>(points1.y,points1.x);
        }
        else
        {
          min = magnitude.at<uchar>(points2.y,points2.x);
        }
        temp = magnitude.at<uchar>(points2.y,points2.x)-magnitude.at<uchar>(points1.y,points1.x) ;
        if(min == 0)
        {
          min = temp*0.01;
        }
}

//空的构造函数
FeatureTracker::FeatureTracker()
{
}

/**
 * @brief   对跟踪点进行排序并去除密集点
 * @Description 对跟踪到的特征点，按照被追踪到的次数排序并依次选点
 *              使用mask进行类似非极大抑制，半径为30，去掉密集点，使特征点分布均匀            
 * @return      void
*/
void FeatureTracker::setMask()
{
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    
    // prefer to keep features that are tracked for long time
    //构造(cnt，pts，id)序列
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    //对光流跟踪到的特征点forw_pts，按照被跟踪到的次数cnt从大到小排序
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    //清空cnt，pts，id并重新存入
    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    for (auto &it : cnt_pts_id)
    {
        if (mask.at<uchar>(it.second.first) == 255)
        {
            //当前特征点位置对应的mask值为255，则保留当前特征点，将对应的特征点位置pts，id，被追踪次数cnt分别存入
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);

            //在mask中将当前特征点周围半径为MIN_DIST的区域设置为0，后面不再选取该区域内的点（使跟踪点不集中在一个区域上）
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

//添将新检测到的特征点n_pts
void FeatureTracker::addPoints()
{
    for (auto &p : n_pts)
    {
        forw_pts.push_back(p);
        ids.push_back(-1);//新提取的特征点id初始化为-1
        track_cnt.push_back(1);//新提取的特征点被跟踪的次数初始化为1
    }
}

//读入图片进行处理
void FeatureTracker::readImage(const cv::Mat &_img,double _cur_time,vector<float> &array_startx, vector<float> &array_starty, vector<float> &array_endx, vector<float> &array_endy,int cnt_img,cv_bridge::CvImageConstPtr ptr)
{
    cv::Mat img;
    cur_time = _cur_time;
    cv::Mat imageMat2;
    imageMat2 = _img.clone();
    

    //如果EQUALIZE=1，表示太亮或太暗
    if (EQUALIZE)//判断是否进行直方图均衡化处理
    {
        //自适应直方图均衡
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(_img, img);
    }
    else
    {
        img = _img;
    }

    if (forw_img.empty())
    {
        cur_img = forw_img = img;
        cur_ptr = forw_ptr = ptr;
    }
    else
    {
        forw_img = img;
        forw_ptr = ptr;
    }
    // imageMat2 = img;
    forw_pts.clear();//此时forw_pts还保存的是上一帧图像中的特征点，所以把它清除




    //-------------------------------------上面是预处理----------------------------------------
    if (cur_pts.size() > 0)
    {
        vector<uchar> status;
        vector<float> err;

        //调用cv::calcOpticalFlowPyrLK()对前一帧的特征点cur_pts进行LK金字塔光流跟踪，得到forw_pts
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        //将位于图像边界外的点标记为0
        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;

        //根据status,把跟踪失败的点剔除
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
    }

    //光流追踪成功,特征点被成功跟踪的次数就加1,数值代表被追踪的次数，数值越大，说明被追踪的就越久
    for (auto &n : track_cnt)
        n++;

    //PUB_THIS_FRAME=1 需要发布特征点
    if (PUB_THIS_FRAME)
    {
        //通过基本矩阵剔除outliers
        rejectWithF();

        ROS_DEBUG("set mask begins");

        setMask();//保证相邻的特征点之间要相隔30个像素,设置mask

        ROS_DEBUG("detect feature begins");

        //计算是否需要提取新的特征点
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0)
        {
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;

            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear();

        ROS_DEBUG("add feature begins");

        //添将新检测到的特征点n_pts添加到forw_pts中，id初始化-1,track_cnt初始化为1.
        addPoints();

    }
    
    vector<cv::Vec4f> lines_temp;
    
     
    for (int i = 0; i < array_startx.size(); i++)
    {
        cv::Vec4f a_line;
        a_line[0] =  array_startx[i];
        a_line[1] =  array_starty[i];
        a_line[2] =  array_endx[i];
        a_line[3] =  array_endy[i];
        if((array_startx[i]-array_endx[i])!=0||(array_starty[i]-array_endy[i])!=0)
        {
            lines_temp.push_back(a_line);
        }
    }

    selectseedconnects(p,connects);

    int width = forw_img.rows-2;
	int height = forw_img.cols-2;
    cv::Mat grad_x, grad_y;
    
    Sobel(forw_img,grad_x,CV_32F,1,0);
	Sobel(forw_img,grad_y,CV_32F,0,1);
    cv::Mat magnitude, angle,magnitude_tmp, angle_tmp,magnitude_show2;
    cv::cartToPolar(grad_x, grad_y, magnitude_tmp, angle_tmp,true);
    magnitude_tmp.convertTo(magnitude, CV_8UC1);
    angle_tmp.convertTo(angle, CV_8UC1,255.0/360);
    forw_lines = lines_temp;
    vector<cv::Vec4f> lines = forw_lines;
    vector<cv::Vec4f> new_lines;
    for(int k=0;k<lines.size();k++)
    {
        cv::Vec4f lines_temp_2;
        lines_temp_2[0] = lines[k][0];
        lines_temp_2[1] = lines[k][1];
        lines_temp_2[2] = lines[k][2];
        lines_temp_2[3] = lines[k][3];
        float x2 = lines_temp_2[0] - lines_temp_2[2];
        float y2 = lines_temp_2[1] - lines_temp_2[3];
        float lengh_line = sqrt(x2*x2 + y2*y2);
        float cos2 = x2 / lengh_line;
        float sin2 = y2 / lengh_line;
        float angle_line = (acos(cos2)*57.6);
        vector<cv::Point2f> seed;
        seed.push_back(cv::Point(lines_temp_2[0],lines_temp_2[1]));
        seed.push_back(cv::Point((lines_temp_2[2]+lines_temp_2[0])/2,(lines_temp_2[3]+lines_temp_2[1])/2));
        seed.push_back(cv::Point(lines_temp_2[2],lines_temp_2[3]));
        if(cos2 * sin2<0)
        {
            if(angle_line<90)
            {
            angle_line  = 180 - angle_line;
            }
        }
        else
        {
            if(angle_line>90)
            {
            angle_line  = 180 - angle_line;
            }
        }
        angle_line = angle_line - 90;
        if(angle_line<0)
        {
            angle_line = angle_line+180;
        }
        angle_line = angle_line/360*255;
        cv::Point2f point_temp;
        vector<cv::Point2f> new_lines_temp_2;
        for(int j=0;j<seed.size();j++)
        {
            cv::Point2f max_position;
            cv::Point2f avg_position;
            float avg_grad=0,all_avg_grad=0;
            float point_count=0;
            int max_grad;
            int max_flag=0;
            
        
            point_temp = seed[j];
            for(int i=0;i<connects.size();i++)
            {
                int tmpX = point_temp.x + connects[i].x;
                int tmpY = point_temp.y + connects[i].y;
                if(tmpX<=0||tmpY<=0||tmpX>height||tmpY>width)
                    continue;
                else
                {
                  float tmp_avg_angel = angle.at<uchar>(tmpY,tmpX);
                  

                    
                    float grayDiff = getThetaDiff(tmp_avg_angel,angle_line,30);
                    if(grayDiff<0)
                    {
                        float tmp_avg_grad = magnitude.at<uchar>(tmpY,tmpX);
                        if(!max_flag)
                        {
                            
                            if(tmp_avg_grad!=0)
                            {
                                point_count++;
                                float avg_tmp;
                                avg_tmp = avg_grad/(avg_grad+tmp_avg_grad);
                                avg_position.x = (avg_position.x*(point_count-1) + tmpX*(1-avg_tmp))/(point_count-avg_tmp);
                                avg_position.y = (avg_position.y*(point_count-1) + tmpY*(1-avg_tmp))/(point_count-avg_tmp);
                                avg_grad = tmp_avg_grad;
                                all_avg_grad = avg_grad;
                            }
                        }
                        else
                        {
                        if(tmp_avg_grad!=0)
                        if((avg_grad - tmp_avg_grad)/tmp_avg_grad<0.25)
                        {
                            point_count++;
                            float avg_tmp;
                            avg_tmp = avg_grad/(avg_grad+magnitude.at<uchar>(tmpY,tmpX));
                            avg_position.x = (avg_position.x*(point_count-1) + tmpX*(1-avg_tmp))/(point_count-avg_tmp);
                            avg_position.y = (avg_position.y*(point_count-1) + tmpY*(1-avg_tmp))/(point_count-avg_tmp);
                            avg_grad = magnitude.at<uchar>(avg_position.y,avg_position.x);
                            all_avg_grad = (all_avg_grad*(point_count-1) + avg_grad)/point_count;
                        }
                        }
                    }
                }
            }
            if(sqrt((avg_position.x-point_temp.x)*(avg_position.x-point_temp.x) + (avg_position.y-point_temp.y)*(avg_position.y - point_temp.y))>10)//||avg_tmp<point_tmp)
              new_lines_temp_2.push_back(point_temp);
            else
            {
              new_lines_temp_2.push_back(avg_position);
            }

        }

        float step = 0.5;
        float step2 = 0.75;
        cv::Point2f mid_temp1,mid_temp2,mid_temp3;
        mid_temp1.x = new_lines_temp_2[0].x*(1-step) + new_lines_temp_2[1].x*step;
        mid_temp1.y = new_lines_temp_2[0].y*(1-step) + new_lines_temp_2[1].y*step;
        mid_temp2.x = new_lines_temp_2[1].x*(1-step) + new_lines_temp_2[2].x*step;
        mid_temp2.y = new_lines_temp_2[1].y*(1-step) + new_lines_temp_2[2].y*step;
        float grad_1 = magnitude.at<uchar>(mid_temp1.y,mid_temp1.x);
        float grad_2 = magnitude.at<uchar>(mid_temp2.y,mid_temp2.x);
        float min1,min2,min3,min4,min5;
        float tmp1,tmp2,tmp3,tmp4,tmp5;
        getmintemp(magnitude,mid_temp1,new_lines_temp_2[0],min1,tmp1);
        getmintemp(magnitude,mid_temp2,new_lines_temp_2[2],min2,tmp2);
        getmintemp(magnitude,new_lines_temp_2[1],new_lines_temp_2[0],min3,tmp3);
        getmintemp(magnitude,new_lines_temp_2[1],new_lines_temp_2[2],min4,tmp4);

        cv::Vec4f new_lines_temp;
        if(tmp3/min3 >2||tmp4/min4 >2||magnitude.at<uchar>((lines_temp_2[1]+lines_temp_2[3])/2,(lines_temp_2[0]+lines_temp_2[2])/2)>
        magnitude.at<uchar>(new_lines_temp_2[1].y,new_lines_temp_2[1].x)||magnitude.at<uchar>((lines_temp_2[1]+lines_temp_2[3])/2,(lines_temp_2[0]+lines_temp_2[2])/2)>
        magnitude.at<uchar>((new_lines_temp_2[0].y+new_lines_temp_2[2].y)/2,(new_lines_temp_2[0].x+new_lines_temp_2[2].x)/2))
        {
            new_lines_temp[0] = lines_temp_2[0];
            new_lines_temp[1] = lines_temp_2[1];
            new_lines_temp[2] = lines_temp_2[2];
            new_lines_temp[3] = lines_temp_2[3];
        }
        else
        {
          if(tmp1/min1 >2 && tmp2/min2 < 0.5)
          {
            cv::Point2f new_lineend;
              cv::Point2f new_lineend_tmp;
              int cnt = 0;
              float lengh = sqrt(pow(new_lines_temp_2[1].x - new_lines_temp_2[2].x,2)+pow(new_lines_temp_2[1].y - new_lines_temp_2[2].y,2));
              for(float i=0;i<lengh;i+=1)
              {
                 new_lineend.x = (new_lines_temp_2[1].x - new_lines_temp_2[2].x)/lengh*i + new_lines_temp_2[1].x;
                 new_lineend.y = (new_lines_temp_2[1].y - new_lines_temp_2[2].y)/lengh*i + new_lines_temp_2[1].y;
                 if(new_lineend.x<0||new_lineend.y<0||new_lineend.x>=height||new_lineend.y>=width)
                    break;
                 getmintemp(magnitude,new_lineend,new_lines_temp_2[1],min5,tmp5);
                 if(tmp5/min5<2)
                 {
                    new_lineend_tmp = new_lineend;
                    flag=1;
                    cnt = 0;
                 }
                 else
                 {
                    cnt++;
                    if(cnt >= lengh/5||cnt>=10)
                      break;
                 }
              }
              if(new_lineend_tmp.x<0||new_lineend_tmp.y<0||new_lineend_tmp.x>=height||new_lineend_tmp.y>=width)
              {
                    new_lines_temp[0] = new_lines_temp_2[0].x;
                    new_lines_temp[1] = new_lines_temp_2[0].y;
                    new_lines_temp[2] = new_lines_temp_2[2].x;
                    new_lines_temp[3] = new_lines_temp_2[2].y;
              }
              else
              {
                new_lines_temp[0] = new_lineend_tmp.x;
                new_lines_temp[1] = new_lineend_tmp.y;
                new_lines_temp[2] = new_lines_temp_2[2].x;
                new_lines_temp[3] = new_lines_temp_2[2].y;               
              }
          }
          else if(tmp1/min1 <0.5 && tmp2/min2 > 2)
          {
            cv::Point2f new_lineend;
              cv::Point2f new_lineend_tmp;
              int cnt = 0;
              float lengh = sqrt(pow(new_lines_temp_2[1].x - new_lines_temp_2[0].x,2)+pow(new_lines_temp_2[1].y - new_lines_temp_2[0].y,2));
              for(float i=0;i<lengh;i+=1)
              {
                 new_lineend.x = (new_lines_temp_2[1].x - new_lines_temp_2[0].x)/lengh*i + new_lines_temp_2[1].x;
                 new_lineend.y = (new_lines_temp_2[1].y - new_lines_temp_2[0].y)/lengh*i + new_lines_temp_2[1].y;
                 if(new_lineend.x<0||new_lineend.y<0||new_lineend.x>=height||new_lineend.y>=width)
                    break;
                 getmintemp(magnitude,new_lineend,new_lines_temp_2[1],min5,tmp5);
                 if(tmp5/min5<2)
                 {
                    new_lineend_tmp = new_lineend;
                    flag=1;
                    cnt = 0;
                 }
                 else
                 {
                    cnt++;
                    if(cnt >= lengh/5||cnt>=10)
                      break;
                 }
              }
              if(new_lineend_tmp.x<0||new_lineend_tmp.y<0||new_lineend_tmp.x>=height||new_lineend_tmp.y>=width)
              {
                    new_lines_temp[0] = new_lines_temp_2[0].x;
                    new_lines_temp[1] = new_lines_temp_2[0].y;
                    new_lines_temp[2] = new_lines_temp_2[2].x;
                    new_lines_temp[3] = new_lines_temp_2[2].y;
              }
              else
              {
                new_lines_temp[0] = new_lines_temp_2[0].x;
                new_lines_temp[1] = new_lines_temp_2[0].y;
                new_lines_temp[2] = new_lineend_tmp.x;
                new_lines_temp[3] = new_lineend_tmp.y;    
              }
          }
          else if(tmp1/min1 >3 && tmp2/min2 > 3)
          {
                new_lines_temp[0] = lines_temp_2[0];
                new_lines_temp[1] = lines_temp_2[1];
                new_lines_temp[2] = lines_temp_2[2];
                new_lines_temp[3] = lines_temp_2[3];
          }
          else
          {
                new_lines_temp[0] = new_lines_temp_2[0].x;
                new_lines_temp[1] = new_lines_temp_2[0].y;
                new_lines_temp[2] = new_lines_temp_2[2].x;
                new_lines_temp[3] = new_lines_temp_2[2].y;

          }
        }
        if(((new_lines_temp[0] - new_lines_temp[2])!=0||(new_lines_temp[1]-new_lines_temp[3])!=0))
        {
            new_lines.push_back(new_lines_temp);
        }
        else
        {
            new_lines.push_back(lines_temp_2);
        }
        
    }
    forw_lines = new_lines;
    connects.clear();
    vector<cv::Vec4f> forw_lines_temp;
    cv::Vec4f line_tmp;

   forw_line_para_a.resize(forw_lines.size());
   forw_line_para_b.resize(forw_lines.size());
   forw_line_para_c.resize(forw_lines.size());
   forw_line_hough.resize(forw_lines.size());
   forw_mid_point.resize(forw_lines.size());

   cur_line_para_a.resize(cur_lines.size());
   cur_line_para_b.resize(cur_lines.size());
   cur_line_para_c.resize(cur_lines.size());
   cur_line_hough.resize(cur_lines.size());
   cur_mid_point.resize(cur_lines.size());
   
   for(int i=0 ; i<forw_lines.size()*sample_point_num;i++)
   {
      int cnt_line = i/sample_point_num;
      int cnt_point = i%sample_point_num;
      forw_point_all.push_back(Point((forw_lines[cnt_line][0]*(cnt_point) + forw_lines[cnt_line][2]*(sample_point_num-cnt_point))/sample_point_num
                                      ,(forw_lines[cnt_line][1]*(cnt_point) + forw_lines[cnt_line][3]*(sample_point_num-cnt_point))/sample_point_num));
      if(cnt_line>=0&&cnt_point==0)
      {
        forw_mid_point[cnt_line].x = (forw_lines[cnt_line][0] + forw_lines[cnt_line][2])*0.5;
        forw_mid_point[cnt_line].y = (forw_lines[cnt_line][1] + forw_lines[cnt_line][3])*0.5;
      }
      if(cnt_point == sample_point_num-1)
      {
        
        forw_line_para_a[cnt_line] = forw_point_all[i].y- forw_point_all[i+1-sample_point_num].y;
        forw_line_para_b[cnt_line] = forw_point_all[i+1-sample_point_num].x-forw_point_all[i].x ;
        forw_line_para_c[cnt_line] = forw_point_all[i].x*forw_point_all[i+1-sample_point_num].y - forw_point_all[i+1-sample_point_num].x*forw_point_all[i].y;
        float temp = sqrt(pow(forw_line_para_a[cnt_line],2)+pow(forw_line_para_b[cnt_line],2));
        forw_line_para_a[cnt_line] /= temp;
        forw_line_para_b[cnt_line] /= temp;
        forw_line_para_c[cnt_line] /= temp;
        if(forw_line_para_c[cnt_line]>0)
        {
            forw_line_para_a[cnt_line] = -forw_line_para_a[cnt_line];
            forw_line_para_b[cnt_line] = -forw_line_para_b[cnt_line];
            forw_line_para_c[cnt_line] = -forw_line_para_c[cnt_line];
        }
        forw_line_hough[cnt_line].x = -forw_line_para_c[cnt_line];
        forw_line_hough[cnt_line].y = acos(forw_line_para_a[cnt_line])*57.3;
        if(forw_line_para_a[cnt_line]<0&&forw_line_para_b[cnt_line]>0)
        {
        forw_line_hough[cnt_line].y = 180 - forw_line_hough[cnt_line].y;
        }
        else if(forw_line_para_a[cnt_line]>0&&forw_line_para_b[cnt_line]<0)
        {
        forw_line_hough[cnt_line].y = 180 - forw_line_hough[cnt_line].y;
        }
        forw_line_hough[cnt_line].y=forw_line_hough[cnt_line].y*4;
      }
   }
   
    vector<int> array;
    vector<float> mid_x;
    mid_x.resize(forw_lines.size());
    array.resize(forw_lines.size());
    pcl::PointCloud<pcl::PointXY>::Ptr cloud (new pcl::PointCloud<pcl::PointXY>);
    cloud->width = forw_lines.size();
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);
    
     for (int i = 0; i < forw_lines.size(); i++)
    {
        (*cloud)[i].x = forw_line_hough[i].x;
        (*cloud)[i].y = forw_line_hough[i].y;
        mid_x[i] = forw_line_hough[i].y;
        array[i] = i;
    }

    vector<int> array_2;
    vector<float> mid_x_2;
    mid_x_2.resize(forw_lines.size());
    array_2.resize(forw_lines.size());
    pcl::PointCloud<pcl::PointXY>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXY>);
    cloud2->width = forw_lines.size();
    cloud2->height = 1;
    cloud2->points.resize (cloud2->width * cloud2->height);

    vector<int> array_3;
    vector<float> mid_x_3;
    mid_x_3.resize(forw_lines.size());
    array_3.resize(forw_lines.size());
    for (int i = 0; i < forw_lines.size(); i++)
    {
        if(forw_line_hough[i].y>90*4)
        {
            (*cloud2)[i].x = forw_line_hough[i].x;
            (*cloud2)[i].y = forw_line_hough[i].y - 180*4;
            mid_x_2[i] = forw_line_hough[i].y - 180*4;
        }
        else
        {
            (*cloud2)[i].x = forw_line_hough[i].x;
            (*cloud2)[i].y = forw_line_hough[i].y;
            mid_x_2[i] = forw_line_hough[i].y;
        }
        mid_x_3[i] = forw_line_hough[i].x;
        array_3[i] = i;
        array_2[i] = i;
    }
    
    pcl::KdTreeFLANN<pcl::PointXY> kdtree;
    if(forw_lines.size()!=0)
       kdtree.setInputCloud (cloud);
    pcl::KdTreeFLANN<pcl::PointXY> kdtree2;
    if(forw_lines.size()!=0)
       kdtree2.setInputCloud (cloud2);

    quickSort(0,mid_x.size()-1,mid_x,array);
    quickSort(0,mid_x_2.size()-1,mid_x_2,array_2);
    quickSort(0,mid_x_3.size()-1,mid_x_3,array_3);
    
     vector<uchar> lines_status;
    vector<float> lines_err;
     vector<int> position_2;
     vector<int> position_2_score;
     vector<int> macthed_position_flag,macthed_position_2;
     position_2.resize(cur_lines.size(),-1);
     position_2_score.resize(cur_lines.size(),-1);
     macthed_position_flag.resize(forw_lines.size(),0);
     macthed_position_2.resize(forw_lines.size(),-1);
    float average_distance=0;

    if(cur_lines.size()>0)
    {
        forw_point_flow.resize(cur_point_all.size());
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_point_all,forw_point_flow,lines_status,lines_err,cv::Size(21, 21), 3);
        vector<float> error_temp_theta;
        vector<float> error_temp_distance;
        vector<float> error_temp_mid;
        vector<int> error_temp_row;
        vector<float> distance_point;
                int cnt_dis = cur_lines.size();
        for(int i=0;i<cur_lines.size();i++)
        {
            float dis = sqrt(pow(forw_point_flow[i*sample_point_num].y - cur_point_all[i*sample_point_num].y,2)+
                                pow(forw_point_flow[i*sample_point_num].x - cur_point_all[i*sample_point_num].x,2));
            if(dis < 90)
                average_distance += dis;
            else
                cnt_dis --;
        }
        if(cnt_dis!=0)
            average_distance = average_distance/cnt_dis;
        else
            average_distance =60;
        if(average_distance >40)
        {
            average_distance = 60;
        }
        else if(average_distance > 0)
        {
            average_distance = average_distance +20;
        }
        else
        {
            average_distance = 20;
        }

        distance_point.resize(sample_point_num);
        for(int cnt_line=0 ; cnt_line<cur_lines.size();cnt_line++)
        {

            int i = cnt_line*sample_point_num + 4;

            cur_mid_point[cnt_line].x = (forw_point_flow[i].x + forw_point_flow[i-sample_point_num+1].x)*0.5;
            cur_mid_point[cnt_line].y = (forw_point_flow[i].y + forw_point_flow[i-sample_point_num+1].y)*0.5;
            cur_line_para_a[cnt_line] = forw_point_flow[i].y- forw_point_flow[i+1-sample_point_num].y;
            cur_line_para_b[cnt_line] = forw_point_flow[i+1-sample_point_num].x-forw_point_flow[i].x ;
            cur_line_para_c[cnt_line] = forw_point_flow[i].x*forw_point_flow[i+1-sample_point_num].y - forw_point_flow[i+1-sample_point_num].x*forw_point_flow[i].y;
            float temp = sqrt(pow(cur_line_para_a[cnt_line],2)+pow(cur_line_para_b[cnt_line],2));

            cur_line_para_a[cnt_line] /= temp;
            cur_line_para_b[cnt_line] /= temp;
            cur_line_para_c[cnt_line] /= temp;
            if(cur_line_para_c[cnt_line]>0)
            {
                cur_line_para_a[cnt_line] = -cur_line_para_a[cnt_line];
                cur_line_para_b[cnt_line] = -cur_line_para_b[cnt_line];
                cur_line_para_c[cnt_line] = -cur_line_para_c[cnt_line];
            }
            cur_line_hough[cnt_line].x = -cur_line_para_c[cnt_line];
            cur_line_hough[cnt_line].y = acos(cur_line_para_a[cnt_line])*57.3;
            if(cur_line_para_a[cnt_line]<0&&cur_line_para_b[cnt_line]>0)
            {
                cur_line_hough[cnt_line].y = 180 - cur_line_hough[cnt_line].y;
            }
            else if(cur_line_para_a[cnt_line]>0&&cur_line_para_b[cnt_line]<0)
            {
                cur_line_hough[cnt_line].y = 180 - cur_line_hough[cnt_line].y;
            }
            cur_line_hough[cnt_line].y=cur_line_hough[cnt_line].y*4;
            vector<int> num1;
            vector<int> num2;
            vector<int> num3;
            vector<int> num;
            vector<int> num_tmp;
            for(int i=0; i<forw_lines.size(); ++i)
            {
                num_tmp.push_back(i);
            }
    //////////////////////////////////////////////////////////////////////////////////////////
            pcl::PointXY pointtemp;
            pcl::PointXY pointtemp2;
            pcl::PointXY pointtemp3;
            // std::vector<int> k_indices;
            std::vector<int> k_indices_1;
            std::vector<float> k_sqr_distances_1;
            std::vector<int> k_indices_2;
            std::vector<float> k_sqr_distances_2;
            std::vector<int> k_indices_3;
            std::vector<float> k_sqr_distances_3;
            pointtemp.x = cur_line_hough[cnt_line].x;
            pointtemp.y = cur_line_hough[cnt_line].y;
            pointtemp3.x = cur_mid_point[cnt_line].x;
            pointtemp3.y = cur_mid_point[cnt_line].y;
            if(cur_line_hough[cnt_line].y>90*4)
            {
                pointtemp2.x = cur_line_hough[cnt_line].x;
                pointtemp2.y = cur_line_hough[cnt_line].y - 180*4;
            }
            else
            {
                pointtemp2.x = cur_line_hough[cnt_line].x;
                pointtemp2.y = cur_line_hough[cnt_line].y;
            }
            int m=kdtree.radiusSearch(pointtemp,50,k_indices_1,k_sqr_distances_1);
            int m2=kdtree2.radiusSearch(pointtemp2,50,k_indices_2,k_sqr_distances_2);
            vector<int>sign_1;
            sign_1.resize(forw_lines.size(),-1);
            for(size_t k =0 ;k<k_indices_1.size();k++)
            {
                int t_temp;
                int t_temp_2;
                t_temp = binarySearch(mid_x,(*cloud)[k_indices_1[k]].y);
                t_temp_2 = binarySearch(mid_x_3,(*cloud)[k_indices_1[k]].x);
                if(sign_1[t_temp]==-1)
                    if(t_temp>=0&&t_temp<array.size())
                    {
                        num1.push_back(array[t_temp]);
                        sign_1[t_temp] = 1;
                    }
                if(sign_1[t_temp_2]==-1)
                    if(t_temp_2>=0&&t_temp_2<array.size()&&t_temp_2!=t_temp)
                    {
                        num1.push_back(array_3[t_temp_2]);
                        sign_1[t_temp_2] = 1;
                    }
            }
            for(size_t k =0 ;k<k_indices_2.size();k++)
            {
                int t_temp;
                int t_temp_2;
                t_temp = binarySearch(mid_x_2,(*cloud2)[k_indices_2[k]].y);
                t_temp_2 = binarySearch(mid_x_3,(*cloud)[k_indices_2[k]].x);
                if(sign_1[t_temp]==-1)
                    if(t_temp>=0&&t_temp<array_2.size())
                    {
                        num2.push_back(array_2[t_temp]);
                        sign_1[t_temp] = 1;
                    }
                if(sign_1[t_temp_2]==-1)
                    if(t_temp_2>=0&&t_temp_2<array.size()&&t_temp_2!=t_temp)
                    {
                        num2.push_back(array_3[t_temp_2]);
                        sign_1[t_temp_2] = 1;
                    }
            }
            std::sort(num1.begin(),num1.end());
            std::sort(num2.begin(),num2.end());
            set_union(num1.begin(),num1.end(),num2.begin(),num2.end(),back_inserter(num));
            if(num.size()>0)
            {
                for(int k=0; k<num.size();k++)
                {

                    int j = num[k];
                    float forw_theta_tmp;
                    float cur_theta_tmp;
                    if(forw_line_hough[j].y<0)
                    {
                        forw_theta_tmp = forw_line_hough[j].y+180*4;
                    }
                    else
                    {
                        forw_theta_tmp = forw_line_hough[j].y;
                    }

                    if(cur_line_hough[cnt_line].y<0)
                    {
                        cur_theta_tmp = cur_line_hough[cnt_line].y+180*4;
                    }
                    else
                    {
                        cur_theta_tmp = cur_line_hough[cnt_line].y;
                    }
                    float theta_temp = abs(forw_theta_tmp - cur_theta_tmp);
                    if(theta_temp>90*4)
                    {
                        theta_temp = 180*4-theta_temp;
                    }

                    float distance_mid_temp = sqrt(pow(forw_mid_point[j].x-cur_mid_point[cnt_line].x,2)+pow(forw_mid_point[j].y-cur_mid_point[cnt_line].y,2));
                    
                    distance_point[0] = abs(forw_point_flow[i].x*forw_line_para_a[j] + forw_point_flow[i].y*forw_line_para_b[j]  +forw_line_para_c[j] );
                    distance_point[1] = abs(forw_point_flow[i-1].x*forw_line_para_a[j] + forw_point_flow[i-1].y*forw_line_para_b[j]  +forw_line_para_c[j] );
                    distance_point[2] = abs(forw_point_flow[i-2].x*forw_line_para_a[j] + forw_point_flow[i-2].y*forw_line_para_b[j]  +forw_line_para_c[j] );
                    distance_point[3] = abs(forw_point_flow[i-3].x*forw_line_para_a[j] + forw_point_flow[i-3].y*forw_line_para_b[j]  +forw_line_para_c[j] );
                    distance_point[4] = abs(forw_point_flow[i-4].x*forw_line_para_a[j] + forw_point_flow[i-4].y*forw_line_para_b[j]  +forw_line_para_c[j] );
                    float distance_temp = 0.2*(distance_point[0] + distance_point[1] + distance_point[2] + distance_point[3] + distance_point[4]);
                    
                    error_temp_row.push_back(j);
                    error_temp_theta.push_back(theta_temp);
                    error_temp_distance.push_back(distance_temp);
                    error_temp_mid.push_back(distance_mid_temp);              
                }
                float min_score = 0 ;
                int min_flag=0;
                int min_position;
                int row = 0;
                for(int j=0 ; j<error_temp_row.size();j++)
                {

                    float temp = sqrt(pow(error_temp_theta[j],2) + pow(error_temp_mid[j],2)+pow(error_temp_distance[j],2));
                    if(min_flag==0)
                    {
                        min_flag=1;
                        min_score = temp;
                        min_position = error_temp_row[j];
                    }
                    else
                    {
                        if(temp<min_score)
                        {
                            min_score = temp;
                            min_position = error_temp_row[j];
                        }
                    }
                }
                position_2[cnt_line] = min_position;
                position_2_score[cnt_line] = min_score;
                if(!macthed_position_flag[min_position])
                {
                    macthed_position_2[min_position] = cnt_line;
                }
                macthed_position_flag[min_position]++ ;
                min_position = 0;
                min_score = 0;
                min_flag = 0;
            }
            error_temp_theta.clear();
            error_temp_mid.clear();
            error_temp_distance.clear();
            error_temp_row.clear();
        }
      
        vector<int> line_ids_temp(forw_lines.size(),-1);
        vector<int> line_track_cnt_temp(forw_lines.size(),0);
        vector<int> line_ids_temp_test(forw_lines.size(),-1);
        vector<int> line_track_cnt_temp_test(forw_lines.size(),0);
        int count=0;
        vector<int> line_min_score(forw_lines.size(),-1);
        vector<int> line_min_position(forw_lines.size(),-1);
        average_distance =100;
        
        for(int i = 0; i<position_2.size(); i++)
        {
            if(position_2[i] != -1)
            {

                if(macthed_position_flag[position_2[i]]<2&&position_2_score[i]<70)
                {
                    line_ids_temp[position_2[i]] = line_ids_2[i];//把上一帧的id赋值给这一帧的id
                    line_track_cnt_temp[position_2[i]] = line_track_cnt_2[i];//把上一阵的追踪数量赋值给这一帧的id
                }
                else if(macthed_position_flag[position_2[i]]>1)
                {
                    if(line_min_score[position_2[i]]==-1&&position_2_score[i]<70)
                    {
                        if(m_line_feature.judgeMidPoint(cur_lines[i], forw_lines[position_2[i]],average_distance))
                        {
                            line_min_score[position_2[i]] = position_2_score[i];
                            line_min_position[position_2[i]] = i;
                            line_ids_temp[position_2[i]] = line_ids_2[i];//把上一帧的id赋值给这一帧的id
                            line_track_cnt_temp[position_2[i]] = line_track_cnt_2[i];//把上一阵的追踪数量赋值给这一帧的id
                        }
                    }
                    else if(line_min_score[position_2[i]] > position_2_score[i]&&position_2_score[i]<200)
                    {
                        if(m_line_feature.judgeMidPoint(cur_lines[i], forw_lines[position_2[i]],average_distance))
                        {
                            line_ids_temp[position_2[i]] = line_ids_2[i];//把上一帧的id赋值给这一帧的id
                            line_track_cnt_temp[position_2[i]] = line_track_cnt_2[i];//把上一阵的追踪数量赋值给这一帧的id
                        }
                    }
                }
            }
        }
        for(int i = 0; i<line_ids_temp.size(); i++)
        {
            line_track_cnt_temp[i]++;
            if(line_ids_temp[i] == -1)//如果上一帧没有匹配到的
                line_ids_temp[i] = n_line_id_2;
            n_line_id_2++;
        }
        line_ids_2= line_ids_temp;
        line_track_cnt_2 =  line_track_cnt_temp;
        for(int j = 0; j<line_ids_temp_test.size(); j++)
        {
            line_track_cnt_temp_test[j]++;
            if(line_ids_temp_test[j] == -1)//如果上一帧没有匹配到的
            {
                float min_score;
                int min_place,min_flag=0;
                double forw_theta_tmp,cur_theta_tmp;
                for(int cnt_line=0;cnt_line<cur_lines.size();cnt_line++)
                {
                    int i = cnt_line*sample_point_num + 4;
                    if(forw_line_hough[j].y<0)
                    {
                        forw_theta_tmp = forw_line_hough[j].y+180*4;
                    }
                    else
                    {
                        forw_theta_tmp = forw_line_hough[j].y;
                    }

                    if(cur_line_hough[cnt_line].y<0)
                    {
                        cur_theta_tmp = cur_line_hough[cnt_line].y+180*4;
                    }
                    else
                    {
                        cur_theta_tmp = cur_line_hough[cnt_line].y;
                    }
                    float theta_temp = abs(forw_theta_tmp - cur_theta_tmp);
                    if(theta_temp>90*4)
                    {
                        theta_temp = 180*4-theta_temp;
                    }

                    float distance_mid_temp = sqrt(pow(forw_mid_point[j].x-cur_mid_point[cnt_line].x,2)+pow(forw_mid_point[j].y-cur_mid_point[cnt_line].y,2));
                    
                    distance_point[0] = abs(forw_point_flow[i].x*forw_line_para_a[j] + forw_point_flow[i].y*forw_line_para_b[j]  +forw_line_para_c[j] );
                    distance_point[1] = abs(forw_point_flow[i-1].x*forw_line_para_a[j] + forw_point_flow[i-1].y*forw_line_para_b[j]  +forw_line_para_c[j] );
                    distance_point[2] = abs(forw_point_flow[i-2].x*forw_line_para_a[j] + forw_point_flow[i-2].y*forw_line_para_b[j]  +forw_line_para_c[j] );
                    distance_point[3] = abs(forw_point_flow[i-3].x*forw_line_para_a[j] + forw_point_flow[i-3].y*forw_line_para_b[j]  +forw_line_para_c[j] );
                    distance_point[4] = abs(forw_point_flow[i-4].x*forw_line_para_a[j] + forw_point_flow[i-4].y*forw_line_para_b[j]  +forw_line_para_c[j] );
                    float distance_temp = 0.2*(distance_point[0] + distance_point[1] + distance_point[2] + distance_point[3] + distance_point[4]);
                    float temp = sqrt(pow(theta_temp,2) + pow(distance_mid_temp,2)+pow(distance_temp,2));
                    if(min_flag==0)
                    {
                        min_flag = 1;
                        min_score = temp;
                        min_place = cnt_line;
                    }
                    else
                    {
                        if(temp<min_score)
                        {
                            min_score = temp;
                            min_place = cnt_line;
                        }
                    }
                }
                float tmp=50;
                if(m_line_feature.judgeAngle(cur_lines[min_place], forw_lines[j])&&m_line_feature.judgeMidPoint(cur_lines[min_place], forw_lines[j],tmp))
                {
                    line_ids_temp_test[j] = line_ids_2[min_place];
                }
                else
                {
                    line_ids_temp_test[j] = -1;
                }
            }

            n_line_id++;
        }
        for(int j = 0; j<line_ids_temp_test.size(); j++)
        {
            line_track_cnt_1++;
        }
        line_ids_2= line_ids_temp;
        line_track_cnt_2 =  line_track_cnt_temp;
        line_ids_3 = line_ids_temp_test;
        line_track_cnt_3 = line_track_cnt_temp_test;
    }
    else
    {
        line_ids_2.resize(forw_lines.size(),-1);
        line_track_cnt_2.resize((forw_lines.size()),0);
        for(int i = 0; i<forw_lines.size(); i++)
        {
            line_track_cnt_1++;
            line_track_cnt_2[i]++;
            if(line_ids_2[i] == -1)
                line_ids_2[i] = n_line_id_2;
            n_line_id_2++;
        }
        n_line_id = n_line_id_2;
        line_ids_3 = line_ids_2;
        line_track_cnt_3 = line_track_cnt_2;
    }
        
    last_line_id = line_ids_2;
    cur_ptr = forw_ptr;
    cur_point_all = forw_point_all;
    forw_point_all.clear();
    //-------------------------------------上面是和线相关的数据处理----------------------------------------


    //当下一帧图像到来时，当前帧数据就成为了上一帧发布的数据

    //把当前帧的数据forw_img、forw_pts赋给上一帧cur_img、cur_pts
    cur_img = forw_img;
    cur_pts = forw_pts;

    cur_lines = forw_lines;
    cur_ldesc = forw_ldesc.clone();

    //根据不同的相机模型去畸变矫正和转换到归一化坐标系上，计算速度
    undistortedPoints();
    prev_time = cur_time;
    flag = 0;
}
/**
 * @brief   通过F矩阵去除outliers
 * @Description 将图像坐标转换为归一化坐标
 *              cv::findFundamentalMat()计算F矩阵
 *              reduceVector()去除outliers 
 * @return      void
*/
void FeatureTracker::rejectWithF()
{
    if (forw_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");//才哟ing的是ransac的方法

        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {

            Eigen::Vector3d tmp_p;
            //根据不同的相机模型将二维坐标转换到三维坐标
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            //转换为归一化像素坐标
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        vector<uchar> status;
        //调用cv::findFundamentalMat对un_cur_pts和un_forw_pts计算F矩阵
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
    }
}

//更新特征点id
bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

//读取相机内参
void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}


//显示去畸变矫正后的特征点  name为图像帧名称
void FeatureTracker::showUndistortion(const string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j);
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b);
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}


//对角点图像坐标进行去畸变矫正，转换到归一化坐标系上，并计算每个角点的速度。                       
void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());

    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
        Eigen::Vector3d b;

        //根据不同的相机模型将二维坐标转换到三维坐标
        m_camera->liftProjective(a, b);

        //再延伸到深度归一化平面上
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }

    // 计算每个特征点的速度到pts_velocity
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time;
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1)
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]);
                if (it != prev_un_pts_map.end())
                {
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0));
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0));
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    prev_un_pts_map = cur_un_pts_map;
}


void FeatureTracker::undistortedline(Vector3d &un_pts_s, Vector3d &un_pts_e, cv::Vec4f line)
{
    Vector2d pts_s, pts_e;
    pts_s<<line[0], line[1];
    pts_e<<line[2], line[3];
    m_camera->liftProjective(pts_s, un_pts_s);
    m_camera->liftProjective(pts_e, un_pts_e);
}