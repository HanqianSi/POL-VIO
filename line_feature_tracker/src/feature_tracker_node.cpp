


// new points velocity is 0, pub or not?
// track cnt > 1 pub?
#include <ros/ros.h>
#include <mutex>
#include <thread>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include "line_feature_tracker/lines2d.h"
#include "line_feature_tracker/lines_devide.h"

#include "feature_tracker.h"

#define SHOW_UNDISTORTION 0

vector<uchar> r_status;
vector<float> r_err;
std::mutex m_buf;
std::mutex n_buf;
queue<sensor_msgs::ImageConstPtr> img_buf;
queue<line_feature_tracker::lines2d> lines_buf;

ros::Publisher pub_img,pub_img2,pub_match,pub_match_point,pub_img_2,pub_img_3,pub_imagemat;
ros::Publisher pub_restart;

//每个相机都有一个FeatureTracker实例，即trackerData[i]
FeatureTracker trackerData[NUM_OF_CAM];

double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;
vector<float> array_startx(200,1);
vector<float> array_starty(200,1);
vector<float> array_endx(200,1);
vector<float> array_endy(200,1);
int cnt1=0;

/**
 * @brief   ROS的回调函数，对新来的图像进行特征点追踪，发布
 * @Description readImage()函数对新来的图像使用光流法进行特征点跟踪
 *              追踪的特征点封装成feature_points发布到pub_img的话题下，
 *              图像封装成ptr发布在pub_match下
 * @param[in]   img_msg 输入的图像
 * @return      void
*/
void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // cnt1++;
    m_buf.lock();
    img_buf.push(img_msg);
    m_buf.unlock();
}

void line_callback(const line_feature_tracker::lines2d::ConstPtr &msg)
{
    line_feature_tracker::lines2d msg2;
    m_buf.lock();
    msg2.header = msg->header;
    msg2.startx = msg->startx;
    msg2.starty = msg->starty;
    msg2.endx = msg->endx;
    msg2.endy = msg->endy;
    lines_buf.push(msg2);
    m_buf.unlock();
}
void sync_process()
{
    while(1)
    {
        
         line_feature_tracker::lines2d msg_line;
         sensor_msgs::ImageConstPtr img_msg;
         sensor_msgs::ImageConstPtr img_line_msg;
            if(!lines_buf.empty()&&!img_buf.empty())
            {
               
            for(int i=0;i<img_buf.size();i++)
            {
                if ((img_buf.front()->header.stamp.toSec() <  lines_buf.front().header.stamp.toSec())) 
                {
                    img_buf.pop();
                }
                else
                {
                    continue;
                }
            }
            if(!img_buf.size())
            {
                continue;
            }
            
            m_buf.lock();
            msg_line = lines_buf.front();


            img_msg = img_buf.front();
            lines_buf.pop();
            img_buf.pop();

            

            m_buf.unlock();
            array_startx.resize(msg_line.startx.size());
            array_starty.resize(msg_line.starty.size());
            array_endx.resize(msg_line.endx.size());
            array_endy.resize(msg_line.endy.size());
            for(int i=0;i<msg_line.startx.size();i++)
            {
                array_startx[i]= msg_line.startx[i];
                array_starty[i]= msg_line.starty[i];
                array_endx[i]= msg_line.endx[i];
                array_endy[i]= msg_line.endy[i];
            }
            //判断是否是第一帧,主要是记录了一下时间戳
            if(first_image_flag)
            {
                first_image_flag = false;
                first_image_time = img_msg->header.stamp.toSec();//记录图像帧的时间
                last_image_time = img_msg->header.stamp.toSec();//上一帧的时间戳
                continue;
            }

            // detect unstable camera stream
            // 通过判断时间间隔，有问题则restart
            if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)//时间戳断开了就立马重启
            {
                first_image_flag = true;
                last_image_time = 0;
                pub_count = 1;
                std_msgs::Bool restart_flag;
                restart_flag.data = true;
                pub_restart.publish(restart_flag);
                continue;
            }
            last_image_time = img_msg->header.stamp.toSec();

            // 发布频率控制
            if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
            {
                PUB_THIS_FRAME = true;
                // 时间间隔内的发布频率十分接近设定频率时，更新时间间隔起始时刻，并将数据发布次数置0
                if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
                {
                    first_image_time = img_msg->header.stamp.toSec();
                    pub_count = 0;
                }
            }
            else
                PUB_THIS_FRAME = false;

            TicToc t_r;
            //-------------------------------------上面仅仅和时间有关，下面才开始处理数据----------------------------------------
            if (PUB_THIS_FRAME)
            {

            cv_bridge::CvImageConstPtr ptr;
            cv_bridge::CvImageConstPtr ptr2;
            //将图像编码8UC1转换为mono8
            if (img_msg->encoding == "8UC1")
            {
                sensor_msgs::Image img;
                img.header = img_msg->header;
                img.height = img_msg->height;
                img.width = img_msg->width;
                img.is_bigendian = img_msg->is_bigendian;
                img.step = img_msg->step;
                img.data = img_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
                ptr2 = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else
            {
                ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
                ptr2 = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
            }
            
            cv::Mat show_img = ptr->image;
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                ROS_DEBUG("processing camera %d", i);
                if (i != 1 || !STEREO_TRACK)//单目
                {
                    cnt1++;
                    trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.toSec(),array_startx,array_starty,array_endx,array_endy,cnt1,cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8));
                }
                else//双目
                {
                    if (EQUALIZE)
                    {
                        //自适应直方图均衡化处理
                        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                        clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
                    }
                    else
                        trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
                }
        #if SHOW_UNDISTORTION
                trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
        #endif
            }

            //更新全局ID
            for (unsigned int i = 0;; i++)
            {
                bool completed = false;
                for (int j = 0; j < NUM_OF_CAM; j++)
                    if (j != 1 || !STEREO_TRACK)
                        completed |= trackerData[j].updateID(i);
                if (!completed)
                    break;
            }


            //-------------------------------------上面处理完成，下面仅仅是进行发布操作----------------------------------------

            //1、将特征点id，矫正后归一化平面的3D点(x,y,z=1)，像素2D点(u,v)，像素的速度(vx,vy),发布到pub_img;
            //2、将图像封装到cv_bridge::cvtColor类型的ptr实例中发布到pub_match
                pub_count++;
                sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
                sensor_msgs::PointCloudPtr feature_lines(new sensor_msgs::PointCloud);
                sensor_msgs::PointCloudPtr feature_lines_2(new sensor_msgs::PointCloud);
                sensor_msgs::PointCloudPtr feature_lines_3(new sensor_msgs::PointCloud);
                sensor_msgs::ChannelFloat32 id_of_point;   //  feature id
                sensor_msgs::ChannelFloat32 u_of_point;    //  u
                sensor_msgs::ChannelFloat32 v_of_point;    //  v
                

                sensor_msgs::ChannelFloat32 id_of_line;   //  feature id
                sensor_msgs::ChannelFloat32 u_of_endpoint;    //  u
                sensor_msgs::ChannelFloat32 v_of_endpoint;    //  v

                sensor_msgs::ChannelFloat32 id_of_line_2;   //  feature id
                sensor_msgs::ChannelFloat32 id_of_line_3;   //  feature id
                sensor_msgs::ChannelFloat32 u_of_endpoint_2;    //  u
                sensor_msgs::ChannelFloat32 v_of_endpoint_2;    //  v
                

                feature_points->header = img_msg->header;
                feature_points->header.frame_id = "world";

                feature_lines->header = img_msg->header;
                feature_lines->header.frame_id = "world";

                feature_lines_2->header = img_msg->header;
                feature_lines_2->header.frame_id = "world";

                feature_lines_3->header = img_msg->header;
                feature_lines_3->header.frame_id = "world";

                vector<set<int>> hash_ids(NUM_OF_CAM);
                for (int i = 0; i < NUM_OF_CAM; i++)//相机个数一般都是1
                {
                    auto &un_pts = trackerData[i].cur_un_pts;//归一化相机坐标系下的坐标
                    auto &cur_pts = trackerData[i].cur_pts;
                    auto &ids = trackerData[i].ids;//能够被跟踪到的特征点的id
                    auto &pts_velocity = trackerData[i].pts_velocity;//当前帧相对前一帧特征点沿x,y方向的像素移动速度
                    for (unsigned int j = 0; j < ids.size(); j++)//遍历所有的特征点
                    {
                        if (trackerData[i].track_cnt[j] > 1)//当前帧中每个特征点被追踪的时间次数大于1
                        {
                            int p_id = ids[j];
                            hash_ids[i].insert(p_id);
                            geometry_msgs::Point32 p;
                            p.x = un_pts[j].x;
                            p.y = un_pts[j].y;
                            p.z = 1;

                            feature_points->points.push_back(p);
                            id_of_point.values.push_back(p_id * NUM_OF_CAM + i);//存储的其实就是空间点的id
                            u_of_point.values.push_back(cur_pts[j].x);
                            v_of_point.values.push_back(cur_pts[j].y);
                            ROS_ASSERT(inBorder(cur_pts[j]));
                        }
                    }

                    auto &line_ids = trackerData[i].line_ids_2;
                    auto &lines = trackerData[i].cur_lines;
                    for(unsigned int j = 0; j < line_ids.size(); j++)
                    {
                        if(trackerData[i].line_track_cnt_2[j] > 2)
                        {
                            Vector3d un_pts_s, un_pts_e;
                            trackerData->undistortedline(un_pts_s, un_pts_e, lines[j]);
                            geometry_msgs::Point32 p;
                            p.x = un_pts_s.x()/un_pts_s.z();
                            p.y = un_pts_s.y()/un_pts_s.z();
                            p.z = 1;
                            feature_lines->points.push_back(p);
                            int p_line_id = line_ids[j];
                            id_of_line.values.push_back(p_line_id * NUM_OF_CAM + i);
                            u_of_endpoint.values.push_back(un_pts_e.x()/un_pts_e.z());
                            v_of_endpoint.values.push_back(un_pts_e.y()/un_pts_e.z());
                        }
                    }

                    auto &line_ids_2 = trackerData[i].line_ids_2;
                    auto &line_ids_3 = trackerData[i].line_ids_3;
                    auto &lines_2 = trackerData[i].cur_lines;
                    for(unsigned int j = 0; j < line_ids_2.size(); j++)
                    {
                            Vector3d un_pts_s, un_pts_e;
                            trackerData->undistortedline(un_pts_s, un_pts_e, lines_2[j]);
                            geometry_msgs::Point32 p;
                            p.x = un_pts_s.x()/un_pts_s.z();
                            p.y = un_pts_s.y()/un_pts_s.z();
                            p.z = 1;
                            feature_lines_2->points.push_back(p);
                            int p_line_id = line_ids_2[j];
                            id_of_line_2.values.push_back(p_line_id * NUM_OF_CAM + i);
                            u_of_endpoint_2.values.push_back(un_pts_e.x()/un_pts_e.z());
                            v_of_endpoint_2.values.push_back(un_pts_e.y()/un_pts_e.z());
                            id_of_line_3.values.push_back(line_ids_3[j]*NUM_OF_CAM + i);
                    }
                    feature_points->channels.push_back(id_of_point);
                    feature_points->channels.push_back(u_of_point);
                    feature_points->channels.push_back(v_of_point);
                    

                    feature_lines->channels.push_back(id_of_line);
                    feature_lines->channels.push_back(u_of_endpoint);
                    feature_lines->channels.push_back(v_of_endpoint);

                    feature_lines_2->channels.push_back(id_of_line_2);
                    feature_lines_2->channels.push_back(u_of_endpoint_2);
                    feature_lines_2->channels.push_back(v_of_endpoint_2);
                    feature_lines_2->channels.push_back(id_of_line_3);

                    if (!init_pub)//第一帧不发布
                    {
                        // init_pub = 1;
                    }
                    else
                    {
                        pub_img.publish(feature_points);
                        pub_img2.publish(feature_lines);
                    }
                   
                }
                //不同的通道存储不同的特征
                
                ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());

                if(!init_pub)
                {
                    init_pub =1;
                }
                else
                {
                    pub_img_2.publish(feature_lines_2);
                    pub_imagemat.publish(img_msg);
                }


                //-------------------------------------上面是发布操作，下面仅仅是显示图片----------------------------------------


                if (1)
                {
                    ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
                    ptr2 = cv_bridge::cvtColor(ptr2, sensor_msgs::image_encodings::BGR8);
                    cv::Mat stereo_img = ptr->image;
                    cv::Mat stereo_img2 = ptr2->image;
                    for (int i = 0; i < NUM_OF_CAM; i++)
                    {
                        cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                        cv::Mat tmp_img2 = stereo_img2.rowRange(i * ROW, (i + 1) * ROW);
                        cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);
                        //显示追踪状态，越红越好，越蓝越不行
                        for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                        {
                            double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                            cv::circle(tmp_img2, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);

                            //draw speed line
                           Vector2d tmp_cur_un_pts (trackerData[i].cur_un_pts[j].x, trackerData[i].cur_un_pts[j].y);
                           Vector2d tmp_pts_velocity (trackerData[i].pts_velocity[j].x, trackerData[i].pts_velocity[j].y);
                           Vector3d tmp_prev_un_pts;
                           tmp_prev_un_pts.head(2) = tmp_cur_un_pts - 0.10 * tmp_pts_velocity;
                           tmp_prev_un_pts.z() = 1;
                           Vector2d tmp_prev_uv;
                           trackerData[i].m_camera->spaceToPlane(tmp_prev_un_pts, tmp_prev_uv);
                           cv::line(tmp_img2, trackerData[i].cur_pts[j], cv::Point2f(tmp_prev_uv.x(), tmp_prev_uv.y()), cv::Scalar(255 , 0, 0), 1 , 8, 0);
                        }

                        for(unsigned int j = 0; j<trackerData[i].forw_lines.size(); j++)
                        {
                            double len = std::min(1.0, 1.0 * trackerData[i].line_track_cnt_2[j] / WINDOW_SIZE);
                            cv::line(tmp_img,
                                    Point(trackerData[i].forw_lines[j][0], trackerData[i].forw_lines[j][1]),
                                    Point(trackerData[i].forw_lines[j][2], trackerData[i].forw_lines[j][3]),
                                    Scalar(255 * (1 - len), 0, 255 * len), 2);
                        }
                    }
                    pub_match.publish(ptr->toImageMsg());
                    pub_match_point.publish(ptr2->toImageMsg());
                }
                
            }
            ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
             
        }
        array_startx.clear();
        array_starty.clear();
        array_endx.clear();
        array_endy.clear();
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    //ros初始化和设置句柄
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n("~");

    //设置logger的级别。 只有级别大于或等于level的日志记录消息才会得到处理。
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    //读取config->euroc->euroc_config.yaml中的一些配置参数,确定相机和IMU的外参是否准确，不准确可以自己标定，确定IMU和时间是否同步，不同步可以设置补偿时间
    readParameters(n);

    //读取每个相机实例读取对应的相机内参
    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    //判断是否加入鱼眼mask来去除边缘噪声
    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    //订阅话题IMAGE_TOPIC(/cam0/image_raw),执行回调函数
    ros::Subscriber sub_img = n.subscribe("/hawp/feature_image", 100, img_callback);
    ros::Subscriber sub_line = n.subscribe("/hawp/Lines2d", 2000, line_callback);


    //发布feature，实例feature_points，跟踪的特征点，给后端优化用
    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000);

    pub_img2 = n.advertise<sensor_msgs::PointCloud>("linefeature", 1000);
    pub_img_2 = n.advertise<sensor_msgs::PointCloud>("linefeature_2", 1000);
    //发布feature_img，实例ptr，跟踪的特征点图，给RVIZ用和调试用
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000);
    pub_match_point = n.advertise<sensor_msgs::Image>("feature_img_point",1000);
    pub_imagemat = n.advertise<sensor_msgs::Image>("linefeature_img_2",1000);
    //发布restart
    std::thread sync_thread{sync_process};
    ros::spin();
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?