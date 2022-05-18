
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/videoio.hpp>
#include "opencv2/highgui.hpp"

#include <sstream>
#include <map>
#include <vector>
#include <iostream>
#include "SurroundSystem.hpp"

std::vector<image_transport::Publisher> pubs;
std::vector<ros::Publisher> infos;
std::map<int, std::vector<std::string>> sp_frames ;
ros::Subscriber quad_sub;

double ff = 270;

double K[] = {ff, 0, 450/2,
              0, ff, 450/2,
              0, 0, 1.0    };
double Pl[]= {ff, 0, 450/2, 0,
              0, ff, 450/2, 0,
              0, 0, 1,     0};                    
double Pr[]= {ff, 0, 450/2, -ff*0.2,
              0, ff, 450/2, 0,
              0, 0, 1,     0}; 
              

SurroundSystem SS; 
int count = 0;

void fillCamInfos(sensor_msgs::CameraInfo &lcam_info, sensor_msgs::CameraInfo &rcam_info)
{
    lcam_info.width = 540;
    lcam_info.height = 540;
    rcam_info.width = 540;
    rcam_info.height = 540;
    for (int c = 0; c < lcam_info.P.size(); c++)
    {
      lcam_info.P.at(c) = Pl[c];
      rcam_info.P.at(c) = Pr[c];
    }
    for (int c = 0; c < lcam_info.K.size(); c++)
    {
      lcam_info.K.at(c) = K[c];
      rcam_info.K.at(c) = K[c];
    }
}


void processImages(cv::Mat& full_frame)
{
    cv::Mat front_left = full_frame(cv::Rect(0, 0, 1080, 1080)).clone();       
    cv::Mat back_right  = full_frame(cv::Rect(1080, 1080, 1080, 1080)).clone();
    cv::Mat back_left  = full_frame(cv::Rect(1080, 0, 1080, 1080)).clone();
    cv::Mat front_right   = full_frame(cv::Rect(0, 1080, 1080, 1080)).clone();
    std::vector<cv::Mat> cams;

    // cv::imshow("flipped", full_frame);
    // cv::waitKey(1);

    std::vector<cv::Mat> remappedCombs = {cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat()};
    SS.getImage(0, SurroundSystem::RECTIFIED, front_left, front_right, remappedCombs[0]);
    SS.getImage(1, SurroundSystem::RECTIFIED, front_right, back_right, remappedCombs[1]);
    SS.getImage(2, SurroundSystem::RECTIFIED, back_left, front_left, remappedCombs[2]);
    SS.getImage(3, SurroundSystem::RECTIFIED, back_right, back_left,  remappedCombs[3]);
                   

    for (int i=0; i<remappedCombs.size(); i++)
    {
        cv::Mat left = remappedCombs[i](cv::Rect(0, 0, 540, 540));
        cv::Mat right = remappedCombs[i](cv::Rect(540, 0, 540, 540));

        std_msgs::Header hdr;
        hdr.stamp = ros::Time::now();
        hdr.seq = count;


        sensor_msgs::CameraInfo linfo;
        sensor_msgs::CameraInfo rinfo;

        hdr.frame_id = sp_frames[i][0];
        linfo.header = hdr;
        sensor_msgs::ImagePtr lmsg = cv_bridge::CvImage(hdr, "bgr8", left).toImageMsg();
        hdr.frame_id = sp_frames[i][1];
        rinfo.header = hdr; 
        sensor_msgs::ImagePtr rmsg = cv_bridge::CvImage(hdr, "bgr8", right).toImageMsg();
        fillCamInfos(linfo, rinfo);

        pubs[2*i].publish(lmsg);
        pubs[2*i+1].publish(rmsg);

        infos[2*i].publish(linfo);
        infos[2*i+1].publish(rinfo);
    }
}

cv::Mat buf_image;
void unityCallback(const sensor_msgs::Image& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  //buf_image = cv_ptr->image;
  cv::flip(cv_ptr->image, buf_image,0);
  //cv::imshow("img", cv_ptr->image);

  processImages(buf_image);
  ++count;
}

int main(int argc, char **argv)
{
  bool use_video_, use_quadrator_;
  std::string source_topic_, source_file_;
  int source_width_, shot_width_, res_width_, source_fps_;

  ros::init(argc, argv, "svs");
  ros::NodeHandle nh;

  // nh.param("use_videofile", use_video_, "false");
  // nh.param("use_quadrator", use_quadrator_, "false");
  // nh.param("source_file", source_file_, "./video.mp4");
  // nh.param("source_topic", source_topic_, "/camera/image_raw");
  // nh.param("source_fps", source_fps_, 30);
  // nh.param("source_width", source_width_, 2160);
  // nh.param("shot_width", shot_width_, 1080);
  // nh.param("result_width", res_width_, 540);

  image_transport::ImageTransport it(nh);

  pubs= {it.advertise("svs/front_cam/left/image_raw", 2), it.advertise("svs/front_cam/right/image_raw", 1),
                                                 it.advertise("svs/right_cam/left/image_raw", 1), it.advertise("svs/right_cam/right/image_raw", 1), 
                                                 it.advertise("svs/left_cam/left/image_raw", 1),  it.advertise("svs/left_cam/right/image_raw", 1),
                                                 it.advertise("svs/back_cam/left/image_raw", 1),  it.advertise("svs/back_cam/right/image_raw", 1)};
  infos = {nh.advertise<sensor_msgs::CameraInfo>("svs/front_cam/left/camera_info", 2), nh.advertise<sensor_msgs::CameraInfo>("svs/front_cam/right/camera_info", 1),
                                       nh.advertise<sensor_msgs::CameraInfo>("svs/right_cam/left/camera_info", 1), nh.advertise<sensor_msgs::CameraInfo>("svs/right_cam/right/camera_info", 1), 
                                       nh.advertise<sensor_msgs::CameraInfo>("svs/left_cam/left/camera_info", 1),  nh.advertise<sensor_msgs::CameraInfo>("svs/left_cam/right/camera_info", 1),
                                       nh.advertise<sensor_msgs::CameraInfo>("svs/back_cam/left/camera_info", 1),  nh.advertise<sensor_msgs::CameraInfo>("svs/back_cam/right/camera_info", 1)};
  sp_frames = { {0, {"fl_r_ph", "fr_l_ph"}}, 
                {1, {"fr_r_ph", "br_l_ph"}},
                {2, {"bl_r_ph", "fl_l_ph"}},
                {3, {"br_r_ph", "bl_l_ph"}}  };
  quad_sub = nh.subscribe("unity/quadrator", 10, unityCallback);                                                        
  // TODO: create dinamically from yaml
  //std::vector<image_transport::Subscriber> fisheye_subs;     

  //ros::Rate loop_rate(10);
  //cv::VideoCapture cap; 
  //cap.open("/home/roser/catkin_ws/src/stereofisheye_ros/movie_002.mp4");
  // if (!cap.isOpened()) {
  //   std::cerr << "ERROR! Unable to open video\n";
  //   return -1;
  // }

  cv::Size origSize(1080, 1080);       //imread(image_list[0], -1).size();
  cv::Size newSize(540, 540);        // origSize * 1;            // determines the size of the output image
  
// Create the stereo system object
  
  KBModel SM0;
  SM0.setIntrinsics({ 0.000757676, -0.000325907, 0.0000403, -0.000001866 }, cv::Vec2d(0, 0), cv::Matx22d(343.536, 0, 0, 343.471));
  SM0.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));  //45^o
  SM0.setCamParams(origSize);
  KBModel SM1;
  SM1.setIntrinsics({ 0.000757676, -0.000325907, 0.0000403, -0.000001866 }, cv::Vec2d(0, 0), cv::Matx22d(343.536, 0, 0, 343.471));
  SM1.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795));  //-45^o
  SM1.setCamParams(origSize);
  KBModel SM2;
  SM2.setIntrinsics({ 0.000757676, -0.000325907, 0.0000403, -0.000001866 }, cv::Vec2d(0, 0), cv::Matx22d(343.536, 0, 0, 343.471));
  SM2.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.9238795, 0.3826834));   //135^o
  SM2.setCamParams(origSize);
  KBModel SM3;
  SM3.setIntrinsics({ 0.000757676, -0.000325907, 0.0000403, -0.000001866 }, cv::Vec2d(0, 0), cv::Matx22d(343.536, 0, 0, 343.471));
  SM3.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, -0.9238795, 0.3826834));  //-135^o
  SM3.setCamParams(origSize);

  SS.addNewCam(SM0);
  SS.addNewCam(SM1);
  SS.addNewCam(SM2);  
  SS.addNewCam(SM3);
  SS.createStereopair(0, 1, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
  SS.createStereopair(1, 3, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
  SS.createStereopair(2, 0, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
  SS.createStereopair(3, 2, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);

  SS.prepareLUTs(); 
  ROS_INFO("LUTs ready");


  //cv::Mat full_frame(cv::Size(1080 * 2, 1080*2), CV_8UC3, cv::Scalar(0, 0, 0));
  ros::spin();
  while (ros::ok())
  {
    //cap >> full_frame;
    // full_frame = buf_image.clone();
    // if (full_frame.size().width == 0)
    // {
    //   ROS_INFO("empty");
    //   continue;
    // }
    // cv::Mat front_right = full_frame(cv::Rect(0, 0, 1080, 1080)).clone();       
    // cv::Mat back_right  = full_frame(cv::Rect(1080, 0, 1080, 1080)).clone();
    // cv::Mat back_left   = full_frame(cv::Rect(0, 1080, 1080, 1080)).clone();
    // cv::Mat front_left  = full_frame(cv::Rect(1080, 1080, 1080, 1080)).clone();
    // std::vector<cv::Mat> cams;

    // std::vector<cv::Mat> remappedCombs = {cv::Mat(), cv::Mat(), cv::Mat(), cv::Mat()};
    // SS.getImage(0, SurroundSystem::RECTIFIED, front_left, front_right, remappedCombs[0]);
    // SS.getImage(1, SurroundSystem::RECTIFIED, front_right, back_right, remappedCombs[1]);
    // SS.getImage(2, SurroundSystem::RECTIFIED, back_left, front_left, remappedCombs[2]);
    // SS.getImage(3, SurroundSystem::RECTIFIED, back_right, back_left,  remappedCombs[3]);
                   

    // for (int i=0; i<remappedCombs.size(); i++)
    // {
    //     cv::Mat left = remappedCombs[i](cv::Rect(0, 0, 540, 540));
    //     cv::Mat right = remappedCombs[i](cv::Rect(540, 0, 540, 540));

    //     std_msgs::Header hdr;
    //     hdr.stamp = ros::Time::now();
    //     hdr.seq = count;


    //     sensor_msgs::CameraInfo linfo;
    //     sensor_msgs::CameraInfo rinfo;

    //     hdr.frame_id = sp_frames[i][0];
    //     linfo.header = hdr;
    //     sensor_msgs::ImagePtr lmsg = cv_bridge::CvImage(hdr, "bgr8", left).toImageMsg();
    //     hdr.frame_id = sp_frames[i][1];
    //     rinfo.header = hdr; 
    //     sensor_msgs::ImagePtr rmsg = cv_bridge::CvImage(hdr, "bgr8", right).toImageMsg();
    //     fillCamInfos(linfo, rinfo);

    //     pubs[2*i].publish(lmsg);
    //     pubs[2*i+1].publish(rmsg);

    //     infos[2*i].publish(linfo);
    //     infos[2*i+1].publish(rinfo);
    // }
    
    //ROS_INFO("SVS: PUB");

//    ros::spinOnce();
    
    //loop_rate.sleep();
    //++count;
  }

  ROS_WARN("LOOP BROKEN");


  return 0;
}