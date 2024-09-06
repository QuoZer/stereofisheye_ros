
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
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

class SFyNode : public rclcpp::Node
{
public:
  SurroundSystem SS; 
  std::vector<image_transport::Publisher> pubs;
  std::vector<rclcpp::Publisher> infos;
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
                

  SFyNode() : Node("stereofisheye_ros_node") 
  {
    RCLCPP_INFO(get_logger(), "Node started!");


    image_transport::ImageTransport it(this);

    this->pubs = {it.advertise("svs/front_cam/left/image_raw", 2), it.advertise("svs/front_cam/right/image_raw", 1),
                  it.advertise("svs/right_cam/left/image_raw", 1), it.advertise("svs/right_cam/right/image_raw", 1), 
                  it.advertise("svs/left_cam/left/image_raw", 1),  it.advertise("svs/left_cam/right/image_raw", 1),
                  it.advertise("svs/back_cam/left/image_raw", 1),  it.advertise("svs/back_cam/right/image_raw", 1)};
    this->infos = {nh.advertise<sensor_msgs::msg::CameraInfo>("svs/front_cam/left/camera_info", 2), nh.advertise<sensor_msgs::msg::CameraInfo>("svs/front_cam/right/camera_info", 1),
                  nh.advertise<sensor_msgs::msg::CameraInfo>("svs/right_cam/left/camera_info", 1), nh.advertise<sensor_msgs::msg::CameraInfo>("svs/right_cam/right/camera_info", 1), 
                  nh.advertise<sensor_msgs::msg::CameraInfo>("svs/left_cam/left/camera_info", 1),  nh.advertise<sensor_msgs::msg::CameraInfo>("svs/left_cam/right/camera_info", 1),
                  nh.advertise<sensor_msgs::msg::CameraInfo>("svs/back_cam/left/camera_info", 1),  nh.advertise<sensor_msgs::msg::CameraInfo>("svs/back_cam/right/camera_info", 1)};
    this->sp_frames = { {0, {"fl_r_ph", "fr_l_ph"}}, 
                  {1, {"fr_r_ph", "br_l_ph"}},
                  {2, {"bl_r_ph", "fl_l_ph"}},
                  {3, {"br_r_ph", "bl_l_ph"}}  };
                   
    quad_sub = create_subscription<sensor_msgs::msg::Image>(
         "/unity/quadrator", rclcpp::SensorDataQoS(), std::bind(&SFyNode::unityCallback, this, _1));
  }

private:

  std::string source_topic_, source_file_;
  bool use_video_, use_quadrator_;
  int source_width_, shot_width_, res_width_, source_fps_, count;
  cv::Mat buf_image;  
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr quad_sub;

  int init_system()
  {
    this->SS = new SurroundSystem;
    this->count = 0;

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

    this->SS.addNewCam(SM0);
    this->SS.addNewCam(SM1);
    this->SS.addNewCam(SM2);  
    this->SS.addNewCam(SM3);
    this->SS.createStereopair(0, 1, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
    this->SS.createStereopair(1, 3, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
    this->SS.createStereopair(2, 0, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
    this->SS.createStereopair(3, 2, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);

    this->SS.prepareLUTs(); 
    RCLCPP_INFO(get_logger(), "LUTs ready");

    return 0;
  }

  void fillCamInfos(sensor_msgs::msg::CameraInfo &lcam_info, sensor_msgs::msg::CameraInfo &rcam_info)
  {
      lcam_info.width = 540;
      lcam_info.height = 540;
      rcam_info.width = 540;
      rcam_info.height = 540;
      for (int c = 0; c < lcam_info.p.size(); c++)
      {
        lcam_info.p.at(c) = Pl[c];
        rcam_info.p.at(c) = Pr[c];
      }
      for (int c = 0; c < lcam_info.k.size(); c++)
      {
        lcam_info.k.at(c) = K[c];
        rcam_info.k.at(c) = K[c];
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
      this->SS.getImage(0, SurroundSystem::RECTIFIED, front_left, front_right, remappedCombs[0]);
      this->SS.getImage(1, SurroundSystem::RECTIFIED, front_right, back_right, remappedCombs[1]);
      this->SS.getImage(2, SurroundSystem::RECTIFIED, back_left, front_left, remappedCombs[2]);
      this->SS.getImage(3, SurroundSystem::RECTIFIED, back_right, back_left,  remappedCombs[3]);
                    

      for (int i=0; i<remappedCombs.size(); i++)
      {
          cv::Mat left = remappedCombs[i](cv::Rect(0, 0, 540, 540));
          cv::Mat right = remappedCombs[i](cv::Rect(540, 0, 540, 540));

          std_msgs::msg::Header hdr;
          hdr.stamp = rclcpp::Clock{}.now();
          // hdr.seq = this->count;         not in ROS2
          sensor_msgs::msg::CameraInfo linfo;
          sensor_msgs::msg::CameraInfo rinfo;

          hdr.frame_id = sp_frames[i][0];
          linfo.header = hdr;
          sensor_msgs::msg::Image::SharedPtr lmsg = cv_bridge::CvImage(hdr, "bgr8", left).toImageMsg();
          hdr.frame_id = sp_frames[i][1];
          rinfo.header = hdr; 
          sensor_msgs::msg::Image::SharedPtr rmsg = cv_bridge::CvImage(hdr, "bgr8", right).toImageMsg();
          fillCamInfos(linfo, rinfo);
          std::vector<sensor_msgs::msg::Image::SharedPtr> msgs = {lmsg, rmsg};
          
          this->pubs.at(2*i).publish(lmsg);
          this->pubs.at(2*i+1).publish(rmsg);

          this->infos.at(2*i).publish(linfo);
          this->infos.at(2*i+1).publish(rinfo);
      }
  }

  void unityCallback(const sensor_msgs::msg::Image& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
    //buf_image = cv_ptr->image;
    cv::flip(cv_ptr->image, buf_image,0);
    //cv::imshow("img", cv_ptr->image);

    this->processImages(buf_image);
    this->count++;
  }


};

int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   auto node = std::make_shared<SFyNode>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
}

