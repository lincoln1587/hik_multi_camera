#include <iostream>
#include "opencv2/opencv.hpp"
#include <vector>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include "hikrobot_camera.hpp"

// 剪裁掉照片和雷达没有重合的视角，去除多余像素可以使rosbag包变小
#define FIT_LIDAR_CUT_IMAGE false
#if FIT_LIDAR_CUT_IMAGE
    #define FIT_min_x 420
    #define FIT_min_y 70
    #define FIT_max_x 2450
    #define FIT_max_y 2000
#endif 

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    //********** variables    **********/
    cv::Mat src[3];
    //string src = "",image_pub = "";
    //********** rosnode init **********/
    ros::init(argc, argv, "hikrobot_camera");
    ros::NodeHandle hikrobot_camera;
    camera::Camera MVS_cap(hikrobot_camera);
    //********** rosnode init **********/
    image_transport::ImageTransport main_cam_image(hikrobot_camera);
    image_transport::CameraPublisher image_pub0 = main_cam_image.advertiseCamera("/hikrobot_camera0/rgb", 1000);
    image_transport::CameraPublisher image_pub1 = main_cam_image.advertiseCamera("/hikrobot_camera1/rgb", 1000);
    image_transport::CameraPublisher image_pub2 = main_cam_image.advertiseCamera("/hikrobot_camera2/rgb", 1000);



    sensor_msgs::Image image_msg0, image_msg1, image_msg2;
    sensor_msgs::CameraInfo camera_info_msg0, camera_info_msg1, camera_info_msg2;
    cv_bridge::CvImagePtr cv_ptr0 = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr0->encoding = sensor_msgs::image_encodings::BGR8;  // 就是rgb格式 
    cv_bridge::CvImagePtr cv_ptr1 = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr1->encoding = sensor_msgs::image_encodings::BGR8;  // 就是rgb格式   
    cv_bridge::CvImagePtr cv_ptr2 = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr2->encoding = sensor_msgs::image_encodings::BGR8;  // 就是rgb格式  
    //********** 10 Hz        **********/
    ros::Rate loop_rate(30);

    while (ros::ok())
    {

        loop_rate.sleep();
        ros::spinOnce();

        MVS_cap.ReadImg(src[0],0);   
        MVS_cap.ReadImg(src[1],1);
        MVS_cap.ReadImg(src[2],2);
        if (src[0].empty() && src[1].empty() && src[2].empty())
        {
            //printf(" no images!!!!!\n");
            continue;
        }
        
#if FIT_LIDAR_CUT_IMAGE
        cv::Rect area(FIT_min_x,FIT_min_y,FIT_max_x-FIT_min_x,FIT_max_y-FIT_min_y); // cut区域：从左上角像素坐标x，y，宽，高
        cv::Mat src_new = src(area);
        cv_ptr->image = src_new;
#else
        cv_ptr0->image = src[0];
        cv_ptr1->image = src[1];
        cv_ptr2->image = src[2];
#endif

/***************camera 0***********/
        image_msg0 = *(cv_ptr0->toImageMsg());
        image_msg0.header.stamp = ros::Time::now();  // ros发出的时间不是快门时间
        image_msg0.header.frame_id = "hikrobot_camera0";

        camera_info_msg0.header.frame_id = image_msg0.header.frame_id;
	    camera_info_msg0.header.stamp = image_msg0.header.stamp;
        image_pub0.publish(image_msg0, camera_info_msg0);


/*****************camera 1****************/
        image_msg1 = *(cv_ptr1->toImageMsg());
        image_msg1.header.stamp = ros::Time::now();  // ros发出的时间不是快门时间
        image_msg1.header.frame_id = "hikrobot_camera1";

        camera_info_msg1.header.frame_id = image_msg1.header.frame_id;
	    camera_info_msg1.header.stamp = image_msg1.header.stamp;
        image_pub1.publish(image_msg1, camera_info_msg1);



/*******************camera 2 ***********************/
        image_msg2 = *(cv_ptr2->toImageMsg());
        image_msg2.header.stamp = ros::Time::now();  // ros发出的时间不是快门时间
        image_msg2.header.frame_id = "hikrobot_camera2";

        camera_info_msg2.header.frame_id = image_msg2.header.frame_id;
	    camera_info_msg2.header.stamp = image_msg2.header.stamp;
        image_pub2.publish(image_msg2, camera_info_msg2);

        //*******************************************************************************************************************/
    }
    return 0;
}
