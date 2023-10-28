#include <ros/ros.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
using namespace cv;
using namespace std;
static const std::string OPENCV_WINDOW = "Raw Image window";
static const std::string OPENCV_WINDOW_1 = "Edge Detection";
class Edge_Detector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  ros::Subscriber obj_sub_;
  image_transport::Publisher image_pub_;
  cv::Mat global_map,src_gray;
  vector<vector<cv::Point>> obj_con;
  
public:
  Edge_Detector()
    : it_(nh_)
  {
    
    // Subscribe to input video feed and publish output video feed
    ROS_INFO("HAHHAH1");

    obj_sub_ = nh_.subscribe("/object_at_robot_height", 1, 
      &Edge_Detector::imageCb, this);
    image_pub_ = it_.advertise("/edge_detector/raw_image", 1);
    global_map = cv::imread("/home/linuxlaitang/ORB_SLAM3/map/map.pgm", 0); 
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy,hierarchy2;
    //normalize(global_map, global_map, 0, 255, cv::NORM_MINMAX);
    ROS_INFO("start cv");

    findContours(global_map,contours,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_TC89_L1 ,cv::Point()); 
        // Simplify the shape of each contour

    std::vector<std::vector<cv::Point>> simplifiedContours(contours.size());
    for (size_t i = 0; i < contours.size(); ++i) {
        cv::approxPolyDP(contours[i], simplifiedContours[i],1.0, true);  // Adjust the epsilon value as needed
    }

    // Draw the simplified contours on a black image
    cv::Mat resultImage = cv::Mat::zeros(global_map.size(), CV_8UC1);
    cv::Mat resultImage2 = cv::Mat::zeros(global_map.size(), CV_8UC1);
    ROS_INFO("start cv2");
    cv::drawContours(resultImage, simplifiedContours, -1, cv::Scalar(255, 255, 255), FILLED);
	  findContours(resultImage,obj_con,hierarchy,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE,cv::Point()); 
    ROS_INFO("start cv3");
    cv::drawContours(resultImage2, obj_con, -1, cv::Scalar(255, 255, 255), 1);
    ROS_INFO("start cv4");
    // ROS_INFO("B4:%d After:%d h:%d",contours.size(),simplifiedContours.size(),hierarchy.size());
  }

  ~Edge_Detector()
  {
    // cv::destroyWindow(OPENCV_WINDOW);
  }
  vector<vector<cv::Point>> get_obj_pt(){
    return obj_con;
  }
  void imageCb(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    pcl::PointCloud<pcl::PointXYZ> latest_cloud;
	pcl::fromROSMsg(*msg, latest_cloud);

	if (latest_cloud.points.size() == 0) return;

	pcl::PointXYZ pt;
	Eigen::Vector3d p3d, p3d_inf;

	for (size_t i = 0; i < latest_cloud.points.size(); ++i) {
		pt = latest_cloud.points[i];
		
	}
	detect_edges(global_map);
    	// image_pub_.publish(cv_ptr->toImageMsg());

  }
  
  cv::Mat detect_edges(cv::Mat img)
  {

   	cv::Mat src, src_gray;
	cv::Mat dst, detected_edges;

	int edgeThresh = 1;
	int lowThreshold = 200;
	int highThreshold =300;
	int kernel_size = 5;

	cv::cvtColor( img, src_gray, CV_RGB2GRAY );
	// cv::Canny( src_gray, detected_edges, lowThreshold, highThreshold, kernel_size );

    cv::imshow(OPENCV_WINDOW, img);
    cv::imshow(OPENCV_WINDOW_1, src_gray);
    cv::waitKey(0);
    return img;
  }	
 
};

