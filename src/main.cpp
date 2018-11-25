#include <ros/ros.h>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>

inline void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> threeCloudsVis (
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2,
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud3)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud2, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud2, single_color2, "cloud2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(cloud3, 0, 0, 255);
  viewer->addPointCloud<pcl::PointXYZ> (cloud3, single_color3, "cloud3");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud3");

  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudFromMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string map_name, int z){
  // 地図の読み込み
  std::string homepath = std::getenv("HOME");
  cv::Mat img = cv::imread(homepath + "/catkin_ws/src/icp_map_combiner/map/" + map_name, 0);
  if(img.empty()){
    ROS_ERROR("map: unable to open the map");
  }else{
    ROS_INFO("map: map loaded");
  }
  
  // 地図データをOpenCVからEigenに渡す
  Eigen::MatrixXd img_e;
  cv::cv2eigen(img, img_e);
  ROS_INFO("map: opencv -> eigen");

  int plot_num = 0;
  for(int i=0; i<img_e.rows(); i++){
    for(int j=0; j<img_e.cols(); j++){
      if(img_e(j, i) < 205){
        plot_num++;
      }
    }
  }

  cloud->width    = plot_num;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);

  int index = 0;
  for(int i=0; i<img_e.rows(); i++){
    for(int j=0; j<img_e.cols(); j++){
      if(img_e(j, i) < 205){
        cloud->points[index].x = j;
        cloud->points[index].y = i;
        cloud->points[index].z = z;
        index++;
      }
    }
  }

  return cloud;
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "icp_map_combiner");

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_in = getPointCloudFromMap(cloud_in, "map1.pgm", 0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_out = getPointCloudFromMap(cloud_out, "map2.pgm", 100);

  // ICP で変換行列を算出
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(cloud_in);
  icp.setInputTarget(cloud_out);
  pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
  icp.align(cloud_source_registered);

  // 変換行列を表示
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
  transformation_matrix = icp.getFinalTransformation ().cast<double>();
  print4x4Matrix (transformation_matrix);

  // 変換行列から ICP で推定された点群 cloud_final を取得
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud_in, *cloud_final, transformation_matrix); 

  // 点群の可視化
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  viewer = threeCloudsVis(cloud_in, cloud_out, cloud_final);
  while (!viewer->wasStopped()) {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}
