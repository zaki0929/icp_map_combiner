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

#define RESOLUTION 0.05

#define MAP1_POS_X 53.2510726225
#define MAP1_POS_Y -43.5945096362
#define MAP1_POS_Z 0.0
#define MAP1_ORI_X 0.0
#define MAP1_ORI_Y 0.0
#define MAP1_ORI_Z 0.802374434033
#define MAP1_ORI_W 0.596820967804

#define MAP2_POS_X 54.2293342001
#define MAP2_POS_Y -38.9508724862
#define MAP2_POS_Z 0.0
#define MAP2_ORI_X 0.0             
#define MAP2_ORI_Y 0.0
#define MAP2_ORI_Z 0.905210257215
#define MAP2_ORI_W 0.424963986984
                   
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

pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudFromMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string map_name, double offset_x, double offset_y, double offset_z)
{
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
        cloud->points[index].x = j + offset_x;
        cloud->points[index].y = i + offset_y;
        cloud->points[index].z = 0 + offset_z;
        index++;
      }
    }
  }

  return cloud;
}

void combine_maps(std::string map1_name, std::string map2_name)
{
  // 地図の読み込み
  std::string homepath = std::getenv("HOME");
  cv::Mat map1 = cv::imread(homepath + "/catkin_ws/src/icp_map_combiner/map/" + map1_name, 0);
  cv::Mat map2 = cv::imread(homepath + "/catkin_ws/src/icp_map_combiner/map/" + map2_name, 0);
  if(map1.empty() || map2.empty()){
    ROS_ERROR("map: unable to open the map");
  }else{
    ROS_INFO("map: map loaded");
  }
  
  // 地図データをOpenCVからEigenに渡す
  Eigen::MatrixXd map1_e;
  Eigen::MatrixXd map2_e;
  cv::cv2eigen(map1, map1_e);
  cv::cv2eigen(map2, map2_e);
  ROS_INFO("map: opencv -> eigen");

  // 地図を動かす
  Eigen::MatrixXd moved_map2_e = Eigen::MatrixXd::Ones(map2_e.rows(), map2_e.cols())*205;
  int diff_x = int((MAP2_POS_X - MAP1_POS_X) / RESOLUTION);
  int diff_y = int((MAP2_POS_Y - MAP1_POS_Y) / RESOLUTION);
  for(int i=0; i<map2_e.rows(); i++){
    for(int j=0; j<map2_e.cols(); j++){
      if(map2_e(i, j) != 205){
        if(i+diff_y < moved_map2_e.rows() && i+diff_y >=0
        && j-diff_x < moved_map2_e.cols() && j-diff_x >= 0){
          moved_map2_e(i+diff_y, j-diff_x) = map2_e(i, j);
        }
      }
    }
  }
  
  // 地図を合成
  for(int i=0; i<map1_e.rows(); i++){
    for(int j=0; j<map1_e.cols(); j++){
      if(map1_e(i, j) == 205){
        map1_e(i, j) = moved_map2_e(i, j);
      }
    }
  }

  cv::eigen2cv(map1_e, map1);
  ROS_INFO("map: eigen -> opencv");
  cv::imwrite(homepath + "/catkin_ws/src/icp_map_combiner/export/combined_map.pgm", map1);
  ROS_INFO("map: combined map exported");
}



int main (int argc, char** argv)
{
  ros::init(argc, argv, "icp_map_combiner");

//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
//  cloud_in = getPointCloudFromMap(cloud_in, "map1.pgm", 0, 0, 0);
//  //cloud_in = getPointCloudFromMap(cloud_in, "map2.pgm", 0, 0, 0);
//
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
//  cloud_out = getPointCloudFromMap(cloud_out, "map1.pgm", 4.0, 4.0, 100);
//  //cloud_out = getPointCloudFromMap(cloud_out, "map2.pgm", 4.0, 4.0, 100);
//
//  // ICP で変換行列を算出
//  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//  icp.setInputCloud(cloud_in);
//  icp.setInputTarget(cloud_out);
//  pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
//  icp.align(cloud_source_registered);
//
//  // 変換行列を表示
//  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
//  transformation_matrix = icp.getFinalTransformation ().cast<double>();
//  print4x4Matrix (transformation_matrix);
//
//  // 変換行列から ICP で推定された点群 cloud_final を取得
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::transformPointCloud(*cloud_in, *cloud_final, transformation_matrix); 
//
//  // 点群の可視化
//  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//  viewer = threeCloudsVis(cloud_in, cloud_out, cloud_final);
//  while (!viewer->wasStopped()) {
//    viewer->spinOnce (100);
//    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//  }
//
  combine_maps("map1.pgm", "map2.pgm");
}
