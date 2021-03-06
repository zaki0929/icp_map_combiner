#include <ros/ros.h>
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
                   
inline void print4x4Matrix (const Eigen::Matrix4d & matrix) {
    printf ("Rotation matrix :\n");
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf ("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf ("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf ("Translation vector :\n");
    printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> threeCloudsVis (
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud3
) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
  
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
  
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(cloud2, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud2, single_color2, "cloud2");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud2");
  
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(cloud3, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ> (cloud3, single_color3, "cloud3");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud3");
  
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    return viewer;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloudFromMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string map_name, double offset_x, double offset_y, double offset_z) {
    // 地図の読み込み
    std::string homepath = std::getenv("HOME");
    cv::Mat img = cv::imread(homepath + "/catkin_ws/src/icp_map_combiner/map/" + map_name, 0);
    if (img.empty()) {
        ROS_ERROR("map: unable to open the map");
    }
    
    // 地図データをOpenCVからEigenに渡す
    Eigen::MatrixXd img_e;
    cv::cv2eigen(img, img_e);
  
    int plot_num = 0;
    for (int i=0; i<img_e.rows(); i++) {
        for (int j=0; j<img_e.cols(); j++) {
            if (img_e(j, i) < 205) {
                plot_num++;
            }
        }
    }
  
    cloud->width    = plot_num;
    cloud->height   = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
  
    int index = 0;
    for (int i=0; i<img_e.rows(); i++) {
        for (int j=0; j<img_e.cols(); j++) {
            if (img_e(i, j) < 205) {
                cloud->points[index].x = j + offset_x;
                cloud->points[index].y = -i + offset_y;
                cloud->points[index].z = 0 + offset_z;
                index++;
            }
        }
    }
    return cloud;
}

Eigen::MatrixXd translateMap(Eigen::MatrixXd map, int x, int y) {
    Eigen::MatrixXd translated_map = Eigen::MatrixXd::Ones(map.rows(), map.cols())*205;
    for (int i=0; i<map.rows(); i++) {
        for (int j=0; j<map.cols(); j++) {
            if (map(i, j) != 205){
                if (i+y < translated_map.rows() && i+y >=0
                &&  j-x < translated_map.cols() && j-x >= 0) {
                    translated_map(i+y, j-x) = map(i, j);
                }
            }
        }
    }
    return translated_map;
}

Eigen::MatrixXd rotateMap(Eigen::MatrixXd map, int x, int y, double th) {
    Eigen::MatrixXd rotated_map = Eigen::MatrixXd::Ones(map.rows(), map.cols())*205;
    for (int i=0; i<map.rows(); i++) {
        for (int j=0; j<map.cols(); j++) {
            if (map(i, j) != 205) {
                int row_index = int( (((j-x+0.5)*std::cos(th)) - ((-y-i-0.5)*std::sin(th))) + x);
                int col_index = int(-(((j-x+0.5)*std::sin(th)) + ((-y-i-0.5)*std::cos(th))) - y);
                rotated_map(col_index, row_index) = map(i, j);
            }
        }
    }
    return rotated_map;
}

Eigen::Matrix4d getTransformationMatFromICP(std::string map1_name, std::string map2_name) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
      cloud_in = getPointCloudFromMap(cloud_in, map1_name, 0, 0, 0);
    
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
      cloud_out = getPointCloudFromMap(cloud_out, map2_name, 0, 0, 0);
    
      // ICP で変換行列を算出
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      icp.setInputCloud(cloud_in);
      icp.setInputTarget(cloud_out);
      pcl::PointCloud<pcl::PointXYZ> cloud_source_registered;
      icp.align(cloud_source_registered);
    
      // 変換行列を表示
      Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
      transformation_matrix = icp.getFinalTransformation().cast<double>();
    
      return transformation_matrix;
}

void combineMaps(std::string map1_name, std::string map2_name) {
    // 地図の読み込み
    std::string homepath = std::getenv("HOME");
    cv::Mat map1 = cv::imread(homepath + "/catkin_ws/src/icp_map_combiner/map/" + map1_name, 0);
    cv::Mat map2 = cv::imread(homepath + "/catkin_ws/src/icp_map_combiner/map/" + map2_name, 0);
    if (map1.empty() || map2.empty()) {
        ROS_ERROR("map: unable to open the map");
    }
    
    // 地図データをOpenCVからEigenに渡す
    Eigen::MatrixXd map1_e;
    Eigen::MatrixXd map2_e;
    cv::cv2eigen(map1, map1_e);
    cv::cv2eigen(map2, map2_e);
  
    // 推定位置で修正
    int diff_x = int((MAP2_POS_X - MAP1_POS_X) / RESOLUTION);
    int diff_y = int((MAP2_POS_Y - MAP1_POS_Y) / RESOLUTION);
    map2_e = translateMap(map2_e, diff_x, diff_y);
  
    double th = std::acos(MAP1_ORI_W) - std::acos(MAP2_ORI_W);
    map2_e = rotateMap(map2_e, MAP1_POS_X / RESOLUTION, MAP1_POS_Y / RESOLUTION, th);
  
    cv::eigen2cv(map2_e, map2);
    cv::imwrite(homepath + "/catkin_ws/src/icp_map_combiner/map/transformed_map2.pgm", map2);
    ROS_INFO("map: transformed map exported");
  
    // ICP で修正
    Eigen::Matrix4d tf_mat = getTransformationMatFromICP("map1.pgm", "transformed_map2.pgm");
    map2_e = translateMap(map2_e, tf_mat(0, 3), tf_mat(1, 3));
    if (tf_mat(1, 0) < 0) {
        th = std::acos(tf_mat(0, 0));
    } else {
        th = -std::acos(tf_mat(0, 0));
    }
    map2_e = rotateMap(map2_e, 0, 0, th);
  
    // 地図を合成
    for (int i=0; i<map1_e.rows(); i++) {
        for (int j=0; j<map1_e.cols(); j++) {
            if (map1_e(i, j) == 205) {
                map1_e(i, j) = map2_e(i, j);
            }
        }
    }
  
    cv::eigen2cv(map1_e, map1);
    cv::imwrite(homepath + "/catkin_ws/src/icp_map_combiner/export/combined_map.pgm", map1);
    ROS_INFO("map: combined map exported");
}

void visualizeICP(std::string map1_name, std::string map2_name) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_in = getPointCloudFromMap(cloud_in, map1_name, 0, 0, 0);
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    cloud_out = getPointCloudFromMap(cloud_out, map2_name, 0, 0, 0);
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_in, *cloud_final, getTransformationMatFromICP(map1_name, map2_name)); 
  
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = threeCloudsVis(cloud_in, cloud_out, cloud_final);
    while (!viewer->wasStopped()) {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "icp_map_combiner");
  
    // 地図の合成
    combineMaps("map1.pgm", "map2.pgm");
  
    // 点群の可視化
    visualizeICP("map1.pgm", "transformed_map2.pgm");
}
