#pragma once

#include <string>
#include <opencv2/opencv.hpp>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>


namespace parkopedia
{
class BayDetector
{
  public:
    BayDetector();
    ~BayDetector();
    bool setInput(std::string inputFile);
    void detectGroundPlane();
    void printPointCloudInfo();
    void downsampleCloud(float vSize = 0.05f);
    cv::Mat getPlaneImage(float resolution = 100);
    void filterPassThrough(std::string dim, float min, float max);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getSubsampledCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getGroundCloud();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getFilteredCloud();



  private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _subsampledCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _groundCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr _filteredCloud;

};
}
