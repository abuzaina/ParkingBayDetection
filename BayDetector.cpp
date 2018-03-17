#include "BayDetector.hpp"

namespace parkopedia
{

BayDetector::BayDetector()
{
    //ctor
}

BayDetector::~BayDetector()
{
    //dtor
}

bool BayDetector::setInput(std::string inputFile)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (inputFile.substr(inputFile.find_last_of(".") + 1) == "ply")
    {
        if (pcl::io::loadPLYFile(inputFile, *cloud) != 0)
        {
            std::cout << "Could not load file \n";
            return 0;
        }
    }

    else if (inputFile.substr(inputFile.find_last_of(".") + 1) == "pcd")
    {
        if (pcl::io::loadPCDFile(inputFile, *cloud) != 0)
        {
            std::cout << "Could not load file \n";
            return 0;
        }
    }

    _cloud = cloud;

    return 1;
}

void BayDetector::printPointCloudInfo()
{
    std::cout << _cloud->width * _cloud->height << " data points \n ";
    //std::cout << "cloud size :" << _cloud->points.size() << "\n";
}


void BayDetector::downsampleCloud(float vSize)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr subsampledCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud(_cloud);
    voxel_grid.setLeafSize(vSize, vSize, vSize);
    voxel_grid.filter(*subsampledCloud);
    std::cout << subsampledCloud->width * subsampledCloud->height << " data points in downsampled cloud \n ";
    _subsampledCloud = subsampledCloud;
    //pcl::io::savePCDFile("cloudSubsampled.pcd", *_subsampledCloud);
}

void BayDetector::detectGroundPlane()
{
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // seg.setOptimizeCoefficients(true);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setDistanceThreshold(0.05);
    // seg.setInputCloud(filteredCloud);
    // seg.segment(*inliers, *coefficients);

    // if (inliers->indices.size() == 0)
    // {
    //     PCL_ERROR("Could not estimate a planar model for the given dataset.");
    // }

    // std::cerr << "Model coefficients: " << coefficients->values[0] << " "
    //           << coefficients->values[1] << " "
    //           << coefficients->values[2] << " "
    //           << coefficients->values[3] << std::endl;

    // pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    // extract.setInputCloud(filteredCloud);
    // extract.setIndices(inliers);
    // extract.setNegative(false);
    // extract.filter(*planeCloud);
    // pcl::io::savePCDFile("planeCloud.pcd", *planeCloud);

    pcl::ModelCoefficients::Ptr coefficients2(new pcl::ModelCoefficients());
    coefficients2->values.resize(4);
    coefficients2->values[0] = coefficients2->values[1] = 0;
    coefficients2->values[2] = 1.0;
    coefficients2->values[3] = 0;
    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(_filteredCloud);
    proj.setModelCoefficients(coefficients2);
    proj.filter(*projected_cloud);

    _groundCloud = projected_cloud;

    //pcl::io::savePCDFile("projected_cloud.pcd", *projected_cloud);
}

void BayDetector::filterPassThrough(std::string dim, float min, float max)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(_cloud);
    pass.setFilterFieldName(dim);
    pass.setFilterLimits(min, max);
    pass.filter(*filteredCloud);

    _filteredCloud = filteredCloud;
    
}

cv::Mat BayDetector::getPlaneImage(float resolution)
{

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D(*_groundCloud, minPt, maxPt);
    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl;

    Eigen::Vector3f y_direction(0.0f, 1.0f, 0.0f);
    Eigen::Vector3f z_axis(0.0f, 0.0f, 1.0f);
    Eigen::Vector3f origin(-minPt.x, -minPt.y, -minPt.z);
    Eigen::Affine3f transformation;

    pcl::getTransformationFromTwoUnitVectorsAndOrigin(y_direction, z_axis, origin, transformation);

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2.translation() << origin;

    pcl::transformPointCloud(*_groundCloud, *transCloud, transform_2);
    pcl::getMinMax3D(*transCloud, minPt, maxPt);

    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl;

    int w = maxPt.x * resolution;
    int h = maxPt.y * resolution;

    cv::Mat image = cv::Mat(h, w, CV_8UC3, cv::Scalar(0, 0, 0));

#pragma omp parallel for
    for (size_t i = 0; i < transCloud->points.size(); i++)
    {
        pcl::PointXYZRGB point = transCloud->points[i];
        float x = w - point.x * resolution;
        float y = h - point.y * resolution;

        // uint32_t rgb = *reinterpret_cast<int *>(&point.rgb);
        // uint8_t r = (rgb >> 16) & 0x0000ff;
        // uint8_t g = (rgb >> 8) & 0x0000ff;
        // uint8_t b = (rgb)&0x0000ff;

        image.at<cv::Vec3b>(x, y)[0] = point.b;
        image.at<cv::Vec3b>(x, y)[1] = point.g;
        image.at<cv::Vec3b>(x, y)[2] = point.r;
    }

    return image;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr BayDetector::getSubsampledCloud()
{
    return _subsampledCloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr BayDetector::getGroundCloud()
{
    return _groundCloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr BayDetector::getFilteredCloud()
{
    return _filteredCloud;
}


}
