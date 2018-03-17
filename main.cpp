#include "BayDetector.hpp"

#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <memory>
#include <iostream>

int main(int argc, char **argv)
{

    std::cout << "This program take a point cloud file and detect parking bays in it  " << std::endl;

    if (argc != 2)
    {
        std::cout << "./BayDetector cloud.ply  " << std::endl;

        return 0;
    }

    auto bayDetector = std::make_shared<parkopedia::BayDetector>();

    if (bayDetector->setInput(std::string(argv[1])))
    {
        bayDetector->printPointCloudInfo();
        
        // bayDetector->downsampleCloud(0.2);
        
        bayDetector->filterPassThrough("z", -0.2, 0.2);

        bayDetector->detectGroundPlane();

        cv::Mat groundPlaneImg;

        groundPlaneImg = bayDetector->getPlaneImage(100);

     
        // cv::Mat resized;
       // cv::resize(groundPlaneImg, resized, cv::Size(), 0.1, 0.1, cv::INTER_AREA);
        imwrite("fp1.jpg", groundPlaneImg);

        
    }

    return 0;
}
