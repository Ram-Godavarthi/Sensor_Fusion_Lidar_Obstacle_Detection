/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"

// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 

    Lidar* myLidar = new Lidar(cars, 0); //Ground plane is at 0 (SObject with slope =0)
    pcl::PointCloud<pcl::PointXYZ>::Ptr myInputCloud = myLidar->scan(); //to create a point cloud
    //renderRays(viewer, myLidar->position, myInputCloud);
    //renderPointCloud(viewer, myInputCloud, "MyInputCloud");


    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;	
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(myInputCloud, 100, 0.2);
    std::cout<<"Simple Highway from environment executed: "<<std::endl;
    renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1,0,0)); 
    //renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 3, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,1), Color(0,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: cloudClusters)
    {
        std::cout<<"Cluster Size: ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster,"ObstacleCloud"+std::to_string(clusterId),colors[clusterId]);

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputPcd)
{
    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputPcd = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer, inputPcd, "inputCloud");
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredPcd = pointProcessorI->FilterCloud(inputPcd, 0.25, Eigen::Vector4f(-11,-6,-2,1), Eigen::Vector4f(35,6,2,1));
    //renderPointCloud(viewer, filteredPcd, "FilteredCloud");

    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr , pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedPcd = pointProcessorI->SegmentPlane(filteredPcd, 20, 0.3);
    //renderPointCloud(viewer, segmentedPcd.first, "segmentedObstacle", Color(1,0,0));
    //renderPointCloud(viewer, segmentedPcd.second, "segmentedPlane", Color(0,1,0));
    
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusteredPcd = pointProcessorI->Clustering(segmentedPcd.first, 0.45 , 12 , 550 );
    int clusterId = 0;
    std::vector<Color> colors = {Color(0,1,1), Color(1,1,0), Color(0,0,1)};

    //Box source_head = {-1, -1.5, -1, 2, 1.4, -0.4};
    //renderBox(viewer, source_head, 0, Color(1, 1, 1), 0.8);
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: clusteredPcd)
    {
        std::cout<<"Cluster Size: ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster,"ClusteredObstacle"+std::to_string(clusterId),colors[clusterId%colors.size()]);

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
    renderPointCloud(viewer, segmentedPcd.second, "segmentatedPlane", Color(0,1,0));
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    //To create a point processor and get the data from file location using streamPcd

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
/*
    //To work and understand the processing of point cloud data
    simpleHighway(viewer);

    //This was used to do filtering, segmentation, clustering on single frame
    cityBlock(viewer);  
*/
    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    } 
}
