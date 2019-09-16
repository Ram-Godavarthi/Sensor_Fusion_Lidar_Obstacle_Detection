/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}


std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{

  auto startTime = std::chrono::steady_clock::now();
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

  while(maxIterations--)
  {
    // Randomly sample subset and fit line
    std::unordered_set<int> inliers;

    //to insert inlier points if they are less.
    while(inliers.size() < 2)
      inliers.insert(rand()%(cloud->points.size()));

    // Getting #2 points frome the point cloud to draw a line
    float x1, y1, x2, y2;



  	// Randomly sample subset and fit line

    auto itr = inliers.begin();
    x1 = cloud->points[*itr].x;
    y1 = cloud->points[*itr].y;

    itr++; //next point
    
    x2 = cloud->points[*itr].x;
    y2 = cloud->points[*itr].y;

    float a = (y1-y2);
    float b = (x2-x1);
    float c = (x1*y2 - x2*y1);

    // Measure distance between every point and fitted line

    for(int index = 0; index < cloud->points.size(); index++)
    {
      if(inliers.count(index) > 0)
        continue;

      pcl::PointXYZ point = cloud->points[index];
      float x3 = point.x;
      float y3 = point.y;


      float d = fabs(a*x3+b*y3+c)/sqrt(a*a+b*b);

      // If distance is smaller than threshold count it as inlier
      if(d <= distanceTol)
        inliers.insert(index);
    }

    // Return indicies of inliers from fitted line with most inliers
    if(inliers.size() > inliersResult.size())
    {
      inliersResult = inliers;
    }

  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout<< "TOtal points in cloud: " << cloud->points.size()<<endl;
  std::cout << "Ransac took " << elapsedTime.count() << " milliseconds" << std::endl;

	
	return inliersResult;

}


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{

  auto startTime = std::chrono::steady_clock::now();
  std::unordered_set<int> inliersResult;
  srand(time(NULL));
  
  // TODO: Fill in this function

  // For max iterations 

  while(maxIterations--)
  {
    // Randomly sample subset and fit line
    std::unordered_set<int> inliers;

    //to insert inlier points if they are less.
    while(inliers.size() < 3) 	//points than 3 -> Segmentation fault error might occur if it is not changed.
      inliers.insert(rand()%(cloud->points.size()));

    // Getting #3 points frome the point cloud to draw a line in 3D
    float x1, y1, z1;
    float x2, y2, z2; 
    float x3, y3, z3;

    // Randomly sample subset and fit the plane by taking 3 points

    auto itr = inliers.begin();
    x1 = cloud->points[*itr].x;
    y1 = cloud->points[*itr].y;
    z1 = cloud->points[*itr].z;

    itr++; //next point

    x2 = cloud->points[*itr].x;
    y2 = cloud->points[*itr].y;
    z2 = cloud->points[*itr].z;

    itr++; //next point

    x3 = cloud->points[*itr].x;
    y3 = cloud->points[*itr].y;
    z3 = cloud->points[*itr].z;

    //To find A, B, C, D in th equation (Ax + By + Cz + D = 0)

    float A = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1); // i equation
    float B = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1); // j equation
    float C = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1); // K equation
   
    //Assigning respective values here
    //float A = i;
    //float B = j;
    //float C = k;
    float D = -(A*x1 + B*y1 + C*z1); // constant term

    // Measure distance between every point and fitted line

    for(int index = 0; index < cloud->points.size(); index++)
    {
      if(inliers.count(index) > 0)
        continue;

      pcl::PointXYZ point = cloud->points[index];
      float x4 = point.x;
      float y4 = point.y;
      float z4 = point.z;


      float dist_3d = fabs(A*x4+B*y4+C*z4 + D)/sqrt(A*A+B*B+C*C);

      // If distance is smaller than threshold count it as inlier
      if(dist_3d <= distanceTol)
        inliers.insert(index);
    }

    // Return indicies of inliers from fitted line with most inliers
    if(inliers.size() > inliersResult.size())
    {
      inliersResult = inliers;
    }

  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout<< "Total points in cloud: " << cloud->points.size()<<endl;
  std::cout << "Ransac Plane took " << elapsedTime.count() << " milliseconds" << std::endl;

  
  return inliersResult;

}

  
int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 10, 0.3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
