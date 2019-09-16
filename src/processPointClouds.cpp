// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "kdtree.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);

    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    //To filter out the regions outside the defined region.

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region(true);
    region.setMax(maxPoint);
    region.setMin(minPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    //To remove the points above the lidar head (roof points).
    
    pcl::CropBox<PointT> roof(true);
    roof.setMax(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMin(Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point : indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract; //created an object extract.
    extract.setInputCloud (cloudRegion); //passing the filtered Cloud Regions
    extract.setIndices (inliers); //passing inliers created previously
    extract.setNegative (true);
    extract.filter (*cloudRegion); //to filter out the roof points


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

    for(int index : inliers->indices){
        planeCloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract; //created an object extract.
    extract.setInputCloud (cloud); //passing cloud inputs
    extract.setIndices (inliers); //passing inliers created previously
    extract.setNegative (true);
    extract.filter (*obstacleCloud); //to filter out the obstacles

    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);

    //Created a pair of obstacle cloud and plane cloud sgementation results.
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    //passing inliers by reference.

    /*
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices}; //helps to seperate the plane from points.
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true); //Optional in the document
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC); 
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    //To segment the planar component from the input cloud

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);  //dereference.
    */

  std::unordered_set<int> inliersResult;
  srand(time(NULL));
  
  // TODO: Fill in this function

  // For max iterations 

  while(maxIterations--)
  {
    // Randomly sample subset and fit line
    std::unordered_set<int> inliers;

    //to insert inlier points if they are less.
    while(inliers.size() < 3)   //points than 3 -> Segmentation fault error might occur if it is not changed.
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
    float D = -1.0*(A*x1 + B*y1 + C*z1); // constant term

    // Measure distance between every point and fitted line

    for(int index = 0; index < cloud->points.size(); index++)
    {
      if(inliers.count(index) > 0)
        continue;

      PointT point = cloud->points[index];
      float x4 = point.x;
      float y4 = point.y;
      float z4 = point.z;


      float dist_3d = fabs(A*x4+B*y4+C*z4 + D)/sqrt(A*A+B*B+C*C);

      // If distance is smaller than threshold count it as inlier
      if(dist_3d <= distanceThreshold)  //changed distanceTOI to distanceThreshold
        inliers.insert(index);
    }

    // Return indices of inliers from fitted line with most inliers
    if(inliers.size() > inliersResult.size())
    {
      inliersResult = inliers;
    }

}

    typename pcl::PointCloud<PointT>::Ptr  pointsInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr  pointsOutliers(new pcl::PointCloud<PointT>());

    for(int index = 0; index < cloud->points.size(); index++) {
        PointT point = cloud->points[index];
        if(inliersResult.count(index)) {
            pointsInliers->points.push_back(point);
        }
        else {
            pointsOutliers->points.push_back(point);
        }   
    }

    if(inliersResult.size()==0){
        std::cout<<"Planar component cannot be extimated for the given dataset"<<std::endl;
    }


    //here, it Outputs the result of what seperate clouds were outputting in the above function -> "segmentPlane"
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(pointsOutliers, pointsInliers);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree *tree = new KdTree; // tree object of a kdTree
    std::vector<std::vector<float>> points; //To collect the cloud points
    for (int i = 0; i < cloud->points.size(); i++)
    {

    //Defining Point vector to collect 3 points form the input cloud.
    std::vector<float> point({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z});
    //collections of all the points into Points vector
    points.push_back(point);
    //insert the poiints into the tree
    tree->insert(points[i], i);
    }

    //finding clusters using euclidean cluster

    std::vector<std::vector<int>> clustersIndices;
    std::vector<bool> processed(points.size(), false);
    
    int i = 0;
    while(i < points.size())
    {
        if(processed[i])
        {
            i++;
            continue;
        }
        std::vector<int> cluster;
        tree->clusterHelper(i, points,cluster, processed, tree, clusterTolerance);
        clustersIndices.push_back(cluster);
        i++;
    }

/*
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);


    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);
*/

    //Getting the indices of the clusters created
    for (std::vector<int> getIndices: clustersIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        //Loop through all the indices.
        for (int index: getIndices)
        {
            cloudCluster->points.push_back(cloud->points[index]);
        }

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        //cloudCluster->is_dense = true;

    // Adding min and max size boundary conditions to remove unwanted clusters.
        if((cloudCluster->width >= minSize) && (cloudCluster->height <= maxSize))
        {
        clusters.push_back(cloudCluster);
        }

    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
