
# mapOptimization

##  main function
```cpp
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    mapOptimization MO;

    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");
    
    std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);

    ros::spin();

    loopthread.join();
    visualizeMapThread.join();

    return 0;
}

```

mapOptimization.cpp에서는 TODO

## main variable
usage index
1. updateInitialGuess 
2. extractSurroundingKeyFrames
3. downsampleCurrentScan 
4. scan2MapOptimization
5. saveKeyFramesAndFactor 
6. correctPoses 
7. publishOdometry , publishFrames 

a. loop closure

|변수이름|type|usage|description|
|------|---|---|---|
|gtSAMgraph|gtsam::NonlinearFactorGraph                     |5||
|initialEstimate|gtsam::Values                              |5||
|isam|gtsam::ISAM2                                          |5||
|isamCurrentEstimate|gtsam::Values                          |5,6||
|poseCovariance|Eigen::MatrixXd                             |5||
|gpsQueue|std::deque<nav_msgs::Odometry>                    |5, gpsHandler()||
|cloudInfo|lio_sam::cloud_info                              |1,4,7||
|cloudKeyPoses3D|pcl::PointCloud\<PointType\>::Ptr          |1,2,4,5,6,7,a||
|cloudKeyPoses6D|pcl::PointCloud\<PointTypePose\>::Ptr      |2,5,6,7,a||
|copy_cloudKeyPoses3D|pcl::PointCloud\<PointType\>::Ptr     |a||
|copy_cloudKeyPoses6D|pcl::PointCloud\<PointTypePose\>::Ptr |a||
|laserCloudCornerLast|pcl::PointCloud\<PointType\>::Ptr     |3||
|laserCloudSurfLast|pcl::PointCloud\<PointType\>::Ptr       |3||
|laserCloudCornerLastDS|pcl::PointCloud\<PointType\>::Ptr   |3,4,7||
|laserCloudSurfLastDS|pcl::PointCloud\<PointType\>::Ptr     |3,4,7||
|laserCloudOri|pcl::PointCloud\<PointType\>::Ptr            |4||
|coeffSel|pcl::PointCloud\<PointType\>::Ptr                 |4||
|laserCloudOriCornerVec|std::vector\<PointType\>            |4||
|coeffSelCornerVec|std::vector\<PointType\>                 |4||
|laserCloudOriCornerFlag|std::vector\<bool\>                |4||
|laserCloudOriSurfVec|std::vector\<PointType\>              |4||
|coeffSelSurfVec|std::vector\<PointType\>                   |4||
|laserCloudOriSurfFlag|std::vector\<bool\>                  |4||
|laserCloudMapContainer|map\<int, pair\<pcl::PointCloud\<PointType\>, pcl::PointCloud\<PointType\>\>\>|2||
|laserCloudCornerFromMap|pcl::PointCloud\<PointType\>::Ptr  |2||
|laserCloudSurfFromMap|pcl::PointCloud\<PointType\>::Ptr    |2||
|laserCloudCornerFromMapDS|pcl::PointCloud\<PointType\>::Ptr|2,4,7||
|laserCloudSurfFromMapDS|pcl::PointCloud\<PointType\>::Ptr  |2,4,7||
|kdtreeCornerFromMap|pcl::KdTreeFLANN\<PointType\>::Ptr     |4||
|kdtreeSurfFromMap|pcl::KdTreeFLANN\<PointType\>::Ptr       |4||
|kdtreeSurroundingKeyPoses|pcl::KdTreeFLANN\<PointType\>::Ptr |2||
|kdtreeHistoryKeyPoses|pcl::KdTreeFLANN\<PointType\>::Ptr   |a||
|downSizeFilterCorner|pcl::VoxelGrid\<PointType\>           |4||
|downSizeFilterSurf|pcl::VoxelGrid\<PointType\>             |4||
|downSizeFilterICP|pcl::VoxelGrid\<PointType\>              |a||
|downSizeFilterSurroundingKeyPoses|pcl::VoxelGrid\<PointType\>|2||



## publisher and subscriber in mapOptimization
많은 publisher와 subscriber가 있지만 대부분은 visualize(rviz)에서 사용하기 위한 토픽들이다. imuIntegration code에서 사용하는 토픽은 pubLaserOdometryGlobal , pubLaserOdometryIncremental이고, mapotimization code 에서 주목해야 할 부분은 lidar 포인트클라우드 callback 함수인 laserCloudInfoHandler() 이다.

```cpp
pubKeyPoses                 = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/trajectory", 1);
pubLaserCloudSurround       = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_global", 1);
pubLaserOdometryGlobal      = nh.advertise<nav_msgs::Odometry> ("lio_sam/mapping/odometry", 1);
pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry> ("lio_sam/mapping/odometry_incremental", 1);
pubPath                     = nh.advertise<nav_msgs::Path>("lio_sam/mapping/path", 1);

subCloud = nh.subscribe<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 1, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
subGPS   = nh.subscribe<nav_msgs::Odometry> (gpsTopic, 200, &mapOptimization::gpsHandler, this, ros::TransportHints().tcpNoDelay());
subLoop  = nh.subscribe<std_msgs::Float64MultiArray>("lio_loop/loop_closure_detection", 1, &mapOptimization::loopInfoHandler, this, ros::TransportHints().tcpNoDelay());

srvSaveMap  = nh.advertiseService("lio_sam/save_map", &mapOptimization::saveMapService, this);

pubHistoryKeyFrames   = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);
pubIcpKeyFrames       = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_corrected_cloud", 1);
pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/lio_sam/mapping/loop_closure_constraints", 1);

pubRecentKeyFrames    = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_local", 1);
pubRecentKeyFrame     = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered", 1);
pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered_raw", 1);

pubSLAMInfo           = nh.advertise<lio_sam::cloud_info>("lio_sam/mapping/slam_info", 1);
```

## laserCloudInfoHandler()
laserCloudInfoHandler에서는 다음과정을 거친다.
1. updateInitialGuess - latest lidar frame에 대한 initial guess
2. extractSurroundingKeyFrames - latest lidar frame과 상대적으로 가까운 포인트클라우드만 추출
3. downsampleCurrentScan - latest lidar frame을 downsampling (extractSurroundingKeyFrames에서 얻은 global map과 일치시키기 위해)
4. scan2MapOptimization - 2,3 에서 얻은 pointcloud을 매칭하여, latest lidar frame 에 대한 좀 더 정확한 포즈를 추정
5. saveKeyFramesAndFactor - 지금까지 입력된 정보를 모두 사용하는 backend? TODO
6. correctPoses - 5. 에서 사용한 정보들의 state 를 업데이트
7. publishOdometry , publishFrames - 


```cpp
void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn)
{
    // extract time stamp
    timeLaserInfoStamp = msgIn->header.stamp;
    timeLaserInfoCur = msgIn->header.stamp.toSec();

    // extract info and feature cloud
    cloudInfo = *msgIn;
    pcl::fromROSMsg(msgIn->cloud_corner,  *laserCloudCornerLast);
    pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);

    std::lock_guard<std::mutex> lock(mtx);

    static double timeLastProcessing = -1;
    if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
    {
        timeLastProcessing = timeLaserInfoCur;

        updateInitialGuess();

        extractSurroundingKeyFrames();

        downsampleCurrentScan();

        scan2MapOptimization();

        saveKeyFramesAndFactor();

        correctPoses();

        publishOdometry();

        publishFrames();
    }
}
```

## updateInitailGuess()

사용하는 변수들
- cloudKeyPoses3D : saveKeyFramesAndFactor()에서 추가되고, correctPoses 에서 조정되는 변수. 현재 global map 에서 tracking되고 있는 frame 포즈들을 의미. 이 함수에서는 initial lidar frame인지 판단하기 위해 사용.
- lastImuPreTransformation : updateInitialGuess()에서 incremental relative pose 를 게산하기 위해 사용하는 변수
- transformTobeMapped : 현재 lidar frame 2 global map 간의 변환을 저장하는 변수. float[6] 형태. 이 함수의 목적임.

아이디어  
각 lidar frame 의 initial pose를 결정하는 함수. 첫 번째 라이다 프레임, 두 번째 라이다 프레임, 이후의 라이다 프레임, 세가지 상태로 나누어서 핸들링함.


```cpp
void updateInitialGuess()
{
    // save current transformation before any processing
    incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

    static Eigen::Affine3f lastImuTransformation;
    // initialization

    // 1. 첫번쨰 프레임(cloudKeyPoses3D->points.empty())인 경우 imu data 의 RPY를 사용해서 현재 프레임의 rotation을 초기화. 
    // global map이 형성되지 않은 상태이므로, translation vector 는 0으로 초기화 한다. (lastImuTransformation = pcl::getTransformation(0, 0, 0,  ... )
    if (cloudKeyPoses3D->points.empty())
    {
        transformTobeMapped[0] = cloudInfo.imuRollInit;
        transformTobeMapped[1] = cloudInfo.imuPitchInit;
        transformTobeMapped[2] = cloudInfo.imuYawInit;

        if (!useImuHeadingInitialization)
            transformTobeMapped[2] = 0;

        lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
        return;
    }

    // use imu pre-integration estimation for pose guess
    static bool lastImuPreTransAvailable = false;
    static Eigen::Affine3f lastImuPreTransformation;
    if (cloudInfo.odomAvailable == true)
    {
        Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX,    cloudInfo.initialGuessY,     cloudInfo.initialGuessZ, 
                                                            cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
        if (lastImuPreTransAvailable == false)
        {
            lastImuPreTransformation = transBack;
            lastImuPreTransAvailable = true;
        } else {
            Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuPreTransformation = transBack;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
            return;
        }
    }

    // use imu incremental estimation for pose guess (only rotation)
    // imu 정보가 있을 경우
    // transBack : T_wc ( latest lidar frame to world (world mean imu RPY))
    // transIncre : T_lc (latest lidar frame to current lidar frame)

    // transTobe : T_il
    // transFinal : T_ic (initial lidar frame to current )
    if (cloudInfo.imuAvailable == true)
    {
        Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
        Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

        Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
        Eigen::Affine3f transFinal = transTobe * transIncre;
        pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

        lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit); // save imu before return;
        return;
    }
}
```

## extractSurroundingKeyFrames()
```cpp
void extractSurroundingKeyFrames()
{
    // global map이 없다면, surrounding key frame 도 없는 것이므로 return
    if (cloudKeyPoses3D->points.empty() == true)
        return; 
    
    // if (loopClosureEnableFlag == true)
    // {
    //     extractForLoopClosure();    
    // } else {
    //     extractNearby();
    // }

    extractNearby();
}

void extractNearby()
{
    pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    // extract all the nearby key poses and downsample them
    kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
    kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
    for (int i = 0; i < (int)pointSearchInd.size(); ++i)
    {
        int id = pointSearchInd[i];
        surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
    }

    downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
    downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
    for(auto& pt : surroundingKeyPosesDS->points)
    {
        kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
        pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
    }

    // also extract some latest key frames in case the robot rotates in one position
    int numPoses = cloudKeyPoses3D->size();
    for (int i = numPoses-1; i >= 0; --i)
    {
        if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
            surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
        else
            break;
    }

    extractCloud(surroundingKeyPosesDS);
}

void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
{
    // fuse the map
    laserCloudCornerFromMap->clear();
    laserCloudSurfFromMap->clear(); 
    for (int i = 0; i < (int)cloudToExtract->size(); ++i)
    {
        if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
            continue;

        int thisKeyInd = (int)cloudToExtract->points[i].intensity;
        if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end()) 
        {
            // transformed cloud available
            *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
            *laserCloudSurfFromMap   += laserCloudMapContainer[thisKeyInd].second;
        } else {
            // transformed cloud not available
            pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(cornerCloudKeyFrames[thisKeyInd],  &cloudKeyPoses6D->points[thisKeyInd]);
            pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(surfCloudKeyFrames[thisKeyInd],    &cloudKeyPoses6D->points[thisKeyInd]);
            *laserCloudCornerFromMap += laserCloudCornerTemp;
            *laserCloudSurfFromMap   += laserCloudSurfTemp;
            laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
        }
        
    }

    // Downsample the surrounding corner key frames (or map)
    downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
    downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
    laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
    // Downsample the surrounding surf key frames (or map)
    downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
    downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
    laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

    // clear map cache if too large
    if (laserCloudMapContainer.size() > 1000)
        laserCloudMapContainer.clear();
}
```

## downsampleCurrentScan()
```cpp
void downsampleCurrentScan()
{
    // Downsample cloud from current scan
    laserCloudCornerLastDS->clear();
    downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
    downSizeFilterCorner.filter(*laserCloudCornerLastDS);
    laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

    laserCloudSurfLastDS->clear();
    downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
    downSizeFilterSurf.filter(*laserCloudSurfLastDS);
    laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
}
```

## scan2MapOptimization()
```cpp
void scan2MapOptimization()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
    {
        kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

        for (int iterCount = 0; iterCount < 30; iterCount++)
        {
            laserCloudOri->clear();
            coeffSel->clear();

            cornerOptimization();
            surfOptimization();

            combineOptimizationCoeffs();

            if (LMOptimization(iterCount) == true)
                break;              
        }

        transformUpdate();
    } else {
        ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
    }
}
```

## 
```cpp    
void saveKeyFramesAndFactor()
{
    if (saveFrame() == false)
        return;

    // odom factor
    addOdomFactor();

    // gps factor
    addGPSFactor();

    // loop factor
    addLoopFactor();

    // cout << "****************************************************" << endl;
    // gtSAMgraph.print("GTSAM Graph:\n");

    // update iSAM
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();

    if (aLoopIsClosed == true)
    {
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
    }

    gtSAMgraph.resize(0);
    initialEstimate.clear();

    //save key poses
    PointType thisPose3D;
    PointTypePose thisPose6D;
    Pose3 latestEstimate;

    isamCurrentEstimate = isam->calculateEstimate();
    latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
    // cout << "****************************************************" << endl;
    // isamCurrentEstimate.print("Current estimate: ");

    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
    cloudKeyPoses3D->push_back(thisPose3D);

    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
    thisPose6D.roll  = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw   = latestEstimate.rotation().yaw();
    thisPose6D.time = timeLaserInfoCur;
    cloudKeyPoses6D->push_back(thisPose6D);

    // cout << "****************************************************" << endl;
    // cout << "Pose covariance:" << endl;
    // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
    poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

    // save updated transform
    transformTobeMapped[0] = latestEstimate.rotation().roll();
    transformTobeMapped[1] = latestEstimate.rotation().pitch();
    transformTobeMapped[2] = latestEstimate.rotation().yaw();
    transformTobeMapped[3] = latestEstimate.translation().x();
    transformTobeMapped[4] = latestEstimate.translation().y();
    transformTobeMapped[5] = latestEstimate.translation().z();

    // save all the received edge and surf points
    pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    pcl::copyPointCloud(*laserCloudCornerLastDS,  *thisCornerKeyFrame);
    pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);

    // save key frame cloud
    cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
    surfCloudKeyFrames.push_back(thisSurfKeyFrame);

    // save path for visualization
    updatePath(thisPose6D);
}
```

## correctPoses()

```cpp
void correctPoses()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    if (aLoopIsClosed == true)
    {
        // clear map cache
        laserCloudMapContainer.clear();
        // clear path
        globalPath.poses.clear();
        // update key poses
        int numPoses = isamCurrentEstimate.size();
        for (int i = 0; i < numPoses; ++i)
        {
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
            cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

            updatePath(cloudKeyPoses6D->points[i]);
        }

        aLoopIsClosed = false;
    }
}
```