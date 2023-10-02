

## imageProjection

###  main function
```cpp
int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    ImageProjection IP;
    
    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    
    return 0;
}
```

imageProjection.cpp 내부에서는 라이다로부터 취득한 포인트 클라우드 토픽을 처리한다. 하지만 raw 포인트 클라우드 토픽을 사용할 경우, 왜곡이 발생하기 때문에 imu나 odometry정보를 이용한다. 왜곡이 발생하는 원인과, 왜곡을 해결하는 방법에 대해서는 후술할 예정이다.

### member variable
```cpp

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock; // imu topic 접근을 제한하는 mutex
    std::mutex odoLock; //odometry topic 접근을 제한하는 mutex

    ros::Subscriber subLaserCloud;
    ros::Publisher  pubLaserCloud;
    
    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn; 
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud; // deskew 를 보정한 뒤의 point cloud

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    lio_sam::cloud_info cloudInfo; // 플래그 및 중간변수를 저장하는데 사용되는 data struct
    double timeScanCur; // lidar frame의 스캔이 시작될 때의 time stamp
    double timeScanEnd; // lidar frame의 스캔이 종료될 때의 time stamp
    std_msgs::Header cloudHeader; //lidar frame의 time stamp을 저장하는 변수

    vector<int> columnIdnCountVec;
```


### publisher and subscriber
imageProjection.cpp에서 사용하는 subscriber, publisher 입니다.


|변수이름|sub/pub|topic type|topic name|callback|usage|
|------|---|---|---|---|---|
|subImu|sub|sensor_msgs::Imu|imuTopic|imuHandler()|-|
|subOdom|sub|nav_msgs::Odometry|odomTopic+"_incremental"|odometryHandler()|-|
|subLaserCloud|sub|sensor_msgs::PointCloud2|pointCloudTopic|cloudHandler()|-|
|pubExtractedCloud|pub|sensor_msgs::PointCloud2|lio_sam/deskew/cloud_deskewed|-|visualize in rviz|
|pubLaserCloudInfo|pub|lio_sam::cloud_info>|lio_sam/deskew/cloud_info|-|used in featureExtraction.cpp|

뒤에 소개 되겠지만, cloud_info 의 message instance 인 cloudInfo.cloud_deskewed을 pubExtractedCloud에서 publish하고, cloud_info 을 pubLaserCloudInfo에서 publish한다. 

```cpp

subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
subOdom       = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());

pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("lio_sam/deskew/cloud_deskewed", 1);
pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/deskew/cloud_info", 1);

```

### imuHandler(), odometryHandler()
imu와 odometry 정보는 lidar 왜곡을 보정하기 위해 사용되기 때문에, lidar callback이 작동하기 전까지 Queue에 데이터만 저장하는 역할을 합니다. lidar callback이 들어올 경우, Queue에 저장된 data를 통합해서 왜곡을 보정합니다.

```cpp
void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
{
    sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

    std::lock_guard<std::mutex> lock1(imuLock);
    imuQueue.push_back(thisImu);
}
```
imuConverter는 utility.h에 정의된 함수로, imu 데이터를 lidar 좌표계와 동일한 axis를 가지도록 변환합니다. (Ext Rotation만 고려하고, Ext translation은 고려 x)

```cpp
    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
    {
        sensor_msgs::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }
};
```

odmetryHandler()

```cpp
void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
{
    std::lock_guard<std::mutex> lock2(odoLock);
    odomQueue.push_back(*odometryMsg);
}
```

### cloudHandler()

cloudHanler 콜백함수에서는 아래와 같이 raw cloudpoint data를 처리합니다.

- cachePointCloud(laserCloudMsg)
- deskewInfo()
- projectPointCloud()
- cloudExtraction()
- publishClouds()
- resetParameters()


```cpp
void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    if (!cachePointCloud(laserCloudMsg))
        return;

    if (!deskewInfo())
        return;

    projectPointCloud();

    cloudExtraction();

    publishClouds();

    resetParameters();
}

```

### cachePointCloud()

```cpp
bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
    // 1. cache point cloud
    //    (laserCloudMsg 를 cloudQueue에 push back)
    // cache point cloud


    // 2. convert cloud
    //    cloudQueue의 front element를 laserCloudIn() 변수 (type : pcl::Pointcloud<PointXYZIRT>)에 할당
    //    velodyne이나 livox면 변환없이 할당
    //    ouster의 경우 preprocessing 을 통해 velodyne format으로 통일

    // 3. get timestamp
    //    cloudPoint time stamp할당

    // 4. check dense flag
    //     is_dense flag 가 false라는 것은 포인트 클라우드 데이터에 inf / NaN 값을 갖는 포인트가 있다는 것을 의미
    //     
    
    // 5. check ring channel
    //    PointXYZIRT 으로 변환된 pointCloud data의 ring flag (XYZI"R"T)가 유효한지 체크

    // 6. check point time
    //    pointXYZIRT 으로 변환된 pointCloud data의 ring flag (XYZIR"T")가 유효한지 체크
    //    deskewFlag는 imageProejction Class가 초기화 될 때 0으로 할당되었음

    return true;
}
```

```cpp
// 1. cache point cloud
//    (laserCloudMsg 를 cloudQueue에 push back)
// cache point cloud
cloudQueue.push_back(*laserCloudMsg);
if (cloudQueue.size() <= 2)
    return false;
```

```cpp
// 2. convert cloud
//    cloudQueue의 front element를 laserCloudIn() 변수 (type : pcl::Pointcloud<PointXYZIRT>)에 할당
//    velodyne이나 livox면 변환없이 할당
//    ouster의 경우 preprocessing 을 통해 velodyne format으로 통일
currentCloudMsg = std::move(cloudQueue.front());
cloudQueue.pop_front();
if (sensor == SensorType::VELODYNE || sensor == SensorType::LIVOX)
{
    pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
}
else if (sensor == SensorType::OUSTER)
{
    // Convert to Velodyne format
    pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
    laserCloudIn->points.resize(tmpOusterCloudIn->size());
    laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
    for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
    {
        auto &src = tmpOusterCloudIn->points[i];
        auto &dst = laserCloudIn->points[i];
        dst.x = src.x;
        dst.y = src.y;
        dst.z = src.z;
        dst.intensity = src.intensity;
        dst.ring = src.ring;
        dst.time = src.t * 1e-9f;
    }
}
else
{
    ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
    ros::shutdown();
}
```

```cpp
// 3. get timestamp
//    cloudPoint time stamp할당
cloudHeader = currentCloudMsg.header;
timeScanCur = cloudHeader.stamp.toSec();
timeScanEnd = timeScanCur + laserCloudIn->points.back().time;
```

```cpp
// 4. check dense flag
//     is_dense flag 가 false라는 것은 포인트 클라우드 데이터에 inf / NaN 값을 갖는 포인트가 있다는 것을 의미
//     
if (laserCloudIn->is_dense == false)
{
    ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
    ros::shutdown();
}
```

```cpp
// 5. check ring channel
//    PointXYZIRT 으로 변환된 pointCloud data의 ring flag (XYZI"R"T)가 유효한지 체크
static int ringFlag = 0;
if (ringFlag == 0)
{
    ringFlag = -1;
    for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
    {
        if (currentCloudMsg.fields[i].name == "ring")
        {
            ringFlag = 1;
            break;
        }
    }
    if (ringFlag == -1)
    {
        ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
        ros::shutdown();
    }
}
```

```cpp
    // 6. check point time
    //    pointXYZIRT 으로 변환된 pointCloud data의 ring flag (XYZIR"T")가 유효한지 체크
    //    deskewFlag는 imageProejction Class가 초기화 될 때 0으로 할당되었음
    if (deskewFlag == 0)
    {
        deskewFlag = -1;
        for (auto &field : currentCloudMsg.fields)
        {
            if (field.name == "time" || field.name == "t")
            {
                deskewFlag = 1;
                break;
            }
        }
        if (deskewFlag == -1)
            ROS_WARN("Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
    }
```


### deskewInfo()
deskewInfo() 에서는 pointcloud 왜곡 보정에 필요한 imu및 odometry 데이터 전처리를 수행합니다.(라이다 프레임 스캔 시작과 종료 타임 스탬프간의 imu relative pose를 계산)

- imuQueue내의 topic이 유효한지 판단
- imuDeskewInfo() : 'lidar scan 시작할 때의 pose' 와 '종료될 때의 pose'의 relative pose를 구함
```cpp
bool deskewInfo()
{
    std::lock_guard<std::mutex> lock1(imuLock);
    std::lock_guard<std::mutex> lock2(odoLock);

    // make sure IMU data available for the scan
    // imuQueue 내에 relative pose 구할 때 사용할수 있는 topic이 있는지 판단
    // 이 분기문에서는 imuQueue의 토픽들이 lidar frame보다 더 넓은 time range를 커버할 것을 요구하고 있음 
    if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd)
    {
        ROS_DEBUG("Waiting for IMU data ...");
        return false;
    }

    imuDeskewInfo();

    odomDeskewInfo();

    return true;
}
```

imuDeskewInfo() 함수에서는 유효한 imu data 에 대해서만 angular velocity를 적분합니다. (lidar frame 시작시간과 종료시간에 존재하는 imu data에 대해)  
적분한 값들은 imuRotX[], imuRotY[], imuRotZ[], imuTime[]에 저장이 되는데, idx 0 값의 경우 roll pitch yaw 값이 0으로 할당이 되며, 나머지 element들은 idx 0 pose에 대한 relative pose 값으로 할당이 됩니다.

```cpp
void imuDeskewInfo()
{
    // imuAvailable flag 초기화 
    // imuDeskewInfo() 함수가 정상적으로 작동했다면 True
    cloudInfo.imuAvailable = false; 

    // lidar frame 시작시간보다 빨리 취득된 imu topic 제거
    while (!imuQueue.empty())
    {
        if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
            imuQueue.pop_front();
        else
            break;
    }

    //queue 가 비어있으면 종료
    if (imuQueue.empty())
        return;

    
    imuPointerCur = 0;
    for (int i = 0; i < (int)imuQueue.size(); ++i)
    {
        sensor_msgs::Imu thisImuMsg = imuQueue[i];
        double currentImuTime = thisImuMsg.header.stamp.toSec();

        
        // get roll, pitch, and yaw estimation for this scan
        // cloudInfo 토픽에 imu rotatino init값 할당
        if (currentImuTime <= timeScanCur)
            imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

        if (currentImuTime > timeScanEnd + 0.01)
            break;

        // idx 0 element의 orientation으로 0으로 초기화
        if (imuPointerCur == 0){
            imuRotX[0] = 0;
            imuRotY[0] = 0;
            imuRotZ[0] = 0;
            imuTime[0] = currentImuTime;
            ++imuPointerCur;
            continue;
        }

        // get angular velocity
        double angular_x, angular_y, angular_z;
        imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

        // integrate rotation
        // idx 0 element에 대한 relative orientation으로 할당
        double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
        imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
        imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
        imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
        imuTime[imuPointerCur] = currentImuTime;
        ++imuPointerCur;
    }

    --imuPointerCur;

    if (imuPointerCur <= 0)
        return;

    cloudInfo.imuAvailable = true;
}
```

odomDeskewInfo()

TODO
```cpp
void odomDeskewInfo()
{
    cloudInfo.odomAvailable = false;

    while (!odomQueue.empty())
    {
        if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
            odomQueue.pop_front();
        else
            break;
    }

    if (odomQueue.empty())
        return;

    if (odomQueue.front().header.stamp.toSec() > timeScanCur)
        return;

    // get start odometry at the beinning of the scan
    nav_msgs::Odometry startOdomMsg;

    for (int i = 0; i < (int)odomQueue.size(); ++i)
    {
        startOdomMsg = odomQueue[i];

        if (ROS_TIME(&startOdomMsg) < timeScanCur)
            continue;
        else
            break;
    }

    tf::Quaternion orientation;
    tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

    double roll, pitch, yaw;
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    // Initial guess used in mapOptimization
    cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
    cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
    cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
    cloudInfo.initialGuessRoll  = roll;
    cloudInfo.initialGuessPitch = pitch;
    cloudInfo.initialGuessYaw   = yaw;

    cloudInfo.odomAvailable = true;

    // get end odometry at the end of the scan
    odomDeskewFlag = false;

    if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
        return;

    nav_msgs::Odometry endOdomMsg;

    for (int i = 0; i < (int)odomQueue.size(); ++i)
    {
        endOdomMsg = odomQueue[i];

        if (ROS_TIME(&endOdomMsg) < timeScanEnd)
            continue;
        else
            break;
    }

    if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
        return;

    Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

    tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

    Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

    float rollIncre, pitchIncre, yawIncre;
    pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

    odomDeskewFlag = true;
}
```

### projectPointCloud()



```cpp
void projectPointCloud()
{
    int cloudSize = laserCloudIn->points.size();
    // range image projection
    for (int i = 0; i < cloudSize; ++i)
    {
        // 1. define (PointType thisPoint) (PointType is XYZI)
        // 2. 포인트가 유효한지 체크
            // lidarMinRange보다 작거나, lidarMaxRange보다 크면 무시
            // ring channel 이 유효하지 않으면 무시 (N_SCAN기준 = lidar channel 수 기준)
            // 만약 downsample 된 라이다 데이터를 사용하고 있었다면, downsampleRate의 정수배인지 체크
        // 3. set columnIdn
            // VELODYNE of OUSTER -> 3d lidar 
                // thisPoint.x , thisPoint.y 를 기준으로 columnIdn을 할당
            // LIVOX -> solid lidar
                //  point array 순서대로 할당
            // * N_SCAN은 z방향 채널수를 의미 (ex vlp 16 -> N_SCAN = 16)
            // * HORIZION_SCAN은 xy방향 채널수를 의미 (ex default HORIZON_SCAN = 1800)
        // 4. columnIdn 이나 rowIdn이 유효하지 않으면 무시
            // columnIdn < 0 || columnIdn >= Horizon_SCAN 일때 무시
            // rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX
        // 5. deskew thisPoint
        // 6. rangeMat 에 range할당
            // range : 원점-thisPoint사이의 거리
        // 7. fullCloud->points 에 point 할당
            //

    }
}
```
3. set columnIdn
```cpp
int columnIdn = -1;
if (sensor == SensorType::VELODYNE || sensor == SensorType::OUSTER)
{
    float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
    static float ang_res_x = 360.0/float(Horizon_SCAN);
    columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
    if (columnIdn >= Horizon_SCAN)
        columnIdn -= Horizon_SCAN;
}
else if (sensor == SensorType::LIVOX)
{
    columnIdn = columnIdnCountVec[rowIdn];
    columnIdnCountVec[rowIdn] += 1;
}
```

5. deskew thisPoint
```cpp
thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);
```

7. fullCloud->points 에 point 할당
```cpp
int index = columnIdn + rowIdn * Horizon_SCAN;
fullCloud->points[index] = thisPoint;
```

### deskewPoint()
deskewPoint()
```cpp
PointType deskewPoint(PointType *point, double relTime)
{
    // 1.deskewFlag 가 유효하지 않거나, imuDeskewInfo() 가 정상적으로 동작하지 않았으면, *point return
    // 2. time을 기준으로 rotation, position interpolation
        // findRotation() , findPosition()
    // 3-1. first Point일 경우, transStartInverse 할당
    // 3-2. 다른 point의 경우, fist Point 기준으로 relative Transformation Matrix 형성
    // 4. relative Transformation Matrix을 기준으로 point 보정
    return newpoint;
}
```

2. rotation, position interpolation
```cpp
float rotXCur, rotYCur, rotZCur;
findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);
float posXCur, posYCur, posZCur;
findPosition(relTime, &posXCur, &posYCur, &posZCur);
```

3-2 relative Transformation Matrix 
```cpp
Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
Eigen::Affine3f transBt = transStartInverse * transFinal; // transStartInverse : firspoint Transfomation Matrix inverse
```

4. relative Transformation Matrix을 기준으로 point 보정
```cpp
PointType newPoint;
newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
newPoint.intensity = point->intensity;
```

### findRotation() and findPosition()
findRotation()

imuDeskewInfo() 에서 할당한 imuTime[], imuRotX[], imuRotY[], imuRotZ[]를 사용해서, pointTime포즈를 interpolation하는 코드
```cpp
void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
{
    *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;
    // 생략
    if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
    {
        // 생략
    } 
    else {
        // imuTime[imuPointerFront - 1] < pointTime < imuTime[imuPointerFront] 을 만족하는 imuPointerFront 을 찾음
        int imuPointerBack = imuPointerFront - 1;
        double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
        double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
        *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
        *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
        *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
    }
}
```
findPosition()  
odometry topic을 사용해서 position interpolation을 하지만, 기본코드에서는 0으로 세팅되어있음
```cpp
void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
{
    *posXCur = 0; *posYCur = 0; *posZCur = 0;

    // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

    // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
    //     return;

    // float ratio = relTime / (timeScanEnd - timeScanCur);

    // *posXCur = ratio * odomIncreX;
    // *posYCur = ratio * odomIncreY;
    // *posZCur = ratio * odomIncreZ;
}
```

### cloudExtraction()

보정을 하면서 유효하지 않은 point cloud들을 필터링을 했었는데, 이 함수에서는 필터링된 pointcloud만 포함한 sensor_msg::Pointcloud2 객체 extractedCloud 를 만든다. 필터링이 된 포인트는 rangeMat 값이 FLT_MAX가 아니므로, FLT_MAX가 아닌 점들만 포함시키면 된다. 
```cpp
void cloudExtraction()
{
    int count = 0;
    // extract segmented cloud for lidar odometry
    for (int i = 0; i < N_SCAN; ++i)
    {
        cloudInfo.startRingIndex[i] = count - 1 + 5;

        for (int j = 0; j < Horizon_SCAN; ++j)
        {
            if (rangeMat.at<float>(i,j) != FLT_MAX)
            {
                // mark the points' column index for marking occlusion later
                cloudInfo.pointColInd[count] = j;
                // save range info
                cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                // save extracted cloud
                extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                // size of extracted cloud
                ++count;
            }
        }
        cloudInfo.endRingIndex[i] = count -1 - 5;
    }
}
```

### publishClouds()
필터링된 포인트클라우드 extractedCloud를 cloudInfo에 넣어서 publish한다.
```cpp
void publishClouds()
{
    cloudInfo.header = cloudHeader;
    cloudInfo.cloud_deskewed  = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
    pubLaserCloudInfo.publish(cloudInfo);
}
```