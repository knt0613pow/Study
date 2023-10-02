
# imuPreintegration

##  main function
```cpp
int main(int argc, char** argv)
{
    ros::init(argc, argv, "roboat_loam");
    
    IMUPreintegration ImuP;

    TransformFusion TF;

    ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");
    
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    
    return 0;
}
```

imuPreintegration.cpp에서는 TODO

## member variable in IMUPreintegration
```cpp
class IMUPreintegration : public ParamServer
{
public:

    std::mutex mtx;

    ros::Subscriber subImu;
    ros::Subscriber subOdometry;
    ros::Publisher pubImuOdometry;

    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;


    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    std::deque<sensor_msgs::Imu> imuQueOpt;
    std::deque<sensor_msgs::Imu> imuQueImu;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;

    int key = 1;
    
    // T_bl: tramsform points from lidar frame to imu frame 
    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    // T_lb: tramsform points from imu frame to lidar frame
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));
```


## publisher and subscriber in IMUPreintegration
imuPreintegration.cpp - IMUPreintegration class 에서 사용하는 subscriber, publisher 입니다.


|변수이름|sub/pub|topic type|topic name|callback|usage|
|------|---|---|---|---|---|
|subImu|sub|sensor_msgs::Imu|imuTopic|imuHandler()|-|
|subImu|sub|nav_msgs::Odometry|lio_sam/mapping/odometry_incremental|odometryHandler()|-|
|pubImuOdometry|pub|nav_msgs::Odometry|odomTopic+"_incremental"|-|-|



```cpp
IMUPreintegration()
{
subImu = nh.subscribe<sensor_msgs::Imu>  (imuTopic,2000, &IMUPreintegration::imuHandler,this, ros::TransportHints().tcpNoDelay());
subOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5,&IMUPreintegration::odometryHandler, this, ros::TransportHints().tcpNoDelay());
pubImuOdometry = nh.advertise<nav_msgs::Odometry> (odomTopic+"_incremental", 2000);

```


## member variable in TransformFusion
```cpp
class TransformFusion : public ParamServer
{
public:
    std::mutex mtx;

    ros::Subscriber subImuOdometry;
    ros::Subscriber subLaserOdometry;

    ros::Publisher pubImuOdometry;
    ros::Publisher pubImuPath;

    Eigen::Affine3f lidarOdomAffine;
    Eigen::Affine3f imuOdomAffineFront;
    Eigen::Affine3f imuOdomAffineBack;

    tf::TransformListener tfListener;
    tf::StampedTransform lidar2Baselink;

    double lidarOdomTime = -1;
    deque<nav_msgs::Odometry> imuOdomQueue;

```

## publisher and subscriber in TransformFusion
imuPreintegration.cpp - TransformFusion class 에서 사용하는 subscriber, publisher 입니다.


|변수이름|sub/pub|topic type|topic name|callback|usage|
|------|---|---|---|---|---|
|subLaserOdometry|sub|nav_msgs::Odometry|lio_sam/mapping/odometry|lidarOdometryHandler()|-|
|subImuOdometry|sub|nav_msgs::Odometry|odomTopic+"_incremental"|imuOdometryHandler()|-|
|pubImuOdometry|pub|nav_msgs::Odometry|odomTopic|-|-|
|pubImuOdometry|pub|nav_msgs::Path|lio_sam/imu/path"|-|-|



```cpp
TransformFusion()
{
    //생략
subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry", 5, &TransformFusion::lidarOdometryHandler, this, ros::TransportHints().tcpNoDelay());
subImuOdometry   = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental",   2000, &TransformFusion::imuOdometryHandler,   this, ros::TransportHints().tcpNoDelay());

pubImuOdometry   = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
pubImuPath       = nh.advertise<nav_msgs::Path>    ("lio_sam/imu/path", 1);
}

```
