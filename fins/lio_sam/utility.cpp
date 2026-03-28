#include "utility.h"

using namespace std;

//Topics
string gpsTopic;

// GPS Settings
bool useImuHeadingInitialization;
bool useGpsElevation;
float gpsCovThreshold;
float poseCovThreshold;

// CPU Params
int numberOfCores;
double mappingProcessInterval;

// Surrounding map
float surroundingkeyframeAddingDistThreshold; 
float surroundingkeyframeAddingAngleThreshold; 
float surroundingKeyframeDensity;
float surroundingKeyframeSearchRadius;

// Loop closure
bool  loopClosureEnableFlag;
float loopClosureFrequency;
int   surroundingKeyframeSize;
float historyKeyframeSearchRadius;
float historyKeyframeSearchTimeDiff;
int   historyKeyframeSearchNum;
float historyKeyframeFitnessScore;

// global map visualization radius
float globalMapVisualizationSearchRadius;
float globalMapVisualizationPoseDensity;
float globalMapVisualizationLeafSize;

float mappingICPSize;

int ikdtreeSearchNeighborNum;

#ifdef USE_ROS1

void read_liosam_params(ros::NodeHandle& nh) {
    // LIO-SAM params

    // Topics
    nh.param<std::string>("lio_sam/gpsTopic", gpsTopic, "odometry/gps");

    // GPS settings
    nh.param<bool>("lio_sam/useImuHeadingInitialization", useImuHeadingInitialization, false);
    nh.param<bool>("lio_sam/useGpsElevation", useGpsElevation, false);
    nh.param<float>("lio_sam/gpsCovThreshold", gpsCovThreshold, 2.0);
    nh.param<float>("lio_sam/poseCovThreshold", poseCovThreshold, 25.0);

    // CPU parameters
    nh.param<int>("lio_sam/numberOfCores", numberOfCores, 2);
    nh.param<double>("lio_sam/mappingProcessInterval", mappingProcessInterval, 0.15);

    // Keyframe Strategy
    nh.param<float>("lio_sam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
    nh.param<float>("lio_sam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
    nh.param<float>("lio_sam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
    nh.param<float>("lio_sam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

    // Loop closeure parameters
    nh.param<bool>("lio_sam/loopClosureEnableFlag", loopClosureEnableFlag, false);
    nh.param<float>("lio_sam/loopClosureFrequency", loopClosureFrequency, 1.0);
    nh.param<int>("lio_sam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
    nh.param<float>("lio_sam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
    nh.param<float>("lio_sam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
    nh.param<int>("lio_sam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
    nh.param<float>("lio_sam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

    // Global pointcloud visualization
    nh.param<float>("lio_sam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
    nh.param<float>("lio_sam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
    nh.param<float>("lio_sam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

    nh.param<float>("lio_sam/mappingICPSize", mappingICPSize, 0.2);
    nh.param<int>("lio_sam/ikdtreeSearchNeighborNum", ikdtreeSearchNeighborNum, 8);
}

#elif defined(USE_ROS2)

void read_liosam_params(const rclcpp::Node::SharedPtr& node) {
    gpsTopic = node->declare_parameter<std::string>("lio_sam.gpsTopic", "odometry/gps");

    useImuHeadingInitialization =
        node->declare_parameter<bool>("lio_sam.useImuHeadingInitialization", false);
    useGpsElevation = node->declare_parameter<bool>("lio_sam.useGpsElevation", false);
    gpsCovThreshold = node->declare_parameter<double>("lio_sam.gpsCovThreshold", 2.0);
    poseCovThreshold = node->declare_parameter<double>("lio_sam.poseCovThreshold", 25.0);

    numberOfCores = node->declare_parameter<int>("lio_sam.numberOfCores", 2);
    mappingProcessInterval =
        node->declare_parameter<double>("lio_sam.mappingProcessInterval", 0.15);

    surroundingkeyframeAddingDistThreshold =
        node->declare_parameter<double>("lio_sam.surroundingkeyframeAddingDistThreshold", 1.0);
    surroundingkeyframeAddingAngleThreshold =
        node->declare_parameter<double>("lio_sam.surroundingkeyframeAddingAngleThreshold", 0.2);
    surroundingKeyframeDensity =
        node->declare_parameter<double>("lio_sam.surroundingKeyframeDensity", 1.0);
    surroundingKeyframeSearchRadius =
        node->declare_parameter<double>("lio_sam.surroundingKeyframeSearchRadius", 50.0);

    loopClosureEnableFlag = node->declare_parameter<bool>("lio_sam.loopClosureEnableFlag", false);
    loopClosureFrequency = node->declare_parameter<double>("lio_sam.loopClosureFrequency", 1.0);
    surroundingKeyframeSize = node->declare_parameter<int>("lio_sam.surroundingKeyframeSize", 50);
    historyKeyframeSearchRadius =
        node->declare_parameter<double>("lio_sam.historyKeyframeSearchRadius", 10.0);
    historyKeyframeSearchTimeDiff =
        node->declare_parameter<double>("lio_sam.historyKeyframeSearchTimeDiff", 30.0);
    historyKeyframeSearchNum = node->declare_parameter<int>("lio_sam.historyKeyframeSearchNum", 25);
    historyKeyframeFitnessScore =
        node->declare_parameter<double>("lio_sam.historyKeyframeFitnessScore", 0.3);

    globalMapVisualizationSearchRadius =
        node->declare_parameter<double>("lio_sam.globalMapVisualizationSearchRadius", 1e3);
    globalMapVisualizationPoseDensity =
        node->declare_parameter<double>("lio_sam.globalMapVisualizationPoseDensity", 10.0);
    globalMapVisualizationLeafSize =
        node->declare_parameter<double>("lio_sam.globalMapVisualizationLeafSize", 1.0);

    mappingICPSize = node->declare_parameter<double>("lio_sam.mappingICPSize", 0.2);
    ikdtreeSearchNeighborNum = node->declare_parameter<int>("lio_sam.ikdtreeSearchNeighborNum", 8);
}



#endif