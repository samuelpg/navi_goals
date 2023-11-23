#include <ros/console.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <string> 
#include <cstdlib>
#include <iomanip>

using namespace std;
using namespace nav_msgs;
using namespace geometry_msgs;
using namespace message_filters;

void path_callback(const PoseWithCovarianceStampedConstPtr& pose_amcl, const OdometryConstPtr& pose_ground)
{
//   ROS_INFO("Hello");
//    ROS_INFO("%i", pose_amcl->header.stamp.sec); 
//    ROS_INFO("%i", pose_amcl->header.stamp.nsec);
//    ROS_INFO("%i", pose_ground->header.stamp.sec); 
//    ROS_INFO("%i", pose_ground->header.stamp.nsec);
//ROS_INFO(std::to_string(pose_ground->header.stamp.sec), std::to_string(pose_ground->header.stamp.nsec));

    double x_diff = abs(pose_amcl->pose.pose.position.x - pose_ground->pose.pose.position.x);
    double y_diff = abs(pose_amcl->pose.pose.position.y - pose_ground->pose.pose.position.y);

    // ROS_INFO("====");
    // ROS_INFO("amcl: [%f, %f]", pose_amcl->pose.pose.position.x, pose_amcl->pose.pose.position.y);
    // ROS_INFO("gzbs: [%f, %f]", pose_ground->pose.pose.position.x, pose_ground->pose.pose.position.y);
    std::cout << "  - point: \n";
    std::cout << "     amcl: [" << pose_amcl->pose.pose.position.x << "," << pose_amcl->pose.pose.position.y << "] \n ";
    std::cout << "    gzbs: [" << pose_ground->pose.pose.position.x << "," << pose_ground->pose.pose.position.y << "] \n";
    std::cout << "     var_x: " << pose_amcl->pose.covariance[0] << "\n";
    std::cout << "     var_y: " << pose_amcl->pose.covariance[7] << "\n";
    std::cout << "     time: " << pose_amcl->header.stamp.sec + pose_amcl->header.stamp.nsec/1000000000.0 << "\n" ;
}

void print_amcl(const PoseWithCovarianceStampedConstPtr& pose_amcl)
{
    std::cout << std::fixed << std::setprecision(20);
    std::cout << "  - point: \n";
    std::cout << "     amcl: [" << pose_amcl->pose.pose.position.x << "," << pose_amcl->pose.pose.position.y << "] \n ";
    std::cout << "    var_x: " << pose_amcl->pose.covariance[0] << "\n";
    std::cout << "     var_y: " << pose_amcl->pose.covariance[7] << "\n";
    std::cout << "     time: " << pose_amcl->header.stamp.sec + pose_amcl->header.stamp.nsec/1000000000.0 << "\n" ;
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "path_comparison_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<PoseWithCovarianceStamped> amcl_sub(nh, "amcl_pose", 1);
    message_filters::Subscriber<Odometry> ground_sub(nh, "ground_truth_pose", 1);
   
    typedef sync_policies::ApproximateTime<PoseWithCovarianceStamped, Odometry> MySyncPolicy;
   
    //TimeSynchronizer<PoseWithCovarianceStamped, Odometry> sync(amcl_sub, ground_sub, 10);
    std::cout << "data: \n";
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), amcl_sub, ground_sub);
    sync.registerCallback(boost::bind(&path_callback, _1, _2));

    // ros::Subscriber sub = nh.subscribe("/amcl_pose", 1000, print_amcl);
    
    ros::spin();
    return 0;
}