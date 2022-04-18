#ifndef ROS_INIT_H
#define ROS_INIT_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
namespace ros_init_treatment
{
    class rosInitManager
    {
        public:
        ros::NodeHandle *nh;
        ros::Publisher ground_posearray_publisher;
        ros::Publisher ground_path_publisher;
        rosInitManager()
        {
            int argc = 0;
            char **argv = NULL;
            std::string node_name = "cvte_rviz_panel";
            ros::init(argc, argv, node_name);
            nh = new ros::NodeHandle();

            ground_posearray_publisher = nh->advertise<geometry_msgs::PoseArray>("/ground_pose_array", 10);
            ground_path_publisher = nh->advertise<nav_msgs::Path>("/ground_path", 10);
        }

        void ground_posearry_pub(geometry_msgs::PoseArray msg_path)
        {
            ground_posearray_publisher.publish(msg_path);
        }

        void ground_path_pub(nav_msgs::Path path)
        {
            ground_path_publisher.publish(path);
        }
    };
}

#endif // TEP