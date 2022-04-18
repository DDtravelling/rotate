#ifndef ROS_INIT_H
#define ROS_INIT_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

namespace ros_init_treatment
{
    class rosInitManager
    {
        public:
        ros::NodeHandle *nh;
        ros::Publisher ground_posearray_publisher;
        rosInitManager()
        {
            int argc = 0;
            char **argv = NULL;
            std::string node_name = "cvte_rviz_panel";
            ros::init(argc, argv, node_name);
            nh = new ros::NodeHandle();

            ground_posearray_publisher = nh->advertise<geometry_msgs::PoseArray>("/ground_pose_array", 10);
        }

        void group_posearry_pub(geometry_msgs::PoseArray msg_path)
        {
            ground_posearray_publisher.publish(msg_path);
        }
    };
}

#endif // TEP