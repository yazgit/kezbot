#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

//TODO implement geometry_msgs instead of turtlesim/Pose
#include <geometry_msgs/Pose.h>


std::string robot_name;

void poseCallback(const turtlesim::PoseConstPtr& msg){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    transform.setRotation(q);

    br.sendTransform(
        tf::StampedTransform(
            transform,
            ros::Time::now(),
            "base_link",
            robot_name+"/base_link"
        )
    );

}

int main(int argc, char **argv){
    ros::init(argc, argv, "my_tf_broadcaster");
    if (argc != 2){
        ROS_ERROR("need robot name as argument");
        return -1;
    };
    robot_name = argv[1];

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe(robot_name+"/pose", 10, &poseCallback);

    ros::spin();
    return 0;


}
