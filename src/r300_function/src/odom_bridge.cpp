#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class OdomBridge {
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
    std::string target_frame_;    // 目标父帧（默认odom）
    std::string child_frame_;     // 目标子帧（默认base_link）

public:
    OdomBridge() {
        // 读取参数（可通过launch配置）
        nh_.param<std::string>("target_frame", target_frame_, "odom");
        nh_.param<std::string>("child_frame", child_frame_, "base_link");
        
        // 订阅激光里程计，发布修正后的里程计
        odom_sub_ = nh_.subscribe("/pointlio/odom", 10, &OdomBridge::odomCb, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);
    }

    void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
        // 1. 复制原始里程计并修改帧ID
        nav_msgs::Odometry odom_out = *msg;
        odom_out.header.frame_id = target_frame_;
        odom_out.child_frame_id = child_frame_;

        // 2. 发布修正后的里程计
        odom_pub_.publish(odom_out);

        // 3. 广播odom→base_link TF（从里程计提取位姿）
        tf::Transform transform;
        tf::poseMsgToTF(odom_out.pose.pose, transform);
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(transform, ros::Time::now(), 
            target_frame_, child_frame_)
        );
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_bridge_node");
    OdomBridge ob;
    ros::spin();
    return 0;
}
