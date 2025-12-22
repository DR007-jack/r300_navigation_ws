#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class IntelligentOdomBridge {
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster tf_broadcaster_;
    
    // 可配置参数
    std::string input_odom_topic_;
    std::string output_odom_topic_;
    std::string output_header_frame_;
    std::string output_child_frame_;

public:
    IntelligentOdomBridge() {
        // 从参数服务器读取配置（支持launch动态配置）
        nh_.param<std::string>("input_odom_topic", input_odom_topic_, "/pointlio/odom");
        nh_.param<std::string>("output_odom_topic", output_odom_topic_, "/odom");
        nh_.param<std::string>("output_header_frame", output_header_frame_, "odom");
        nh_.param<std::string>("output_child_frame", output_child_frame_, "base_link");
        
        // 初始化订阅与发布
        odom_sub_ = nh_.subscribe(input_odom_topic_, 10, &IntelligentOdomBridge::odomCb, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(output_odom_topic_, 10);
        
        ROS_INFO("Intelligent Odom Bridge initialized:");
        ROS_INFO("Input: %s", input_odom_topic_.c_str());
        ROS_INFO("Output: %s (frame: %s -> %s)", output_odom_topic_.c_str(), output_header_frame_.c_str(), output_child_frame_.c_str());
    }

    void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
        // 1. 复制原始消息，仅修正frame_id（保留原始时间戳、位姿、 covariance）
        nav_msgs::Odometry odom_out = *msg;
        odom_out.header.frame_id = output_header_frame_;
        odom_out.child_frame_id = output_child_frame_;

        // 2. 发布修正后的里程计
        odom_pub_.publish(odom_out);

        // 3. 广播TF（与里程计消息时间戳同步，避免插值错误）
        tf::Transform transform;
        tf::poseMsgToTF(odom_out.pose.pose, transform);
        tf_broadcaster_.sendTransform(
            tf::StampedTransform(transform, odom_out.header.stamp, 
            output_header_frame_, output_child_frame_)
        );
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "intelligent_odom_bridge_node");
    IntelligentOdomBridge bridge;
    ros::spin();
    return 0;
}

