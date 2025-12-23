#include <memory>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <std_srvs/Trigger.h>

// 引入ugv_sdk核心头文件（包含ProtocolVersion定义的头文件）
#include "ugv_sdk/details/interface/parser_interface.hpp"  // 直接引入ProtocolVersion所在头文件
#include "ugv_sdk/mobile_robot/scout_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"
#include "scout_base/scout_messenger.hpp"

using namespace westonrobot;

// 全局机器人对象指针
std::unique_ptr<ScoutRobot> robot;

static bool HandleClearFaults(std_srvs::Trigger::Request &req,
                              std_srvs::Trigger::Response &res) {
  (void)req;

  if (!robot) {
    res.success = false;
    res.message = "Robot not initialized";
    return true;
  }

  try {
    // Re-send commanded mode (idempotent) and then request state/fault reset.
    robot->EnableCommandedMode();
    robot->ResetRobotState();
    res.success = true;
    res.message = "Sent EnableCommandedMode and ResetRobotState";
  } catch (const std::exception &e) {
    res.success = false;
    res.message = std::string("Failed: ") + e.what();
  }

  return true;
}

int main(int argc, char **argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "scout_odom");
  ros::NodeHandle node(""), private_node("~");

  // 先读取端口参数，后续协议检测 & 连接都使用同一个端口，避免重复占用 CAN 资源
  std::string port_name;
  private_node.param<std::string>("port_name", port_name, std::string("can0"));

  // 读取机器人类型参数（scout mini/omni）
  bool is_scout_mini = false;
  static bool is_scout_omni = false;
  private_node.getParam("is_scout_mini", is_scout_mini);
  ROS_INFO("Working as scout mini: %d", is_scout_mini);
  private_node.getParam("is_scout_omni", is_scout_omni);
  ROS_INFO("Working as scout omni: %d", is_scout_omni);

  // 协议检测（使用正确拼写的ProtocolDectctor）
  // 注意：协议检测会占用一次 CAN 连接，因此必须确保 detector 在 robot->Connect() 前析构释放。
  try {
    // 这个 SDK 的枚举里使用了拼写 UNKONWN（不是 UNKNOWN），因此不要引用不存在的 UNKNOWN
    auto proto = ProtocolVersion::UNKONWN;
    {
      westonrobot::ProtocolDectctor detector;
      detector.Connect(port_name);
      proto = detector.DetectProtocolVersion(5);
    }  // detector 在此处析构，释放端口资源

    std::cout << "Detected protocol version: " << static_cast<int>(proto) << std::endl;

    // 若未检测到协议版本，继续构造/连接可能触发 SDK 内部空指针导致段错误。
    // 直接退出并提示检查 CAN 通讯与协议版本。
    if (proto == ProtocolVersion::UNKONWN) {
      ROS_ERROR("Failed to detect Scout protocol version on '%s' (got UNKONWN). "
                "Please verify CAN bitrate (e.g. 500000), wiring, and that frames like 0x151/0x221/0x241 are present in 'candump %s'.",
                port_name.c_str(), port_name.c_str());
      ros::shutdown();
      return -1;
    }

    // 根据机器人类型创建机器人对象（直接使用检测到的proto构造，避免显式引用ProtocolVersion::AGX_Vx）
    if (is_scout_mini && is_scout_omni) {
      // Scout Mini Omni机器人
      std::cout << "Creating Scout Mini Omni Robot object" << std::endl;
      robot = std::unique_ptr<ScoutMiniOmniRobot>(new ScoutMiniOmniRobot(proto));
    } else {
      // 普通Scout机器人
      std::cout << "Creating Scout Robot object (mini: " << is_scout_mini << ")" << std::endl;
      robot = std::unique_ptr<ScoutRobot>(new ScoutRobot(proto, is_scout_mini));
    }

    // 检查机器人对象是否创建成功
    if (robot == nullptr) {
      ROS_ERROR("Failed to create robot object");
      ros::shutdown();
      return -1;
    }
  } catch (const std::exception &error) {
    ROS_ERROR("Protocol detection failed on port '%s'. Please bring up CAN bus or make sure the port exists. Error: %s",
              port_name.c_str(), error.what());
    ros::shutdown();
    return -1;
  }

  // 创建ROS消息发布器对象
  ScoutROSMessenger messenger(robot.get(), &node, is_scout_omni);

  // 读取配置参数
  private_node.param<std::string>("odom_frame", messenger.odom_frame_, std::string("odom"));
  private_node.param<std::string>("base_frame", messenger.base_frame_, std::string("base_link"));
  private_node.param<bool>("simulated_robot", messenger.simulated_robot_, false);
  private_node.param<int>("control_rate", messenger.sim_control_rate_, 50);
  private_node.param<std::string>("odom_topic_name", messenger.odom_topic_name_, std::string("odom"));
  private_node.param<bool>("pub_tf", messenger.pub_tf, true);

  // 非仿真模式下连接机器人
  if (!messenger.simulated_robot_) {
    if (port_name.find("can") != std::string::npos) {
      // 避免重复 Connect 导致 SDK 内部资源/线程死锁：前面协议检测已释放端口，这里只做一次连接。
      try {
        robot->Connect(port_name);
        robot->EnableCommandedMode();
        ROS_INFO("Using CAN bus to talk with the robot on %s", port_name.c_str());
      } catch (const std::exception &e) {
        ROS_ERROR("Failed to connect/enable commanded mode on %s: %s", port_name.c_str(), e.what());
        ros::shutdown();
        return -1;
      }
    } else {
      ROS_INFO("Only CAN bus interface is supported for now");
    }
  }

  // Expose an explicit fault clear service for higher-level bringup scripts.
  // NOTE: This does not bypass hardware safety chain (E-Stop, driver lockout, etc.).
  ros::ServiceServer clear_faults_srv =
      node.advertiseService("/scout/clear_faults", HandleClearFaults);
  ROS_INFO("Service ready: /scout/clear_faults (std_srvs/Trigger)");

  // 设置ROS订阅器
  messenger.SetupSubscription();

  // 主循环：50Hz发布机器人状态
  ros::Rate rate(50);
  while (ros::ok()) {
    if (!messenger.simulated_robot_) {
      messenger.PublishStateToROS();
    } else {
      double linear, angular;
      messenger.GetCurrentMotionCmdForSim(linear, angular);
      messenger.PublishSimStateToROS(linear, angular);
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}


