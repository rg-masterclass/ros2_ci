#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

#include "tortoisebot_waypoints/action/waypoint_action.hpp"

#define DISTANCE_PRECISION 0.1
#define YAW_PRECISION M_PI / 90

using std::placeholders::_1;
using std::placeholders::_2;

class WaypointActionServer : public rclcpp::Node {
public:
  using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
  using GoalHandle = rclcpp_action::ServerGoalHandle<WaypointAction>;

  WaypointActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("tortoisebot_as", options) {
    action_server = rclcpp_action::create_server<WaypointAction>(
        this, "tortoisebot_as",
        std::bind(&WaypointActionServer::handle_goal, this, _1, _2),
        std::bind(&WaypointActionServer::handle_cancel, this, _1),
        std::bind(&WaypointActionServer::handle_accepted, this, _1));

    cmd_vel_publisher =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&WaypointActionServer::odom_callback, this, _1));

    RCLCPP_INFO(
        this->get_logger(),
        "The action server has been started and waiting for the requests...");
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position = msg->pose.pose.position;

    tf2::Quaternion qyaw;
    tf2::convert(msg->pose.pose.orientation, qyaw);
    current_yaw = tf2::getYaw(qyaw);
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const WaypointAction::Goal> goal) {
    (void)uuid;
    (void)goal;

    RCLCPP_INFO(this->get_logger(), "Goal: ACCEPT_AND_EXECUTE");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Goal cancel request.");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
    std::thread{std::bind(&WaypointActionServer::run, this, _1), goal_handle}
        .detach();
  }

  void run(const std::shared_ptr<GoalHandle> goal_handle) {

    const auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<WaypointAction::Feedback>();
    auto result = std::make_shared<WaypointAction::Result>();

    rclcpp::Rate loop_rate(25);

    auto des_pos = goal->position;
    auto des_yaw =
        atan2(des_pos.y - current_position.y, des_pos.x - current_position.x);

    float err_pos = 0.0;
    float err_yaw = 0.0;

    bool is_active = true;

    while (is_active) {

      err_pos = sqrt(pow(des_pos.y - current_position.y, 2) +
                     pow(des_pos.x - current_position.x, 2));

      err_yaw = des_yaw - current_yaw;

      //   RCLCPP_INFO(this->get_logger(), "Current Yaw: %f", current_yaw);
      //   RCLCPP_INFO(this->get_logger(), "Desired Yaw: %f", des_yaw);
      //   RCLCPP_INFO(this->get_logger(), "Error Yaw: %f", err_yaw);

      RCLCPP_INFO(this->get_logger(), "Desired Pos: %f/%f", des_pos.x,
                  des_pos.y);
      RCLCPP_INFO(this->get_logger(), "Current Pos: %f/%f", current_position.x,
                  current_position.y);
      RCLCPP_INFO(this->get_logger(), "Error Pos: %f", err_pos);

      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);

        feedback->state = "CANCELLED";
        goal_handle->publish_feedback(feedback);

        is_active = false;
        rclcpp::shutdown();
      }

      if (abs(err_yaw) > YAW_PRECISION) {
        cmd_vel_msg.angular.z = (err_yaw > 0) ? 0.65 : -0.65;
        cmd_vel_publisher->publish(cmd_vel_msg);
      } else {
        cmd_vel_msg.linear.x = 0.6;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_publisher->publish(cmd_vel_msg);
      }

      feedback->position = current_position;
      feedback->state = "GO_TO_POINT";

      goal_handle->publish_feedback(feedback);

      if (err_pos < DISTANCE_PRECISION) {
        is_active = false;

        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_publisher->publish(cmd_vel_msg);

        feedback->state = "SUCCESS";
        goal_handle->publish_feedback(feedback);

        result->success = true;
        goal_handle->succeed(result);
      }

      loop_rate.sleep();
    }
  }

private:
  geometry_msgs::msg::Twist cmd_vel_msg;
  geometry_msgs::msg::Point current_position;
  float current_yaw;

  rclcpp_action::Server<WaypointAction>::SharedPtr action_server;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto server = std::make_shared<WaypointActionServer>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(server);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}