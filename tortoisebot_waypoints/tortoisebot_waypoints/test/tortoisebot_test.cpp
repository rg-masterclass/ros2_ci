#include "rclcpp/rclcpp.hpp"
#include "gtest/gtest.h"
#include <memory>

#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

#include "tortoisebot_waypoints/action/waypoint_action.hpp"

#define DISTANCE_TEST_PRECISION 0.1
#define YAW_TEST_PRECISION 5.0

using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

class RclCppFixture {
public:
  RclCppFixture() { rclcpp::init(0, nullptr); }
  ~RclCppFixture() { rclcpp::shutdown(); }
};
RclCppFixture g_rclcppfixture;

class WaypointActionServerTestFixture : public ::testing::Test {
public:
  using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
  using GoalHandle = rclcpp_action::ClientGoalHandle<WaypointAction>;

  WaypointActionServerTestFixture() {

    node = rclcpp::Node::make_shared("waypoint_action_server_test_node");

    odom_subscriber = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&WaypointActionServerTestFixture::odom_callback, this, _1));

    action_client =
        rclcpp_action::create_client<WaypointAction>(node, "tortoisebot_as");

    this->goal.x = 1.0;
    this->goal.y = 1.0;

    timer = node->create_wall_timer(
        100ms,
        std::bind(&WaypointActionServerTestFixture::timer_callback, this));
  }

  bool PositionTest() {
    while (is_waiting_for_results) {
      rclcpp::spin_some(node);
    }

    float x_error = abs(goal.x - current_position.x);
    float y_error = abs(goal.y - current_position.y);

    if (x_error <= DISTANCE_TEST_PRECISION &&
        y_error <= DISTANCE_TEST_PRECISION) {
      return true;
    }

    return false;
  }
  bool OrientationTest() {
    while (is_waiting_for_results) {
      rclcpp::spin_some(node);
    }

    float goal_yaw =
        atan2(goal.y - initial_position.y, goal.x - initial_position.x);

    float yaw_error = abs(goal_yaw - current_yaw);

    if (yaw_error <= YAW_TEST_PRECISION) {
      return true;
    }

    return false;
  }

private:
  std::shared_ptr<rclcpp::Node> node;

  void
  goal_response_callback(std::shared_future<GoalHandle::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandle::SharedPtr,
      const std::shared_ptr<const WaypointAction::Feedback> feedback) {}

  void result_callback(const GoalHandle::WrappedResult &result) {
    is_waiting_for_results = false;
  }

  void timer_callback() {
    this->timer->cancel();

    if (!action_client->wait_for_action_server(10s)) {
      RCLCPP_ERROR(node->get_logger(), "Action server not available");
    }

    auto goal_msg = WaypointAction::Goal();
    goal_msg.position = this->goal;

    auto goal_options =
        rclcpp_action::Client<WaypointAction>::SendGoalOptions();

    goal_options.goal_response_callback = std::bind(
        &WaypointActionServerTestFixture::goal_response_callback, this, _1);

    goal_options.feedback_callback = std::bind(
        &WaypointActionServerTestFixture::feedback_callback, this, _1, _2);

    goal_options.result_callback =
        std::bind(&WaypointActionServerTestFixture::result_callback, this, _1);

    auto future = action_client->async_send_goal(goal_msg, goal_options);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!is_initialized) {
      initial_position = msg->pose.pose.position;
      is_initialized = true;
    }

    current_position = msg->pose.pose.position;

    tf2::Quaternion current_orientation;
    tf2::convert(msg->pose.pose.orientation, current_orientation);
    current_yaw = tf2::getYaw(current_orientation);
  }

  bool is_initialized = false;
  bool is_waiting_for_results = true;

  geometry_msgs::msg::Point goal;

  geometry_msgs::msg::Point initial_position;
  geometry_msgs::msg::Point current_position;

  float current_yaw;

  rclcpp_action::Client<WaypointAction>::SharedPtr action_client;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;

  rclcpp::TimerBase::SharedPtr timer;
};

TEST_F(WaypointActionServerTestFixture, Position) {
  EXPECT_TRUE(PositionTest());
}

TEST_F(WaypointActionServerTestFixture, Orientation) {
  EXPECT_TRUE(OrientationTest());
}