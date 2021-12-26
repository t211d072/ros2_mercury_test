#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/pose.hpp>

/*** グローバル変数の宣言 ***/
rclcpp::Node::SharedPtr g_node = nullptr;

/*** Int32型をサブスクライブするコールバック関数 ***/
void onPoseSubscribed(const geometry_msgs::msg::Pose::SharedPtr pose)
{
  RCLCPP_INFO_STREAM(g_node->get_logger(), "x: " << pose->position.x << ", y:" << pose->position.y);
}
int main(int argc, char **argv)
{
  /*** プロセスの初期化 ***/
  rclcpp::init(argc, argv);
  /*** ノードの初期化 ***/
  g_node = rclcpp::Node::make_shared("subscriber");
  /*** サブスクライバの初期化 ***/
  auto sub = g_node->create_subscription<geometry_msgs::msg::Pose>("/rs_path_following/target_pose", 10, onPoseSubscribed);
  /*** コールバックの発生を待機 ***/
  rclcpp::spin(g_node);
  /*** プロセスの終了 ***/
  g_node = nullptr;
  rclcpp::shutdown();
  return 0;
}