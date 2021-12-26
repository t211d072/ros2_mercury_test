#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <practice/prac_lib/prac_lib.hpp>
#include <mutex>

namespace project_dil {

class ROS2Template : public rclcpp::Node {
public:
  ROS2Template();
  ~ROS2Template();
  
  void run();

private:
  void initParams();
  void initTopic();
  void initService();
  void onPointCloudSubscribed(const sensor_msgs::msg::PointCloud::SharedPtr msg);

  std::thread *thread_to_spin_;

  sensor_msgs::msg::PointCloud pointcloud_;

  bool isObstacleNear_ = false;

  // Publisherの登録
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_pose_;
  // Subscriberの登録
  rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr sub_pointcloud_;
  // ライブラリのインスタンス化
  practice::PracLib prac_lib_;

  std::mutex mutex_;
};

ROS2Template:: ROS2Template(): Node("dil_template"){

  using std::placeholders::_1;

  initParams();
  initTopic();
  initService();

  sub_pointcloud_ = this->create_subscription<sensor_msgs::msg::PointCloud>("c32/points", 10, 
                    std::bind(&ROS2Template::onPointCloudSubscribed, this, _1));

  // メインループ用スレッドの作成
  thread_to_spin_ = new std::thread([this]{
    rclcpp::spin(this->get_node_base_interface());
  });
  thread_to_spin_->detach();
}

ROS2Template::~ROS2Template(){
  // メインループ用スレッドの削除
  delete thread_to_spin_;
}

/**********
 * メインループ
 **********/
void ROS2Template::run(){
  // ループ周期の登録
  rclcpp::WallRate loop_rate(30);

  sensor_msgs::msg::PointCloud pointcloud = pointcloud_;

  float angle = 90;
  int time_count = 0;

  while(rclcpp::ok()){
    geometry_msgs::msg::Pose pose;

    RCLCPP_INFO_STREAM(this->get_logger(), "flag: " << isObstacleNear_);

    if(isObstacleNear_ == false){
      if(time_count < 150) angle = 0;
      if(time_count >= 150 && time_count < 300) angle = -45;
      if(time_count >= 300 && time_count < 450) angle = 45;
      if(time_count > 450) rclcpp::shutdown();

      cv::Point2f coordinate = prac_lib_.cvtAngleToPoint(angle);

      pose.position.x = coordinate.x;
      pose.position.y = coordinate.y;
      
      // // 前方
      //pose.position.x = 1.0;
      //pose.position.y = 0.0;

      // // 右前方
      // pose.position.x = 1.0;
      // pose.position.y = -0.5;

      // // 左前方
      // pose.position.x = 1.0;
      // pose.position.y = 0.5;
      
      // // 停止
      // pose.position.x = 0.0;
      // pose.position.y = 0.0;

      pub_pose_->publish(pose);
      RCLCPP_INFO_STREAM(this->get_logger(), "x: " << pose.position.x << ", y:" << pose.position.y);
    }

    for(int i = 0, size = pointcloud.points.size(); i < size; i++){
      if((pointcloud.points[i].x >  0   && pointcloud.points[i].x < 1.5) &&
         (pointcloud.points[i].y > -0.5 && pointcloud.points[i].y < 0.5)){
        isObstacleNear_ = true;
      }else isObstacleNear_ = false;
    }

    // 一定周期にするためのウェイト
    time_count++;
    loop_rate.sleep();
  }
}

/**********
 * パラメータの登録
 **********/
void ROS2Template::initParams(){

}

/**********
 * トピックの登録
 **********/
void ROS2Template::initTopic(){
  pub_pose_ = this->create_publisher<geometry_msgs::msg::Pose>("/rs_path_following/target_pose", 10);
}

/**********
 * サービスの登録
 **********/
void ROS2Template::initService(){

}

/**********
 * Pose型をサブスクライブするコールバック関数
 **********/
void ROS2Template::onPointCloudSubscribed(const sensor_msgs::msg::PointCloud::SharedPtr msg)
{
  //RCLCPP_INFO_STREAM(this->get_logger(), "sub_x: " << pose->position.x << ", sub_y:" << pose->position.y);
  std::lock_guard<std::mutex> lock(mutex_);
  pointcloud_ = *msg;
}

} // project_dil

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<project_dil::ROS2Template>();

    node->run();

    rclcpp::shutdown();
    return 0;
}