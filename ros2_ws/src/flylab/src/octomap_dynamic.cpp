// octomap_dynamic —— octomap 动态清理节点 (建图链路)
//
// 功能: 周期调用 /octomap_server/clear_bbox 服务 (BoundingBoxQuery),
//       以无人机当前位置为中心, 清理 6 个方向上远离无人机的旧地图区域,
//       防止建图链路长时间飞行后地图无限膨胀。
// 参数: range_around_drone (默认 5.0 米, 无人机周围保留范围),
//       map_range (默认 200.0 米, 总地图范围)。
// 输入: nav_msgs/Odometry 无人机里程计; 移植自 hw_insight/octomap_dynamic.py。
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap_msgs/srv/bounding_box_query.hpp>

using namespace std::chrono_literals;

// 动态清理 octomap：周期调用 /octomap_server/clear_bbox 服务，
// 按无人机当前位置轮流清理 6 个方向上远离无人机的旧地图区域，
// 防止地图随飞行无限膨胀。
// 移植自 hw_insight/octomap_dynamic.py
class OctomapDynamic : public rclcpp::Node
{
public:
  OctomapDynamic() : Node("octomap_dynamic"), x_(0.0), y_(0.0), z_(0.0), idx_(0)
  {
    // 无人机周围保留范围（米）与地图总尺寸（米）
    range_around_drone_ = declare_parameter<double>("range_around_drone", 5.0);
    map_range_ = declare_parameter<double>("map_range", 200.0);
    RCLCPP_INFO(get_logger(), "range_around_drone = %.2f, map_range = %.2f",
                range_around_drone_, map_range_);

    clear_bbox_client_ = create_client<octomap_msgs::srv::BoundingBoxQuery>(
      "/octomap_server/clear_bbox");

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/airsim_node/PX4/odometry", 1,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg)
      {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        z_ = msg->pose.pose.position.z;
      });

    timer_ = create_wall_timer(100ms, std::bind(&OctomapDynamic::timerCallback, this));
  }

private:
  void timerCallback()
  {
    if (!clear_bbox_client_->service_is_ready())
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                           "等待 /octomap_server/clear_bbox 服务...");
      return;
    }

    auto request = std::make_shared<octomap_msgs::srv::BoundingBoxQuery::Request>();
    // 轮流清理 6 个方向：+x / -x / +y / -y / +z / -z，
    // 每次只保留无人机周围 range_around_drone 米内的那一侧，其余方向清空
    const double min = -map_range_;
    const double max = map_range_;
    switch (idx_)
    {
      case 0:
        request->min.x = x_ + range_around_drone_; request->max.x = max;
        request->min.y = min; request->max.y = max;
        request->min.z = min; request->max.z = max;
        break;
      case 1:
        request->min.x = min; request->max.x = x_ - range_around_drone_;
        request->min.y = min; request->max.y = max;
        request->min.z = min; request->max.z = max;
        break;
      case 2:
        request->min.x = min; request->max.x = max;
        request->min.y = y_ + range_around_drone_; request->max.y = max;
        request->min.z = min; request->max.z = max;
        break;
      case 3:
        request->min.x = min; request->max.x = max;
        request->min.y = min; request->max.y = y_ - range_around_drone_;
        request->min.z = min; request->max.z = max;
        break;
      case 4:
        request->min.x = min; request->max.x = max;
        request->min.y = min; request->max.y = max;
        request->min.z = z_ + range_around_drone_; request->max.z = max;
        break;
      default: // 5
        request->min.x = min; request->max.x = max;
        request->min.y = min; request->max.y = max;
        request->min.z = min; request->max.z = z_ - range_around_drone_;
        break;
    }

    clear_bbox_client_->async_send_request(request);
    idx_ = (idx_ + 1) % 6;
  }

  double range_around_drone_;
  double map_range_;
  double x_, y_, z_;
  int idx_;

  rclcpp::Client<octomap_msgs::srv::BoundingBoxQuery>::SharedPtr clear_bbox_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapDynamic>());
  rclcpp::shutdown();
  return 0;
}
