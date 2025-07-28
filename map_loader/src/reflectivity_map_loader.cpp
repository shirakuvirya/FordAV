#include <chrono>
#include <cmath>
#include <condition_variable>
#include <dirent.h>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>

using namespace std::chrono_literals;

//----------------------------------------
// Helper struct & functions
//----------------------------------------
struct Tile
{
  std::string path;
  float origin_x;
  float origin_y;
  int id;
  std::vector<Tile*> neighbors;
};

bool checkNeighbor(const Tile &a, const Tile &b, float thresh)
{
  float dist = std::abs(a.origin_x - b.origin_x)
             + std::abs(a.origin_y - b.origin_y);
  return dist <= thresh;
}

bool getFileNames(const std::string &folderPath,
                  std::vector<std::string> &pcdPaths)
{
  DIR *dirp = opendir(folderPath.c_str());
  if (!dirp) {
    perror("Cannot open dir");
    return false;
  }
  struct dirent *dp;
  while ((dp = readdir(dirp)) != nullptr) {
    std::string name = dp->d_name;
    if (name.size() > 4 && name.substr(name.size() - 4) == ".pcd") {
      pcdPaths.push_back(folderPath + "/" + name);
    }
  }
  closedir(dirp);
  return true;
}

//----------------------------------------
// PointCloudMapLoader
//----------------------------------------
class PointCloudMapLoader
  : public std::enable_shared_from_this<PointCloudMapLoader>
{
public:
  explicit PointCloudMapLoader(rclcpp::Node::SharedPtr node)
  : node_(node), pose_available_(false)
  {
    readParameters();
    setupCommunication();
    createTilesFromFolder();
    computeNeighbors();
  }

private:
  // ROS node handle
  rclcpp::Node::SharedPtr node_;

  // Publisher & Subscriber
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  // Timer for periodic map publishing
  rclcpp::TimerBase::SharedPtr timer_;

  // Internal state
  Eigen::Vector3d current_pose_;
  bool pose_available_{false};
  std::string map_folder_;
  std::string pcd_topic_;
  std::string pose_topic_;
  float neighbor_dist_{128.0f};
  float publish_rate_{1.0f};

  std::vector<Tile> all_tiles_;

  //--------------------------------------
  // Read params from parameter server
  //--------------------------------------
  void readParameters()
  {
    node_->declare_parameter<std::string>("map_folder", "");
    node_->declare_parameter<std::string>("pcd_topic", "/pointcloud_map");
    node_->declare_parameter<std::string>("pose_topic", "/pose_ground_truth");
    node_->declare_parameter<float>("neighbor_dist", 128.0f);
    node_->declare_parameter<float>("publish_rate", 1.0f);

    node_->get_parameter("map_folder", map_folder_);
    node_->get_parameter("pcd_topic", pcd_topic_);
    node_->get_parameter("pose_topic", pose_topic_);
    node_->get_parameter("neighbor_dist", neighbor_dist_);
    node_->get_parameter("publish_rate", publish_rate_);
  }

  //--------------------------------------
  // Setup publisher, subscriber, and timer
  //--------------------------------------
  void setupCommunication()
  {
    // Publisher
    pcd_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      pcd_topic_, rclcpp::QoS(10));

    // Subscriber
    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, rclcpp::QoS(10),
      std::bind(&PointCloudMapLoader::poseCallback, this, std::placeholders::_1));

    // Timer: call getPoseTile() at publish_rate_ Hz
    auto period = std::chrono::duration<float>(1.0f / publish_rate_);
    timer_ = node_->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&PointCloudMapLoader::getPoseTile, this));
  }

  //--------------------------------------
  // Callback: update current pose
  //--------------------------------------
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    current_pose_(0) = msg->pose.position.x;
    current_pose_(1) = msg->pose.position.y;
    current_pose_(2) = msg->pose.position.z;
    pose_available_ = true;
  }

  //--------------------------------------
  // Build tile list from map_folder_
  //--------------------------------------
  void createTilesFromFolder()
  {
    std::vector<std::string> pcdPaths;
    if (!getFileNames(map_folder_, pcdPaths)) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to read PCD files from '%s'", map_folder_.c_str());
      rclcpp::shutdown();
      return;
    }
    int id_counter = 0;
    for (const auto &path : pcdPaths) {
      // extract tile center from filename, e.g. "12_34.pcd"
      float x, y;
      parseTileCenter(path, x, y);
      Tile t;
      t.path = path;
      t.origin_x = x;
      t.origin_y = y;
      t.id = id_counter++;
      all_tiles_.push_back(t);
    }
    RCLCPP_INFO(node_->get_logger(), "Loaded %zu tiles", all_tiles_.size());
  }

  //--------------------------------------
  // Compute neighbor relationships
  //--------------------------------------
  void computeNeighbors()
  {
    for (auto &ti : all_tiles_) {
      for (auto &tj : all_tiles_) {
        if (checkNeighbor(ti, tj, neighbor_dist_)) {
          ti.neighbors.push_back(&tj);
        }
      }
    }
  }

  //--------------------------------------
  // Periodic: find current tile & publish
  //--------------------------------------
  void getPoseTile()
  {
    if (!pose_available_) {
      RCLCPP_INFO(node_->get_logger(), "Waiting for pose...");
      return;
    }
    for (auto &t : all_tiles_) {
      if (onTile(current_pose_, t)) {
        publishNeighborTiles(t);
        return;
      }
    }
  }

  //--------------------------------------
  // Publish PCDs of neighbors of tile t
  //--------------------------------------
  void publishNeighborTiles(const Tile &t)
  {
    std::vector<std::string> paths;
    for (auto *nbr : t.neighbors) {
      paths.push_back(nbr->path);
    }
    RCLCPP_INFO(node_->get_logger(), "Publishing %zu neighbor tiles", paths.size());
    sensor_msgs::msg::PointCloud2 full_cloud, part_cloud;
    for (const auto &p : paths) {
      if (full_cloud.width == 0) {
        pcl::io::loadPCDFile(p, full_cloud);
      } else {
        pcl::io::loadPCDFile(p, part_cloud);
        full_cloud.width    += part_cloud.width;
        full_cloud.row_step += part_cloud.row_step;
        full_cloud.data.insert(full_cloud.data.end(),
                               part_cloud.data.begin(),
                               part_cloud.data.end());
      }
      if (!rclcpp::ok()) return;
    }
    full_cloud.header.frame_id = "map";
    full_cloud.header.stamp = node_->now();
    pcd_publisher_->publish(full_cloud);
  }

  //--------------------------------------
  // Check if pose lies within tile t
  //--------------------------------------
  bool onTile(const Eigen::Vector3d &pose, const Tile &t)
  {
    return (pose(0) > t.origin_x &&
            pose(1) > t.origin_y &&
            pose(0) < t.origin_x + 64.0f &&
            pose(1) < t.origin_y + 64.0f);
  }

  //--------------------------------------
  // Extract x,y from filename "x_y.pcd"
  //--------------------------------------
  void parseTileCenter(const std::string &path, float &x, float &y)
  {
    auto slash = path.find_last_of('/');
    std::string name = path.substr(slash + 1);
    auto underscore = name.find('_');
    auto dot        = name.find('.');
    x = std::stof(name.substr(0, underscore));
    y = std::stof(name.substr(underscore + 1, dot - underscore - 1));
  }
};

//----------------------------------------
// Main
//----------------------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("point_cloud_map_loader");
  // The first CLI arg is the map folder
  if (argc < 2) {
    RCLCPP_ERROR(node->get_logger(), "Usage: point_cloud_map_loader <map_folder>");
    return 1;
  }
  // Pass map_folder as parameter
  node->set_parameter(rclcpp::Parameter("map_folder", std::string(argv[1])));

  auto loader = std::make_shared<PointCloudMapLoader>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
