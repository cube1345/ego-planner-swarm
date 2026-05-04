#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Geometry>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class MultiSensorFusionNode final : public rclcpp::Node
{
public:
  using PointT = pcl::PointXYZ;
  using CloudT = pcl::PointCloud<PointT>;
  using CloudMsg = sensor_msgs::msg::PointCloud2;
  using ImageMsg = sensor_msgs::msg::Image;
  using SyncCloudPolicy = message_filters::sync_policies::ApproximateTime<CloudMsg, CloudMsg>;
  using SyncImagePolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, CloudMsg>;

  MultiSensorFusionNode()
      : Node("multi_sensor_fusion_node"),
        tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_)
  {
    declareAndLoadParameters();

    fusion_pub_ = create_publisher<CloudMsg>(output_topic_, rclcpp::SensorDataQoS());

    if (depth_input_is_image_)
    {
      depth_image_sub_ = std::make_shared<message_filters::Subscriber<ImageMsg>>(
          this, depth_topic_, rmw_qos_profile_sensor_data);
      lidar_sub_ = std::make_shared<message_filters::Subscriber<CloudMsg>>(
          this, lidar_topic_, rmw_qos_profile_sensor_data);
      sync_image_lidar_ = std::make_shared<message_filters::Synchronizer<SyncImagePolicy>>(
          SyncImagePolicy(sync_queue_size_), *depth_image_sub_, *lidar_sub_);
      sync_image_lidar_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_sec_));
      sync_image_lidar_->registerCallback(
          std::bind(&MultiSensorFusionNode::onImageLidarSync, this, std::placeholders::_1, std::placeholders::_2));

      RCLCPP_INFO(get_logger(), "Fusion mode: depth image + lidar cloud");
    }
    else
    {
      depth_cloud_sub_ = std::make_shared<message_filters::Subscriber<CloudMsg>>(
          this, depth_topic_, rmw_qos_profile_sensor_data);
      lidar_sub_ = std::make_shared<message_filters::Subscriber<CloudMsg>>(
          this, lidar_topic_, rmw_qos_profile_sensor_data);
      sync_cloud_lidar_ = std::make_shared<message_filters::Synchronizer<SyncCloudPolicy>>(
          SyncCloudPolicy(sync_queue_size_), *depth_cloud_sub_, *lidar_sub_);
      sync_cloud_lidar_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_sec_));
      sync_cloud_lidar_->registerCallback(
          std::bind(&MultiSensorFusionNode::onCloudLidarSync, this, std::placeholders::_1, std::placeholders::_2));

      RCLCPP_INFO(get_logger(), "Fusion mode: depth cloud + lidar cloud");
    }

    RCLCPP_INFO(get_logger(), "Fusion node ready. depth_topic=%s lidar_topic=%s output_topic=%s target_frame=%s",
                depth_topic_.c_str(), lidar_topic_.c_str(), output_topic_.c_str(), target_frame_.c_str());
  }

private:
  void declareAndLoadParameters()
  {
    declare_parameter("depth_input_is_image", false);
    declare_parameter("depth_topic", "/drone_0_pcl_render_node/cloud");
    declare_parameter("lidar_topic", "/drone_0_lidar/points");
    declare_parameter("output_topic", "/fusion_cloud");
    declare_parameter("target_frame", "world");
    declare_parameter("depth_frame_override", "");
    declare_parameter("lidar_frame_override", "");

    declare_parameter("sync_queue_size", 20);
    declare_parameter("sync_slop_sec", 0.08); // TODO: verify per onboard clock jitter.
    declare_parameter("tf_lookup_timeout_sec", 0.02);

    // TODO: calibrate camera intrinsics for real sensor.
    declare_parameter("camera_fx", 387.229248046875);
    declare_parameter("camera_fy", 387.229248046875);
    declare_parameter("camera_cx", 321.04638671875);
    declare_parameter("camera_cy", 243.44969177246094);
    declare_parameter("depth_scale", 1000.0); // 16UC1 depth unit -> meters
    declare_parameter("depth_min_m", 0.20);
    declare_parameter("depth_max_m", 8.0);
    declare_parameter("depth_stride", 2);

    declare_parameter("range_min_m", 0.20);
    declare_parameter("range_max_m", 12.0);
    declare_parameter("depth_hfov_deg", 88.0);
    declare_parameter("depth_vfov_deg", 58.0);
    declare_parameter("lidar_hfov_deg", 360.0);
    declare_parameter("lidar_vfov_deg", 40.0);

    declare_parameter("voxel_leaf_size", 0.10);
    declare_parameter("enable_sor", true);
    declare_parameter("sor_mean_k", 12);
    declare_parameter("sor_stddev_mul", 1.0);

    declare_parameter("warn_min_hz", 10.0);

    get_parameter("depth_input_is_image", depth_input_is_image_);
    get_parameter("depth_topic", depth_topic_);
    get_parameter("lidar_topic", lidar_topic_);
    get_parameter("output_topic", output_topic_);
    get_parameter("target_frame", target_frame_);
    get_parameter("depth_frame_override", depth_frame_override_);
    get_parameter("lidar_frame_override", lidar_frame_override_);

    get_parameter("sync_queue_size", sync_queue_size_);
    get_parameter("sync_slop_sec", sync_slop_sec_);
    get_parameter("tf_lookup_timeout_sec", tf_lookup_timeout_sec_);

    get_parameter("camera_fx", camera_fx_);
    get_parameter("camera_fy", camera_fy_);
    get_parameter("camera_cx", camera_cx_);
    get_parameter("camera_cy", camera_cy_);
    get_parameter("depth_scale", depth_scale_);
    get_parameter("depth_min_m", depth_min_m_);
    get_parameter("depth_max_m", depth_max_m_);
    get_parameter("depth_stride", depth_stride_);

    get_parameter("range_min_m", range_min_m_);
    get_parameter("range_max_m", range_max_m_);
    get_parameter("depth_hfov_deg", depth_hfov_deg_);
    get_parameter("depth_vfov_deg", depth_vfov_deg_);
    get_parameter("lidar_hfov_deg", lidar_hfov_deg_);
    get_parameter("lidar_vfov_deg", lidar_vfov_deg_);

    get_parameter("voxel_leaf_size", voxel_leaf_size_);
    get_parameter("enable_sor", enable_sor_);
    get_parameter("sor_mean_k", sor_mean_k_);
    get_parameter("sor_stddev_mul", sor_stddev_mul_);

    get_parameter("warn_min_hz", warn_min_hz_);
  }

  void onCloudLidarSync(const CloudMsg::ConstSharedPtr &depth_msg,
                        const CloudMsg::ConstSharedPtr &lidar_msg)
  {
    if (!convertCloudMsg(*depth_msg, depth_local_cloud_) || !convertCloudMsg(*lidar_msg, lidar_local_cloud_))
    {
      ++drop_count_;
      return;
    }

    processAndPublish(*depth_msg, *lidar_msg, depth_msg->header.stamp, false);
  }

  void onImageLidarSync(const ImageMsg::ConstSharedPtr &depth_msg,
                        const CloudMsg::ConstSharedPtr &lidar_msg)
  {
    if (!convertDepthImageToCloud(*depth_msg, depth_local_cloud_) || !convertCloudMsg(*lidar_msg, lidar_local_cloud_))
    {
      ++drop_count_;
      return;
    }

    processAndPublish(*depth_msg, *lidar_msg, (depth_msg->header.stamp.sec >= lidar_msg->header.stamp.sec) ? depth_msg->header.stamp : lidar_msg->header.stamp, true);
  }

  template <typename DepthMsgT>
  void processAndPublish(const DepthMsgT &depth_msg,
                         const CloudMsg &lidar_msg,
                         const builtin_interfaces::msg::Time &stamp,
                         bool from_depth_image)
  {
    const std::string depth_frame = depth_frame_override_.empty() ? depth_msg.header.frame_id : depth_frame_override_;
    const std::string lidar_frame = lidar_frame_override_.empty() ? lidar_msg.header.frame_id : lidar_frame_override_;

    preprocessSensorCloud(depth_local_cloud_, true);
    preprocessSensorCloud(lidar_local_cloud_, false);

    if (!transformCloudToTarget(depth_local_cloud_, depth_target_cloud_, depth_frame, stamp) ||
        !transformCloudToTarget(lidar_local_cloud_, lidar_target_cloud_, lidar_frame, stamp))
    {
      ++drop_count_;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Skip frame due to TF transform failure.");
      return;
    }

    fuseClouds(depth_target_cloud_, lidar_target_cloud_, fused_cloud_);
    postFilterCloud(fused_cloud_, filtered_cloud_);
    publishFusionCloud(filtered_cloud_, stamp);

    ++frame_count_;
    updateRuntimeStats(from_depth_image);
  }

  bool convertCloudMsg(const CloudMsg &msg, CloudT::Ptr &cloud)
  {
    if (msg.data.empty())
    {
      return false;
    }
    pcl::fromROSMsg(msg, *cloud);
    return !cloud->points.empty();
  }

  bool convertDepthImageToCloud(const ImageMsg &msg, CloudT::Ptr &cloud)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, msg.encoding);
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "cv_bridge exception: %s", e.what());
      return false;
    }

    cloud->clear();
    const int rows = cv_ptr->image.rows;
    const int cols = cv_ptr->image.cols;
    cloud->points.reserve(static_cast<size_t>((rows / std::max(depth_stride_, 1)) * (cols / std::max(depth_stride_, 1))));

    const bool is_16u = (msg.encoding == sensor_msgs::image_encodings::TYPE_16UC1);
    const bool is_32f = (msg.encoding == sensor_msgs::image_encodings::TYPE_32FC1);
    if (!is_16u && !is_32f)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Unsupported depth encoding: %s (expect 16UC1 or 32FC1)", msg.encoding.c_str());
      return false;
    }

    for (int v = 0; v < rows; v += std::max(depth_stride_, 1))
    {
      for (int u = 0; u < cols; u += std::max(depth_stride_, 1))
      {
        float z = 0.0F;
        if (is_16u)
        {
          const auto raw = cv_ptr->image.at<uint16_t>(v, u);
          if (raw == 0U)
          {
            continue;
          }
          z = static_cast<float>(raw / depth_scale_);
        }
        else
        {
          z = cv_ptr->image.at<float>(v, u);
        }

        if (!std::isfinite(z) || z < depth_min_m_ || z > depth_max_m_)
        {
          continue;
        }

        PointT p;
        p.x = static_cast<float>((static_cast<double>(u) - camera_cx_) * z / camera_fx_);
        p.y = static_cast<float>((static_cast<double>(v) - camera_cy_) * z / camera_fy_);
        p.z = z;
        cloud->points.emplace_back(p);
      }
    }

    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;
    return !cloud->points.empty();
  }

  void preprocessSensorCloud(CloudT::Ptr &cloud, bool is_depth)
  {
    if (cloud->empty())
    {
      return;
    }

    CloudT filtered;
    filtered.points.reserve(cloud->points.size());

    const double hfov = (is_depth ? depth_hfov_deg_ : lidar_hfov_deg_) * M_PI / 180.0;
    const double vfov = (is_depth ? depth_vfov_deg_ : lidar_vfov_deg_) * M_PI / 180.0;
    const double hfov_half = 0.5 * hfov;
    const double vfov_half = 0.5 * vfov;

    for (const auto &p : cloud->points)
    {
      const double r = std::sqrt(static_cast<double>(p.x * p.x + p.y * p.y + p.z * p.z));
      if (!std::isfinite(r) || r < range_min_m_ || r > range_max_m_)
      {
        continue;
      }

      double h_angle = 0.0;
      double v_angle = 0.0;
      if (is_depth)
      {
        // Camera optical frame assumption: x right, y down, z forward.
        h_angle = std::atan2(static_cast<double>(p.x), static_cast<double>(p.z));
        v_angle = std::atan2(-static_cast<double>(p.y),
                             std::sqrt(static_cast<double>(p.x * p.x + p.z * p.z)) + 1e-6);
      }
      else
      {
        // LiDAR frame assumption: x forward, y left, z up.
        h_angle = std::atan2(static_cast<double>(p.y), static_cast<double>(p.x));
        v_angle = std::atan2(static_cast<double>(p.z),
                             std::sqrt(static_cast<double>(p.x * p.x + p.y * p.y)) + 1e-6);
      }

      if (std::abs(h_angle) > hfov_half || std::abs(v_angle) > vfov_half)
      {
        continue;
      }

      filtered.points.push_back(p);
    }

    filtered.width = static_cast<uint32_t>(filtered.points.size());
    filtered.height = 1;
    filtered.is_dense = false;
    cloud->swap(filtered);
  }

  bool transformCloudToTarget(const CloudT::ConstPtr &input,
                              CloudT::Ptr &output,
                              const std::string &source_frame,
                              const builtin_interfaces::msg::Time &stamp)
  {
    if (input->empty())
    {
      output->clear();
      return true;
    }

    geometry_msgs::msg::TransformStamped tf_stamped;
    try
    {
      tf_stamped = tf_buffer_.lookupTransform(
          target_frame_, source_frame, rclcpp::Time(stamp), rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF lookup failed: %s", ex.what());
      return false;
    }

    const auto &t = tf_stamped.transform.translation;
    const auto &q_msg = tf_stamped.transform.rotation;
    Eigen::Quaternionf q(static_cast<float>(q_msg.w), static_cast<float>(q_msg.x),
                         static_cast<float>(q_msg.y), static_cast<float>(q_msg.z));

    Eigen::Matrix4f tf_matrix = Eigen::Matrix4f::Identity();
    tf_matrix.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    tf_matrix(0, 3) = static_cast<float>(t.x);
    tf_matrix(1, 3) = static_cast<float>(t.y);
    tf_matrix(2, 3) = static_cast<float>(t.z);

    pcl::transformPointCloud(*input, *output, tf_matrix);
    output->width = static_cast<uint32_t>(output->points.size());
    output->height = 1;
    output->is_dense = false;
    return true;
  }

  void fuseClouds(const CloudT::ConstPtr &depth_cloud,
                  const CloudT::ConstPtr &lidar_cloud,
                  CloudT::Ptr &output)
  {
    output->clear();
    output->points.reserve(depth_cloud->points.size() + lidar_cloud->points.size());
    output->points.insert(output->points.end(), depth_cloud->points.begin(), depth_cloud->points.end());
    output->points.insert(output->points.end(), lidar_cloud->points.begin(), lidar_cloud->points.end());
    output->width = static_cast<uint32_t>(output->points.size());
    output->height = 1;
    output->is_dense = false;
  }

  void postFilterCloud(const CloudT::ConstPtr &input, CloudT::Ptr &output)
  {
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setLeafSize(static_cast<float>(voxel_leaf_size_), static_cast<float>(voxel_leaf_size_),
                             static_cast<float>(voxel_leaf_size_));
    voxel_filter.setInputCloud(input);
    voxel_filter.filter(*output);

    if (!enable_sor_ || output->points.size() < static_cast<size_t>(std::max(4, sor_mean_k_)))
    {
      return;
    }

    pcl::StatisticalOutlierRemoval<PointT> sor_filter;
    sor_filter.setInputCloud(output);
    sor_filter.setMeanK(std::max(sor_mean_k_, 4));
    sor_filter.setStddevMulThresh(std::max(sor_stddev_mul_, 0.1));
    sor_filter.filter(*output);
  }

  void publishFusionCloud(const CloudT::ConstPtr &cloud, const builtin_interfaces::msg::Time &stamp)
  {
    CloudMsg output_msg;
    pcl::toROSMsg(*cloud, output_msg);
    output_msg.header.stamp = stamp;
    output_msg.header.frame_id = target_frame_;
    fusion_pub_->publish(output_msg);
  }

  void updateRuntimeStats(bool from_depth_image)
  {
    const auto now_time = this->now();
    if (last_success_stamp_.nanoseconds() > 0)
    {
      const double dt = (now_time - last_success_stamp_).seconds();
      if (dt > 1e-4)
      {
        const double inst_hz = 1.0 / dt;
        running_hz_ = (running_hz_ <= 1e-6) ? inst_hz : 0.9 * running_hz_ + 0.1 * inst_hz;
      }
    }
    last_success_stamp_ = now_time;

    if (last_log_stamp_.nanoseconds() == 0 || (now_time - last_log_stamp_).seconds() > 1.0)
    {
      last_log_stamp_ = now_time;
      RCLCPP_INFO(get_logger(),
                  "fusion_hz=%.2f frames=%zu drops=%zu mode=%s out_points=%zu",
                  running_hz_, frame_count_, drop_count_, from_depth_image ? "image+lidar" : "cloud+lidar",
                  filtered_cloud_->points.size());
      if (running_hz_ > 1e-6 && running_hz_ < warn_min_hz_)
      {
        RCLCPP_WARN(get_logger(), "Fusion rate %.2f Hz is below target %.2f Hz", running_hz_, warn_min_hz_);
      }
    }
  }

private:
  bool depth_input_is_image_{false};
  std::string depth_topic_;
  std::string lidar_topic_;
  std::string output_topic_;
  std::string target_frame_;
  std::string depth_frame_override_;
  std::string lidar_frame_override_;

  int sync_queue_size_{20};
  double sync_slop_sec_{0.08};
  double tf_lookup_timeout_sec_{0.02};

  double camera_fx_{387.229248046875};
  double camera_fy_{387.229248046875};
  double camera_cx_{321.04638671875};
  double camera_cy_{243.44969177246094};
  double depth_scale_{1000.0};
  double depth_min_m_{0.2};
  double depth_max_m_{8.0};
  int depth_stride_{2};

  double range_min_m_{0.2};
  double range_max_m_{12.0};
  double depth_hfov_deg_{88.0};
  double depth_vfov_deg_{58.0};
  double lidar_hfov_deg_{360.0};
  double lidar_vfov_deg_{40.0};

  double voxel_leaf_size_{0.10};
  bool enable_sor_{true};
  int sor_mean_k_{12};
  double sor_stddev_mul_{1.0};

  double warn_min_hz_{10.0};
  size_t frame_count_{0};
  size_t drop_count_{0};
  double running_hz_{0.0};
  rclcpp::Time last_success_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_log_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<CloudMsg>::SharedPtr fusion_pub_;

  std::shared_ptr<message_filters::Subscriber<CloudMsg>> depth_cloud_sub_;
  std::shared_ptr<message_filters::Subscriber<ImageMsg>> depth_image_sub_;
  std::shared_ptr<message_filters::Subscriber<CloudMsg>> lidar_sub_;

  std::shared_ptr<message_filters::Synchronizer<SyncCloudPolicy>> sync_cloud_lidar_;
  std::shared_ptr<message_filters::Synchronizer<SyncImagePolicy>> sync_image_lidar_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  CloudT::Ptr depth_local_cloud_{std::make_shared<CloudT>()};
  CloudT::Ptr lidar_local_cloud_{std::make_shared<CloudT>()};
  CloudT::Ptr depth_target_cloud_{std::make_shared<CloudT>()};
  CloudT::Ptr lidar_target_cloud_{std::make_shared<CloudT>()};
  CloudT::Ptr fused_cloud_{std::make_shared<CloudT>()};
  CloudT::Ptr filtered_cloud_{std::make_shared<CloudT>()};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiSensorFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
