#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

/**
 * @brief Vehicle GPS position uORB topic data callback
 */
class VehicleGpsPositionListener : public rclcpp::Node
{
public:
  explicit VehicleGpsPositionListener() : Node("bridge_test") {
    trajectory_setpoint_publisher_ =
        this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic", 10);
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/omnihex_setpoint",
#ifdef ROS_DEFAULT_API
        10,
#endif
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          std::cout << "\n\n\n\n\n\n\n\n\n\n";
          std::cout << msg->pose.position.x   << std::endl;
          std::cout << msg->pose.position.y   << std::endl;
          std::cout << msg->pose.position.z   << std::endl;
          Eigen::Quaterniond quat(msg->pose.orientation.w,
                                  msg->pose.orientation.x,
                                  msg->pose.orientation.y,
                                  msg->pose.orientation.z);
          Eigen::Vector3d euler_angle = quat.toRotationMatrix().eulerAngles(2, 1, 0);
          px4_msgs::msg::TrajectorySetpoint traj_setpoint_msg{};
          traj_setpoint_msg.x = msg->pose.position.x;
          traj_setpoint_msg.y = msg->pose.position.y;
          traj_setpoint_msg.z = msg->pose.position.z;
          traj_setpoint_msg.yaw = euler_angle[2];
          traj_setpoint_msg.pitch = euler_angle[1];
          traj_setpoint_msg.roll = euler_angle[0];
          trajectory_setpoint_publisher_->publish(traj_setpoint_msg);
        });
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      subscription_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
      trajectory_setpoint_publisher_;
};

int main(int argc, char *argv[])
{
  std::cout << "Starting vehicle_global_position listener node..." << std::endl;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleGpsPositionListener>());

  rclcpp::shutdown();
  return 0;
}
