#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <string>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace trajectory_msgs::msg;

float cal_pose_diff(const geometry_msgs::msg::PoseStamped& current_pose, const MultiDOFJointTrajectoryPoint& target);

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control")
	{
#ifdef ROS_DEFAULT_API
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("OffboardControlMode_PubSubTopic", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("VehicleCommand_PubSubTopic", 10);
#else
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("OffboardControlMode_PubSubTopic");
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic");
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("VehicleCommand_PubSubTopic");
#endif

		// get common timestamp
		traj_sub_ = 
			this->create_subscription<MultiDOFJointTrajectory>("/omnihex_traj", 1,
							[this](const MultiDOFJointTrajectory::SharedPtr msg)
									{
										trajectoryPtr = msg;
										traj_updated = true;
									});
		pos_subscription_ = 
			this->create_subscription<VehicleLocalPosition>("VehicleLocalPosition_PubSubTopic", 1, 
							[this](const VehicleLocalPosition::SharedPtr msg)
									{
										current_pose_.header.stamp = get_clock()->now();
										current_pose_.pose.position.x = msg->y;
										current_pose_.pose.position.y = msg->x;
										current_pose_.pose.position.z =-msg->z;
									});
		att_subscription_ = 
			this->create_subscription<VehicleAttitude>("VehicleAttitude_PubSubTopic", 1, 
							[this](const VehicleAttitude::SharedPtr msg)
									{
										current_pose_.header.stamp = get_clock()->now();
										current_pose_.pose.orientation.x = msg->q[1];
										current_pose_.pose.orientation.y = msg->q[2];
										current_pose_.pose.orientation.z = msg->q[3];
										current_pose_.pose.orientation.w = msg->q[0];
									});
		timesync_sub_ =
			this->create_subscription<Timesync>("Timesync_PubSubTopic", 10,
									[this](const Timesync::UniquePtr msg)
									{
										timestamp_.store(msg->timestamp);
									});
		offboard_setpoint_counter_ = 0;
		set_trajectory_setpoint(0.0, 0.0, 1.0);

		auto timer_callback = [this]() -> void
		{
			if (offboard_setpoint_counter_ == 10)
			{
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}
			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint(target_traj_setpoint);
			
			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11)
			{
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm() const;
	void disarm() const;

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr pos_subscription_;
	rclcpp::Subscription<VehicleAttitude>::SharedPtr att_subscription_;
	rclcpp::Subscription<MultiDOFJointTrajectory>::SharedPtr traj_sub_;

	std::atomic<uint64_t> timestamp_; //!< common synced timestamped

	uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent

public:

	float threshold = 1.0;
	geometry_msgs::msg::PoseStamped current_pose_{};
	TrajectorySetpoint target_traj_setpoint{};
	// px4_msgs::msg::VehicleLocalPosition current_position_;
	MultiDOFJointTrajectory::SharedPtr trajectoryPtr;
	bool traj_updated = false;

	void publish_offboard_control_mode() const;
	void set_trajectory_setpoint(const MultiDOFJointTrajectoryPoint& target);
	void set_trajectory_setpoint(float x, float y, float z, 
						float roll = 0.0, float pitch = 0.0, float yaw = 0.0);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
								 float param2 = 0.0) const;
	void publish_trajectory_setpoint(TrajectorySetpoint msg);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() const
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() const
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode() const
{
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = true;
	msg.body_rate = false;

	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::set_trajectory_setpoint(float x, float y, float z, 
						float roll, float pitch, float yaw)
{
	// msg.timestamp = timestamp_.load();
	target_traj_setpoint.x = y;
	target_traj_setpoint.y = x;
	target_traj_setpoint.z = -z;

	// msg.x = target_pose_.transforms[0].translation.x;
	// msg.y = -target_pose_.transforms[0].translation.y;
	// msg.z = -target_pose_.transforms[0].translation.z;
	target_traj_setpoint.roll = pitch; // target_pose_.x;
	target_traj_setpoint.pitch = roll; // target_pose_.x;
	target_traj_setpoint.yaw = -yaw + M_PI / 2; // target_pose_.x;
}

void OffboardControl::publish_trajectory_setpoint(TrajectorySetpoint msg)
{
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
											  float param2) const
{
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}

void OffboardControl::set_trajectory_setpoint(const MultiDOFJointTrajectoryPoint& target) {
	// msg.timestamp = timestamp_.load();
	target_traj_setpoint.x = target.transforms[0].translation.y;
	target_traj_setpoint.y = target.transforms[0].translation.x;
	target_traj_setpoint.z = -target.transforms[0].translation.z;
	tf2::Quaternion q;
	tf2::convert(target.transforms[0].rotation, q);
	std::cout << "q: " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
	// Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z());
	// Eigen::Vector3d eulerAngle = q_eigen.matrix().eulerAngles(2,1,0);
	// calculate euler angles
	// roll (x-axis rotation)
	double roll = 0.0, pitch = 0.0, yaw = 0.0;
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    yaw = std::atan2(siny_cosp, cosy_cosp);

	std::cout << roll << " " << pitch << " " << yaw << std::endl;
	// todo: eigen quaternion to euler angle
	target_traj_setpoint.roll = roll;
	target_traj_setpoint.pitch = -pitch; 
	target_traj_setpoint.yaw = -yaw + M_PI / 2; 
	// std::cout << "target position: " << target_traj_setpoint.x << " "  
	// 								 << target_traj_setpoint.y << " " 
	// 								 << target_traj_setpoint.z << " " 
	// 								 << target_traj_setpoint.roll << " " 
	// 								 << target_traj_setpoint.pitch << " " 
	// 								 << target_traj_setpoint.yaw << std::endl;
}

float cal_pose_diff(const geometry_msgs::msg::PoseStamped& current_pose, const MultiDOFJointTrajectoryPoint& target) {
	float delta_x = current_pose.pose.position.x - target.transforms[0].translation.x;
	float delta_y = current_pose.pose.position.y - target.transforms[0].translation.y;
	float delta_z = current_pose.pose.position.z - target.transforms[0].translation.z;
	tf2::Quaternion q_target, q_current;
	tf2::convert(target.transforms[0].rotation, q_target);
	tf2::convert(current_pose.pose.orientation, q_current);

	// Eigen::Quaterniond q_target_eigen(q_target.w(), q_target.x(), q_target.y(), q_target.z());
	// Eigen::Quaterniond q_current_eigen(q_current.w(), q_current.x(), q_current.y(), q_current.z());
	// Eigen::Vector3d target_eulerAngle = q_target_eigen.matrix().eulerAngles(2,1,0);
	// Eigen::Vector3d current_eulerAngle = q_current_eigen.matrix().eulerAngles(2,1,0);
	// std::cout << "target_quaternion: " << q_target.w() << " " << q_target.x() << " " << q_target.y() << " " << q_target.z() << std::endl;
	// std::cout << "current_quaternion: " << q_current.w() << " " << q_current.x() << " " << q_current.y() << " " << q_current.z() << std::endl;

	tf2::Quaternion q_diff = q_target - q_current;

	float diff = delta_x * delta_x + delta_y * delta_y + delta_z * delta_z
				+ q_diff.dot(q_diff);
	return diff;
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::Rate loop_rate(5);
	auto node = std::make_shared<OffboardControl>();
	while (rclcpp::ok()) {
		
		if (node->traj_updated) {
			for (auto point : node->trajectoryPtr->points) {
				int counter = 0;
				// std::cout << "current pose: " << node->current_pose_.pose.position.x << " "
				// 							  << node->current_pose_.pose.position.y << " "
				// 							  << node->current_pose_.pose.position.z << std::endl;


				while (cal_pose_diff(node->current_pose_, point) > node->threshold && counter <= 100) {
					counter++;
					// std::cout << "current pose: " << node->current_pose_.pose.position.x << " "
					// 						  << node->current_pose_.pose.position.y << " "
					// 						  << node->current_pose_.pose.position.z << std::endl;
					// std::cout << "Pose diff	" << cal_pose_diff(node->current_pose_, point) << std::endl;
					node->set_trajectory_setpoint(point);
					rclcpp::spin_some(node);
					loop_rate.sleep();
				}
				if (counter > 100) {
					std::cout << "Path Failed" << std::endl;
					node->set_trajectory_setpoint(node->current_pose_.pose.position.x,
												node->current_pose_.pose.position.y,
												node->current_pose_.pose.position.z,
												0.0, 0.0, 0.0);
					break;
				}
				else {
					std::cout << "Path Finished" << std::endl;
				}
			}
			node->traj_updated = false;
			
		}
		rclcpp::spin_some(node);
	}
	rclcpp::shutdown();
	return 0;
}
