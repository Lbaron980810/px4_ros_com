#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <string>

#include <cmath>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

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
			this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>("/omnihex_traj", 1,
							[this](const trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr msg)
															   {
																   trajectoryPtr = msg;
																   traj_updated = true;
															   });
		pos_subscription_ = 
			this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("VehicleLocalPosition_PubSubTopic", 1, 
							[this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
															   {
																	current_position_.x = msg->x;
																	current_position_.y = -msg->y;
																	current_position_.z = -msg->z;
															   });
		offboard_setpoint_counter_ = 0;
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("Timesync_PubSubTopic", 10,
															   [this](const px4_msgs::msg::Timesync::UniquePtr msg)
															   {
																   timestamp_.store(msg->timestamp);
															   });
		offboard_setpoint_counter_ = 0;

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
			publish_trajectory_setpoint(1.0, 1.0, 1.0);
			
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
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_subscription_;
	rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_sub_;

	std::atomic<uint64_t> timestamp_; //!< common synced timestamped

	uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent

public:

	float threshold = 1.0;
	px4_msgs::msg::VehicleLocalPosition current_position_;
	trajectory_msgs::msg::MultiDOFJointTrajectory::SharedPtr trajectoryPtr;
	bool traj_updated = false;

	void publish_offboard_control_mode() const;
	void publish_trajectory_setpoint(float x, float y, float z, 
						float roll = 0.0, float pitch = 0.0, float yaw = 0.0);
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
								 float param2 = 0.0) const;
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
void OffboardControl::publish_trajectory_setpoint(float x, float y, float z, 
						float roll, float pitch, float yaw)
{
	TrajectorySetpoint msg{};
	// msg.timestamp = timestamp_.load();
	msg.x = x;
	msg.y = -y;
	msg.z = -z;

	// msg.x = target_pose_.transforms[0].translation.x;
	// msg.y = -target_pose_.transforms[0].translation.y;
	// msg.z = -target_pose_.transforms[0].translation.z;
	msg.roll = roll; // target_pose_.x;
	msg.pitch = pitch; // target_pose_.x;
	msg.yaw = yaw; // target_pose_.x;

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

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	auto node = std::make_shared<OffboardControl>();
	while (rclcpp::ok()) {
		if (node->traj_updated) {
			for (auto point : node->trajectoryPtr->points) {
				std::cout << "current position: " << node->current_position_.x << " "  
							<< node->current_position_.y << " " << node->current_position_.z << std::endl;
				std::cout << "target position: " << point.transforms[0].translation.x << " "  
						<< point.transforms[0].translation.y << " " << point.transforms[0].translation.z << std::endl;
				float delta_x = node->current_position_.x - point.transforms[0].translation.x;
				float delta_y = node->current_position_.y - point.transforms[0].translation.y;
				float delta_z = node->current_position_.z - point.transforms[0].translation.z;
				float delta = delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
				while (delta > node->threshold) {
					delta_x = node->current_position_.x - point.transforms[0].translation.x;
					delta_y = node->current_position_.y - point.transforms[0].translation.y;
					delta_z = node->current_position_.z - point.transforms[0].translation.z;
					delta = delta_x * delta_x + delta_y * delta_y + delta_z * delta_z;
					// std::cout << "delta: " << delta << std::endl;
					node->publish_offboard_control_mode();
					node->publish_trajectory_setpoint(point.transforms[0].translation.x, point.transforms[0].translation.y, point.transforms[0].translation.z);
					rclcpp::spin_some(node);
					// rclcpp::sleep_for(std::chrono::seconds(1));
				}
			}
			node->traj_updated = false;
			std::cout << "Path Finished" << std::endl;
		}
		rclcpp::spin_some(node);
	}

	rclcpp::shutdown();
	return 0;
}
