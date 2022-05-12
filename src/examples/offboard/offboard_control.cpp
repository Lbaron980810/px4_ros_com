/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>

 * The TrajectorySetpoint message and the OFFBOARD mode in general are under an ongoing update.
 * Please refer to PR: https://github.com/PX4/PX4-Autopilot/pull/16739 for more info. 
 * As per PR: https://github.com/PX4/PX4-Autopilot/pull/17094, the format
 * of the TrajectorySetpoint message shall change.
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <string>

#include <cmath>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

void setAngularVel(double &wx, double &wy, double &wz,
                   const double &roll, const double &pitch,
                   const double &d_roll, const double &d_pitch, const double &d_yaw)
{
    double phi = roll;
    double theta = pitch;
    wx = d_roll - sin(theta) * d_yaw;
    wy = cos(phi) * d_pitch + cos(theta) * sin(phi) * d_yaw;
    wz = -sin(phi) * d_pitch + cos(theta) * cos(phi) * d_yaw;
}

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
        log_switch_publisher_ =
            this->create_publisher<std_msgs::msg::Bool>("L1MPC/LogSwitch", 10);
#else
		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("OffboardControlMode_PubSubTopic");
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("TrajectorySetpoint_PubSubTopic");
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("VehicleCommand_PubSubTopic");
#endif

		// get common timestamp
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
			publish_trajectory_setpoint();

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
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr log_switch_publisher_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;

	std::atomic<uint64_t> timestamp_; //!< common synced timestamped
	uint64_t t0_{};
	bool t0_set_{false};
    int lap_count_{0};

	uint64_t offboard_setpoint_counter_; //!< counter for the number of setpoints sent

	void publish_offboard_control_mode() const;
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
								 float param2 = 0.0) const;
    bool log_flag_{false};
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
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = true;
	msg.body_rate = true;

	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{
	if(offboard_setpoint_counter_ == 11 && (!t0_set_))
	{
		t0_ = timestamp_.load();
		t0_set_ = true;
	}
	
	TrajectorySetpoint msg{};
	uint64_t t = timestamp_.load();

    // circle trajectory
//	float theta = 0.3 * 1e-6 * (t - t0_);
//	msg.timestamp = t;
//	msg.x = 0 + 4 * sin(theta);
//	msg.y = 4 - 4 * cos(theta);
//	msg.z = -2.5 + sin(theta);
//	msg.roll = 0 + sin(theta);
//	msg.pitch = 0 + sin(theta);
//	msg.yaw = M_PI_2 + theta; // [-PI:PI]

    // 8 shape trajectory
    double T = 45, t_1 = T / (3 * M_PI + 4), t_2 = 3 * M_PI * T / (6 * M_PI + 8);
    double r = 0.6, h = 1.4, d_h = 0.3;
	double kroll = 6.0/9, kpitch = 1;
	double x_init = 0, y_init = 0;

    double t_ = 1e-6 * (double)(t - t0_);
    if(t0_set_ && ((int)t_ / (int)T > lap_count_))
    {
        log_flag_ = true;
        std::cout << "write log status: " << log_flag_ << std::endl;
        std_msgs::msg::Bool log_switch_msg{};
        log_switch_msg.data = log_flag_;
        log_switch_publisher_->publish(log_switch_msg);
        lap_count_ = (int)t_ / (int)T;
		if(lap_count_ == 3)
			exit(0);
    }
    while(t_ > T)
        t_ -= T;
    
    msg.z = -h - d_h + d_h * cos(2 * M_PI / (2 * t_1 + t_2) * t_);
    msg.vz = -d_h * 2 * M_PI / (2 * t_1 + t_2) * sin(2 * M_PI / (2 * t_1 + t_2) * t_);

    msg.pitch = -msg.vz;
    double dPitch = 4 * pow(M_PI, 2) * d_h * cos(2 * M_PI * t_ / (2 * t_1 + t_2)) / pow((2 * t_1 + t_2), 2);
    double dRoll, dYaw;

    if(t_ >= 0 && t_ < t_1)
    {
        msg.x = 0;
        msg.vx = 0;

        msg.y = r / t_1 * t_;
        msg.vy = r / t_1;

        msg.roll = 0;
        dRoll = 0;

        msg.yaw = M_PI_2;
        dYaw = 0;
    }
    if(t_ >= t_1 && t_ < t_1 + t_2)
    {
        double omega = 3 * M_PI / (2 * t_2);

        msg.x = -r + r * cos(omega * (t_ - t_1));
        msg.vx = -r * omega * sin(omega*(t_ - t_1));

        msg.y = r + r * sin(omega * (t_ - t_1));
        msg.vy = r * omega * cos(omega * (t_ - t_1));

        msg.roll = M_PI / 4 - M_PI / 4 * cos(2 * M_PI / t_2 * (t_ - t_1));
        dRoll = M_PI / 4 * 2 * M_PI / t_2 * sin(2 * M_PI / t_2 * (t_ - t_1));

        msg.yaw = M_PI / 2 + omega * (t_ - t_1);
        dYaw = omega;
    }
    if(t_ >= t_1 + t_2 && t_ < 3 * t_1 + t_2)
    {
        msg.x = -r + r / t_1 * (t_ - t_1 - t_2);
        msg.vx = r / t_1;

        msg.y = 0;
        msg.vy = 0;

        msg.roll = 0;
        dRoll = 0;

        msg.yaw = 0;
        dYaw = 0;
    }
    if(t_ >= 3 * t_1 + t_2 && t_ < T - t_1)
    {
        double omega = 3 * M_PI / (2 * t_2);

        msg.x = r + r * sin(omega * (t_ - 3 * t_1 - t_2));
        msg.vx = r * omega * cos(omega * (t_ - 3 * t_1 - t_2));

        msg.y = -r + r * cos(omega * (t_ - 3 * t_1 - t_2));
        msg.vy = -r * omega * sin(omega * (t_ - 3 * t_1 - t_2));

        msg.roll = -M_PI / 4 + M_PI / 4 * cos(2 * M_PI / t_2 * (t_ - 3 * t_1 - t_2));
        dRoll = -M_PI / 4 * 2 * M_PI / t_2 * sin(2 * M_PI / t_2 * (t_ - 3 * t_1 - t_2));

        msg.yaw = -omega * (t_ - 3 * t_1 - t_2);
        dYaw = -omega;
    }
    if(t_ >= T - t_1 && t_ < T)
    {
        msg.x = 0;
        msg.vx = 0;

        msg.y = -r + r / t_1 * (t_ - (T - t_1));
        msg.vy = r / t_1;

        msg.roll = 0;
        dRoll = 0;

        msg.yaw = M_PI_2;
        dYaw = 0;
    }

    msg.roll *= kroll;
    dRoll *= kpitch;
	msg.x += x_init;
	msg.y += y_init;

    double wx, wy, wz;
    setAngularVel(wx, wy, wz, (double)msg.roll, (double)msg.pitch, dRoll, dPitch, dYaw);
    msg.rollspeed = wx;
    msg.pitchspeed = wy;
    msg.yawspeed = wz;

	while(msg.yaw > M_PI)
		msg.yaw -= M_PI * 2;
	while(msg.yaw < -M_PI)
		msg.yaw += M_PI * 2;

    if(t0_set_)
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
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
