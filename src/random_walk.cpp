/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 * 
 *  Copyright (c) Rui P. Rocha, 2025
 »
 *  All rights reserved.
 * 
 *  Version 1.0.0, Mar. 26, 2025
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>		// laser scans
#include <geometry_msgs/msg/twist.hpp>			// twist messages
#include <geometry_msgs/msg/twist_stamped.hpp>	// twist messages with timestamp
#include <stdlib.h>
#include <chrono>
#include <limits>

#define GO_STRAIGHT 	0
#define ADJUST_YAW		1
#define CHECK_SAFETY	2


class CRandomWalk : public rclcpp::Node {
private:
	// node parameters
	double linear_speed;
	double angular_speed;
	double safe_distance_th;
	double critical_distance_th;
	double alpha;
	int max_delta_yaw;
	double lidar_yaw_offset;
	bool stamped;

	// other attributes
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_twist_pub;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
	unsigned char state;	// 0:go straight; 1:adjust yaw
	bool rotate_anti_clockwise;
	rclcpp::Time end_time;

	// private methods
	double min_distance_to_obstacles(const sensor_msgs::msg::LaserScan &scan,
		double &angle_min_distance);
	void publish_twist(const geometry_msgs::msg::Twist &);
	void scan_callback(const sensor_msgs::msg::LaserScan &);
	rclcpp::Duration motionTimePredict(double) const;

	// static methods
	static rclcpp::Duration convertTime(double);
	static double rndAngle(int);

public:
	CRandomWalk();
};


CRandomWalk::CRandomWalk(): Node("random_walk"){
	// declare node parameters
	this->declare_parameter("linear_speed",			1.0);
	this->declare_parameter("angular_speed",		1.0);
	this->declare_parameter("safe_distance_th",		1.0);
	this->declare_parameter("critical_distance_th",	0.25);
	this->declare_parameter("alpha",				0.25);
	this->declare_parameter("max_delta_yaw",		180);
	this->declare_parameter("lidar_yaw_offset",		0.0);
	this->declare_parameter("stamped",				false);

	// get parameter values
	linear_speed =			this->get_parameter("linear_speed").as_double();
	angular_speed =			this->get_parameter("angular_speed").as_double();
	safe_distance_th =		this->get_parameter("safe_distance_th").as_double();
	critical_distance_th =	this->get_parameter("critical_distance_th").as_double();
	alpha =					this->get_parameter("alpha").as_double();
	max_delta_yaw =			this->get_parameter("max_delta_yaw").as_int();
	lidar_yaw_offset = 		this->get_parameter("lidar_yaw_offset").as_double();
	stamped =				this->get_parameter("stamped").as_bool();

	if (alpha > 1.0) alpha = 1.0;
	if (alpha < 0.0) alpha = 0.0;

	while (max_delta_yaw >= 180) max_delta_yaw -= 360;
	while (max_delta_yaw < -180) max_delta_yaw += 360;
	if (max_delta_yaw < 0.0) max_delta_yaw = -max_delta_yaw;

	RCLCPP_INFO_STREAM(this->get_logger(),
		"linear_speed = " << linear_speed << ", angular_speed = " << angular_speed <<
		", safe_distance_th = " << safe_distance_th << ", critical_distance_th = " << critical_distance_th <<
		", alpha = " << alpha << ", max_delta_yaw = " << max_delta_yaw <<
		", lidar_yaw_offset = " << lidar_yaw_offset << ", stamped = " << (stamped?"True":"False")
	);
	lidar_yaw_offset *= M_PI/180.0; // convert offset to radians

	// subscriber
	laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan",
		rclcpp::SensorDataQoS(),
		std::bind(&CRandomWalk::scan_callback, this, std::placeholders::_1));

	// publisher
	if (stamped)
 		stamped_twist_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 1);
 	else
 		twist_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

 	state = GO_STRAIGHT;

 	// initialize random number generator
	using namespace std::chrono;
	system_clock::time_point tp = system_clock::now();
  	system_clock::duration dtn = tp.time_since_epoch();
  	unsigned int seed = dtn.count() % 10000000000; // get 10 less significant decimal digits
  	RCLCPP_INFO_STREAM(this->get_logger(),"random number generator seed set to " << seed);
  	srand(seed);  	

 	publish_twist(geometry_msgs::msg::Twist()); // publish zero twist message
}


double CRandomWalk::min_distance_to_obstacles(const sensor_msgs::msg::LaserScan &scan,
		double &angle_min_distance){
	// consider only readings between angles -PI/2 and +PI/2
	double angle_1st_reading = scan.angle_min + lidar_yaw_offset;
	double k_floating = (-M_PI/2.0 - angle_1st_reading) / scan.angle_increment;
	long unsigned int k = (unsigned long int) k_floating;
	if ( ((double) k) != k_floating) ++k;
	double angle = angle_1st_reading + k * scan.angle_increment;
	double min_dist = std::numeric_limits<double>::max();
	angle_min_distance = angle;
	double previous_range = min_dist;

	// constants to avoid outliers in laser scan
	const double max_range_variation_within_cluster = 0.1;
	const double epsilon = 0.05;

	for (; k < scan.ranges.size() && angle <= M_PI/2.0; ++k){
		if (!std::isnan(scan.ranges[k]) && scan.ranges[k] > epsilon &&
			fabs(previous_range - scan.ranges[k]) < max_range_variation_within_cluster &&
			min_dist > scan.ranges[k])
		{
			min_dist = scan.ranges[k];
			angle_min_distance = angle;
		}
		previous_range = scan.ranges[k];
		angle += scan.angle_increment;
	}

	RCLCPP_DEBUG_STREAM(this->get_logger(),
		"min_dist = " << min_dist << ", angle_min_distance = " << angle_min_distance
		);
	return min_dist;
}


void CRandomWalk::scan_callback(const sensor_msgs::msg::LaserScan &scan){
	double angle_min_distance;
	double min_dist;

	if (state == GO_STRAIGHT || state == CHECK_SAFETY)
		min_dist = min_distance_to_obstacles(scan, angle_min_distance);

	geometry_msgs::msg::Twist twist;

	// check state transitions
	switch (state){
		case GO_STRAIGHT:
			if (min_dist <= critical_distance_th) {
				state = ADJUST_YAW;
				rotate_anti_clockwise = (angle_min_distance <= 0.0);
				twist.angular.z = (rotate_anti_clockwise? angular_speed:-angular_speed);
				publish_twist(twist);
				end_time = this->get_clock()->now() + 
					motionTimePredict(rndAngle(max_delta_yaw));
			}
			break;
		case ADJUST_YAW:
			if (end_time <= this->get_clock()->now()) {
				state = CHECK_SAFETY;
				publish_twist(twist); // zero twist
			}
			break;
		case CHECK_SAFETY:
			if (min_dist > critical_distance_th) state = GO_STRAIGHT;
			else {
				state = ADJUST_YAW;
				twist.angular.z = (rotate_anti_clockwise? angular_speed:-angular_speed);
				publish_twist(twist);
				end_time = this->get_clock()->now() + 
					motionTimePredict(rndAngle(max_delta_yaw));
			}
	};

	// perform actions
	if (state == GO_STRAIGHT) {
		if (min_dist >= safe_distance_th) twist.linear.x = linear_speed;
		else if (min_dist <= critical_distance_th) twist.linear.x = 0.0;
		else twist.linear.x =  linear_speed / (safe_distance_th - critical_distance_th) *
				( (1.0 - alpha)*min_dist + alpha*safe_distance_th - critical_distance_th );
		publish_twist(twist);
	}
}


void CRandomWalk::publish_twist(const geometry_msgs::msg::Twist &cmd_vel){

	if (stamped){
		geometry_msgs::msg::TwistStamped cmd_vel_stamped;

		cmd_vel_stamped.header.stamp = this->get_clock()->now();
		cmd_vel_stamped.header.frame_id = "base_link";
		cmd_vel_stamped.twist = cmd_vel;
		stamped_twist_pub->publish(cmd_vel_stamped);
	}
	else twist_pub->publish(cmd_vel);
}


rclcpp::Duration CRandomWalk::convertTime(double secs){
	int s = (int) secs;
	unsigned int ns = (unsigned int) ( (secs - ((double) s)) * 1e9);
	return rclcpp::Duration(s, ns);
}


#define ANGACCEL 45.0
rclcpp::Duration CRandomWalk::motionTimePredict(double angle) const{
	double t;
	double tacc = angular_speed / ANGACCEL; // acceleration time to maximum angular speed
	double alpha_acc = 0.5 * ANGACCEL * pow(tacc, 2.0);
	angle = fabs(angle); // the following calculations assume that angle >= 0.0
	if (alpha_acc < angle){
		double t2 = (angle - alpha_acc) / angular_speed;
		t = tacc + t2;
	}
	else t = sqrt(angle * 0.5 / ANGACCEL);

	RCLCPP_DEBUG_STREAM(this->get_logger(), "Predicted time to rotate " << angle <<
		" radians is " << t << " s");
	return CRandomWalk::convertTime(t);
}


double CRandomWalk::rndAngle(int max_angle){
	int deg = (rand() % max_angle) + 1; // random angle between 1 and max_angle, max_angle >= 0.0
	return ((double) deg) / 180.0 * M_PI;
}


int main(int argc, char** argv){
	// setup ROS node
	rclcpp::init(argc, argv);
	std::shared_ptr<rclcpp::Node> node = std::make_shared<CRandomWalk>();

	// spin without controlling explicitly in main() the execution loop
	rclcpp::spin(node->get_node_base_interface()); //trigger callbacks and prevents exiting
	rclcpp::shutdown();
	return(0);
}
