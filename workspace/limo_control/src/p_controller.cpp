#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class PController : public rclcpp::Node
{
public:
    PController() : Node("p_controller")
    {
        // Initialize parameters
        this->declare_parameter("kp_linear", 1.0);
        this->declare_parameter("kp_angular", 2.0);
        this->declare_parameter("kp_angular_trim", 0.5);  // Smaller gain for orientation trimming
        this->declare_parameter("max_linear_vel", 0.5);
        this->declare_parameter("max_angular_vel", 1.0);
        this->declare_parameter("goal_x", 2.0);
        this->declare_parameter("goal_y", 2.0);
        this->declare_parameter("goal_tolerance", 0.1);
        this->declare_parameter("angular_tolerance", 0.05);  // Threshold to switch to drive phase
        
        // Get parameters
        kp_linear_ = this->get_parameter("kp_linear").as_double();
        kp_angular_ = this->get_parameter("kp_angular").as_double();
        kp_angular_trim_ = this->get_parameter("kp_angular_trim").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        goal_x_ = this->get_parameter("goal_x").as_double();
        goal_y_ = this->get_parameter("goal_y").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        angular_tolerance_ = this->get_parameter("angular_tolerance").as_double();
        
        // Create publishers and subscribers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PController::odomCallback, this, std::placeholders::_1));
        
        // Create timer for control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&PController::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "P Controller initialized with staged approach");
        RCLCPP_INFO(this->get_logger(), "Goal: (%.2f, %.2f)", goal_x_, goal_y_);
        RCLCPP_INFO(this->get_logger(), "Kp_linear: %.2f, Kp_angular: %.2f, Kp_angular_trim: %.2f", 
                    kp_linear_, kp_angular_, kp_angular_trim_);
        RCLCPP_INFO(this->get_logger(), "Angular tolerance: %.3f rad (%.1f deg)", 
                    angular_tolerance_, angular_tolerance_ * 180.0 / M_PI);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        // Extract yaw from quaternion
        double x = msg->pose.pose.orientation.x;
        double y = msg->pose.pose.orientation.y;
        double z = msg->pose.pose.orientation.z;
        double w = msg->pose.pose.orientation.w;
        
        current_yaw_ = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        has_odom_ = true;
    }
    
    void controlLoop()
    {
        if (!has_odom_) {
            return;
        }
        
        // Calculate errors
        double error_x = goal_x_ - current_x_;
        double error_y = goal_y_ - current_y_;
        double distance_error = std::sqrt(error_x * error_x + error_y * error_y);
        double angle_to_goal = std::atan2(error_y, error_x);
        double angle_error = angle_to_goal - current_yaw_;
        
        // Normalize angle error to [-pi, pi]
        while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI) angle_error += 2.0 * M_PI;
        
        // Check if goal is reached
        if (distance_error < goal_tolerance_) {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd_vel);
            
            if (!goal_reached_) {
                RCLCPP_INFO(this->get_logger(), "Goal reached! Distance error: %.3f", distance_error);
                goal_reached_ = true;
            }
            return;
        }
        
        geometry_msgs::msg::Twist cmd_vel;
        double linear_vel = 0.0;
        double angular_vel = 0.0;
        
        // Stage 1: Turn-in-place until aligned with goal
        if (std::abs(angle_error) > angular_tolerance_) {
            // Only rotate, no linear motion
            angular_vel = kp_angular_ * angle_error;
            linear_vel = 0.0;
            
            // Apply angular velocity limit
            angular_vel = std::max(-max_angular_vel_, std::min(max_angular_vel_, angular_vel));
            
            // Log turn-in-place phase
            if (!in_drive_phase_) {
                RCLCPP_DEBUG(this->get_logger(), 
                    "TURN-IN-PLACE: Angle error: %.3f rad (%.1f deg), Angular vel: %.3f",
                    angle_error, angle_error * 180.0 / M_PI, angular_vel);
            }
            
            in_drive_phase_ = false;
        }
        // Stage 2: Drive to goal with orientation trimming
        else {
            // Apply linear velocity for distance
            linear_vel = kp_linear_ * distance_error;
            
            // Apply small angular correction for orientation trimming
            angular_vel = kp_angular_trim_ * angle_error;
            
            // Apply velocity limits
            linear_vel = std::max(-max_linear_vel_, std::min(max_linear_vel_, linear_vel));
            angular_vel = std::max(-max_angular_vel_, std::min(max_angular_vel_, angular_vel));
            
            // Log transition to drive phase
            if (!in_drive_phase_) {
                RCLCPP_INFO(this->get_logger(), 
                    "Switching to DRIVE-TO-GOAL phase (angle error: %.3f rad, %.1f deg)",
                    angle_error, angle_error * 180.0 / M_PI);
                in_drive_phase_ = true;
            }
            
            RCLCPP_DEBUG(this->get_logger(), 
                "DRIVE-TO-GOAL: Dist: %.3f, Angle: %.3f, Vel: (%.3f, %.3f)",
                distance_error, angle_error, linear_vel, angular_vel);
        }
        
        // Publish velocity command
        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;
        cmd_vel_pub_->publish(cmd_vel);
    }
    
    // Parameters
    double kp_linear_, kp_angular_, kp_angular_trim_;
    double max_linear_vel_, max_angular_vel_;
    double goal_x_, goal_y_;
    double goal_tolerance_, angular_tolerance_;
    
    // State variables
    double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0;
    bool has_odom_ = false;
    bool goal_reached_ = false;
    bool in_drive_phase_ = false;
    
    // ROS2 objects
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PController>());
    rclcpp::shutdown();
    return 0;
}