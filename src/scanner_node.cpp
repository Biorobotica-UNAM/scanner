#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <algorithm>

#include "std_msgs/msg/float64_multi_array.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "actionlib_msgs/msg/goal_status.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"

class HeadBrigdeNode : public rclcpp::Node
{
public:
  HeadBrigdeNode() : Node("head_control_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
  {

    // Declare parameters
    this->declare_parameter<bool>("use_takeshi", false);
    bool use_takeshi = this->get_parameter("use_takeshi").as_bool();


    this->declare_parameter<std::string>( "head_goal_topic",          "/hardware/head/goal_pose");
    this->declare_parameter<std::string>( "head_current_pose_topic",  "/hardware/head/current_pose");
    //this->declare_parameter<std::string>( "head_goal_reached_topic",  "/hardware/head/goal_reached");

    this->declare_parameter<std::string>("base_frame", "base_link");
    if (!use_takeshi) {
        this->declare_parameter<std::string>("camera_frame",                "camera_link");
        this->declare_parameter<std::string>("neck_frame",                  "neck_link");
    } else {
        this->declare_parameter<std::string>("camera_frame",                "head_rgbd_sensor_rgb_frame");
        this->declare_parameter<std::string>("neck_frame",                  "torso_lift_link");
    }

    this->declare_parameter<double>("pan_min", -1.57);
    this->declare_parameter<double>("pan_max", 1.57);
    this->declare_parameter<double>("tilt_min", -0.9);
    this->declare_parameter<double>("tilt_max", 0.47);
    this->declare_parameter<double>("dead_zone_threshold", 0.05);

    head_goal_topic_ = this->get_parameter("head_goal_topic").as_string();
    head_cp_topic_ = this->get_parameter("head_current_pose_topic").as_string();

    camera_frame_ = this->get_parameter("camera_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    neck_frame_ = this->get_parameter("neck_frame").as_string();

    pan_min = static_cast<float>(this->get_parameter("pan_min").as_double());
    pan_max = static_cast<float>(this->get_parameter("pan_max").as_double());
    tilt_min = static_cast<float>(this->get_parameter("tilt_min").as_double());
    tilt_max = static_cast<float>(this->get_parameter("tilt_max").as_double());
    dead_zone_threshold = static_cast<float>(this->get_parameter("dead_zone_threshold").as_double());
    // Service Client
    
    // Subscribers
    head_cp_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        head_cp_topic_, 10,
        std::bind(&HeadBrigdeNode::headCurrentPoseCallback, this, std::placeholders::_1));
    
    look_at_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/hardware/head/look_at", 1,
        std::bind(&HeadBrigdeNode::lookAtCallback, this, std::placeholders::_1));

    nav_status_subscriber_ = this->create_subscription<actionlib_msgs::msg::GoalStatus>(
        "/navigation/status", 1,
        std::bind(&HeadBrigdeNode::navigationStatusCallback, this, std::placeholders::_1));

        
        // Publishers
    head_goal_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        head_goal_topic_, 10);
    
    // Timers

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(40), std::bind(&HeadBrigdeNode::control_cycle, this));
            
    RCLCPP_INFO(this->get_logger(), "HEAD CONTROL NODE BRIDGE BY RUSANROD HAS BEEN STARTED.");
}
private:
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr head_cp_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr look_at_subscriber_;
    rclcpp::Subscription<actionlib_msgs::msg::GoalStatus>::SharedPtr nav_status_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr head_goal_publisher_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Parameters
    std::string head_goal_topic_;
    std::string head_cp_topic_;

    std::string camera_frame_;
    std::string base_frame_;
    std::string neck_frame_;
    
    geometry_msgs::msg::PointStamped::SharedPtr latest_target_point_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // System variables
    float pan_min, pan_max, tilt_min, tilt_max, dead_zone_threshold;

    float current_pan = 0.0;
    float current_tilt = 0.0;
    float alpha = 0.85;

    bool new_target_ = false;

    // Utility functions

    std::vector<float> clampHeadPose(float goalPan, float goalTilt)
    {
        // Pan limits
        float clampedPan = std::clamp(goalPan, pan_min, pan_max);
        float clampedTilt = std::clamp(goalTilt, tilt_min, tilt_max);

        return {clampedPan, clampedTilt};
    }

    // Callbacks
    void headCurrentPoseCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Current pose message has less than 2 elements.");
            return;
        }

        current_pan = static_cast<float>(msg->data[0]);
        current_tilt = static_cast<float>(msg->data[1]);
    }

    void lookAtCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg){
        latest_target_point_ = msg;
        new_target_ = true;
    }

    void control_cycle() {
        if (!new_target_ || !latest_target_point_) return;

        new_target_ = false;  // Reset flag

        try {
            // 1. Transformar el punto al frame fijo del cuello (torso_lift_link o similar)
            geometry_msgs::msg::PointStamped p_neck;
            tf_buffer_.transform(*latest_target_point_, p_neck, neck_frame_, tf2::durationFromSec(0.01));

            // 2. Obtener la posición de la CÁMARA respecto al cuello (OFFSET estático)
            // Esto nos dice dónde está el "ojo" respecto al eje de rotación.
            geometry_msgs::msg::TransformStamped cam_tf = 
                tf_buffer_.lookupTransform(neck_frame_, camera_frame_, tf2::TimePointZero);
            
            double off_x = cam_tf.transform.translation.x;
            double off_y = cam_tf.transform.translation.y;
            double off_z = cam_tf.transform.translation.z;

            // 3. El punto relativo a la cámara, pero expresado en los ejes del cuello
            // IMPORTANTE: Aquí está el truco. Queremos que el vector Cámara -> Punto 
            // se alinee con el eje de visión.
            double dx = p_neck.point.x - off_x;
            double dy = p_neck.point.y - off_y;
            double dz = p_neck.point.z - off_z;

            // 4. Calcular PAN (Rotación en Z)
            // Si dy es positivo (izquierda), atan2 nos da un ángulo positivo (izquierda)
            double pan_target = std::atan2(dy, dx);

            // 5. Calcular TILT (Rotación en Y)
            // Distancia en el plano horizontal (suelo)
            double dist_horizontal = std::sqrt(dx * dx + dy * dy);
            // dz positivo es arriba -> atan2 positivo es arriba.
            double tilt_target = std::atan2(dz, dist_horizontal);

            // cambio 
            //double pan_diff = std::abs(pan_target - current_pan);
            //double tilt_diff = std::abs(tilt_target - current_tilt);

            //if (pan_diff > dead_zone_threshold || tilt_diff > dead_zone_threshold){
                float smooth_pan = (alpha * pan_target) + ((1.0 - alpha) * current_pan);
                float smooth_tilt = (alpha * tilt_target) + ((1.0 - alpha) * current_tilt);
            

            // 6. Límites y publicación
            //cambiox2
                auto clamped = clampHeadPose(smooth_pan, smooth_tilt);
            //auto clamped = clampHeadPose(static_cast<float>(pan_target), static_cast<float>(tilt_target));
            
                std_msgs::msg::Float64MultiArray head_goal_msg;
                head_goal_msg.data = {clamped[0], clamped[1]};
                head_goal_publisher_->publish(head_goal_msg);
            


        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF wait: %s", ex.what());
        }
    }

    void navigationStatusCallback(const actionlib_msgs::msg::GoalStatus::SharedPtr msg)
    {
        // Check if the navigation goal has been reached
        if (msg->status == actionlib_msgs::msg::GoalStatus::SUCCEEDED) {
            std_msgs::msg::Float64MultiArray head_goal_msg;
            head_goal_msg.data = {0.0, 0.0}; // Reset to default pose
            head_goal_publisher_->publish(head_goal_msg);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeadBrigdeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
