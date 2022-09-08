#include <functional>
#include <chrono>
#include <memory>
#include <string>

#include <atrix/matrix.h>
#include <atrix/linalg/algorithms.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64.hpp"

using namespace std::chrono_literals;

class KF_Node : public rclcpp::Node {

public:
    KF_Node(Matrix::Matrix<double>,
            Matrix::Matrix<double>,
            Matrix::Matrix<double>,
            Matrix::Matrix<double>,
            Matrix::Matrix<double>,
            Matrix::Matrix<double>);
    void predict();
    void update(Matrix::Vector<double>);

private:
    Matrix::Matrix<double> P;
    Matrix::Matrix<double> F;
    Matrix::Matrix<double> B;
    Matrix::Matrix<double> Q;
    Matrix::Matrix<double> H;
    Matrix::Matrix<double> R;
    Matrix::Matrix<double> K;

    Matrix::Matrix<double> u;
    Matrix::Matrix<double> x; // state
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr kalman_pub;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gps_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr imu_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr real_odometry_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr          steering_sub;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr            velocity_sub;
    

    void gps_callback(geometry_msgs::msg::PoseStamped::SharedPtr);
    void imu_callback(geometry_msgs::msg::PoseStamped::SharedPtr) const;
    void real_odometry_callback(geometry_msgs::msg::PoseStamped::SharedPtr) const;
    
    void steering_callback(std_msgs::msg::Float64::SharedPtr);
    void velocity_callback(std_msgs::msg::Int64::SharedPtr); 
};