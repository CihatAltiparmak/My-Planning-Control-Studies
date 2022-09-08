#include "kf.hpp"
#include <atrix/matrix.h>

#include <math.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#pragma GCC diagnostic ignored "-Wunused-variable"

std::string GPS_TOPIC = "/jarcar/gps";
std::string IMU_TOPIC = "/jarcar/imu";
std::string REAL_ODOMETRY_TOPIC = "/jarcar/real_odometry";
std::string STEERING_TOPIC = "/jarcar/steering";
std::string VELOCITY_TOPIC = "/jarcar/velocity";
std::string KALMAN_FILTER_TOPIC = "/jarcar/kalman_filtered_odom";

KF_Node::KF_Node(
    Matrix::Matrix<double> P,
    Matrix::Matrix<double> F,
    Matrix::Matrix<double> B,
    Matrix::Matrix<double> Q,
    Matrix::Matrix<double> H,
    Matrix::Matrix<double> R
    // Matrix::Matrix<double> z
): Node("Kalman_Filter_Introduction_Node"),
    P(P), F(F), B(B), Q(Q), H(H), R(R)
{
    x = Matrix::zeros<double>(2, 1);
    u = Matrix::zeros<double>(2, 1);

    kalman_pub        = create_publisher<geometry_msgs::msg::PoseStamped>(KALMAN_FILTER_TOPIC, 10);

    gps_sub           = create_subscription<geometry_msgs::msg::PoseStamped>(GPS_TOPIC, 10, std::bind(&KF_Node::gps_callback, this, std::placeholders::_1));
    imu_sub           = create_subscription<geometry_msgs::msg::PoseStamped>(IMU_TOPIC, 10, std::bind(&KF_Node::imu_callback, this, std::placeholders::_1));
    real_odometry_sub = create_subscription<geometry_msgs::msg::PoseStamped>(REAL_ODOMETRY_TOPIC, 10, std::bind(&KF_Node::real_odometry_callback, this, std::placeholders::_1));

    steering_sub      = create_subscription<std_msgs::msg::Float64>(STEERING_TOPIC, 10, std::bind(&KF_Node::steering_callback, this, std::placeholders::_1));
    velocity_sub      = create_subscription<std_msgs::msg::Int64>(VELOCITY_TOPIC, 10, std::bind(&KF_Node::velocity_callback, this, std::placeholders::_1));

}

void KF_Node::predict() {
    double dt  = 0.1;
    double yaw = x(2, 0);
    double L = 2;

    B(0, 1) = B(1, 1) = B(2, 0) = 0.0;
    B(0, 0) = cos(yaw) * dt;
    B(1, 0) = sin(yaw) * dt;
    B(2, 1) = dt / L;

    x = Matrix::dot(F, x) + Matrix::dot(B, u);
    P = Matrix::dot(F, Matrix::dot(P, Matrix::transpoze(F))) + Q;
}

void KF_Node::update(Matrix::Vector<double> z) {
    auto S = Matrix::inv( Matrix::dot( H, Matrix::dot(P, Matrix::transpoze(H) ) ) + R);
    K = Matrix::dot( P, Matrix::dot( Matrix::transpoze(H), S) );

    x += Matrix::dot( K, z - Matrix::dot( H, x ) );
    P -= Matrix::dot( K, Matrix::dot( H, P ) );

    auto msg = geometry_msgs::msg::PoseStamped();
    msg.pose.position.x = x(0, 0);
    msg.pose.position.y = x(1, 0);

    kalman_pub->publish(msg);
}

void KF_Node::gps_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    double x_gps = msg->pose.position.x;
    double y_gps = msg->pose.position.y;

    Matrix::Vector<double> z(2, 1);

    z(0, 0) = x_gps;
    z(1, 0) = y_gps;

    this->predict();
    this->update(z);
    std::cout << "[GPS CALLBACK]" << x_gps << " | " << y_gps << std::endl;
}

void KF_Node::imu_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg) const {
    double x_imu = msg->pose.position.x;
    double y_imu = msg->pose.position.y;
    std::cout << "[IMU CALLBACK]" << x_imu << " | " << y_imu << std::endl;
}

void KF_Node::real_odometry_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg) const {
    double x_real = msg->pose.position.x;
    double y_real = msg->pose.position.y;
    std::cout << "[REAL ODOMETRY CALLBACK][" << x_real << " | " << y_real << "]" << std::endl;
    std::cout << "[KALMAN FILTER CALLBACK][" << x(0, 0) << "|" << x(1, 0) << "]" << std::endl;
}

void KF_Node::steering_callback(std_msgs::msg::Float64::SharedPtr msg) {
    double steering = msg->data;
    u(0, 0) = steering;
}

void KF_Node::velocity_callback(std_msgs::msg::Int64::SharedPtr msg) {
    double velocity = msg->data;
    u(1, 0) = velocity;
}


int main(int argc, char** argv) {
    Matrix::Matrix<double> P = Matrix::identity<double>(3) * 0.1;
    Matrix::Matrix<double> F = Matrix::identity<double>(3);
    Matrix::Matrix<double> B = Matrix::zeros<double>(3, 2);
    Matrix::Matrix<double> Q = Matrix::identity<double>(3) * 0.1;
    Matrix::Matrix<double> H = Matrix::identity<double>(3);
    Matrix::Matrix<double> R = Matrix::identity<double>(3) * 0.1;
    // Matrix::Matrix<double> u(2);
    // Matrix::Matrix<double> x(2); // state
    // Matrix::Matrix<double> z(1);
   
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KF_Node>(P, F, B, Q, H, R));
    rclcpp::shutdown();
}