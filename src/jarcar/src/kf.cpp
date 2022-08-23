#include "kf.hpp"
#include <atrix/matrix.h>

#include "geometry_msgs/msg/pose_stamped.hpp"
#pragma GCC diagnostic ignored "-Wunused-variable"
KF_Node::KF_Node(
    Matrix::Matrix<double> F,
    Matrix::Matrix<double> B,
    Matrix::Matrix<double> Q,
    Matrix::Matrix<double> H,
    Matrix::Matrix<double> R,
    Matrix::Matrix<double> K,
    Matrix::Matrix<double> u,
    Matrix::Matrix<double> x,
    Matrix::Matrix<double> z
)
 : Node("Kalman_Filter_Introduction_Node"),
   F(F),
   B(B),
   Q(Q),
   H(H),
   R(R),
   K(K),
   u(u),
   x(x),
   z(z)
 {
 }

 void KF_Node::predict() {

 }

 void KF_Node::update() {

 }

 void KF_Node::gps_callback(geometry_msgs::msg::PoseStamped::SharedPtr msg) const {
    double x_gps = msg->pose.position.x;
    double y_gps = msg->pose.position.y;
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
    std::cout << "[REAL ODOMETRY CALLBACK]" << x_real << " | " << y_real << std::endl;
 }

 int main(int argc, char** argv) {
    Matrix::Matrix<double> P = Matrix::zeros<double>(4, 4);

    Matrix::Matrix<double> F(1);
    Matrix::Matrix<double> B(1);
    Matrix::Matrix<double> Q(1);
    Matrix::Matrix<double> H(1);
    Matrix::Matrix<double> R(1);
    Matrix::Matrix<double> K(1);

    Matrix::Matrix<double> u(1);
    Matrix::Matrix<double> x(1); // state
    Matrix::Matrix<double> z(1);

    P(0, 0) = 0.1;
    P(1, 1) = 0.1;
    P(2, 2) = 0.1;
    P(3, 3) = 0.1;


    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KF_Node>(F, B, Q, H, R, K, u, x, z));
    rclcpp::shutdown();
 }