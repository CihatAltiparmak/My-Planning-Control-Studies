#include <functional>
#include <chrono>
#include <memory>
#include <string>

#include <atrix/matrix.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class KF_Node : public rclcpp::Node {

public:
    KF_Node(Matrix::Matrix<double>,
            Matrix::Matrix<double>,
            Matrix::Matrix<double>,
            Matrix::Matrix<double>,
            Matrix::Matrix<double>,
            Matrix::Matrix<double>,
            Matrix::Matrix<double>,
            Matrix::Matrix<double>,
            Matrix::Matrix<double>);
    void predict();
    void update();

private:
    Matrix::Matrix<double> F;
    Matrix::Matrix<double> B;
    Matrix::Matrix<double> Q;
    Matrix::Matrix<double> H;
    Matrix::Matrix<double> R;
    Matrix::Matrix<double> K;

    Matrix::Matrix<double> u;
    Matrix::Matrix<double> x; // state
    Matrix::Matrix<double> z;
    
    void gps_callback(geometry_msgs::msg::PoseStamped::SharedPtr) const;
    void imu_callback(geometry_msgs::msg::PoseStamped::SharedPtr) const;
    void real_odometry_callback(geometry_msgs::msg::PoseStamped::SharedPtr) const;
};