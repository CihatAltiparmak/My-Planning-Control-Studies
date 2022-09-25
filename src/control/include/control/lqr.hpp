#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "jarcar_msgs/msg/car_control.hpp"

#include <atrix/matrix.h>
#include <atrix/vector.h>

class LQR : public rclcpp::Node {
    public:
        LQR(Matrix::Matrix<double>,
            Matrix::Matrix<double>);

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr new_trajectory_sub;

        rclcpp::Publisher<jarcar_msgs::msg::CarControl>::SharedPtr car_control_pub;
    private:
        Matrix::Matrix<double> A;
        Matrix::Matrix<double> B;

        Matrix::Matrix<double> Q;
        Matrix::Matrix<double> R;

        nav_msgs::msg::Path path;
        bool is_started;
        size_t path_index;

        void new_trajectory_callback(nav_msgs::msg::Path::SharedPtr);
        void odom_callback(geometry_msgs::msg::PoseStamped::SharedPtr);

        Matrix::Matrix<double> solve();
        geometry_msgs::msg::PoseStamped find_the_nearest_way(geometry_msgs::msg::PoseStamped);

        geometry_msgs::msg::PoseStamped switch_next_point(geometry_msgs::msg::PoseStamped);

};