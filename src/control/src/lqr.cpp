#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "jarcar_msgs/msg/car_control.hpp"

#include "control/lqr.hpp"

#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-parameter"

#include <atrix/matrix.h>
#include <atrix/vector.h>
#include <atrix/linalg/algorithms.h>
#include <vector>

std::string ODOM_TOPIC           = "/jarcar/real_odometry";
std::string NEW_TRAJECTORY_TOPIC = "/control/lqr/new_trajectory";
std::string CAR_CONTROL_TOPIC    = "/jarcar/ackermann_control";

int ITERATION_NUMBER = 3000; // 400; // 6000;

double calculate_de (geometry_msgs::msg::PoseStamped start_point, 
                     geometry_msgs::msg::PoseStamped target_point,
                     geometry_msgs::msg::PoseStamped actual_point) {
    
    double a = target_point.pose.position.y - start_point.pose.position.y;
    double b = start_point.pose.position.x - target_point.pose.position.x;
    double c = (start_point.pose.position.y * target_point.pose.position.x) - (target_point.pose.position.y * start_point.pose.position.x);

    double dist = (a * actual_point.pose.position.x + b * actual_point.pose.position.y + c) / std::sqrt(a * a + b * b);

    std::cout << "[calculate_e_g_debug        dist]" << dist << std::endl;
    std::cout << "[calculate_e_g_debug start_point]" << start_point.pose.position.x << " | " << start_point.pose.position.y << std::endl;
    std::cout << "[calculate_e_g_debug target_point]" << target_point.pose.position.x << " | " << target_point.pose.position.y << std::endl;
    return dist;
}

double calculate_euclidian_distance(geometry_msgs::msg::PoseStamped actual_point,
                                    geometry_msgs::msg::PoseStamped target_point) {

    double dif_x = target_point.pose.position.x - actual_point.pose.position.x;
    double dif_y = target_point.pose.position.y - actual_point.pose.position.y;

    double dist = std::sqrt(dif_x * dif_x + dif_y * dif_y);

    return dist;
}

LQR::LQR(Matrix::Matrix<double> Q,
         Matrix::Matrix<double> R) 
: Node("LQR_Controller"), Q(Q), R(R)
{
    odom_sub                = create_subscription<geometry_msgs::msg::PoseStamped>(ODOM_TOPIC, 10, std::bind(&LQR::odom_callback, this, std::placeholders::_1));
    new_trajectory_sub      = create_subscription<nav_msgs::msg::Path>(NEW_TRAJECTORY_TOPIC, 10, std::bind(&LQR::new_trajectory_callback, this, std::placeholders::_1));

    car_control_pub         = create_publisher<jarcar_msgs::msg::CarControl>(CAR_CONTROL_TOPIC, 10);

    is_started              = false;
    path_index = 0;

    // just for testing
    std::vector<double> x_axis   = {0.0, 50.0, 60.0, 100.0, 140.0}; // {0.0, 50.0, 60.0, 70.0};
    std::vector<double> y_axis   = {0.0, 100.0, 200.0, 200.0, 100.0}; // {0.0, 100.0, 200.0, -100.0};
    std::vector<double> yaw_axis = {std::atan(2.0), std::atan(2.0), std::atan(10), std::atan(0), -std::atan(2.5)}; // {std::atan(2.0), std::atan(2.0), std::atan(10), -std::atan(30)};// {1.107148717794, 1.1071487, 1.4711276, 1.5374753};

    for (int i = 0; i < 5; i++) {
        auto p = geometry_msgs::msg::PoseStamped();
        p.pose.position.x    = x_axis[i];
        p.pose.position.y    = y_axis[i];
        p.pose.orientation.w = yaw_axis[i];
        path.poses.push_back(p);
    }
}

void LQR::odom_callback(geometry_msgs::msg::PoseStamped::SharedPtr odom_msg) {

    if (!is_started) {
        return;
    }

    double dt       = 0.1;
    double velocity = 5;
    double L        = 2;

    A = Matrix::identity<double>(3);
    A(0, 2) = -velocity * dt;
    A(1, 2) = +velocity * dt;

    B = Matrix::zeros<double>(3, 1);
    B(2, 0) = -(velocity / L) * dt;

    auto actual_point = *odom_msg;
    auto target_point = switch_next_point(actual_point);
    auto start_point  = path.poses[std::max(0, (int)path_index - 1)];

    auto K = solve();
    
    auto x_error = Matrix::zeros<double>(3, 1);

    double de      = calculate_de(start_point, target_point, actual_point);
    double se      = std::sqrt( std::pow( calculate_euclidian_distance(actual_point, target_point), 2) - std::pow( de, 2 ) );
    double theta_e = target_point.pose.orientation.w - actual_point.pose.orientation.w;

    x_error(0, 0) = se;
    x_error(1, 0) = de;
    x_error(2, 0) = theta_e;

    auto u_star = Matrix::dot(K, x_error);


    auto car_control_msg = jarcar_msgs::msg::CarControl();
    car_control_msg.velocity = velocity;
    car_control_msg.steering = u_star(0, 0);
    car_control_pub->publish(car_control_msg);

    std::cout << "[ LQR DEBUG ] : Velocity |" << car_control_msg.velocity << std::endl;
    std::cout << "[ LQR DEBUG ] : Steering |" << car_control_msg.steering << std::endl;

}

void LQR::new_trajectory_callback(nav_msgs::msg::Path::SharedPtr msg) {
    is_started = true;
    // path.poses = msg->poses;
}

Matrix::Matrix<double> LQR::solve() {

    Matrix::Matrix<double> K = Matrix::identity<double>(2);
    Matrix::Matrix<double> P = Q;

    for (int i = 0; i < ITERATION_NUMBER; i++) {
        auto AT_P = Matrix::dot( Matrix::transpoze(A), P );
        auto BT_P = Matrix::dot( Matrix::transpoze(B), P );

        P = Q + Matrix::dot( AT_P, A ) - Matrix::dot( AT_P, B, Matrix::inv( R + Matrix::dot( BT_P, B) ), BT_P, A );
    }

    auto AT_P = Matrix::dot( Matrix::transpoze(A), P );
    auto BT_P = Matrix::dot( Matrix::transpoze(B), P );

    K = Matrix::dot( Matrix::inv( R + Matrix::dot( BT_P, B) ), BT_P, A ) * -1;

    return K;
}

geometry_msgs::msg::PoseStamped LQR::find_the_nearest_way(geometry_msgs::msg::PoseStamped odom) {
    if (path.poses.empty()) {
        std::cout << "Surprize matafaka" << std::endl;
    }
    geometry_msgs::msg::PoseStamped closest_point;
    double closest_distance = 1e8;
    double yaw_of_closest_point = 0;
    for (size_t i = 0; i < path.poses.size() - 1; i++) {

        // auto or geometry::msgs::PoseStamped raises Segmentation Fault. Search about it
        const geometry_msgs::msg::PoseStamped& pose1 = path.poses[i];
        const geometry_msgs::msg::PoseStamped& pose2 = path.poses[i + 1];

        double pose1_x = path.poses[i].pose.position.x;
        double pose1_y = path.poses[i].pose.position.y;

        double pose2_x = path.poses[i + 1].pose.position.x;
        double pose2_y = path.poses[i + 1].pose.position.y;

        double dif_x2_x1 = pose2_x - pose1_x;
        double dif_y2_y1 = pose2_y - pose1_y;
        double dif_x1_x0 = pose1_x - odom.pose.position.x;
        double dif_y1_y0 = pose1_y - odom.pose.position.y;


        double distance = std::abs(dif_x2_x1 * dif_y1_y0 - dif_x1_x0 * dif_y2_y1) / std::sqrt(dif_x2_x1 * dif_x2_x1 + dif_y2_y1 * dif_y2_y1);

        if (distance <= closest_distance) {
            closest_distance = distance;
            closest_point = pose2;

            yaw_of_closest_point = std::atan2(dif_y2_y1, dif_x2_x1);
        }
    }

    closest_point.pose.orientation.w = yaw_of_closest_point;
    return closest_point;
}

geometry_msgs::msg::PoseStamped LQR::switch_next_point(geometry_msgs::msg::PoseStamped estimated_point) {
    auto current_point = path.poses[path_index];

    auto diff_x = estimated_point.pose.position.x - current_point.pose.position.x;
    auto diff_y = estimated_point.pose.position.y - current_point.pose.position.y;

    auto dist = std::sqrt( diff_x * diff_x + diff_y * diff_y );

    if (dist <= 0.5 && path_index < path.poses.size()) {
        path_index++;
    }

    return path.poses[path_index];
}

int main(int argc, char** argv) {

    Matrix::Matrix<double> Q = Matrix::identity<double>(3);
    Matrix::Matrix<double> R = Matrix::identity<double>(1);

    R(0, 0) = 0.001; // 100;

    Q(0, 0) = 1; // 5000;
    Q(1, 1) = 10000000; // 5000;
    Q(2, 2) = 10000000; // 5000;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LQR>(Q, R));
    rclcpp::shutdown();
}