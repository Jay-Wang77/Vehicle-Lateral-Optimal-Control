#include "carla_shenlan_lqr_pid_controller/lqr_controller.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include "math.h"

using namespace std;

namespace shenlan {
namespace control {

LqrController::LqrController() {}

LqrController::~LqrController() {}

// lqr configuration
void LqrController::LoadControlConf() {
    ts_ = 0.01;    // time step

    cf_ = 155494.663;                              // cornering stiffness of front tire.
    cr_ = 155494.663;                              // cornering stiffness of rear tire.
    wheelbase_ = 2.852;                            
    steer_ratio_ = 16;                             // Ratio coefficient between steering wheel angle and tire rotation angle
    steer_single_direction_max_degree_ = 470.0;    // max steering angle

    const double mass_fl = 1845.0/4;                     // mass of the front left suspension
    const double mass_fr = 1845.0/4;
    const double mass_rl = 1845.0/4;
    const double mass_rr = 1845.0/4;
    const double mass_front = mass_fl + mass_fr;
    const double mass_rear = mass_rl + mass_rr;
    mass_ = mass_front + mass_rear;

    lf_ = wheelbase_ * (1.0 - mass_front / mass_);    // the front wheel to the center point
    lr_ = wheelbase_ * (1.0 - mass_rear / mass_);

    // moment of inertia
    iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

    lqr_eps_ = 0.01;              // LQR iterative solution accuracy
    lqr_max_iteration_ = 1500;    // number of iterations

    return;
}

// Initialize controller
void LqrController::Init() {
    // Matrix init operations.
    const int matrix_size = basic_state_size_;
    matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    /*
    A matrix (Gear Drive)
    [0.0,                             1.0,                           0.0,                                            0.0;
     0.0,          (-(c_f + c_r) / m) / v,               (c_f + c_r) / m,                (l_r * c_r - l_f * c_f) / m / v;
     0.0,                             0.0,                           0.0,                                            1.0;
     0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z, (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */
    // Initialize the constant items of the A matrix
    matrix_a_(0, 1) = 1.0;
    matrix_a_(1, 2) = (cf_ + cr_) / mass_;
    matrix_a_(2, 3) = 1.0;
    matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
    // Initialize the non-constant items of the A matrix
    matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);
    matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
    matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
    matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
    matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

    /*
    b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
    */
    // Initialize B matrix
    matrix_b_ = Matrix::Zero(basic_state_size_, 1);
    matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
    matrix_b_(1, 0) = cf_ / mass_;
    matrix_b_(3, 0) = lf_ * cf_ / iz_;
    matrix_bd_ = matrix_b_ * ts_;

    // state matrix
    matrix_state_ = Matrix::Zero(matrix_size, 1);
    // feedback matrix
    matrix_k_ = Matrix::Zero(1, matrix_size);
    // param. R, weight of input
    matrix_r_ = Matrix::Identity(1, 1);
    matrix_r_(0, 0) = 10;
    // param. Q, weight of state
    matrix_q_ = Matrix::Zero(matrix_size, matrix_size);

    // int q_param_size = 4;
    matrix_q_(0, 0) = 2;    // lateral_error
    matrix_q_(1, 1) = 1;    // lateral_error_rate
    matrix_q_(2, 2) = 0.1;    // heading_error
    matrix_q_(3, 3) = 0.1;    // heading__error_rate

    matrix_q_updated_ = matrix_q_;

    return;
}

// distance between two points
double PointDistanceSquare(const TrajectoryPoint &point, const double x, const double y) {
    double dx = point.x - x;
    double dy = point.y - y;
    return dx * dx + dy * dy;
}

// normalize the angle (radians) between [-M_PI, M_PI]
double NormalizeAngle(const double angle) {
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}
// Convert degrees to radians
double atan2_to_PI(const double atan2) { return atan2 * M_PI / 180; }

// Calculate and output control commands. This function is called at a fixed frequency.
bool LqrController::ComputeControlCommand(const VehicleState &localization, const TrajectoryData &planning_published_trajectory, ControlCmd &cmd) {
    // trajectory of planning
    trajectory_points_ = planning_published_trajectory.trajectory_points;
    /*
    A matrix (Gear Drive)
    [0.0,                               1.0,                            0.0,                                               0.0;
     0.0,            (-(c_f + c_r) / m) / v,                (c_f + c_r) / m,                   (l_r * c_r - l_f * c_f) / m / v;
     0.0,                               0.0,                            0.0,                                               1.0;
     0.0,   ((lr * cr - lf * cf) / i_z) / v,   (l_f * c_f - l_r * c_r) / i_z,   (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */
    // configure state matrix A
    double v_x = localization.velocity;
    matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / (v_x + 0.01);    // Avoid using the denominator when the speed is 0
    matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / (v_x + 0.01);
    matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / (v_x + 0.01);
    matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / (v_x + 0.01);
    /*
    b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
    */
    // configure input matrix B
    matrix_bd_ = matrix_bd_;
    // cout << "matrix_bd_.row(): " << matrix_bd_.rows() << endl;
    // cout << "matrix_bd_.col(): " << matrix_bd_.cols() << endl;
    // Update state = [Lateral Error, Lateral Error Rate, Heading Error, Heading Error Rate]

    // calculate lateral error and update state x
    UpdateState(localization);

    // Update the state matrix A and discretize the state matrix A
    UpdateMatrix(localization);

    // cout << "matrix_bd_.row(): " << matrix_bd_.rows() << endl;
    // cout << "matrix_bd_.col(): " << matrix_bd_.cols() << endl;

    // Solve Lqr Problem
    SolveLQRProblem(matrix_ad_, matrix_bd_, matrix_q_, matrix_r_, lqr_eps_, lqr_max_iteration_, &matrix_k_);

    // calculate feedback, steer = -K * state.
    std::cout << "matrix_k_: " << matrix_k_ << std::endl;
    double steer_angle_feedback = -(matrix_k_ * matrix_state_)(0, 0);

    // calculate feedforward control and steady state steer angle
    double steer_angle_feedforward = 0.0;
    steer_angle_feedforward = ComputeFeedForward(localization, ref_curv_);
    double steer_angle = steer_angle_feedback - 0.9 * steer_angle_feedforward;
    // limit max steer angle of front wheel
    if (steer_angle >= atan2_to_PI(20.0)) {
        steer_angle = atan2_to_PI(20.0);
    } else if (steer_angle <= -atan2_to_PI(20.0)) {
        steer_angle = -atan2_to_PI(20.0);
    }
    // Set the steer commands
    cmd.steer_target = steer_angle;

    return true;
}

void LqrController::UpdateState(const VehicleState &vehicle_state) {
    // LateralControlError lat_con_err;
    std::shared_ptr<LateralControlError> lat_con_err = std::make_shared<LateralControlError>();
    // calculate lateral error;
    ComputeLateralErrors(vehicle_state.x, vehicle_state.y, vehicle_state.heading, vehicle_state.velocity, vehicle_state.angular_velocity, vehicle_state.acceleration, lat_con_err);

    // State matrix update;
    matrix_state_(0, 0) = lat_con_err->lateral_error;
    matrix_state_(1, 0) = lat_con_err->lateral_error_rate;
    matrix_state_(2, 0) = lat_con_err->heading_error;
    matrix_state_(3, 0) = lat_con_err->heading_error_rate;

    // cout << "lateral_error: " << (lat_con_err->lateral_error) << endl;
    // cout << "heading_error: " << (lat_con_err->heading_error) << endl;
}

void LqrController::UpdateMatrix(const VehicleState &vehicle_state) {
    Eigen::MatrixXd matrix_a_temp = Eigen::MatrixXd::Identity(matrix_a_.rows(), matrix_a_.cols());
    matrix_ad_ = matrix_a_ * ts_ + matrix_a_temp;
}

double LqrController::ComputeFeedForward(const VehicleState &localization, double ref_curvature) {
    if (isnan(ref_curvature)) {
        ref_curvature = 0;
    }
    // Feedforward control part that calculates the required steering angle of the vehicle's front wheels
    return atan(ref_curvature * (lf_ + lr_));
}

void LqrController::ComputeLateralErrors(const double x, const double y, const double theta, const double linear_v, const double angular_v, const double linear_a, LateralControlErrorPtr &lat_con_err) {
    TrajectoryPoint current_closest_point = this->QueryNearestPointByPosition(x, y);

    // double closest_point_x_in_vehicle_coordinate =   (current_closest_point.x - x) * cos(theta) + (current_closest_point.y - y) * sin(theta);
    // Rotation matrix
    double e_y = -(current_closest_point.x - x) * sin(theta) + (current_closest_point.y - y) * cos(theta);
    // The reference heading angle of the point closest to the vehicle on the path.
    // If it is greater than the current heading angle of the vehicle, the vehicle should turn left to track the heading.
    double e_theta = current_closest_point.heading - theta;

    if (e_theta > M_PI) {
        e_theta = e_theta - M_PI * 2;
    }
    if (e_theta < -M_PI) {
        e_theta = e_theta + M_PI * 2;
    }
    // angular_v: Rate of change of vehicle heading angle
    // current_closest_point.v: velocity of reference
    // current_closest_point.kappa: curvature
    double e_y_dot = 0 + linear_v * std::sin(e_theta);
    double e_theta_dot = angular_v - current_closest_point.kappa * current_closest_point.v;

    lat_con_err->lateral_error = e_y;
    lat_con_err->lateral_error_rate = e_y_dot;
    lat_con_err->heading_error = e_theta;
    lat_con_err->heading_error_rate = e_theta_dot;
}

// Query the trajectory point and curvature closest to the current position
TrajectoryPoint LqrController::QueryNearestPointByPosition(const double x, const double y) {
    double d_min = PointDistanceSquare(trajectory_points_.front(), x, y);
    size_t index_min = 0;

    for (size_t i = 1; i < trajectory_points_.size(); ++i) {
        double d_temp = PointDistanceSquare(trajectory_points_[i], x, y);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    ref_curv_ = trajectory_points_[index_min].kappa;    // curvature closest to the current position

    double front_index = index_min + 5;
    if (front_index >= trajectory_points_.size()) {
        ref_curv_front_ = trajectory_points_[index_min].kappa;
    } else {
        ref_curv_front_ = trajectory_points_[front_index].kappa;
    }

    return trajectory_points_[index_min];
}

void LqrController::SolveLQRProblem(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R, const double tolerance, const uint max_num_iteration, Matrix *ptr_K) {
    // prevent matrix dimension error causing subsequence operation to fail
    if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() || Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols()) {
        std::cout << "LQR solver: one or more matrices have incompatible dimensions." << std::endl;
        return;
    }
    Eigen::MatrixXd P = Q;
    Eigen::MatrixXd AT = A.transpose();
    Eigen::MatrixXd BT = B.transpose();
    Eigen::MatrixXd Rinv = R.inverse();

    double diff;
    Eigen::MatrixXd P_next;
    for (size_t i = 0; i < max_num_iteration; i++) {
        P_next = AT * P * A - AT * P * B * (R + BT * P * B).inverse() * BT * P * A + Q;
        diff = fabs((P_next - P).maxCoeff());
        P = P_next;
        // the change in P is less than the preset tolerance
        if (diff < tolerance) {
            break;
        }
    }
    *ptr_K = (R + BT * P * B).inverse() * BT * P * A;
}

}    // namespace control
}    // namespace shenlan
