#include "ekf.hpp"
#include <iostream>

ExtendedKalmanFilter::ExtendedKalmanFilter(int state_dim, int control_dim, double dt)
    : state_dim_(state_dim), control_dim_(control_dim), dt_(dt) {
    // Initialize the state estimate vector (x) to zero
    x_ = Eigen::VectorXd::Zero(state_dim_);

    // Initialize the covariance matrix (P) to small uncertainty values
    P_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 0.1;

    // Initialize process noise covariance matrix (Q)
    Q_ = Eigen::MatrixXd::Identity(state_dim_, state_dim_) * 0.01;

    // Initialize measurement noise covariance matrix (R) (this will be set later)
    R_ = Eigen::MatrixXd::Identity(1, 1) * 0.1;

    // Initialize the control input (u) to zero
    u_ = Eigen::VectorXd::Zero(control_dim_);

    // Initialize the measurement matrix (H) for the simple case
    H_ = Eigen::MatrixXd::Zero(1, state_dim_);
    H_(0, 0) = 1;  // Measurement only depends on the position (1D)
}

void ExtendedKalmanFilter::initialize(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& H) {
    // Set the initial state estimate
    x_ = initial_state;

    // Set the process noise covariance
    Q_ = Q;

    // Set the measurement noise covariance
    R_ = R;

    // Set the measurement matrix
    H_ = H;
}

Eigen::VectorXd ExtendedKalmanFilter::state_transition_function(const Eigen::VectorXd& x, const Eigen::VectorXd& u, double dt) {
    // Extract position, velocity, and control input (acceleration)
    double position = x(0);
    double velocity = x(1);
    double acceleration = u(0);

    // Use kinematic equations to compute the new position and velocity
    double new_position = position + velocity * dt + 0.5 * acceleration * dt * dt;
    double new_velocity = velocity + acceleration * dt;

    // Return the updated state vector
    Eigen::VectorXd new_state(2);
    new_state << new_position, new_velocity;
    return new_state;
}

Eigen::MatrixXd ExtendedKalmanFilter::calculate_F_jacobian(double dt) {
    // Jacobian matrix of the state transition function
    Eigen::MatrixXd F(2, 2);
    F << 1, dt,  // The partial derivative of position w.r.t. position is 1, and w.r.t. velocity is dt
         0, 1;    // The partial derivative of velocity w.r.t. position is 0, and w.r.t. velocity is 1
    return F;
}

void ExtendedKalmanFilter::predict() {
    // Predict the next state based on the state transition function
    x_ = state_transition_function(x_, u_, dt_);

    // Compute the Jacobian of the state transition function
    Eigen::MatrixXd F = calculate_F_jacobian(dt_);

    // Update the covariance matrix
    P_ = F * P_ * F.transpose() + Q_;
}

void ExtendedKalmanFilter::update(const Eigen::VectorXd& z) {
    // Compute the innovation covariance (S)
    Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;

    // Compute the Kalman gain (K)
    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

    // Compute the innovation (residual between measured and predicted states)
    Eigen::VectorXd y = z - H_ * x_;

    // Update the state estimate with the weighted innovation
    x_ = x_ + K * y;

    // Update the covariance matrix
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
    P_ = (I - K * H_) * P_;
}

void ExtendedKalmanFilter::set_control_input(const Eigen::VectorXd& u) {
    u_ = u;
}

void ExtendedKalmanFilter::set_measurement_noise(const Eigen::MatrixXd& R) {
    R_ = R;
}
