#ifndef EKF_HPP
#define EKF_HPP

#include <Eigen/Dense>
#include <vector>

class ExtendedKalmanFilter {
public:
    /**
     * Constructor to initialize the EKF.
     * 
     * @param state_dim Number of state variables (e.g., position, velocity).
     * @param control_dim Number of control input variables (e.g., acceleration).
     * @param dt Time step between predictions.
     */
    ExtendedKalmanFilter(int state_dim, int control_dim, double dt);

    // Predicts the next state based on the model and updates the covariance.
    void predict();

    // Updates the state with a measurement.
    void update(const Eigen::VectorXd& z);

    // Sets the control input vector.
    void set_control_input(const Eigen::VectorXd& u);

    // Sets the measurement noise covariance matrix.
    void set_measurement_noise(const Eigen::MatrixXd& R);

    // Initializes the filter with an initial state, process noise, measurement noise, and measurement matrix.
    void initialize(const Eigen::VectorXd& initial_state, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R, const Eigen::MatrixXd& H);

private:
    // Function for state transition (model of system dynamics).
    Eigen::VectorXd state_transition_function(const Eigen::VectorXd& x, const Eigen::VectorXd& u, double dt);

    // Function to compute the Jacobian of the state transition function.
    Eigen::MatrixXd calculate_F_jacobian(double dt);

    int state_dim_;  // Dimension of the state vector
    int control_dim_;  // Dimension of the control vector
    double dt_;  // Time step

    Eigen::VectorXd x_;  // State vector (e.g., position, velocity)
    Eigen::MatrixXd P_;  // Covariance matrix
    Eigen::MatrixXd Q_;  // Process noise covariance
    Eigen::MatrixXd R_;  // Measurement noise covariance
    Eigen::MatrixXd H_;  // Measurement matrix
    Eigen::VectorXd u_;  // Control input (e.g., acceleration)
};

#endif  // EKF_HPP
