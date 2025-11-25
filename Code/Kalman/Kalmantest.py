import numpy as np  # Import NumPy for matrix and vector calculations

# ==========================
# PARAMETERS
# ==========================

Ts = 1.0  # Sampling period in seconds (camera and filter update every 1 second)

# Process noise covariance matrix Q (model uncertainty)
# You should tune these numbers based on how noisy your odometry is.
Q = np.diag([0.01, 0.01, 0.001])  # Diagonal matrix: [q_x, q_y, q_theta]

# Measurement noise covariance matrix R for the camera
# This describes how noisy the camera pose measurement is.
R_cam = np.diag([0.02, 0.02, 0.002])  # Diagonal matrix: [r_x, r_y, r_theta]

# ==========================
# HELPER FUNCTIONS
# ==========================

def wrap_angle(angle):
    """
    Wrap an angle in radians to the range [-pi, pi].
    This keeps theta from growing without bound.
    """
    # Use modulo operation to bring the angle into [-pi, pi]
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


# ==========================
# PREDICTION STEP
# ==========================

def ekf_predict(x, P, v, omega, Ts, Q):
    """
    Extended Kalman Filter prediction step for a unicycle model.

    Inputs:
        x     : 3x1 state vector [x, y, theta]^T (as a NumPy array)
        P     : 3x3 covariance matrix of the state estimate
        v     : linear velocity (e.g. m/s or mm/s, consistent with x,y units)
        omega : angular velocity (rad/s)
        Ts    : sampling time (s)
        Q     : 3x3 process noise covariance matrix

    Outputs:
        x_pred : 3x1 predicted state vector after Ts seconds
        P_pred : 3x3 predicted covariance matrix
    """

    # Extract current state components for readability
    x_pos = x[0, 0]  # Current x position
    y_pos = x[1, 0]  # Current y position
    theta = x[2, 0]  # Current orientation (rad)

    # ---------- Nonlinear state prediction (unicycle model) ----------
    # x_{k+1} = x_k + Ts * v * cos(theta_k)
    x_pos_pred = x_pos + Ts * v * np.cos(theta)
    # y_{k+1} = y_k + Ts * v * sin(theta_k)
    y_pos_pred = y_pos + Ts * v * np.sin(theta)
    # theta_{k+1} = theta_k + Ts * omega
    theta_pred = theta + Ts * omega
    # Wrap orientation so it stays in [-pi, pi]
    theta_pred = wrap_angle(theta_pred)

    # Stack predicted components into a 3x1 state vector
    x_pred = np.array([[x_pos_pred],
                       [y_pos_pred],
                       [theta_pred]])

    # ---------- Jacobian of the motion model wrt state (A_k) ----------
    # A_k = df/dx evaluated at current x and u

    A = np.array([
        [1.0, 0.0, -Ts * v * np.sin(theta)],
        [0.0, 1.0,  Ts * v * np.cos(theta)],
        [0.0, 0.0,  1.0]
    ])

    # ---------- Covariance prediction ----------
    # P_{k+1|k} = A_k * P_{k|k} * A_k^T + Q
    P_pred = A @ P @ A.T + Q

    # Return predicted state and covariance
    return x_pred, P_pred


# ==========================
# UPDATE STEP (CAMERA POSE)
# ==========================

def ekf_update_camera(x_pred, P_pred, z_cam, R_cam):
    """
    EKF update step with a full pose measurement from camera.

    Measurement model:
        z = H * x + v, with H = I_3 (identity)

    Inputs:
        x_pred : 3x1 predicted state vector [x, y, theta]^T
        P_pred : 3x3 predicted covariance matrix
        z_cam  : 3x1 camera measurement [x_cam, y_cam, theta_cam]^T
        R_cam  : 3x3 measurement noise covariance for the camera

    Outputs:
        x_upd : 3x1 updated state estimate
        P_upd : 3x3 updated covariance
    """

    # Measurement matrix H = identity (3x3), because camera directly measures [x, y, theta]
    H = np.eye(3)

    # ---------- Innovation (measurement residual) ----------
    # y = z - H * x_pred
    y = z_cam - (H @ x_pred)

    # Ensure the orientation residual is wrapped nicely:
    # Only the third component (theta) needs wrapping.
    y[2, 0] = wrap_angle(y[2, 0])

    # ---------- Innovation covariance ----------
    # S = H * P_pred * H^T + R
    S = H @ P_pred @ H.T + R_cam

    # ---------- Kalman gain ----------
    # K = P_pred * H^T * S^{-1}
    K = P_pred @ H.T @ np.linalg.inv(S)

    # ---------- Updated state ----------
    # x_upd = x_pred + K * y
    x_upd = x_pred + K @ y

    # Wrap theta again for safety
    x_upd[2, 0] = wrap_angle(x_upd[2, 0])

    # ---------- Updated covariance ----------
    # P_upd = P_pred - K * H * P_pred
    P_upd = P_pred - K @ H @ P_pred

    # Return updated state and covariance
    return x_upd, P_upd


# ==========================
# EXAMPLE USAGE
# ==========================

if __name__ == "__main__":
    # Initial state: assume we start at origin, heading 0 rad
    x_est = np.array([[0.0],   # x position
                      [0.0],   # y position
                      [0.0]])  # theta (rad)

    # Initial covariance: large uncertainty in position, medium in theta
    P_est = np.diag([1.0, 1.0, 0.5])

    # Example: simulate 5 time steps, with odometry + camera each second
    for k in range(5):
        print(f"\nTime step k = {k}")

        # ----- Fake odometry for this 1-second interval -----
        # Example: robot drives 0.1 m/s straight, omega = 0.05 rad/s
        v_k = 0.1      # 0.1 m/s
        omega_k = 0.05 # 0.05 rad/s

        # ----- Prediction step -----
        x_pred, P_pred = ekf_predict(x_est, P_est, v_k, omega_k, Ts, Q)

        print("Predicted state x_pred =")
        print(x_pred)
        print("Predicted covariance P_pred =")
        print(P_pred)

        # ----- Camera measurement available every second -----
        # Here we create a noisy measurement around the true predicted state
        # In reality, this comes from your vision system.
        noise = np.array([[np.random.normal(0.0, 0.1)],   # noise in x
                          [np.random.normal(0.0, 0.1)],   # noise in y
                          [np.random.normal(0.0, 0.05)]]) # noise in theta

        # Camera measures: predicted pose + some noise
        z_cam = x_pred + noise

        # ----- Update step with camera -----
        x_est, P_est = ekf_update_camera(x_pred, P_pred, z_cam, R_cam)

        print("Updated state x_est =")
        print(x_est)
        print("Updated covariance P_est =")
        print(P_est)


