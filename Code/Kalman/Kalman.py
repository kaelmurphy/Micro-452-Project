import numpy as np

# ==========================
# ANGLE WRAP
# ==========================
def wrap_angle(a):
    return (a + np.pi) % (2*np.pi) - np.pi


# ==========================
# EKF PREDICTION STEP
# ==========================
def ekf_predict(x, P, v, omega, Ts, Q):
    x_pos, y_pos, theta = x.flatten()

    # Nonlinear model
    x_pos_pred = x_pos + Ts * v * np.cos(theta)
    y_pos_pred = y_pos + Ts * v * np.sin(theta)
    theta_pred = wrap_angle(theta + Ts * omega)

    x_pred = np.array([[x_pos_pred],
                       [y_pos_pred],
                       [theta_pred]])

    # Jacobian
    A = np.array([
        [1, 0, -Ts * v * np.sin(theta)],
        [0, 1,  Ts * v * np.cos(theta)],
        [0, 0,  1]
    ])

    # Covariance
    P_pred = A @ P @ A.T + Q
    return x_pred, P_pred


# ==========================
# EKF UPDATE STEP (CAMERA)
# ==========================
def ekf_update_camera(x_pred, P_pred, z_cam, R_cam):
    H = np.eye(3)

    # Innovation
    y = z_cam - H @ x_pred
    y[2,0] = wrap_angle(y[2,0])

    # Innovation covariance
    S = H @ P_pred @ H.T + R_cam

    # Kalman Gain
    K = P_pred @ H.T @ np.linalg.inv(S)

    # Updated state
    x_upd = x_pred + K @ y
    x_upd[2,0] = wrap_angle(x_upd[2,0])

    # Updated covariance
    P_upd = P_pred - K @ H @ P_pred

    return x_upd, P_upd


# ==========================
# EKF STEP = WHAT YOU CALL EVERY TIME
# ==========================
def ekf_step(x_prev, P_prev,
             v_wheel, omega_wheel,
             z_cam, camera_on,
             Ts, Q, R_cam):
    """
    Computes ONE EKF iteration.

    Parameters
    ----------
    x_prev : (3,1)
        Previous estimated state.
    P_prev : (3,3)
        Previous covariance.
    v_wheel, omega_wheel : floats
        Odometry (linear and angular velocity).
    z_cam : (3,) or None
        Camera measurement (x, y, theta) when available.
    camera_on : bool
        Whether to use z_cam.
    Ts : float
        Sampling period (seconds)
    Q : (3,3)
        Process noise
    R_cam : (3,3)
        Camera noise

    Returns
    -------
    x_new, P_new : updated state and covariance
    """

    # ---- PREDICTION ----
    x_pred, P_pred = ekf_predict(x_prev, P_prev, v_wheel, omega_wheel, Ts, Q)

    # ---- UPDATE ----
    if camera_on and z_cam is not None:
        z = np.array([[z_cam[0]],
                      [z_cam[1]],
                      [wrap_angle(z_cam[2])]])
        x_new, P_new = ekf_update_camera(x_pred, P_pred, z, R_cam)
    else:
        x_new, P_new = x_pred, P_pred

    return x_new, P_new



