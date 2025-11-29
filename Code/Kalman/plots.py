def ekf_step(x_prev, P_prev,
             v_wheel, omega_wheel,
             z_cam, camera_on,
             Ts, Q, R_cam,
             xlog, Plog, innovlog, inovusedlog):
    """
    Computes ONE EKF iteration and logs state, covariance and innovation.

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
    xlog : list
        List to store state estimates over time.
    Plog : list
        List to store covariance matrices over time.
    innovlog : list
        List to store innovation vectors over time.
    inovusedlog : list
        List to store booleans indicating when innovation is valid.

    Returns
    -------
    x_new, P_new : updated state and covariance
    """
    """
    xlog = []
    Plog = []
    innovlog = []
    inovusedlog = []
    """

    # ---- PREDICTION ----
    x_pred, P_pred = ekf_predict(x_prev, P_prev, v_wheel, omega_wheel, Ts, Q)

    # ---- UPDATE ----
    x_new, P_new = x_pred, P_pred

    # Flag if the camera was used this step
    used_this_step = False
    innovation = np.zeros((3, 1))

    # If the camera is on and a measurement is available, perform the update.
    if camera_on and (z_cam is not None):
        z = np.array([[z_cam[0]],
                      [z_cam[1]],
                      [wrap_angle(z_cam[2])]])
        x_new, P_new = ekf_update_camera(x_pred, P_pred, z, R_cam)
        H = np.eye(3)
        innovation = z - H @ x_new
        innovation[2, 0] = wrap_angle(innovation[2, 0])
        used_this_step = True

    # ---- LOGGING ----
    xlog.append(x_new.copy())
    Plog.append(P_new.copy())
    innovlog.append(innovation.copy())
    inovusedlog.append(used_this_step)

    return x_new, P_new

def plot_covariance_history(ekf_P_log, Ts):
    """
    Plots the diagonal elements of the covariance matrix P over time.

    Parameters
    ----------
    ekf_P_log : list of np.array, each shape (3,3)
        Covariance matrices stored at each time step.
    Ts : float
        Sampling time (seconds).
    """

    P_array = np.array(ekf_P_log)
    N = P_array.shape[0]
    t = np.arange(N) * Ts

    var_x = P_array[:, 0, 0]
    var_y = P_array[:, 1, 1]
    var_theta = P_array[:, 2, 2]

    plt.figure()
    plt.plot(t, var_x, label="Var(x)")
    plt.plot(t, var_y, label="Var(y)")
    plt.plot(t, var_theta, label="Var(theta)")
    plt.grid(True)
    plt.title("EKF covariance diagonal over time")
    plt.xlabel("Time [s]")
    plt.ylabel("Variance")
    plt.legend()
    plt.show()



def plot_innovation_history(innov_log, innov_used_log, Ts):
    """
    Plots the innovation (measurement residual) for x, y, theta over time.

    
    Parameters
    ----------
    innov_log : list of np.array, each shape (3,1)
        Innovation vectors recorded at each time step.
    innov_used_log : list of bool
        Flags indicating whether the innovation is valid (camera was used).
    Ts : float
        Sampling time.
    """

    innov_array = np.array([y.flatten() for y in innov_log])
    N = innov_array.shape[0]
    t = np.arange(N) * Ts

    # Create a boolean mask where camera was actually used.
    mask = np.array(innov_used_log, dtype=bool)

    plt.figure()
    plt.plot(t[mask], innov_array[mask, 0], "o", label="innovation x")
    plt.plot(t[mask], innov_array[mask, 1], "o", label="innovation y")
    plt.plot(t[mask], innov_array[mask, 2], "o", label="innovation theta")
    plt.grid(True)
    plt.title("Innovation (measurement residual) at camera update times")
    plt.xlabel("Time [s]")
    plt.ylabel("Innovation")
    plt.legend()
    plt.show()


def plot_error_vs_sigma(ekf_x_log, ref_x_log, ekf_P_log, Ts):
    """
    Compares EKF error to the predicted standard deviation (sqrt of variance).

    Parameters
    ----------
    ekf_x_log : list of np.array, each shape (3,1)
        EKF state estimates over time.
    ref_x_log : list of np.array, each shape (3,1)
        Reference or "ground truth" states over time.
    ekf_P_log : list of np.array, each shape (3,3)
        Covariance matrices over time.
    Ts : float
        Sampling time.
    """

    x_est = np.array([x.flatten() for x in ekf_x_log])
    x_ref = np.array([x.flatten() for x in ref_x_log])
    P_array = np.array(ekf_P_log)
    N = x_est.shape[0]
    t = np.arange(N) * Ts

    # Compute error for x, y, theta as arrays of length N.
    err_x = x_est[:, 0] - x_ref[:, 0]
    err_y = x_est[:, 1] - x_ref[:, 1]
    err_theta = x_est[:, 2] - x_ref[:, 2]

    # Wrap the theta error to stay within [-pi, pi].
    err_theta = np.array([wrap_angle(e) for e in err_theta])

    # Compute standard deviation (sqrt of variance) for each state from P.
    sigma_x = np.sqrt(P_array[:, 0, 0])
    sigma_y = np.sqrt(P_array[:, 1, 1])
    sigma_theta = np.sqrt(P_array[:, 2, 2])

    # Plot x error vs ±2 sigma_x.
    plt.figure()
    plt.plot(t, err_x, label="error x")
    plt.plot(t, 2 * sigma_x, "--", label="+2σ_x")
    plt.plot(t, -2 * sigma_x, "--", label="-2σ_x")
    plt.grid(True)
    plt.title("Error in x vs ±2σ_x")
    plt.xlabel("Time [s]")
    plt.ylabel("Error [m]")
    plt.legend()
    plt.show()

    # Plot y error vs ±2 sigma_y.
    plt.figure()
    plt.plot(t, err_y, label="error y")
    plt.plot(t, 2 * sigma_y, "--", label="+2σ_y")
    plt.plot(t, -2 * sigma_y, "--", label="-2σ_y")
    plt.grid(True)
    plt.title("Error in y vs ±2σ_y")
    plt.xlabel("Time [s]")
    plt.ylabel("Error [m]")
    plt.legend()
    plt.show()

    # Plot theta error vs ±2 sigma_theta.
    plt.figure()
    plt.plot(t, err_theta, label="error theta")
    plt.plot(t, 2 * sigma_theta, "--", label="+2σ_theta")
    plt.plot(t, -2 * sigma_theta, "--", label="-2σ_theta")
    plt.grid(True)
    plt.title("Error in theta vs ±2σ_theta")
    plt.xlabel("Time [s]")
    plt.ylabel("Error [rad]")
    plt.legend()
    plt.show()




import matplotlib.pyplot as plt
import numpy as np

plt.ion()  # interactive mode ON

fig, ax = plt.subplots()
line_x, = ax.plot([], [], label="Var(x)")
line_y, = ax.plot([], [], label="Var(y)")
line_th, = ax.plot([], [], label="Var(theta)")

ax.set_xlabel("Time step")
ax.set_ylabel("Variance")
ax.legend()
ax.grid(True)

def realtime_plot_P(Plog):
    # Extract diagonal elements
    var_x = [P[0,0] for P in Plog]
    var_y = [P[1,1] for P in Plog]
    var_th = [P[2,2] for P in Plog]

    t = np.arange(len(Plog))

    # Update plot data
    line_x.set_data(t, var_x)
    line_y.set_data(t, var_y)
    line_th.set_data(t, var_th)

    ax.relim()
    ax.autoscale_view()
    plt.pause(0.001)   # plot refresh


# After each ekf_step(...)
if len(Plog) % 5 == 0:   # update the plot every 5 iterations
    realtime_plot_P(Plog)
