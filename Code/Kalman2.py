# ==========================
# IMPORTS
# ==========================
import numpy as np                      
import matplotlib.pyplot as plt         


# ==========================
# GLOBAL LOGS
# ==========================
xlog = []                               
Plog = []                               
innovlog = []                           
inovusedlog = []                        
zlog = []                               


# ==========================
# ANGLE WRAP
# ==========================
def wrap_angle(a):
    """
    Wraps an angle to the range [-pi, pi].

    """
    return (a + np.pi) % (2 * np.pi) - np.pi  


# ==========================
# EKF PREDICTION STEP
# ==========================
def ekf_predict(x, P, v, omega, Ts, Q):
    """
    EKF prediction step for a unicycle / differential-drive style model.

    """
    x_pos, y_pos, theta = x.flatten()                    

    # Nonlinear motion model (discrete-time)
    x_pos_pred = x_pos + Ts * v * np.cos(theta)          
    y_pos_pred = y_pos + Ts * v * np.sin(theta)          
    theta_pred = wrap_angle(theta + Ts * omega)          

    # Construct predicted state vector
    x_pred = np.array([[x_pos_pred],
                       [y_pos_pred],
                       [theta_pred]])                    

    # Jacobian of the motion model w.r.t. state (A matrix)
    A = np.array([
        [1, 0, -Ts * v * np.sin(theta)],                 
        [0, 1,  Ts * v * np.cos(theta)],                 
        [0, 0,  1]                                       
    ])

    # Covariance prediction using linearized model
    P_pred = A @ P @ A.T + Q                             

    return x_pred, P_pred                                


# ==========================
# EKF UPDATE STEP (CAMERA)
# ==========================
def ekf_update_camera(x_pred, P_pred, z_cam, R_cam):
    """
    EKF update step using a camera measurement.
    """
    H = np.eye(3)                                       

    # Innovation (measurement residual) based on predicted state
    y = z_cam - H @ x_pred                              
    y[2, 0] = wrap_angle(y[2, 0])                       

    # Innovation covariance
    S = H @ P_pred @ H.T + R_cam                        

    # Kalman gain
    K = P_pred @ H.T @ np.linalg.inv(S)                 

    # Updated state estimate
    x_upd = x_pred + K @ y                              
    x_upd[2, 0] = wrap_angle(x_upd[2, 0])               

    # Updated covariance matrix (Joseph form for better numerical stability)
    I = np.eye(3)                                       
    P_upd = (I - K @ H) @ P_pred @ (I - K @ H).T + K @ R_cam @ K.T  # Joseph form covariance update

    return x_upd, P_upd, y                            


# ==========================
# EKF STEP
# ==========================
def ekf_step(x_prev, P_prev,
             v_wheel, omega_wheel,
             z_cam, camera_on,
             Ts, Q, R_cam):
    """
    Perform one full EKF step: prediction + optional camera update,
    and log all relevant quantities.
    """
    global xlog, Plog, zlog, innovlog, inovusedlog      

    # ---- PREDICTION ----
    x_pred, P_pred = ekf_predict(x_prev, P_prev, v_wheel, omega_wheel, Ts, Q)  

    # Initialize flags and innovation for this step
    used_this_step = False                            
    innovation = np.zeros((3, 1))                     

    # ---- UPDATE (if camera is on and measurement is available) ----
    if camera_on and (z_cam is not None):              
        # Convert raw measurement to 3x1 numpy array and wrap the angle
        z = np.array([[z_cam[0]],                     
                      [z_cam[1]],                     
                      [wrap_angle(z_cam[2])]])        

        # Perform EKF camera update
        x_new, P_new, innovation = ekf_update_camera(x_pred, P_pred, z, R_cam)  

        used_this_step = True                          
        zlog.append(z.copy())                          
    else:
        # No measurement update: just carry prediction 
        x_new, P_new = x_pred, P_pred                  
        zlog.append(None)                              

    # ---- LOGGING ----
    xlog.append(x_new.copy())                          
    Plog.append(P_new.copy())                          
    innovlog.append(innovation.copy())                 
    inovusedlog.append(used_this_step)                 

    return x_new, P_new                                


# ==========================
# Covariance History
# ==========================
def plot_covariance_history(Ts):
    """
    Plots the diagonal elements of the covariance matrix P over time.

    Uses the global Plog list.

    """
    global Plog                                        

    P_array = np.array(Plog)                           
    if P_array.size == 0:                              
        print("No covariance data to plot.")           
        return                                         

    N = P_array.shape[0]                               
    t = np.arange(N) * Ts                              

    var_x = P_array[:, 0, 0]                           
    var_y = P_array[:, 1, 1]                           
    var_theta = P_array[:, 2, 2]                       

    fig, ax1 = plt.subplots()
    line_x, = ax1.plot(t, var_x, label="Var(x)")
    line_y, = ax1.plot(t, var_y, label="Var(y)")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel(r"$\sigma_x^2,\,\sigma_y^2\;\;[\mathrm{mm}^2]$")
    ax1.grid(True)
    ax2 = ax1.twinx()
    line_theta, = ax2.plot(t, var_theta, label="Var(theta)", color = 'green')
    ax2.set_ylabel(r"$\sigma_\theta^2\;[\mathrm{rad}^2]$")
    lines = [line_x, line_y, line_theta]
    labels = [line.get_label() for line in lines]
    ax1.legend(lines, labels, loc="upper left")
    plt.title("EKF covariance diagonal over time (two y-axes)")
                                
# ==========================
# Innovation History
# ==========================
def plot_innovation_history(Ts):
    """
    Plots the innovation (measurement residual) for x, y, theta over time,
    only at timesteps where the camera update was used.

    Uses global innovlog and inovusedlog.

    """
    global innovlog, inovusedlog

    # If there are no innovations at all, exit early.
    if len(innovlog) == 0:
        print("No innovation data to plot.")
        return

    # Convert innovation list into an array shape (N, 3)
    innov_array = np.array([y.flatten() for y in innovlog])
    N = innov_array.shape[0]

    # Create time vector in seconds
    t = np.arange(N) * Ts

    # Mask where camera was used
    mask = np.array(inovusedlog, dtype=bool)

    if not np.any(mask):
        print("No camera updates used; nothing to plot.")
        return

    # Extract individual series
    innov_x = innov_array[:, 0]
    innov_y = innov_array[:, 1]
    innov_theta = innov_array[:, 2]

    # ---- FIGURE + AXIS SETUP ----
    fig, ax1 = plt.subplots()

    # LEFT AXIS → x & y innovations
    line_x = ax1.plot(t[mask], innov_x[mask], label="innovation x")[0]
    line_y = ax1.plot(t[mask], innov_y[mask], label="innovation y")[0]

    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Innovation (x, y)")
    ax1.grid(True)

    # RIGHT AXIS → theta innovations
    ax2 = ax1.twinx()
    line_theta = ax2.plot(
        t[mask], innov_theta[mask], color="purple", label="innovation theta"
    )[0]

    ax2.set_ylabel("Innovation (theta)")

    # ---- COMBINED LEGEND ----
    lines = [line_x, line_y, line_theta]
    labels = [line.get_label() for line in lines]
    ax1.legend(lines, labels, loc="upper left")

    # ---- TITLE ----
    plt.title("Innovation history (x,y on left axis — theta on right axis)")                        


# ==========================
# Plot error vs Sigma
# ==========================
def plot_error_vs_sigma(Ts):
    """
    Compares EKF error to the predicted standard deviation (sqrt of variance).

    Here we compare the EKF estimate xlog against the measurements stored in zlog
    (only at timesteps where a measurement exists). If you have real ground truth,
    you can store it in zlog instead of raw measurements.
    """
    global xlog, zlog, Plog                            

    if len(xlog) == 0 or len(Plog) == 0:               
        print("No data in xlog or Plog to plot error vs sigma.")  
        return                                        

    # Mask only the timesteps where a reference/measurement exists
    mask = np.array([z is not None for z in zlog], dtype=bool)   

    if not np.any(mask):                              
        print("No reference measurements in zlog; cannot plot error vs sigma.") 
        return                                         

    # Convert entire logs to arrays
    x_est_all = np.array([x.flatten() for x in xlog]) 
    P_all = np.array(Plog)                            

    # Apply mask to keep only measured timesteps
    x_est = x_est_all[mask]                           
    P_array = P_all[mask]                             

    # Build array of reference states / measurements
    zref = np.array([z.flatten() for z in zlog if z is not None]) 

    N = x_est.shape[0]                                
    t = np.arange(N) * Ts                             

    # Compute error for x, y, theta as arrays of length N
    err_x = x_est[:, 0] - zref[:, 0]                 
    err_y = x_est[:, 1] - zref[:, 1]                 
    err_theta = x_est[:, 2] - zref[:, 2]              

    # Wrap the theta error to stay within [-pi, pi]
    err_theta = np.array([wrap_angle(e) for e in err_theta])  

    # Compute standard deviation (sqrt of variance) for each state from P
    sigma_x = np.sqrt(P_array[:, 0, 0])                
    sigma_y = np.sqrt(P_array[:, 1, 1])                
    sigma_theta = np.sqrt(P_array[:, 2, 2])            

    # Plot x error vs ±2 sigma_x
    plt.figure()                                      
    plt.plot(t, err_x, label="error x")                
    plt.plot(t, 2 * sigma_x, "--", label="+2σ_x")      
    plt.plot(t, -2 * sigma_x, "--", label="-2σ_x")     
    plt.grid(True)                                    
    plt.title("Error in x vs ±2σ_x")                   
    plt.xlabel("Time [s]")                             
    plt.ylabel("Error [mm]")                            
    plt.legend()                                        

    # Plot y error vs ±2 sigma_y
    plt.figure()                                       
    plt.plot(t, err_y, label="error y")                
    plt.plot(t, 2 * sigma_y, "--", label="+2σ_y")      
    plt.plot(t, -2 * sigma_y, "--", label="-2σ_y")    
    plt.grid(True)                                     
    plt.title("Error in y vs ±2σ_y")                  
    plt.xlabel("Time [s]")                             
    plt.ylabel("Error [mm]")                            
    plt.legend()                                        

    # Plot theta error vs ±2 sigma_theta
    plt.figure()                                      
    plt.plot(t, err_theta, label="error theta")        
    plt.plot(t, 2 * sigma_theta, "--", label="+2σ_theta")  
    plt.plot(t, -2 * sigma_theta, "--", label="-2σ_theta") 
    plt.grid(True)                                     
    plt.title("Error in theta vs ±2σ_theta")          
    plt.xlabel("Time [s]")                             
    plt.ylabel("Error [rad]")                          
    plt.legend()                                        


# ==========================
# LOG RESET HELPER
# ==========================
def reset_ekf_logs():
    """
    Reset all EKF logs. Call this once before starting a new simulation/run.
    """
    global xlog, Plog, innovlog, inovusedlog, zlog     # Use global log variables
    xlog = []                                          # Clear state log
    Plog = []                                          # Clear covariance log
    innovlog = []                                      # Clear innovation log
    inovusedlog = []                                   # Clear camera usage log
    zlog = []                                          # Clear measurement / reference log
